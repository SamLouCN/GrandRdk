"""相机修正工具（独立文件，依赖 numpy、opencv-python 或 opencv-contrib-python）。

默认自动加载项目根目录 camera_correction_params.json：640×480，空气焦距 318.8 px，畸变按零，
清水折射率 1.333，针孔到玻璃距离 0.013 m（估计值）。
算法针对下视相机、平面防水窗及近似平面的池底；标定使用空气照片。
改参数后，原任务程序需要重新示教。本文件不会修改原工程配置。

直接调用：
    corrector = create_corrector().for_frame(frame_1280x960)
    frame = cv2.rotate(frame_1280x960, cv2.ROTATE_180)
    fixed = corrector.undistort(frame, floor_distance_m=0.98)
    mask = corrector.valid_mask(0.98)

命令行：
    python camera_correction.py info
    python camera_correction.py export --output camera_params.json
    python camera_correction.py image --input raw.jpg --output fixed.png --floor-distance 0.98
    python camera_correction.py video --input raw.avi --output fixed.avi --depth-cm 12
    python camera_correction.py calibrate --images air_photos --columns 9 --rows 6 --square-mm 25 --output air_camera.json

--floor-distance 单位米（沿用主工程相机离底高度的传入约定）。
--depth-cm 是艇顶压力计读数；默认池水深 1.3 m，相机低于压力计 0.20 m。
默认保持输入分辨率，按录像模式换算内参；本工具不能从裁剪图恢复丢失视场。
--rotate-180 只用于尚未转正的原始画面，已转正的录像不要重复旋转。
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

DEFAULT_PARAMS_PATH = Path(__file__).resolve().parents[1] / 'camera_correction_params.json'


def default_camera_params():
    """返回新的参数字典；焦距来自实跑反算，畸变为零假设，玻璃距离未实测。"""
    return dict(model="air_intrinsics_flat_port", width=640, height=480,
                matrix=[[318.8, 0, 320.0], [0, 318.8, 240.0], [0, 0, 1]],
                distortion=None, n_water=1.333, glass_distance_m=0.013,
                note="10-06 实跑按一格接头中心距 0.82 m 反算；畸变按零；玻璃距离为估计值")


def load_camera_params(path=None):
    """读取独立相机 JSON 或部署 event_params.json 的 overrides.camera。

    未给文件时自动读取项目根目录 JSON，文件不存在才使用代码默认值。
    标定文件没有折射率、玻璃距离时补入上述默认值。
    """
    if path is None:
        if not DEFAULT_PARAMS_PATH.is_file():
            return default_camera_params()
        path = DEFAULT_PARAMS_PATH
    document = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if "overrides" in document:
        camera = document["overrides"].get("camera", default_camera_params())
    else:
        camera = document
    if camera is None:
        raise ValueError("camera 为 null，没有可供本工具使用的相机参数")
    for name in ("width", "height", "matrix"):
        if name not in camera:
            raise ValueError(f"相机文件缺少 {name}")
    return dict(camera, n_water=camera.get("n_water", 1.333),
                glass_distance_m=camera.get("glass_distance_m", 0.013))


def save_camera_params(camera, path):
    """保存独立 JSON；保留已有文件，不覆盖既有标定结果。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(camera, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def camera_height_m(depth_cm, water_depth_m=1.3, camera_below_sensor_m=0.20):
    """压力计深度换算为相机离池底高度，沿用 event_task.py 的定义与 0.05 m 下限。"""
    return max(0.05, water_depth_m-float(depth_cm)/100-camera_below_sensor_m)


def adapt_camera_params(camera, input_width, input_height, fit='center-crop'):
    """将标定内参换算到录像模式，输出保持输入分辨率。

    center-crop 假设不同长宽比录像来自同一视场居中裁切；resize 假设
    标定整幅视场被缩放到录像尺寸；strict 只接受相同长宽比。
    无法仅凭视频尺寸判断相机的真实裁切模式，假设需由采集设置核对。
    """
    iw, ih = int(input_width), int(input_height)
    rw, rh = int(camera['width']), int(camera['height'])
    if min(iw, ih, rw, rh) < 2:
        raise ValueError('视频和内参尺寸必须至少为2像素')
    if fit not in ('center-crop', 'resize', 'strict'):
        raise ValueError('未知内参适配模式')
    same_aspect = iw * rh == ih * rw
    if fit == 'strict' and not same_aspect:
        raise ValueError(f'视频{iw}×{ih}与内参{rw}×{rh}比例不一致；'
                         '请提供此录像模式内参或使用 --camera-fit center-crop')
    if fit == 'resize':
        transform = np.diag([iw / rw, ih / rh, 1.0])
    else:
        scale = max(iw / rw, ih / rh)
        transform = np.array([[scale, 0, (iw - rw * scale) / 2],
                              [0, scale, (ih - rh * scale) / 2], [0, 0, 1]])
    width, height = iw, ih
    matrix = transform @ np.asarray(camera['matrix'], dtype=float)
    adapted = dict(camera, width=width, height=height, matrix=matrix.tolist())
    info = dict(mode=fit, reference_size=[rw, rh], input_size=[iw, ih],
                processing_size=[width, height], aspect_changed=not same_aspect,
                assumption='same field of view, centered crop' if fit == 'center-crop'
                else 'same aspect ratio' if fit == 'strict' else 'full field of view resized',
                reference_to_processing=transform.tolist(), preserve_resolution=True)
    return adapted, info


def prepare_frame(frame, width=640, height=480, rotate_180=False):
    """显式尺寸变换的旧接口；原分辨率流程使用 for_frame/undistort。

    width/height 应与内参一致。禁止把不同长宽比的画面拉伸。
    """
    h, w = frame.shape[:2]
    if abs(w * height - h * width) > max(w, h) or w < width or h < height:
        raise ValueError(f"输入{w}×{h}无法直接适配内参{width}×{height}；"
                         "请先调用 adapt_camera_params 换算录像模式内参")
    if (w, h) != (width, height):
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    return cv2.rotate(frame, cv2.ROTATE_180) if rotate_180 else frame


class FlatPortCorrector:
    def __init__(self, width, height, matrix, distortion=None, n_water=1.333, glass_distance_m=0.0):
        self.w, self.h = int(width), int(height)
        m = np.asarray(matrix, float)
        if (self.w <= 0 or self.h <= 0 or m.shape != (3, 3)
                or not np.isfinite(m).all() or m[0, 0] <= 0 or m[1, 1] <= 0
                or m[0, 1] != 0 or m[1, 0] != 0 or not np.allclose(m[2], [0, 0, 1])):
            raise ValueError('需要正尺寸、正焦距、零 skew 的有限 3×3 空气内参')
        self.fx, self.fy, self.cx, self.cy = float(m[0, 0]), float(m[1, 1]), float(m[0, 2]), float(m[1, 2])
        self.dist = np.zeros(5) if distortion is None else np.asarray(distortion, float).reshape(-1)
        self.n = float(n_water)
        self.d = float(glass_distance_m)
        if (not np.isfinite(self.dist).all() or len(self.dist) not in (4, 5)
                or not math.isfinite(self.n) or not math.isfinite(self.d)
                or self.n < 1 or self.d < 0):
            raise ValueError("折射率不小于 1，玻璃距离不为负")
        # 传感器上下/左右边缘对应的水中角；输出焦距取能把全部内容放进画布的最大值。
        self.theta_w_v = self._water(math.atan((self.h/2)/self.fy))
        self.theta_w_h = self._water(math.atan((self.w/2)/self.fx))
        self.f_out = min((self.h/2)/math.tan(self.theta_w_v), (self.w/2)/math.tan(self.theta_w_h))
        self._maps = None
        self._maps_key = None
        self._frame_correctors = {}

    def for_frame(self, frame, fit='center-crop'):
        """按输入分辨率换算内参并缓存修正器，帧本身不缩放。"""
        h, w = frame.shape[:2]
        if (w, h) == (self.w, self.h):
            return self
        key = (w, h, fit)
        if key not in self._frame_correctors:
            camera = dict(model='air_intrinsics_flat_port', width=self.w, height=self.h,
                          matrix=[[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]],
                          distortion=self.dist.tolist(), n_water=self.n, glass_distance_m=self.d)
            adapted, _ = adapt_camera_params(camera, w, h, fit)
            self._frame_correctors[key] = create_corrector(adapted)
        return self._frame_correctors[key]

    def _water(self, theta_air):
        return math.asin(min(1.0, math.sin(theta_air)/self.n))

    def water_fov_deg(self):
        """按空气内参估计的水下视场；真实值以示教录像里量到的格线间距核对（event_teaching.compile_teaching）。"""
        diag = self._water(math.atan(math.hypot(self.w/2/self.fx, self.h/2/self.fy)))
        return dict(vertical=2*math.degrees(self.theta_w_v), horizontal=2*math.degrees(self.theta_w_h),
                    diagonal=2*math.degrees(diag), output_focal_px=self.f_out)

    def _lut(self, ratio):
        """空气角 -> 玻璃面视点看到的地面归一化偏移 X/D。ratio = 玻璃距离 / 相机离底高度。"""
        limit = math.atan(math.hypot(self.w/2/self.fx, self.h/2/self.fy))*1.05
        theta_a = np.linspace(0.0, min(limit, 1.55), 4000)
        s = np.sin(theta_a)/self.n
        keep = s < 0.9995
        theta_a, s = theta_a[keep], s[keep]
        xi = ratio*np.tan(theta_a)+np.tan(np.arcsin(s))
        return xi, theta_a

    def maps(self, floor_distance_m=None):
        """输出像素 -> 原图像素的采样表。不知道离底高度时按视点在玻璃面处理（d 的影响忽略）。"""
        key = self._distance_ratio(floor_distance_m)
        if self._maps is not None and self._maps_key == key:
            return self._maps
        xx, yy = np.meshgrid(np.arange(self.w, dtype=np.float64), np.arange(self.h, dtype=np.float64))
        projected = self.corrected_to_raw(np.stack([xx, yy], axis=-1), floor_distance_m)
        map_x, map_y = projected[..., 0].astype(np.float32), projected[..., 1].astype(np.float32)
        invalid = ~np.isfinite(projected).all(axis=-1)
        map_x[invalid] = -1
        map_y[invalid] = -1
        self._maps, self._maps_key = (map_x, map_y), key
        return self._maps

    def _distance_ratio(self, distance):
        if distance is not None and (not math.isfinite(float(distance)) or distance <= 0):
            raise ValueError('平面距离必须为正有限值')
        ratio = 0.0 if distance is None or self.d <= 0 else self.d / max(float(distance), 0.05)
        return round(ratio, 3)

    @property
    def output_matrix(self):
        return np.array([[self.f_out, 0, self.cx], [0, self.f_out, self.cy], [0, 0, 1]])

    def corrected_to_raw(self, points, floor_distance_m=None):
        """校正坐标 -> 原始640×480坐标；用于回画同一组检测，域外点返回 NaN。

        与 remap 使用完全相同的模型和距离量化。整条边必须采样映射，
        校正图上的直线在原图上通常为曲线，不能只映射两端后画直线。
        """
        points = np.asarray(points, dtype=np.float64)
        if points.shape[-1] != 2:
            raise ValueError('点坐标最后一维必须为2')
        xi_grid, theta_grid = self._lut(self._distance_ratio(floor_distance_m))
        dx, dy = points[..., 0]-self.cx, points[..., 1]-self.cy
        r_out = np.hypot(dx, dy)
        xi = r_out/self.f_out
        theta_a = np.interp(xi, xi_grid, theta_grid, right=np.nan)
        tan_a = np.tan(theta_a)
        with np.errstate(invalid="ignore", divide="ignore"):
            ux = np.where(r_out > 1e-9, dx/np.maximum(r_out, 1e-9), 0.0)
            uy = np.where(r_out > 1e-9, dy/np.maximum(r_out, 1e-9), 0.0)
        x, y = tan_a*ux, tan_a*uy
        # 空气内参附带的镜头残余畸变按 OpenCV 顺序 k1 k2 p1 p2 k3 正向套用。
        k1, k2, p1, p2, k3 = (list(self.dist)+[0.0]*5)[:5]
        r2 = x*x+y*y
        radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        xd = x*radial+2*p1*x*y+p2*(r2+2*x*x)
        yd = y*radial+p1*(r2+2*y*y)+2*p2*x*y
        return np.stack([self.fx*xd+self.cx, self.fy*yd+self.cy], axis=-1)

    def valid_mask(self, floor_distance_m=None):
        map_x, map_y = self.maps(floor_distance_m)
        return (map_x >= 0) & (map_x <= self.w-1) & (map_y >= 0) & (map_y <= self.h-1)

    def undistort(self, rgb, floor_distance_m=None):
        if rgb.shape[0] != self.h or rgb.shape[1] != self.w:
            raise ValueError("图像尺寸与空气内参不一致")
        map_x, map_y = self.maps(floor_distance_m)
        return cv2.remap(rgb, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

    def assess_distortion(self, corrected_frame, corners, floor_distance_m=None):
        """从实测红杆中心采样评估前后直线度，不从任意单帧猜测标定系数。

        门杆为直线是本诊断的假设。颜色干扰、遮挡和模型误检会影响结果，
        因此它只提供畸变证据，不决定或重估校正参数。
        """
        result = dict(status='insufficient-evidence', parameter_estimation=False, edges={})
        if corners is None or len(corners) != 4:
            return result
        points = np.asarray(corners, dtype=float)
        if not np.isfinite(points).all():
            return result
        red = corrected_frame[..., 2].astype(np.float32) - corrected_frame[..., 1]
        before, after = [], []
        for name, a, b in zip(('top', 'right', 'bottom', 'left'), points, np.roll(points, -1, axis=0)):
            length = np.linalg.norm(b-a)
            if length < 40:
                continue
            normal = np.array([-(b-a)[1], (b-a)[0]]) / length
            radius = max(6, int(12 * self.w / 640))
            offsets = np.arange(-radius, radius + 1, dtype=np.float32)
            centers = a + np.linspace(0.15, 0.85, 25)[:, None] * (b-a)
            samples = centers[:, None, :] + offsets[None, :, None] * normal
            values = cv2.remap(red, samples[..., 0].astype(np.float32),
                               samples[..., 1].astype(np.float32), cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=-255)
            observed = []
            for center, profile in zip(centers, values):
                baseline = float(np.percentile(profile, 25))
                weights = np.maximum(profile - baseline - 10, 0)
                # Profiles filled by a solid blob or black-border transition are unreliable.
                active = int(np.count_nonzero(weights))
                if 1 <= active <= 1.5 * radius and profile.max() - baseline >= 22:
                    observed.append(center + normal * float(np.dot(weights, offsets) / weights.sum()))
            if len(observed) < 12:
                continue
            observed = np.asarray(observed)
            raw = self.corrected_to_raw(observed, floor_distance_m)
            keep = np.isfinite(raw).all(axis=1)
            if np.count_nonzero(keep) < 12:
                continue
            raw_rms, raw_rel = line_straightness(raw[keep])
            fixed_rms, fixed_rel = line_straightness(observed[keep])
            result['edges'][name] = dict(samples=int(keep.sum()), before_rms_px=raw_rms,
                                         after_rms_px=fixed_rms, before_relative=raw_rel,
                                         after_relative=fixed_rel)
            before.append(raw_rel)
            after.append(fixed_rel)
        if len(before) >= 2:
            raw_rel, fixed_rel = float(np.median(before)), float(np.median(after))
            result.update(before_relative=raw_rel, after_relative=fixed_rel)
            if raw_rel > 0.003 and raw_rel - fixed_rel > 0.001:
                result['status'] = 'curvature-reduced'
            elif fixed_rel - raw_rel > 0.001:
                result['status'] = 'curvature-increased'
            else:
                result['status'] = 'no-clear-change'
        return result

    @classmethod
    def from_document(cls, camera, n_water=1.333, glass_distance_m=0.0):
        """读 calibrate_camera.py 生成的空气内参文件（width/height/matrix/distortion）。"""
        if camera.get('model', 'air_intrinsics_flat_port') != 'air_intrinsics_flat_port':
            raise ValueError('当前平面窗修正需要空气内参，不能使用水下等效内参再次修正')
        if 'refraction approximated locally' in camera.get('method', ''):
            raise ValueError('这是旧工具生成的水下等效内参，请改用空气照片重新标定')
        return cls(camera["width"], camera["height"], camera["matrix"], camera.get("distortion"),
                   n_water, glass_distance_m)


def line_straightness(points):
    """实测点到最佳拟合直线的 RMS，以及除以像长后的相对 RMS。"""
    points = np.asarray(points, dtype=float)
    centered = points - points.mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    rms = float(np.sqrt(np.mean((centered @ vh[-1]) ** 2)))
    span = float(np.ptp(centered @ vh[0]))
    return rms, rms / max(span, 1e-9)


def air_focal_for_water_fov(height_px, water_fov_y_deg, n_water=1.333):
    """仿真反推：传感器竖直边缘看到的水中角已知时，同一传感器在空气中的焦距（像素）。"""
    s = n_water*math.sin(math.radians(water_fov_y_deg)/2)
    if s >= 1:
        raise ValueError("水下视场超过平面窗临界角")
    return height_px/(2*math.tan(math.asin(s)))


def calibrate(paths, columns, rows, square_m):
    objects = np.zeros((rows*columns, 3), np.float32)
    objects[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)*square_m
    object_views, image_views, used, rejected = [], [], [], []
    size = None
    for path in paths:
        # imdecode 支持 Windows 中文路径。
        image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            rejected.append(str(path))
            continue
        current = image.shape[1], image.shape[0]
        if size is not None and current != size:
            raise ValueError("标定照片分辨率不一致，请只使用实际运行分辨率的一组图片")
        size = current
        found, corners = cv2.findChessboardCornersSB(image, (columns, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if found:
            object_views.append(objects.copy())
            image_views.append(corners)
            used.append(str(Path(path).resolve()))
        else:
            rejected.append(str(path))
    if len(used) < 8:
        raise ValueError(f"只有 {len(used)} 张找到完整内角点。请提供至少 8 张不同位置和倾角的清晰照片。")
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(object_views, image_views, size, None, None)
    per_view = []
    for world_points, image_points, rvec, tvec in zip(object_views, image_views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(world_points, rvec, tvec, matrix, distortion)
        per_view.append(float(np.sqrt(np.mean((projected-image_points)**2))))
    return dict(purpose="calibrated_from_images", model="air_intrinsics_flat_port", method="OpenCV air pinhole + lens distortion",
        width=size[0], height=size[1], matrix=matrix.tolist(), distortion=distortion.reshape(-1).tolist(),
        rms_reprojection_px=float(rms), per_view_error_px=per_view, used_images=used, rejected_images=rejected,
        board_inner_corners=[columns, rows], square_m=square_m,
        note="空气标定；平面窗距离另量。重投影误差不是水下误差，需另用实际水下格板核对修正后尺度。")


def create_corrector(camera=None):
    """创建一个修正器供多帧复用；采样表由原算法按高度缓存。"""
    camera = load_camera_params() if camera is None else camera
    return FlatPortCorrector.from_document(camera, camera.get("n_water", 1.333),
                                           camera.get("glass_distance_m", 0.013))


def correct_image(frame, corrector, floor_distance_m=None, rotate_180=False):
    """保持原分辨率修正折射与镜头畸变，按输入尺寸缓存适配后的内参。"""
    corrector = corrector.for_frame(frame)
    if rotate_180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    return corrector.undistort(frame, floor_distance_m)


def correct_image_file(input_path, output_path, corrector, floor_distance_m=None, rotate_180=False):
    """读取并修正单张图片；支持 Windows 中文路径。"""
    frame = cv2.imdecode(np.fromfile(input_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError(f"无法读取图片：{input_path}")
    output_path = Path(output_path)
    if output_path.exists():
        raise FileExistsError(output_path)
    fixed = correct_image(frame, corrector, floor_distance_m, rotate_180)
    ok, encoded = cv2.imencode(output_path.suffix, fixed)
    if not ok:
        raise ValueError(f"无法编码图片：{output_path.suffix}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    encoded.tofile(output_path)
    return fixed


def correct_video_file(input_path, output_path, corrector, floor_distance_m=None, rotate_180=False):
    """逐帧修正，输出 MJPG AVI（无音轨）；高度参数在整个视频内保持不变。

    真实运行深度变化时，请逐帧调用 correct_image 并传入对应遥测高度。
    """
    output_path = Path(output_path)
    if output_path.suffix.lower() != ".avi":
        raise ValueError("视频输出请使用 .avi（MJPG）")
    if output_path.exists():
        raise FileExistsError(output_path)
    capture = cv2.VideoCapture(str(input_path))
    writer = None
    count = 0
    try:
        if not capture.isOpened():
            raise ValueError(f"无法打开视频：{input_path}")
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("视频帧率无效")
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            fixed = correct_image(frame, corrector, floor_distance_m, rotate_180)
            if writer is None:
                output_path.parent.mkdir(parents=True, exist_ok=True)
                writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"MJPG"),
                                         fps, (fixed.shape[1], fixed.shape[0]))
                if not writer.isOpened():
                    raise ValueError(f"无法创建视频：{output_path}")
            writer.write(fixed)
            count += 1
        if not count:
            raise ValueError("视频中没有可读取的帧")
    finally:
        capture.release()
        if writer is not None:
            writer.release()
    return count


def calibrate_camera_from_folder(images_dir, columns, rows, square_mm):
    """空气棋盘格标定：columns/rows 是内角点数，square_mm 是格子边长。

    至少 8 张清晰完整棋盘格照片，位置和倾角应有变化；分辨率须与运行一致。
    输出内参和畸变、重投影误差、有效/拒绝照片列表，不自动写入任务配置。
    """
    if columns < 3 or rows < 3 or square_mm <= 0:
        raise ValueError("棋盘格参数不合法")
    paths = sorted(p for p in Path(images_dir).iterdir()
                   if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"))
    return calibrate(paths, columns, rows, square_mm/1000)


def main():
    parser = argparse.ArgumentParser(description="独立相机工具：镜头畸变 + 平面窗折射修正")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("info", "export", "image", "video"):
        command = commands.add_parser(name)
        command.add_argument("--params", type=Path, help="独立相机 JSON 或部署 event_params.json")
        if name != "info":
            command.add_argument("--output", type=Path, required=True)
        if name in ("image", "video"):
            command.add_argument("--input", type=Path, required=True)
            distances = command.add_mutually_exclusive_group()
            distances.add_argument("--floor-distance", type=float, help="相机离底高度，米")
            distances.add_argument("--depth-cm", type=float, help="艇顶压力计读数，厘米")
            command.add_argument("--water-depth", type=float, default=1.3)
            command.add_argument("--camera-below-sensor", type=float, default=0.20)
            command.add_argument("--rotate-180", action="store_true", help="原始相机装反时转正")
    calibration = commands.add_parser("calibrate")
    calibration.add_argument("--images", type=Path, required=True)
    calibration.add_argument("--columns", type=int, required=True)
    calibration.add_argument("--rows", type=int, required=True)
    calibration.add_argument("--square-mm", type=float, required=True)
    calibration.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "calibrate":
        if args.output.exists():
            raise FileExistsError(args.output)
        camera = calibrate_camera_from_folder(args.images, args.columns, args.rows, args.square_mm)
        save_camera_params(camera, args.output)
        print(f"已保存 {args.output}；有效照片 {len(camera['used_images'])}；RMS {camera['rms_reprojection_px']:.3f} px")
        return
    camera = load_camera_params(args.params)
    if args.command == "export":
        save_camera_params(camera, args.output)
        print(f"已保存 {args.output}")
        return
    corrector = create_corrector(camera)
    if args.command == "info":
        print(json.dumps(dict(camera=camera, water_fov=corrector.water_fov_deg()), ensure_ascii=False, indent=2))
        return
    height = args.floor_distance
    if args.depth_cm is not None:
        height = camera_height_m(args.depth_cm, args.water_depth, args.camera_below_sensor)
    if height is not None and height <= 0:
        raise ValueError("相机离底高度必须为正")
    if height is None:
        print("未传离底高度：按原算法忽略针孔到玻璃距离的影响。")
    if args.command == "image":
        correct_image_file(args.input, args.output, corrector, height, args.rotate_180)
        print(f"已保存 {args.output}")
    else:
        count = correct_video_file(args.input, args.output, corrector, height, args.rotate_180)
        print(f"已保存 {args.output}；处理 {count} 帧")


if __name__ == "__main__":
    main()
