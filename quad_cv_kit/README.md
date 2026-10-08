# quad_cv_kit

视频：读取相机参数 → 校正帧 → 整门 YOLO 检测 → 选取当前画面面积最大的门框 → OpenCV 拟合 → 标注四边像素长度及水平线倾角。校正前、校正后视频共用这一次识别的结果。

## YOLO 与改进 CV 联动的最近门视频实验

`src/detect_red_gate.py` 提取红管中心线、连接白色弯头和可见管端，支持只有部分边可见的门框。`demo/demo_video_cv_improved.py` 默认批量处理 `E:/TEST` 中的视频，使用 `model/best.pt` 和 CPU 推理。`src/yolo_red_gate.py` 负责 YOLO 目标选择、搜索区域和短时跟踪的联动。

```powershell
# 在 GrandRdk 项目根目录运行；也可以传入单个视频或其他目录
python quad_cv_kit/demo/demo_video_cv_improved.py
python quad_cv_kit/demo/demo_video_cv_improved.py E:/TEST/DOOR_TEST_1.mp4 --max-frames 150
# 对照此前不依赖 YOLO 的检测
python quad_cv_kit/demo/demo_video_cv_improved.py --cv-only --out quad_cv_kit/runs/cv_only
```

每个视频输出到 `quad_cv_kit/runs/cv_improved/<视频名>/`：`nearest_gate_before.mp4`（原画面）、`nearest_gate_after.mp4`（校正画面）、`rows.jsonl`、`camera_used.json`、`summary.json` 及左右对照抽帧拼图 `contact.jpg`。黄色框显示本帧所有 YOLO 整门检测及置信度，当前 CV 目标带 `[selected]`；绿色线沿实际检测到的门管绘制，残缺门只标注可见边。校正前的黄色框逐边采样回映射，可能呈曲线，无效区间不连接。YOLO 漏检时不把缓存框画作本帧识别框。`rows.jsonl` 的 `yolo.display_detections/raw_box_geometry` 记录两份画面的识别框显示数据。

YOLO 每帧选择面积最大的整门框作为最近门代理。已跟踪的前景门贴边且仍有匹配的贴边检测时优先维持该目标，避免残缺后跳到远处完整门。普通搜索区域在 YOLO 框四周外扩 8%（`--roi-padding`）；触及输入边界时允许更宽的溢出，每侧至少外扩画面宽/高的 25%。边界检查也考虑校正图内部的无效黑边。目标切换清除旧 CV 跟踪。

CV 在所选搜索区域内按可见管子的像素粗细选择，粗细相近时结合连续性与可见长度；溢出候选必须有实际边段落在所选 YOLO 框附近。贴边目标的细背景管不能直接替换仍获图像支持的粗前景管。这假设各门使用相近的实际管径，不是米制测距。面积、框重叠和管径都是启发式信息，高度重叠的门仍可能被错误关联。内部将整张干净校正帧按比例缩放并补边至 640×360，再用掩膜限制搜索区域，避免独立缩放 ROI 改变管径。`nearest_gate` 坐标已扣除补边并映射到原分辨率的校正画面，`coordinate_space=corrected`。`raw_geometry.edge_paths` 保存同一组门边在原画面中的曲线；逐边采样映射，无效区间不连接。两份视频保持源尺寸、同帧数、同帧率（编码需要时右/下补一个像素），共用一次检测。

默认每三帧进行一次新 CV 检测，中间使用当前图像光流和红管证据跟踪，距最后真实 CV 检测最多保持 0.2 秒；YOLO 漏检时只允许同样时限的图像跟踪，不启动新 CV 来续期。JSON 的 `observation` 区分 `detected/tracked`，无有效目标时 `nearest_gate=null`；`yolo` 记录检测框、目标、搜索区域和选择原因，`cv_enabled` 表示新 CV 检测是否允许，`detection_ran` 表示本帧是否实际执行。`summary.json` 的 `cv_detection_frames` 统计实际执行次数。`--detect-every 1` 每帧执行 CV，`--hold-seconds 0` 关闭跟踪保持，`--show` 开启前后对照预览。模型参数为 `--weights/--classes/--conf/--iou/--imgsz/--device`。默认加载 `camera_correction_params.json` 进行平面窗折射和镜头畸变校正，使用 `--camera-fit center-crop` 适配录像内参，与现有视频入口一致；`--camera-params` 可指定其他参数，`--plane-distance` 仅在已知适用的场景平面距离时提供。实际采用的参数和适配假设写入 `camera_used.json`。安装根目录 `requirements.txt` 中的依赖；`--cv-only` 只需 OpenCV、NumPy。视频优先使用 PATH 中的 FFmpeg 输出 H.264，缺失时使用 OpenCV MPEG-4。小门、低红色对比度及大倾角杆仍可能漏检；YOLO 区域限制本身不会消除这些 CV 限制。

## CV 预处理与 UI 回正提示

改进 demo 默认在 CV 输入上以 HSV 的亮度通道做 1.12 倍温和对比度增强（`--cv-contrast 1` 关闭，允许 1..1.5）。YOLO 输入和输出视频的底图保持原校正图，CV 光流和新检测都使用相同预处理。校正黑边保持不变。预处理参数写入 `camera_used.json`、逐帧记录和汇总。

UI 后处理：当前所选 YOLO 框触及图像外边界或校正有效区边界时，显示朝相应缺失侧的橙色箭头；两侧同时缺失导致方向冲突时显示 `DIRECTION UNCERTAIN`。这是由边界接触推断出画方向，不能判断遮挡造成的框内残缺。YOLO 漏检时暂停方向和姿态提示。仅当目标不贴边且 CV 提供两横两竖四条有效观测线时，以直线交点得到按 TL/TR/BR/BL 排列的四点，浅绿色只绘制观测管端到交点之间的补线。超过外推上限、交点越界、非凸或越出目标范围时拒绝补线。实际观测管端单独保存在 `nearest_gate.observed_segments`，补线在 `guidance.completion`，不写回检测或跟踪状态。

四点以校正输出内参、零残余畸变和门管中心距 0.70×0.50m 做平面 PnP；`--gate-width-m/--gate-height-m` 可指定实测值。UI 用相机右手坐标系 X 向右、Y 向下、Z 向前，角度是相机需要旋转的局部欧拉角，`R=Rz*Ry*Rx`，并非重力系航向角。先沿 `d=t-(t·n)n` 移到穿过门中心的法线，再按估计旋转使前进轴与法线同向；完成后门中心处于前进轴上。显示 `Shift px-equiv` 的 X/Y/Z 为 `(fx*dx,fy*dy,mean(fx,fy)*dz)/t_z`，X/Y 表示当前深度下的像素等效位移，Z 是虚拟像素等效量，不是可直接测得的像素深度。`center_offset_xy_px` 另存中心偏移。未设定穿门前后距离，因此 Z 不表示前进至门的距离命令。

姿态结果在 `guidance.alignment` 内，保存角度、相机坐标平移、法线、像素等效量、重投影误差及尺寸假设；平面姿态有明显双解、拟合残差过大或数据不足时显示 unavailable。跟踪四边的结果标为 tracked；推算与跟踪都不冒充实测。现有校正参数和门尺寸未经本次实测，这些量用于 UI 示意，不输出运动控制。`guidance_*_before/after.jpg` 保存首次方向、补线和姿态提示样例；`summary.json.guidance_frames` 统计状态。

## 模型与目录

默认 `model/best.pt` 来自本次放入项目根目录的新 `best.pt`，已实测类别为 `{0: door}`。旧四角模型保留为 `model/best_corners.pt`；新权重原始副本保留为 `model/imported_best.pt`。来源和 SHA256 记录在 `model/provenance.json`。运行流程只使用整门检测框，不合并四角检测框。

```text
model/                  模型及来源记录
src/yolo_quad.py        YOLO → 最大框 → OpenCV、绘制
src/quad_cv_det.py      红杆提取、直线拟合和结构检查
src/pole_lines.py       独立杆线候选、稳健中心线拟合、完整边段验证
src/temporal_overlay.py 短时光流跟踪、显示平滑和当前红杆证据检查
src/quad_geom.py        四边形几何、边长和倾角
src/camera_correction.py 平面窗折射、镜头畸变校正、坐标映射和弯曲诊断
src/media_demo.py       图片/视频输入、标注和 JSONL 输出
src/frame_io.py         板端共享帧读取（选装）
src/cv_quad_if.py       板端接口（选装，依赖外部 quad_vision）
demo/                   图片、视频入口
tests/                  几何与流程回归测试
runs/                   运行输出
```

## 运行

Python 3.10 或更高，在项目根目录执行：

```powershell
python -m pip install -r requirements.txt

# 视频：先校正，检测出最大门框后立即开启 OpenCV，导出前后两份标注视频
python demo/demo_video.py "D:/Projects/door-train/test.avi" --device cpu --out runs/correction_compare

# 快速处理前 100 帧
python demo/demo_video.py "D:/Projects/door-train/test.avi" --max-frames 100 --device cpu

# 图片：选择最大门框，立即调用 OpenCV
python demo/demo_images.py "D:/data/frame.jpg" --device cpu
python demo/demo_images.py "D:/data/images" --device cpu
```

`--weights` 指定其他本地整门权重，`--classes` 可指定整门类别名或 ID。默认类别名为 `door/gate/ring`。四角模型会报错，不会把角点的小框当作整门。`--device 0` 可选择已配置好的 GPU。

公共推理模块 `src/yolo_quad.py` 在导入 Ultralytics / PyTorch 前通过 `os.environ` 配置：`PYTORCH_TUNABLEOP_ENABLED=1`、`PYTORCH_TUNABLEOP_TUNING_DURATION=short`、`MIOPEN_FIND_MODE=FAST`、`PYTORCH_MIOPEN_SUGGEST_NHWC=0`、`TORCH_BLAS_PREFER_HIPBLASLT=0`、`MIOPEN_DEBUG_CONV_DIRECT=0`、`MIOPEN_DEBUG_CONV_IMPLICIT_GEMM=0`。这些值会覆盖进程中已有的同名变量；`MIOPEN_DEBUG_CONV_WINOGRAD` 保留为注释。自定义程序应先导入本模块，再导入 PyTorch。

## 最大框选择

每帧从符合类别和 `--conf` 的整门检测中，按推理帧中的框面积 `(x2-x1)*(y2-y1)` 选择最大门（视频使用校正帧）；面积相同时按置信度选择。随后立即使用当前框和干净帧执行 OpenCV，每帧最多处理一个门。目标切换或重新出现时也立即处理；没有门检测时不执行 OpenCV，不复用上一帧的多边形。

图片、视频与库调用使用同一流程。稳定等待、连续帧数、稳定 IoU 和最大间隔参数已经移除。

视频显示默认另做短时稳定处理：从首帧立即显示最大目标，先用前后向LK光流和RANSAC估计它在当前帧的位置，再按当前观测权重0.65平滑小幅框线波动。YOLO暂时漏检时最多跟踪0.2秒；CV暂时失败时，只有光流成功且当前帧红杆仍支持该多边形，才在同样时限内显示跟踪估计。时限从最后一次真实观测计起，不因预测成功而延长。长时丢失、光流失败或明显不同的最大目标会清除旧状态。推理仍每帧运行、仍选当前最大框并立即执行CV，不增加等待。

默认只显示最大目标；`--show-all-boxes` 可同时显示其他当前YOLO框，这些次要框没有时序稳定。`--hold-seconds 0.2` 调节短时跟踪时限（0..1秒），`--smooth-alpha 0.65` 调节当前观测的权重，越小平滑越强但响应越慢。`--no-stabilize` 关闭显示跟踪和平滑。跟踪帧用 `[YOLO gap: tracked]` / `[CV tracked]` 标记；它们是当前画面支持的显示估计，不是本帧新检测。

| 参数 | 默认 | 含义 |
|---|---|---|
| `--conf` | 0.25 | YOLO 置信度阈值 |
| `--iou` | 0.7 | YOLO NMS IoU 阈值 |
| `--imgsz` | 640 | YOLO 推理分辨率 |

## 视频畸变校正和自动诊断

默认自动读取根目录 `camera_correction_params.json`。数据为640×480、空气内参 fx=fy=318.8px、主点(320,240)、水折射率1.333、针孔到玻璃距离0.013m；镜头畸变系数为null，按零使用。校正后的输出焦距约474.70px。JSON中的这些数据是现场估计参数，不是新增的棋盘格实测畸变系数。

视频先读取实际帧尺寸，再适配内参并生成校正表。1280×960输出1280×960，1280×720输出1280×720；校正和OpenCV均处理原分辨率干净帧。YOLO内部仍按 `--imgsz` 缩放推理，Ultralytics把框映射回原分辨率。奇数尺寸视频只在右/下边补1像素黑边以满足编码要求，不丢弃原始像素。原始画面如已转正，不要再使用 `--rotate-180`。

`--camera-fit center-crop` 为默认：假设不同长宽比的录像来自同一相机视场居中裁切，再同步换算焦距和主点。例如640×480内参用于1280×720录像，fx=fy=637.6、主点(640,360)。仅凭尺寸无法确认真实采集模式；如录像保留整个标定视场并进行了非等比缩放，选择 `--camera-fit resize`；如需拒绝所有比例不匹配输入，选择 `--camera-fit strict`。有此录像模式的实测内参时应优先用 `--camera-params` 指定它。镜头归一化畸变系数不因分辨率变化而缩放；像素单位的内参需要换算。

视频默认使用已知参数直接校正，采样表缓存复用。未提供场景平面的垂直距离时忽略针孔到玻璃的视点偏移，不会假设门距离等于池底高度。已知且适用时可加 `--plane-distance 0.98`，或用 `--camera-params` 指定另一份空气内参。

自动画面诊断从所识别红杆的实际中心采样，比较校正前后的直线拟合残差，报告 `curvature-reduced`、`curvature-increased`、`no-clear-change` 或 `insufficient-evidence`。诊断以红杆本应为直线为假设，遮挡、颜色干扰或错误拟合会影响它；它不从任意单帧反推新标定参数，也不因某帧缺少证据而跳过配置指定的校正。当前模型按平面防水窗和近似平面场景设计，仍需用相机实际图像核对参数适用性。

YOLO只读取干净的校正帧，OpenCV随后在同一校正帧上拟合最大门。原始画面不另跑一套模型：将这些YOLO框边界和OpenCV多边形逐边采样，使用与校正相同的非线性映射回画。原始画面上的框可能是曲线；两份视频目标、数量、类别、置信度和测量一致，坐标和边界形状不同。曲线映射中的无效部分不连线；校正帧黑边为无原始像素可用的区域。

## 标注与输出

- 黄色框：YOLO 检测到的整门框；当前最大框附带 `[selected]` 标注。
- 绿色多边形：OpenCV 四根杆中心线的交点；红点为左上、右上、右下、左下四角。
- 四边 `top/right/bottom/left: 123.4px +5.6deg`：像素长度和相对图像水平线的倾角。
- 角度范围 `(-90°, 90°]`，水平 0°、竖直 90°；向右上倾斜为正，向右下倾斜为负。
- `lvl=4` 表示通过完整结构检查；候选四边形若未通过检查，标注 `fit only`、`geometry_valid=false`。

视频目录输出 `before_correction.mp4`（校正前画面＋回映射框）和 `after_correction.mp4`（校正后画面＋框），两者保留源图分辨率、同帧数、同帧率、无音轨。校正前画面仅按选项转正，不缩放。两份视频文字标注均为校正坐标中的像素长度和倾角，不代表原始曲线的弧长。

图片 demo 保持原始图像识别，输出标注 JPG。两种 demo 都输出 `rows.jsonl`，包含检测框及四边测量；视频额外记录 `raw_geometry`（原始画面映射路径）、`distortion_diagnostic`（弯曲诊断）和 `coordinate_space=corrected`。`camera_used.json` 保存原标定参数、换算后的 `adapted_camera`、`camera_adaptation` 适配假设、输出内参、源图尺寸、距离假设和有效像素比例。

`detections` / `raw_geometry` 始终保存实际推理观测；实际视频绘制使用 `display_detections` / `display_raw_geometry`，两份视频共用同一份显示数据。`temporal_status` 和 `tracking` 记录状态、光流有效性和距真实观测的帧数。跟踪多边形标为 `observation=tracked`、`geometry_valid=false`；平滑多边形标为 `observation=smoothed`。显示边长和倾角按当前显示四角重算；`fields.display_estimate=true` 提醒它们是显示估计。控制或定量评估应读取真实 `detections`，不要把预测当实测。

```text
gate_status.state            searching / processing
gate_status.cv_enabled       本帧是否启动 OpenCV
detections[].selected        当前最大门框的标记
detections[].quad.corners    OpenCV 四角；视频为校正帧坐标，图片为原图坐标
quad.edges.top.length_px     上边像素长度，其余三边同理
quad.edges.top.angle_deg     上边相对图像水平线的倾角
quad.geometry_valid         完整结构是否可信
quad.lvl / why / diag        观测等级、原因和中间量
```

长度是图像像素长度，倾角是二维像面角度；真实米制长度、相对重力水平面的倾角需要相机标定及深度或姿态信息。

## 库调用

```python
import cv2
from src.yolo_quad import YoloQuadDetector, draw_detections
from src.camera_correction import create_corrector

corrector = create_corrector()  # 自动读取根目录 JSON
pipeline = YoloQuadDetector(device="cpu", opts={"fx": corrector.f_out})
cap = cv2.VideoCapture("input.mp4")
while True:
    ok, frame = cap.read()
    if not ok:
        break
    frame_corrector = corrector.for_frame(frame)  # 按原分辨率换算内参，缓存复用
    fixed = frame_corrector.undistort(frame)
    pipeline.opts = {"fx": frame_corrector.f_out}
    detections = pipeline.detect(fixed)
    annotated = draw_detections(fixed, detections)
    # 将 annotated 保存、显示或交给后续处理
cap.release()
```

单张图片同样使用 `pipeline.detect(frame)`。已有整门框可调用 `GateQuadProcessor.process(frame, detections)` 使用同一最大框选择流程；仅需要 OpenCV 时直接调用 `src.quad_cv_det.detect(frame, bbox)`。

备用模式仍可使用：图片 `--bbox-source json` 读取同名 JSON 中的 `dets`（含 `label/score/bbox`），视频 `--bbox-source red` 使用最大红色连通域定位。两者均选择最大门框并立即调用 OpenCV。

## OpenCV 原理

整门框外扩15%作为颜色ROI → 相对红度 `(R-G)-percentile(R-G,25)` 大于22形成掩码 → 去噪与红球过滤 → 以实际YOLO框定位四边搜索带 → Hough生成独立杆线候选 → 沿杆等距采样红色截面的中心 → Huber稳健拟合和残差剔除 → 覆盖度、密度和连续性检查 → 四线求交 → 检查交点是否在YOLO框附近、整条可见边至少70%有红色支持 → 凸性、面积和宽高比检查。

像素阈值、形态学核和面积门槛随原图分辨率换算，颜色阈值保持原值。搜索带中混入背景红杆时，不再把所有红像素直接最小二乘拟合成一条线。交点严重外推或整边证据不足时不输出完整的本帧实测多边形；出画横杆也必须有足够实际证据才可给四角。短时视频显示稳定独立于这套检测，不修改或替代实际观测。`--opts '{"robust_lines": false}'` 仅用于对照旧算法。

默认按红色 70×50cm 矩形门设计，宽高比为 1.40±0.35。红色像素太少时尝试 Canny 兜底。CV 参数通过 `--opts` JSON 覆盖 `src/quad_cv_det.py` 的 `DEFAULTS`。

`lvl` 为 4 时完整几何可用；3/2/1 为降级观测，0 无有效观测。`psi=(ρ-1)/(ρ+1)` 是左右立柱像长比形成的无量纲偏航量，并非本次标注的边倾角。输入必须是没有叠加框线的干净帧，识别完毕才在副本上画图。

## 重叠门框的剩余限制

仅选择最大 YOLO 框不能排除框内另一扇门的红杆。

现在按独立候选杆线、位置先验和实际整边支持选线，能减少背景门杆把中心线拉歪。视频增加了短时运动跟踪，但尚未实现四边候选的联合实例归属；高度重叠时仍可能混用两扇门的有效杆线。后续可联合组合候选四边，再按实例分割或可追踪特征辨别归属。

未经校正图像训练的YOLO可能受到校正引起的形状和视场变化影响；标签没有考虑畸变不代表模型一定失效。OpenCV不依赖训练标注，但依赖YOLO定位、像素颜色、校正精度和杆线可见性；校正残差导致杆仍弯曲时，直线拟合也可能拒绝它。应比较两种输入的验证集表现，并在必要时用一致校正方式变换训练图与标注后训练。

## 验证

```powershell
python tests/test_quad_cv.py
python tests/test_yolo_quad.py
python tests/test_camera_correction.py
python tests/test_pole_lines.py
python tests/test_temporal_overlay.py
python tests/test_red_gate_improved.py
python tests/test_yolo_red_gate.py
python tests/test_gate_guidance.py
```

23 项原有合成场景回归；流程测试覆盖首帧立即执行、最大面积优先、目标切换立即执行、丢失后不复用旧结果、重新出现立即执行、长度倾角和四角模型误用。

13项流程测试验证最大门框立即处理及独立 YOLO 框接口不启动旧 CV；13项校正测试验证默认参数加载、映射一致性、无畸变恒等变换、镜头系数生效、弯曲诊断、16:9居中裁切的内参换算、原分辨率处理及缓存和双视频每帧只在校正图上推理一次；3项杆线测试验证重叠背景杆干扰、定位框轻微抖动和整边证据拒绝。旧出画场景中强制给完整四角的两项断言已改为验证证据不足时降级。

改进检测的9项测试验证管径选择、原分辨率坐标、真实观测超时、非线性边段回映射和无效区间断开；8项联动测试验证普通 ROI 隔离、贴边允许溢出、残缺前景连续性、YOLO 缺失禁止新 CV、漏检时限、目标切换清除旧跟踪、校正黑边和固定尺度管径。

6项时序测试使用真实光流，验证首帧立即显示、漏检时随当前帧移动、0.2秒时限到期、CV丢失时的独立时限、画面切换/目标变化清除旧框、显示平滑及边长倾角一致性；原始检测数据始终保留。

随机复查相同帧和相同YOLO框（无需再跑YOLO，原分辨率校正后对比新旧OpenCV）：

```powershell
python demo/compare_cv_frames.py runs/rec_cam1_corrected/rows.jsonl --out runs/cv_native_comparison --samples 12 --seed 1006
```

输出逐帧对比图、`comparison.jpg` 和 `summary.json`。抽样范围是有YOLO目标的帧，不要求旧CV成功。这是可视化诊断，不等同于带人工真值的准确率评估。
