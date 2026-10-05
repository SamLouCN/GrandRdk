# -*- coding: utf-8 -*-
"""视觉接口 —— 读 /dev/shm/momo_det_*.json，返回「识别到的目标物 + 距瞄准点的偏差」

[AUV-MISSION 2026-09-26 新增] 任务脚本的视觉子程序入口。
任务状态机只消费本模块的返回值，不直接碰共享内存。

输入（front.py / bottom.py 覆盖写，采集 640x480）:
    {"frame": 12345, "ts": 1758000000.12,
     "dets": [{"label":"door","score":0.87,"bbox":[x1,y1,x2,y2],"center":[cx,cy]}]}

输出（poll 的返回值，None = 本帧没有符合要求的目标）:
    {'label','canon','score','cx','cy','dx','dy','ex','ey','w','h',
     'clip_l','clip_r','clip_t','clip_b','age_s','frame'}

★ dx/dy 就是「目标中心距瞄准点的带符号像素偏差」，ex/ey 是它的归一化形式。
★ 没有真实距离字段 —— 模型只给像素。需要距离请另接单目测距。

实现要点（照抄 viskf 已验证的做法，别自创）:
  1. 帧号去重：同一帧不重复出控制量
  2. mtime 新鲜度：写端挂了不能拿旧帧做控制
  3. 同标签取 score 最高
  4. clip 分边判定：贴边时 w/h 不可信（撞球判距、过门判穿都要避开）
  5. 读 JSON 一律 try/except 兜空（写端正覆盖写时可能读到半个 JSON）
"""
import json
import os

# ---------------------------------------------------------------- 标签别名归一化
# 新模型的类别名还没定，可能是 ball / red_ball / red-ball / sphere。
# 业务代码一律用 canonical 名（'ball' / 'gate'），命中任一同义标签即算命中。
# [AUV-MISSION 2026-09-26 新增段] 新模型定名后只改这张表 + to32_config.AUV_LABEL_*
CANON = {
    'door': 'gate', 'gate': 'gate', 'doorway': 'gate',
    'red-ball': 'ball', 'red_ball': 'ball', 'ball': 'ball',
    'sphere': 'ball', 'redball': 'ball', 'target-ball': 'ball',
    'yellow-ball': 'ball_y', 'yellow_ball': 'ball_y', 'yellowball': 'ball_y',
}


def canon_label(name):
    """把模型输出的类别名归一化成 canonical 名（未登记的按小写原名返回）"""
    s = str(name).strip().lower().replace(' ', '_').replace('-', '_')
    return CANON.get(s, CANON.get(str(name).strip().lower(), s))


class VisionIF(object):
    """视觉接口：poll(cam, want, now) -> dict | None"""

    def __init__(self, cfg, shm_dir=None, log=None):
        """初始化：解析两路相机的文件名、画面尺寸、贴边余量

        shm_dir 可注入（台架就是靠它把读端指向临时目录，不碰真 /dev/shm）。
        所有配置项都用 getattr 兜底 —— 板端配置段没同步时不能 ImportError。
        """
        self.cfg = cfg                                       # 配置对象（to32_config）
        self.log = log                                       # 日志函数（可为 None）
        self.shm_dir = shm_dir or str(getattr(cfg, 'AUV_SHM_DIR', '/dev/shm'))
        self.file = {'front': str(getattr(cfg, 'AUV_DET_FRONT', 'momo_det_front.json')),
                     'bottom': str(getattr(cfg, 'AUV_DET_BOTTOM', 'momo_det_bottom.json'))}
        self.w = float(getattr(cfg, 'AUV_IMG_W', 640.0))     # 画面宽（归一化分母）
        self.h = float(getattr(cfg, 'AUV_IMG_H', 480.0))     # 画面高
        self.margin = float(getattr(cfg, 'AUV_CLIP_MARGIN_PX', 4.0))   # 贴边判定的边沿余量
        self._last_frame = {}                                # (cam,want) -> 上次已出过的帧号
        self._warned = set()                                 # 同类警告只打一次

    # ------------------------------------------------------------ 内部工具
    def _warn_once(self, key, msg):
        """同类警告只打一次 —— 20Hz 循环里"文件不存在/数据超期"会每拍都触发，刷爆日志"""
        if key in self._warned:
            return
        self._warned.add(key)
        if self.log:
            self.log('[vision_if] ' + msg)

    def _read_json(self, path):
        """读一个可能被并发覆盖写的 JSON；失败返回 None（绝不抛）"""
        try:
            st = os.stat(path)
            with open(path, 'r') as f:
                obj = json.load(f)
        except Exception:
            return None, None
        if not isinstance(obj, dict):
            return None, st.st_mtime
        return obj, st.st_mtime

    # ------------------------------------------------------------ 主接口
    def poll(self, cam, want, now, aim=None):
        """取一路相机里 canonical 名为 want 的最优目标。

        cam  : 'front' | 'bottom'
        want : canonical 名，如 'ball' / 'gate'
        aim  : 瞄准点 (px,py)；None = 用配置里的默认瞄准点
        返回 None 表示「本帧没有可用目标」——调用方必须能接受 None。
        """
        fname = self.file.get(cam)
        if fname is None:
            self._warn_once('cam_' + str(cam), '未知相机名 %r（只认 front/bottom）' % cam)
            return None
        path = os.path.join(self.shm_dir, fname)
        if not os.path.isfile(path):
            self._warn_once('nofile_' + cam, '%s 不存在（写端没起？）' % path)
            return None

        obj, mtime = self._read_json(path)
        if obj is None:
            return None                                      # 读到半个 JSON：本次当没数据
        age = (now - mtime) if mtime is not None else 9.9
        stale_s = float(getattr(self.cfg, 'AUV_DET_STALE_S', 0.5))
        if mtime is None or age > stale_s:
            self._warn_once('stale_' + cam, '%s 数据超期 age=%.2fs > %.2fs，按无数据处理'
                            % (fname, age, stale_s))
            return None

        frame = obj.get('frame')
        key = (cam, str(want))
        if frame is not None and self._last_frame.get(key) == frame:
            return None                                      # 同一帧不重复出控制量
        self._last_frame[key] = frame

        min_score = float(getattr(self.cfg, 'AUV_MIN_SCORE', 0.5))
        best = None
        for d in (obj.get('dets') or []):
            if not isinstance(d, dict):
                continue
            if canon_label(d.get('label', '')) != want:
                continue
            try:
                sc = float(d.get('score', 0.0))
            except Exception:
                continue
            if sc < min_score:
                continue
            if best is None or sc > best[0]:
                best = (sc, d)
        if best is None:
            return None
        score, d = best

        # ---- 几何：优先用 center，缺失时用 bbox 中点兜底
        bbox = d.get('bbox') or []
        w = h = 0.0
        if len(bbox) >= 4:
            try:
                x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
                w, h = abs(x2 - x1), abs(y2 - y1)
            except Exception:
                x1 = y1 = x2 = y2 = 0.0
        else:
            x1 = y1 = x2 = y2 = 0.0
        ctr = d.get('center') or []
        if len(ctr) >= 2:
            try:
                cx, cy = float(ctr[0]), float(ctr[1])
            except Exception:
                cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
        else:
            cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)

        # ---- 瞄准点：默认画面中心，各阶段可覆盖
        if aim is None:
            aim = (0.5 * self.w, 0.5 * self.h)
        px, py = float(aim[0]), float(aim[1])
        dx, dy = cx - px, cy - py
        ex = dx / (0.5 * self.w) if self.w > 0 else 0.0
        ey = dy / (0.5 * self.h) if self.h > 0 else 0.0

        m = self.margin
        clip_l = bool(x1 <= m)
        clip_t = bool(y1 <= m)
        clip_r = bool(x2 >= self.w - m)
        clip_b = bool(y2 >= self.h - m)

        return {
            'label': d.get('label'), 'canon': want, 'score': score,
            'cx': cx, 'cy': cy, 'dx': dx, 'dy': dy, 'ex': ex, 'ey': ey,
            'w': w, 'h': h, 'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            'clip_l': clip_l, 'clip_r': clip_r, 'clip_t': clip_t, 'clip_b': clip_b,
            'clip': (clip_l or clip_r or clip_t or clip_b),
            'age_s': age, 'frame': frame,
        }
