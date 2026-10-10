# -*- coding: utf-8 -*-
"""按赛段选择模型；修改 MODELS 中的文件名和类别表后重启视觉进程。

一个赛段的前视、下视使用同一份模型配置，阈值/BPU 等未覆盖项沿用各自配置。
换模型时同步修改 class_names，其顺序必须与训练输出一致。
相对模型路径以工程 models/ 为基准；pick 类别沿用 main_config 的 bottom 模型说明。
Mission 写 momo_stage.json，视觉 worker 每帧读取；没有 stage 时用原配置。
"""
import json
import os
import threading
import time


MODELS = {
    'ball': {
        'model_path': 'test_nashe_640x640_nv12.hbm',
        'class_names': ['door', 'red-ball', 'yellow-ball'],
        'target_class_names': ['red-ball'],
        'target_class_ids': [],
    },
    'gate': {
        'model_path': 'door_6_nashe_640x640_nv12.hbm',
        'class_names': ['door'],
        'target_class_names': ['door'],
        'target_class_ids': [],
    },
    'pick': {
        'model_path': 'bottom.hbm',
        'class_names': ['ring', 'red-ball', 'yellow-ball'],
        'target_class_names': ['red-ball'],
        'target_class_ids': [],
    },
}

# 键对应 Stage.NAME；同一赛段的多个 stage 指向同一份配置，不重复加载。
# PassGate / PassDoorV2 / PickBall 都是预留/新流程名称：当前正式序列走 PassGate，
# PassDoorV2（t_pass_door_v2.py）是"对中→前冲+高度计突变"新过门流程，同用 gate 模型。
# 未列出的 stage（含 IDLE / DONE）使用原 quick_config 配置。
STAGE_MODELS = {
    'Task1': 'ball',
    'SearchBall': 'ball',
    'HitBall': 'ball',
    'HitBall_v2': 'ball',
    'Task2': 'gate',
    'PassGate': 'gate',
    'PassDoorV2': 'gate',
    'PickBall': 'pick',
}

# 下视相机启用阶段集：仅这些阶段 bottom.py 才打开相机采集，
# 其余阶段 V4L2 release、进程常驻（由采集线程按 stage 门控，见 src/bottom.py producer）。
BOTTOM_ACTIVE_STAGES = {'SearchBall', 'PickBall'}


def stage_path(shm_dir=None):
    return os.path.join(os.environ.get('GRDK_SHM_DIR') or shm_dir or '/dev/shm',
                        'momo_stage.json')


def publish_stage(stage, shm_dir=None):
    """原子发布阶段；目录由宿主准备，离线测试无需创建 /dev/shm。"""
    path = stage_path(shm_dir)
    tmp = '%s.%s.%s.tmp' % (path, os.getpid(), threading.get_ident())
    try:
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump({'stage': stage}, f)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)


def read_stage(shm_dir=None):
    """读当前发布的阶段名；文件缺失视为 IDLE，解析失败返回 None。

    与 StageDetector._stage() 同源（都读 momo_stage.json），供不持有检测器的
    调用方（如 bottom.py 采集线程做相机门控）轻量读取，不触发模型加载。
    """
    path = stage_path(shm_dir)
    try:
        with open(path, encoding='utf-8') as f:
            stage = json.load(f)['stage']
        return stage if isinstance(stage, str) else None
    except FileNotFoundError:
        return 'IDLE'
    except (OSError, ValueError, KeyError, TypeError):
        return None


def model_config(stage, base_cfg):
    profile = STAGE_MODELS.get(stage)
    cfg = dict(base_cfg)
    if profile is not None:
        cfg.update(MODELS[profile])
        root = os.environ.get('GRDK_ROOT') or os.path.dirname(os.path.dirname(__file__))
        if not os.path.isabs(cfg['model_path']):
            cfg['model_path'] = os.path.join(root, 'models', cfg['model_path'])
    return cfg


class StageDetector:
    """每个 worker 独立持有检测器，只在有效模型配置变化时重新加载。"""

    def __init__(self, base_cfg, factory, shm_dir=None, log=print):
        self.base_cfg = base_cfg
        self.factory = factory
        self.path = stage_path(shm_dir)
        self.log = log
        self.stage = 'IDLE'
        self.cfg = None
        self._loaded_cfg = None
        self._detector = None
        self._failed_cfg = None
        self._retry_at = 0.0
        self.ready = False

    def _stage(self):
        try:
            with open(self.path, encoding='utf-8') as f:
                stage = json.load(f)['stage']
            return stage if isinstance(stage, str) else self.stage
        except FileNotFoundError:
            return 'IDLE'
        except (OSError, ValueError, KeyError, TypeError):
            return self.stage

    def current_stage(self):
        """供前视预处理选择赛段；实际推理仍会再次核对模型阶段。"""
        return self._stage()

    def detect(self, frame, nv12=None):
        self.stage = self._stage()
        self.cfg = model_config(self.stage, self.base_cfg)
        self.ready = self.cfg == self._loaded_cfg
        if not self.ready:
            if self.cfg == self._failed_cfg and time.monotonic() < self._retry_at:
                return []
            try:
                detector = self.factory(self.cfg)
            except Exception as exc:
                self.log('[stage_model] %s 模型加载失败: %s' % (self.stage, exc))
                self._failed_cfg = dict(self.cfg)
                self._retry_at = time.monotonic() + 1.0
                return []  # 继续回传相机画面，不用旧模型冒充当前模型。
            self._detector = detector
            self._loaded_cfg = dict(self.cfg)
            self._failed_cfg = None
            self.ready = True
            self.log('[stage_model] %s -> %s' % (self.stage, self.cfg['model_path']))
        return self._detector.detect(frame, nv12=nv12)

    def is_current(self):
        """加载/推理期间若已经切到另一模型，丢弃这次过时结果。"""
        return self.cfg == model_config(self._stage(), self.base_cfg)
