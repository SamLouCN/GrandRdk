# -*- coding: utf-8 -*-
"""卡尔曼进程托管 —— 深度 / 图像两路，生命周期都跟着 AUV 模式走

★★ [2026-10-11 用户要求] **深度卡尔曼已从板端移除** ★★
  - 本模块的 ``DepthKalmanLauncher``（深度路）**已不再被 mode_auv / test_runner 使用**；
  - 板端 ``run.sh`` 默认也不启动 depth_kalman（``NO_DEPTHKF=1``；``--depthkf`` 可临时恢复）；
  - 所有深度判定统一改为读**固件深度计遥测 actual_depth_cm**（$TEL 帧内，
    与下发 depth_cm 是同一个固件帧，可直接相减）；
  - 本文件保留：``KalmanLauncher`` 基类与 ``ViskfLauncher``（图像路）仍可复用；
    将来若要恢复深度路，把 ``DepthKalmanLauncher`` 重新挂回 mode_auv 即可（相关键都还在 task_config）。

为什么做成"进程托管"而不是"接进 GrandRDK/run.sh"
------------------------------------------------
用户明确要求：**上位机切到 AUV 模式就自动可用，不要额外的 start/stop 脚本**。
所以两个卡尔曼的起停都挂在 ``mode_auv.on_enter / on_exit`` 上：
  * 进 AUV → ``ensure_started()``：没在跑就起一个，已经在跑就复用；
  * 出 AUV → ``stop()``：只停**自己起的那个**，别人（手动 `./run.sh --daemon`）起的原样留着。

两路各自独立，互不影响
----------------------
`DepthKalmanLauncher`（深度，`momo_depth.json`）与 `ViskfLauncher`（图像，`momo_viskf.json`）
是两个独立对象，**各自的幂等判据、各自的进程句柄**。一路起不来不影响另一路，
也不影响 AUV 主流程 —— 最坏只是少一路观测源。

为什么是子进程而不是 import 进来跑
----------------------------------
两个卡尔曼都是**独立工程**（50Hz 循环 + 自己的日志轮转 + 自己的一套模块），
PYTHONPATH 与 to32 完全不同；塞进 to32 的 20Hz tick 会互相拖慢。保持进程隔离最省心，
两侧只通过 `/dev/shm/momo_*.json` 打交道。

目录布局（2026-10-01 定稿：**按 GrandRDK 的分类归档**）
------------------------------------------------------
``config/`` 收所有配置、``src/`` 收所有源码，所以两个卡尔曼**不再各自带 config/ 与 src/**：

    config/depth_config.py                   ← 深度卡尔曼的配置
    config/viskf_config.py                   ← 图像卡尔曼的配置
    src/kalman/depth_kalman/{main,fusion,model,sources,sinks,kf,samples,cfgutil}.py
    src/kalman/depth_kalman/{run.sh,stop.sh,tests/,logs/}
    src/kalman/camera_kalman/{viskf.py,run.sh,stop.sh,tests/,logs/}

⚠ 因此拉起子进程时 PYTHONPATH 必须是 ``<宿主>/config`` + ``<工程根>`` 两段：
前者给 `import depth_config` / viskf 找 `viskf_config.py`，后者给工程自己的模块。
⚠ 刻意**不**把源码摊到 ``src/`` 顶层：depth_kalman 的主程序叫 ``main.py``，
而板端已有 ``src/to32/main.py``，摊平后 `pkill -f 'src/main.py'` 这类操作会误伤。

幂等性（最重要的一条）
----------------------
**绝不能起出两个同款卡尔曼** —— 两个进程同时原子写同一个 JSON，
读端会看到两个写端交替覆盖，时间戳跳动、σ 时大时小，非常难查。
所以 ``ensure_started()`` 有三道"已在跑"的判据，任一命中就复用、不再 spawn：
  ① 本对象上一拍起的进程还活着；
  ② ``logs/<工程名>.pid`` 里的 pid 还活着（手动 ./run.sh --daemon 起的会落这个）；
  ③ 输出 JSON 的 mtime 在 ``*_FRESH_S`` 内（终态判据，兜住前两条都失效的情况）。

失败永远不外抛
--------------
本模块属于"观测/辅助"链路，任何异常都吞掉并返回 False。
卡尔曼没起来最坏的结果是任务走降级路径（无深度源走定时放行、过门走原始像素伺服），
**绝不能因为它拖垮 AUV 主流程**。
"""
import os
import subprocess
import time


class _FakeProc(object):
    """台架用的假进程句柄：只记录被调用过什么，不真的起进程

    台架**绝对不能**真的 spawn 一个卡尔曼（会写 /dev/shm、占 CPU、
    还可能和板端真跑的实例打架）。所以 Popen 做成可注入的，测试时传这个类。
    """
    def __init__(self, tag=''):
        """tag 只用于台架日志里区分不同用例"""
        self.tag = tag            # 标记：区分不同用例
        self.alive = True         # 存活标志：kill() 后置 False
        self.signaled = []        # 记录收到的信号（terminate/kill）

    def poll(self):
        """返回 None 表示还活着（与 subprocess.Popen.poll 语义一致）"""
        return None if self.alive else 0

    def terminate(self):
        """记录 SIGTERM 但不真杀（其实也没进程可杀）"""
        self.signaled.append('SIGTERM')

    def kill(self):
        """记录 SIGKILL 并置为已退出"""
        self.signaled.append('SIGKILL')
        self.alive = False

    def wait(self, timeout=None):
        """立刻返回 0：无需等待"""
        return 0


# ------------------------------------------------------------------ 参数规格
def _f(cfg, name, default):
    """读配置并强制转 float；缺键或转失败一律回落缺省（板端可能没同步新配置段）"""
    try:
        return float(getattr(cfg, name, default))
    except Exception:
        return float(default)


def _host_config_dir(root):
    """由卡尔曼工程根推导宿主的 config/ 目录

    布局（2026-10-01）：`/userdata/GrandRDK/src/kalman/<名字>/` → 宿主根是往上 3 级。
    用逐级向上的推导而不是写死常量，是为了整包拷到别处跑时仍然成立
    （真找不到就返回 None，调用方退化成"只用自己的工程根"，不会因此起不来）。
    """
    if not root:
        return None
    up = os.path.abspath(root)
    for _ in range(3):
        up = os.path.dirname(up)
    d = os.path.join(up, 'config')
    return d if os.path.isdir(d) else None


def _spec_cfg_dirs(cfg, root, override_attr):
    """配置目录列表：显式覆盖 > 宿主 config/ > （空）"""
    explicit = str(getattr(cfg, override_attr, '') or '')
    if explicit:
        return [explicit]
    d = _host_config_dir(root)
    return [d] if d else []


def _depth_spec(cfg):
    """深度卡尔曼这一路的规格（`AUV_KALMAN_*` + 共用 `AUV_SHM_DIR` / `AUV_DEPTH_FILE`）

    [2026-10-01 新布局] 配置在宿主的 `config/depth_config.py`，源码平铺在
    `src/kalman/depth_kalman/`（不再套一层 src/），主程序就是 `<root>/main.py`。
    """
    root = str(getattr(cfg, 'AUV_KALMAN_DIR', '') or '')
    return {
        'label': '深度卡尔曼',
        'enabled': bool(getattr(cfg, 'AUV_KALMAN_AUTOSTART', False)),
        'root': root,
        'script': 'main.py',                           # depth_kalman 的主程序（平铺在根下）
        'args': ('--quiet',),
        'out_file': str(getattr(cfg, 'AUV_DEPTH_FILE', 'momo_depth.json')),
        'wait_s': _f(cfg, 'AUV_KALMAN_START_WAIT_S', 3.0),
        'fresh_s': _f(cfg, 'AUV_KALMAN_FRESH_S', 1.5),
        'shm_dir': str(getattr(cfg, 'AUV_SHM_DIR', '/dev/shm')),
        'cfg_dirs': _spec_cfg_dirs(cfg, root, 'AUV_KALMAN_CONFIG_DIR'),
    }


def _viskf_spec(cfg):
    """图像卡尔曼（viskf）这一路的规格（`AUV_VISKF_*`）

    同新布局：配置 `config/viskf_config.py`，源码 `src/kalman/camera_kalman/viskf.py`。
    """
    root = str(getattr(cfg, 'AUV_VISKF_DIR', '') or '')
    return {
        'label': '图像卡尔曼(viskf)',
        'enabled': bool(getattr(cfg, 'AUV_VISKF_AUTOSTART', False)),
        'root': root,
        'script': 'viskf.py',                          # camera_kalman 的主程序（平铺在根下）
        'args': ('--quiet',),
        'out_file': str(getattr(cfg, 'AUV_VISKF_FILE', 'momo_viskf.json')),
        'wait_s': _f(cfg, 'AUV_VISKF_START_WAIT_S', 3.0),
        'fresh_s': _f(cfg, 'AUV_VISKF_FRESH_S', 1.5),
        'shm_dir': str(getattr(cfg, 'AUV_SHM_DIR', '/dev/shm')),
        'cfg_dirs': _spec_cfg_dirs(cfg, root, 'AUV_VISKF_CONFIG_DIR'),
    }


class KalmanLauncher(object):
    """卡尔曼进程托管基类：ensure_started() / stop() / running()"""

    def __init__(self, cfg, log=None, popen=None, spec=None):
        """初始化：按 spec 记住这一路的一切（spec 由子类从 cfg 读出来）

        popen 可注入（台架用它换成 _FakeProc 工厂，避免真起进程）。
        cfg 上缺任何键都按"关闭托管"处理，绝不让 AUV 起不来。
        """
        s = spec or {}
        self.cfg = cfg                                        # 配置对象（to32_config）
        self.log = log                                        # 日志函数（可为 None）
        self.label = s.get('label', 'kalman')                 # 日志里显示的名字
        self.root = str(s.get('root', '') or '')              # 卡尔曼工程根（src/kalman/<名字>）
        self.enabled = bool(s.get('enabled', False))          # 总开关
        self.script = str(s.get('script', 'main.py'))         # 主程序（相对工程根）
        self.args = tuple(s.get('args') or ('--quiet',))      # 追加参数
        # ★ 配置目录 [2026-10-01] 两个卡尔曼的 config 已收进宿主的 config/
        #   （/userdata/GrandRDK/config/{depth_config,viskf_config}.py），
        #   所以 PYTHONPATH 必须带上宿主 config/，只给 <root> 会 ImportError。
        self.cfg_dirs = list(s.get('cfg_dirs') or [])         # 额外要进 PYTHONPATH 的配置目录
        self.out_file = str(s.get('out_file', ''))            # 它写的共享内存文件名
        self.wait_s = float(s.get('wait_s', 3.0) or 0.0)      # 优雅退出等待
        self.fresh_s = float(s.get('fresh_s', 1.5) or 0.0)    # 输出新鲜度判据
        self.shm_dir = str(s.get('shm_dir', '/dev/shm'))
        self.py = str(getattr(cfg, 'AUV_KALMAN_PY', 'python3'))  # 解释器：默认 python3
        self._popen = popen or subprocess.Popen                # 可注入的进程工厂
        self.proc = None                                       # 自己起的进程句柄
        self.owned = False                                     # True = 我们起的，退出时该我们停
        self.started_once = False                              # 本轮是否已尝试过（防重复）

    # ------------------------------------------------------------------ 日志
    def _log(self, msg, level='info'):
        """内部日志：log 为 None 时静默；任何异常都不能冒出去"""
        if self.log is None:
            return
        try:
            self.log('[%s] %s' % (self.label, msg))
        except Exception:
            pass

    # ------------------------------------------------------------------ 判活
    @staticmethod
    def _alive(pid):
        """pid 是否存活：用 signal 0 探活（不发真信号）。异常一律当不存活"""
        try:
            pid = int(pid)
        except Exception:
            return False
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)          # 信号 0 = 只探活
            return True
        except Exception:
            return False             # 包括 ProcessLookupError / PermissionError

    def _pidfile_pid(self):
        """读 logs/<工程名>.pid（run.sh --daemon 会写）。读不到返回 None

        文件名取工程根的 basename —— 恰好与两家 run.sh 的写法一致
        （depth_kalman.pid / camera_kalman.pid），所以改目录名不用改代码。
        """
        if not self.root:
            return None
        p = os.path.join(self.root, 'logs', os.path.basename(self.root.rstrip('/')) + '.pid')
        try:
            with open(p, 'r') as f:
                return int((f.read() or '').strip())
        except Exception:
            return None

    def _out_fresh(self):
        """终态判据：输出 JSON 是否在 fresh_s 内被更新过"""
        if not self.out_file:
            return False
        p = os.path.join(self.shm_dir, self.out_file)
        try:
            age = time.time() - os.stat(p).st_mtime
        except Exception:
            return False
        return age <= self.fresh_s

    def running(self):
        """三道判据：任一命中即认为"已经有人在跑这一路" """
        if self.proc is not None and self.proc.poll() is None:
            return True                                  # ① 我们自己起的还活着
        if self._alive(self._pidfile_pid()):
            return True                                  # ② 别人用 run.sh --daemon 起的
        if self._out_fresh():
            return True                                  # ③ 输出还在被写
        return False

    # ------------------------------------------------------------------ 起
    def ensure_started(self):
        """确保这一路在跑。幂等：已经在跑就复用，绝不起第二个。

        返回 True = 可用（起来的或本来就有的）；False = 不可用（关了/路径错/起不来）。
        **任何情况下都不抛异常。**
        """
        try:
            if not self.enabled:                          # 总开关关了：直接跳过
                self._log('自动拉起已关闭（总开关=False）—— 需要就手动'
                          ' cd %s && ./run.sh --daemon' % (self.root or '<目录未配置>'))
                return False
            if not self.root or not os.path.isdir(self.root):
                self._log('工程目录不存在: %r —— 跳过自动拉起，任务将走降级路径'
                          % self.root, 'warn')
                return False
            if self.running():                            # 幂等核心：有人跑了就复用
                self._log('已在运行，复用（不重复起）')
                self.started_once = True
                return True

            main_py = os.path.join(self.root, self.script)
            if not os.path.isfile(main_py):
                self._log('缺 %s —— 跳过自动拉起' % main_py, 'warn')
                return False

            # 环境变量：卡尔曼靠 PYTHONPATH 找配置与自己的模块。
            # [2026-10-01] 布局改成「宿主 config/ + 工程根平铺」，所以是 宿主config/ : <root>
            # （老布局是 <root>/config : <root>/src，跟 run.sh 一起改掉了）
            env = dict(os.environ)
            pp = list(self.cfg_dirs) + [self.root]
            old = env.get('PYTHONPATH', '')
            env['PYTHONPATH'] = os.pathsep.join(pp + ([old] if old else []))
            logs_dir = os.path.join(self.root, 'logs')
            try:
                os.makedirs(logs_dir, exist_ok=True)
            except Exception:
                pass

            self._log('拉起: %s %s %s' % (self.py, main_py, ' '.join(self.args)))
            self.proc = self._popen([self.py, main_py] + list(self.args),
                                    cwd=self.root, env=env,
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.STDOUT)
            self.owned = True                              # 是我们起的 → 退出时由我们停
            self.started_once = True
            return True
        except Exception as exc:                           # 吞掉一切：辅助链路不许拖垮主流程
            self._log('自动拉起失败（任务继续，走降级路径）: %r' % (exc,), 'warn')
            self.proc = None
            self.owned = False
            return False

    # ------------------------------------------------------------------ 停
    def stop(self):
        """停掉**自己起的那个**。不是自己起的（别人手动起的）原样留着，不抢别人的进程。"""
        if self.proc is None:
            return
        if not self.owned:
            self._log('当前实例不是本模块起的，跳过停止（不抢别人的进程）')
            self.proc = None
            return
        try:
            self._log('停止')
            self.proc.terminate()                          # 先 SIGTERM（主程序注册了信号处理器）
            deadline = time.time() + max(0.0, self.wait_s)
            while time.time() < deadline:
                if self.proc.poll() is not None:
                    break
                time.sleep(0.05)
            if self.proc.poll() is None:                   # 优雅退出超时 → SIGKILL 兜底
                self.proc.kill()
                self.proc.wait(timeout=1.0)
        except Exception as exc:
            self._log('停止时出错（已忽略）: %r' % (exc,), 'warn')
        finally:
            self.proc = None
            self.owned = False
            self.started_once = False


class DepthKalmanLauncher(KalmanLauncher):
    """深度卡尔曼托管（`AUV_KALMAN_*`，输出 `momo_depth.json`）

    ★ [2026-10-11] **已停用** —— 深度卡尔曼已从板端移除，mode_auv / test_runner
    不再构造本类；深度判定改走固件深度计遥测 actual_depth_cm。保留以备恢复。
    """

    def __init__(self, cfg, log=None, popen=None):
        """从 cfg 读 `AUV_KALMAN_*` 组装规格（每次构造都重读，方便台架改配置后重建）"""
        KalmanLauncher.__init__(self, cfg, log=log, popen=popen, spec=_depth_spec(cfg))


class ViskfLauncher(KalmanLauncher):
    """图像卡尔曼（viskf）托管（`AUV_VISKF_*`，输出 `momo_viskf.json`）"""

    def __init__(self, cfg, log=None, popen=None):
        """从 cfg 读 `AUV_VISKF_*` 组装规格"""
        KalmanLauncher.__init__(self, cfg, log=log, popen=popen, spec=_viskf_spec(cfg))
