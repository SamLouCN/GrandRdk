# -*- coding: utf-8 -*-
"""★★ 测试配置 —— 测试阶段的**唯一**入口文件（水池实机 / 本机离线都改这里）

定位：
  * 板端水池测试：改本文件 → 重启中位机即生效（**不用改 config/auv_config.py**）
  * 本机离线仿真：tests/ 下的台架也读本文件（阶段参数 / 仿真参数 / 视觉剧本）

优先级（避免"改一处不生效"的争吵）：
  test_config.py  >  auv_config.py 的 AUV_TEST_*（后者保留兼容，但**不再推荐改**）
  ⚠ 唯一例外是"或"语义的开关：TEST_ENABLED 与 AUV_TEST_MODE **任一个为 True 就进测试模式**
    （防止有人在 auv_config 里开了测试模式、又在 test_config 里关掉却以为关掉了）。

------------------------------------------------------------------------------
怎么用（三步）
------------------------------------------------------------------------------
1) TEST_ENABLED = True                       开测试模式（False = 全流程作业/验收）
2) TEST_STAGES  = ['SIT_BOTTOM']             选要测的功能模块（支持 * ? 通配）
3) STAGE_PARAMS = {'SIT_BOTTOM': {...}}      给这个模块单独配参数

想知道"某个模块到底做了什么、参数都是啥"：
    cd tests && python run_task.py --stage SIT_BOTTOM -v

------------------------------------------------------------------------------
TEST_STAGES 三种写法（可混写）
------------------------------------------------------------------------------
  ['SEEK_BALL_F']        按**阶段名**选（精确，推荐）
  ['task:PassGateTask']  按**任务类**选（选中所有用这个类的阶段，如 4 个 PASS_GATE_n）
  ['SEEK_GATE_*']        通配
  []                     空 = 全部跑（但仍受 TEST_HOLD_MAX_S 压缩）

★ SURFACE / DONE 在 FORCED 里，**任何配置都删不掉** —— 上浮不允许被关闭。
  选段里写了不存在的名字 → 启动打 ERROR 并**自动退回全流程**（Fail-Safe：
  宁可跑全程，也不让你以为"这段没问题"）。

------------------------------------------------------------------------------
STAGE_PARAMS 的键也可以是上面那三种写法，优先级
------------------------------------------------------------------------------
  精确阶段名  >  task:<类名>  >  通配  >  plan.py 里的默认值
"""
import fnmatch  # 标准库: 选段通配匹配


# ============================================================================
# 一、测试总开关
# ============================================================================
TEST_ENABLED = True           # ★ True = 测试模式；False = 全流程作业（验收/比赛必须 False）
                              #   ⚠ 验收/上场前必须改回 False
TEST_STAGES = []              # 要测哪些模块（阶段名 / task:<类名> / 通配）；[] = 全部
                              #   只想验一段就写成一个元素的列表，例：['SEEK_BALL_F']
                              #   想验某一类（4 个门）写：['task:PassGateTask']
TEST_HOLD_MAX_S = None        # 保持判据上限(s)：默认 None = 沿用 auv_config 的 AUV_TEST_HOLD_MAX_S
                              #   （那边默认 5.0）。想在本文件里定死就写数字，钳到 [1.0, 30.0]。
                              #   ⚠ 钳下界是防止写 0 —— 判据会"瞬间满足"，测出来的时序全是假的。
TEST_LOOP = False             # 单段循环调试：跑完最后一段回到第一段（默认 False = 跑完就上浮）
TEST_AUTO_SURFACE = True      # ★ 恒 True：跑完选中的段一定上浮（写 False 无效，代码强制）

# ============================================================================
# 二、各功能模块的参数覆盖（★ "相应的各项参数" 主要在这里配）
# ============================================================================
# 键 = 阶段名 / task:<类名> / 通配；值 = 要覆盖的参数字典。
#
# 参数写法沿用框架约定（写错会静默失效，务必看 §五速查）：
#   大写键 + 字符串  → 查配置 `AUV_<字符串>`（如 'SURGE': 'SURGE_SEEK' → AUV_SURGE_SEEK）
#   大写键 + 数字    → 字面量（如 'TIMEOUT_S': 10.0）
#   小写键           → 字面量（cam / scan / next / fail）
#
# ⚠ 想改的是"全局配置"（增益、阈值、池深…）请去 GLOBAL_PARAMS；这里只改**某个阶段**的行为。
STAGE_PARAMS = {
    # ---- 例：把三段搜索的超时压到 10s，水池里别干等 30s
    # 'SEEK_BALL_F': {'TIMEOUT_S': 10.0, 'SETTLE_S': 0.5, 'scan': True},
    # 'SEEK_BALL_B': {'TIMEOUT_S': 10.0, 'SETTLE_S': 0.5},
    # 'task:SeekTask': {'TIMEOUT_S': 12.0, 'SETTLE_S': 0.5},

    # ---- 例：撞球调推力 / 武装延迟 / 撞击阈值
    # 'RAM_BALL': {'SURGE': 0.35, 'ARM_S': 0.3, 'TIMEOUT_S': 12.0},

    # ---- 例：坐底调下沉速率与净空阈值（RATE_CMS 越大越猛）
    # 'SIT_BOTTOM': {'RATE_CMS': 10.0, 'CLEAR_CM': 8.0, 'TIMEOUT_S': 15.0},

    # ---- 例：对中调容差（px）与保持时间
    # 'CENTER_BALL': {'TOL_PX': 40.0, 'HOLD_S': 1.0, 'TIMEOUT_S': 10.0},

    # ---- 例：穿门调推力与超时
    # 'task:PassGateTask': {'SURGE': 0.35, 'TIMEOUT_S': 20.0},

    # ---- 例：投放调触发延时与总时长（舵机未实装时可把总时长压到 1s）
    # 'RELEASE_DROP': {'ARM_S': 0.5, 'TOTAL_S': 2.0},

    # ---- 例：转向（现场发现转不够就把 DEG 调大）
    # 'TURN_120': {'DEG': 130.0, 'TIMEOUT_S': 12.0},

    # ---- 通配兜底（对全部阶段生效，精确写法优先）
    # '*': {'TIMEOUT_S': 20.0},
}

# ============================================================================
# 三、全局参数覆盖（改 AUV_* 或任何配置键，不用去动 auv_config.py）
# ============================================================================
# 这些会**直接覆盖**配置对象上的同名键（板端就是 to32_config 模块对象），
# 启动时在日志里打一行"[AUV] test_config 全局覆盖 N 项"，一眼能看出有没有生效。
#
# 常用场景：水池里临时改增益/阈值试效果，试好了再**写回** auv_config.py 并清空这里。
# ⚠ 验收/比赛前请清空本字典（或至少确认里面没有临时值）。
GLOBAL_PARAMS = {
    # 'AUV_POOL_DEPTH_CM': 130.0,        # 🔴 池深(cm)：到场第一件事就是量它
    # 'AUV_YAW_RATE_DPS': 30.0,          # 🔴 转向速率(°/s)：发满舵→计时→量角度
    # 'AUV_TURN_RIGHT_SIGN': 1.0,        # 🔴 右转到底是 yaw 增(+1)还是减(-1)
    # 'AUV_SPEED_MPS': 0.25,             # 🔴 巡航速度(m/s)：航位推算的距离全靠它
    # 'AUV_SURGE_SEEK': 0.25,
    # 'AUV_SURGE_RAM': 0.40,
    # 'AUV_RAM_W_PX': 220.0,             # 🔴 撞球时球宽拍下来取阈值
    # 'AUV_GATE_COUNT': 4,               # 现场数门
    # 'AUV_GATE_SEQ': 'low,high,low,high',
    # 'AUV_BUDGET_S': 900.0,             # 想快速验证流程可临时压到 60~120
    # 'AUV_TEST_HOLD_MAX_S': 5.0,        # 也可在这里写（等价于上面的 TEST_HOLD_MAX_S）
}

# ============================================================================
# 四、离线仿真参数（只影响本机 tests/，板端不看）
# ============================================================================
SIM = {
    'dt': 0.05,               # 仿真步长(s)：20Hz
    'max_s': 600.0,           # 最长仿真时长(s)
    'pool_m': 1.30,           # 仿真池深(m)
    'depth_rate_mps': 0.15,   # 垂向最大速率(m/s)：模拟固件定深闭环 + 水阻
    'start_depth_m': 0.0,     # 起始深度(m)：测 DIVE 用 0，测 SIT_BOTTOM 时可设 1.0
    'depth_ok': True,         # False = 模拟深度源掉线（测降级路径）
    'viskf_ok': False,        # True = 模拟图像卡尔曼可用（测滤波伺服路径）
    'servo_ready': True,      # False = 模拟舵机未实装（测 ServoStub 降级）
    'echo_log': False,        # True = 仿真时逐条打印日志
}

# ============================================================================
# 五、视觉剧本（离线仿真里"什么时候看得见目标"，板端不看）
# ============================================================================
# 通用规则：进入阶段 → 等 delay 秒 → 看见目标 → 目标框宽按 grow px/s 长大 →
#          宽度超过 disappear 就"出画消失"（用来触发撞球/穿门的尺度判据）。
# stages 表按阶段名精确覆盖（支持 * ? 通配），优先级高于通用规则。
VISION_SCRIPT = {
    'enabled': True,                # False = 全程看不见目标（测搜索超时→中止路径）
    'default_delay_s': 3.0,         # 没写 stages 条目时的默认"多久才看见"
    'see_immediately': [],          # 这些阶段一进去就看得见（delay=0）
    'never_see': [],                # 这些阶段永远看不见（例：['SEEK_BALL_F'] 测中止）
    'grow_w_px_per_s': 120.0,       # 通用：目标框宽度增长速率(px/s)
    'disappear_w_px': 560.0,        # 通用：宽度超过它就"出画消失"
    'start_w_px': 80.0,             # 通用：刚看见时的目标框宽度(px)
    # ---- 逐段剧本（键支持 * ? 通配；不写的阶段用上面的通用规则）
    'stages': {
        'SEEK_BALL_F': {'label': 'ball', 'cx': 380.0, 'cy': 300.0,
                        'w': 80.0, 'grow': 0.0, 'delay': 3.0},
        'RAM_BALL':    {'label': 'ball', 'cx': 330.0, 'cy': 250.0,
                        'w': 80.0, 'grow': 120.0, 'delay': 0.0},
        'SEEK_GATE_*': {'label': 'gate', 'cx': 330.0, 'cy': 250.0,
                        'w': 120.0, 'grow': 0.0, 'delay': 3.0},
        'PASS_GATE_*': {'label': 'gate', 'cx': 330.0, 'cy': 250.0,
                        'w': 120.0, 'grow': 200.0, 'delay': 0.0, 'disappear': 560.0},
        'SEEK_BALL_B': {'label': 'ball', 'cx': 330.0, 'cy': 250.0,
                        'w': 60.0, 'grow': 0.0, 'delay': 2.0},
        'CENTER_BALL': {'label': 'ball', 'cx': 322.0, 'cy': 242.0,
                        'w': 60.0, 'grow': 0.0, 'delay': 0.0},
    },
}


# ============================================================================
# 六、状态提示回传（$MSG 帧 → 上位机终端显示）
# ============================================================================
# AUV 在跑什么、出了什么警告/报错，会以 $MSG 文本帧推给上位机（默认与 $AUV 同端口 8085）。
# 板端与本机离线**共用这一份**配置；离线仿真默认没 start()，不会真的往外发 UDP。
#
# 想让上位机终端"什么都别刷"：把 min_level 提到 'WARN'（只看警告和报错）。
NOTIFY = {
    'enabled': True,        # False = 完全静默（NoMsg，什么都不发）
    'min_level': 'INFO',    # INFO / WARN / ERROR（低于它的直接丢）
    'hz': 2.0,              # INFO 级提示的发送节拍(Hz)；WARN/ERROR 立即发，不受它限制
    'heartbeat_s': 5.0,     # 心跳(s)：长时间没新提示时播报"还在跑 + 当前阶段"；0=关
    'dedup_s': 2.0,         # 相同的(级别,码,文本)在 N 秒内不重复发
}


# ============================================================================
# 七、辅助函数（改配置不用动这一节）
# ============================================================================
def _match(pool, pat):
    """大小写不敏感的通配匹配（pool 里已是大写串）"""
    return bool(fnmatch.filter(list(pool), str(pat).upper()))


def _up(x):
    return str(x).upper()


def stage_params(name, task=None):
    """取某个阶段的参数覆盖：精确阶段名 > task:<类名> > 通配

    name = 阶段名（如 SEEK_BALL_F）；task = 任务类名（如 SeekTask）
    """
    out = {}
    if not STAGE_PARAMS:
        return out
    n = _up(name)
    tn = ('TASK:' + _up(task)) if task else ''

    def _rank(p):
        """0=通配兜底 / 1=task:<类名> / 2=精确阶段名（数字越大优先级越高）"""
        if p == '*':
            return 0
        return 1 if p.startswith('TASK:') else 2

    items = []
    for pat, val in STAGE_PARAMS.items():
        if isinstance(val, dict):
            items.append((_rank(_up(pat)), _up(pat), val))
    items.sort(key=lambda x: x[0])                        # 低优先级先套，后套的覆盖先套的

    for rank, p, val in items:
        hit = (rank == 0) \
            or (rank == 1 and tn and fnmatch.fnmatch(tn, p)) \
            or (rank == 2 and fnmatch.fnmatch(n, p))
        if hit:
            out.update(val)
    return out


def apply_global(cfg):
    """把 GLOBAL_PARAMS 覆盖到配置对象上（板端就是 to32_config 模块对象）

    返回被覆盖的键名列表（便于启动日志里打一行，确认到底生效没有）。
    """
    hit = []
    for k, v in (GLOBAL_PARAMS or {}).items():
        try:
            setattr(cfg, k, v)
            hit.append(str(k))
        except Exception:
            pass
    return hit


def _keep(name, task=None):
    """该阶段是否保留在测试流程里"""
    if not TEST_ENABLED:
        return True
    n = _up(name)
    if n in ('SURFACE', 'DONE'):                          # FORCED：删不掉
        return True
    if not TEST_STAGES:                                   # 没写次级开关 = 全部跑
        return True
    tn = ('TASK:' + _up(task)) if task else ''
    for p in TEST_STAGES:
        pu = _up(p)
        if fnmatch.fnmatch(n, pu) or (tn and fnmatch.fnmatch(tn, pu)):
            return True
    return False


def validate(names_tasks):
    """校验 TEST_STAGES 里有没有写错的名字

    names_tasks: [(阶段名, 任务类名), ...] 或 [阶段名, ...]
    返回 (ok, unknown, kept_names)
      ok=False 表示"有未知名字"，调用方应退回全流程（Fail-Safe）。
    """
    pairs = []
    for item in (names_tasks or []):
        if isinstance(item, (list, tuple)):
            pairs.append((item[0], (item[1] if len(item) > 1 else None)))
        else:
            pairs.append((item, None))
    all_names = [p[0] for p in pairs]

    if not TEST_ENABLED or not TEST_STAGES:
        return True, [], all_names

    pool = set()
    for n, t in pairs:
        pool.add(_up(n))
        if t:
            pool.add('TASK:' + _up(t))
    unknown = [p for p in TEST_STAGES if not _match(pool, p)]
    if unknown:
        return False, unknown, all_names
    return True, [], [n for n, t in pairs if _keep(n, t)]


def summary():
    """一行摘要（进 AUV 时打出来，一眼确认现在在测什么）"""
    if not TEST_ENABLED:
        return '全流程作业'
    if not TEST_STAGES:
        return '测试模式(全部模块, 保持判据≤%ss)' % (TEST_HOLD_MAX_S or '默认')
    return '测试模式(仅 %s, 保持判据≤%ss)' % (','.join(TEST_STAGES), TEST_HOLD_MAX_S or '默认')
