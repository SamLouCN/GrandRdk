# -*- coding: utf-8 -*-
"""viskf 参数总表 —— camera_kalman 工程的**唯一调参入口**。

读取方式
--------
``src/viskf.py`` 的 load_cfg() 按以下顺序找配置（命中即用，**刻意不用 import**）:

    1. 命令行 ``--config <文件>`` / 环境变量 ``VISKF_CONFIG`` 指定的文件
    2. ``<工程根>/config/viskf_config.py``   ← 本文件（独立部署）
    3. ``<工程根>/config/quick_config.py``   ← 回落（并入 GrandRDK 时读其 VISKF_* 段）

工程根 = ``src/viskf.py`` 的上一级目录。都找不到就用内置缺省，**不报错**。
用 ``python3 src/viskf.py --status`` 看当前**实际命中**的来源与全部生效值。

键名规则
--------
一律 ``VISKF_<内置键名大写>``。**本文件没写的键 = 用内置缺省**
（完整清单见 ``src/viskf.py`` 顶部的 ``DEFAULTS``），所以这里只列值得调的。

🔴 = 上真机前必须实测/标定。未标定前输出只能看趋势，**别直接喂控制环**。
"""

# ==================================================================== 数据源 / 输出
# viskf 只读共享内存、只写一个 JSON，**绝不碰相机与串口**。
# 注意姿态源 momo_telemetry.json 由中位机 To32 的 shm_sink 写（属 GrandRDK 侧改动）。
# 该文件不存在时滤波器**自动降级**（flags.att_degraded=True，当机体水平跑），
# 不报错、不阻塞 —— 所以本工程可以先只跑 x 通道，姿态通道等遥测通了再开。
VISKF_ENABLED        = True
VISKF_SHM_DIR        = '/dev/shm'
VISKF_DET_PATH       = '/dev/shm/momo_det_front.json'   # front.py 写（前视 YOLO 检测框）
VISKF_TEL_PATH       = '/dev/shm/momo_telemetry.json'   # To32 shm_sink 写（缺则降级）
VISKF_OUT_PATH       = '/dev/shm/momo_viskf.json'       # 本进程写（原子替换）
# VISKF_LOG_DIR      = '/userdata/kalman/camera_kalman/logs'  # 不写 = <工程根>/logs
VISKF_DET_STALE_S    = 0.5      # det 文件 mtime 超时 → 当作没数据（写端挂了）
VISKF_TEL_STALE_S    = 1.0      # 遥测年龄 < 此值：姿态全补偿
VISKF_TEL_DEGRADE_S  = 3.0      # 此值以内：用保持值；超过：当水平跑（att_degraded）

# ==================================================================== 识别目标
VISKF_TARGET_LABEL   = 'door'   # 必须与 front.py 实际筛出的类别一致
VISKF_MIN_SCORE      = 0.5      # 🔴 按实测检测质量调（front.py 已按 SCORE_THRES 先筛过一遍）
VISKF_CLIP_MARGIN_PX = 2.0      # bbox 距画面边缘小于此值判"该边贴边"
VISKF_CLIP_R_MULT    = 4.0      # 贴边帧观测方差放大倍数
# 贴边**分边放大**，不是"一贴边就三通道全放大"：
#   左右贴边 → 框宽略偏（实测只小 7.6%）→ 只放大 e_x
#   上下贴边 → 框高被裁 ~10%、中心偏移 → 只放大 s_n 与 el
# e_x 不吃上下贴边（它只用 w），el 不吃左右贴边（无 roll 时只依赖 cy）。
# ⚠ e_x 的归一化口径**恒定**（永远除门框宽），贴边也不切换 ——
#   老实现切到"按画面宽归一"会让尺度跳 1.5~2.1 倍，真机实测制造 0.209 的假跳变
#   （正常帧间只有 0.004），把滤波输出抖动放大到观测的 3.9 倍。别再改回去。

# ==================================================================== 相机几何
<<<<<<< HEAD
VISKF_W_IMG          = 640
VISKF_H_IMG          = 480
=======
VISKF_W_IMG          = 640      # [2026-10-10] 由 1280 校正为 640：前视采集/标定都是 640×480，
VISKF_H_IMG          = 480      #   与下面的主点 (320,240) 自洽（原 1280×720 + 主点 320 自相矛盾）。
                                #   注：该键目前在 camera_kalman 代码里未被引用，仅作口径记录。
>>>>>>> 9c99bbf (door适应性修改)
VISKF_X0_PX          = 320.0    # 🔴 主点
VISKF_Y0_PX          = 240.0
VISKF_DX0_PX         = 0.0      # 期望门中心相对主点的横向偏移（对准光轴 = 0）
VISKF_FOCAL_PX       = 554.0    # 🔴 **占位值**。只影响 el（俯仰补偿与到位判据），
                                #    e_x / s_n 免标定、不依赖它。

# ==================================================================== 姿态补偿
VISKF_ATT_ENABLE     = True
VISKF_PITCH_SIGN     = 1.0      # 🔴 符号是 E8 类坑：对着固定门摇机体，补偿后 el 应基本不动，动则翻号
VISKF_ROLL_SIGN      = 1.0      # 🔴 同上（roll 走像面反旋转，精确且不需要焦距）
VISKF_ATT_SIGMA      = 0.01     # 姿态 σ(rad)，进 el 观测方差

# ==================================================================== 观测噪声 σ
# 2026-09-23 真机视频实测回填（已排除贴边帧）。原始出处：
#   D:/RC/Test_any/README_测试报告.md §3.2
#   · e_x  0.0177  （原占位 0.02 基本对）
#   · el   0.0036  （占位 0.005 保守 1.4x）
#   · s_n  0.0043  （占位 0.01  保守 2.3x）
# 仍建议真机静止对着门录 30s 复核一遍再闭环（P2）。
VISKF_SIG_EX         = 0.0177   # e_x（占门宽的比例，无量纲）
VISKF_SIG_EL         = 0.0036   # el (rad)
VISKF_SIG_S          = 0.0043   # s_n

# ==================================================================== 过程噪声密度
# 连续白噪声加速度模型（每通道独立）。调大 = 更信观测（响应快、噪声大）；调小反之。
VISKF_Q_EX           = 0.05
VISKF_Q_EL           = 0.02
VISKF_Q_S            = 0.10     # 接近段 ṡ_n 大，尺寸通道 Q 偏大

# ==================================================================== 门限 / 丢失
VISKF_GATE_NSIGMA    = 3.0      # 新息门限（|新息| / √(S+floor²) 超过即拒收该帧）
VISKF_GATE_RESET_N   = 5        # 连续拒绝 N 帧 → 该通道重置（重捕 / 换门的关键路径）
VISKF_COAST_S        = 0.5      # 无观测超时 → 纯预测滑行（flags.coasting）
VISKF_LOST_S         = 2.0      # 无观测超时 → lost（再出现则整体重置）
# 新息门限的**附加底噪**（模型误差：目标机动 / 线性化误差 / 姿态补偿残差）。
# 这些量都会进新息但**不在 R 里** —— 实测跟踪残差是观测 σ 的 1.5~3.5 倍。
# 取与 σ 同量级 → 门限放宽 √2 倍。不加它的话，实测 σ 回填后拒收从 20 涨到 35 拍
# （好观测被误拒）。同一条经验在 depth_kalman 里也踩过。
VISKF_GATE_FLOOR_X   = 0.018    # ≈ SIG_EX
VISKF_GATE_FLOOR_EL  = 0.0036   # ≈ SIG_EL
VISKF_GATE_FLOOR_S   = 0.0043   # ≈ SIG_S

# ==================================================================== 外推防护
# 真机实测：门离场 6.68s 时老实现仍在 20Hz 输出，e_x 一路自由外推到 +2.38
# （正常跟踪上限只有 +0.60），sig_x 到 2.40（正常 ≤0.50）。数值本身是垃圾。
VISKF_REACQUIRE_SKIP_GATE = True   # coast 后的重捕首帧不做新息门限（此刻状态比观测更不可信）
VISKF_TRUST_AGE_S    = 0.2      # 距最近有效观测超过此值 → trust=False
VISKF_MAX_SIG_X      = 1.0      # sig_x 超此值 → trust=False（track 时 ≤0.50，外推可到 2.31）
# ⚠ 控制侧应判 **trust**，不要只看 gate_visible：门离场后 gate_visible 立刻为 False，
#   但 e_x 仍在持续外推。trust = track 且 age≤TRUST_AGE_S 且 sig_x≤MAX_SIG_X。

# ==================================================================== 节奏
VISKF_LOOP_HZ        = 50.0     # 主循环频率（det 源约 150fps，50Hz 已足够且留余量）
VISKF_OUT_HZ         = 20.0     # 输出 JSON 频率（与控制侧读取节奏对齐）
VISKF_PRINT_HZ       = 2.0      # 终端打印频率
VISKF_LOG_EVERY_S    = 1.0      # 日志文件写入间隔
