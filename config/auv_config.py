# -*- coding: utf-8 -*-
"""AUV 自主任务配置 —— 【唯一调参入口】

[AUV-MISSION 2026-09-26 新增] 与 AUV 任务脚本有关的**所有**可调参数都在这一个文件里，
不要再往 to32_config.py / quick_config.py 里加 AUV_* 键（避免"改一处不生效"）。

本模块被 to32_config.py 用 `from auv_config import *` 整体并入，
因此业务代码继续写 `import to32_config as C` 然后 `C.AUV_XXX` 即可，**不需要改调用方式**。
单独调试时也可以直接 `import auv_config`。

约定:
  * 长度一律 cm（深度/净空阈值），角度一律 deg，时间一律 s，推力一律归一化 [-1,1]
  * 🔴 标记 = 上真机前必须实测标定，当前是占位值，占位值下只能验结构不能验性能

设计前提（与 v1 方案不同，改参数前务必知悉）:
  * 深度主源 = depth_kalman 的 momo_depth.json，不是遥测 raw（遥测链路当前不通）
  * 撞球判定不看 IMU，看视觉（IMU 参数保留，遥测通后 AUV_USE_IMU=True 即启用）
  * 坐底判定看离底净空 clearance（绝对量），不看"深度停滞"
  * 转向是开环定时（无航向反馈），AUV_YAW_RATE_DPS 未标定时转不准

流程: 下潜1m定深 → 前视撞球 → 右转120° → 过门 → 下视捡球坐底 → 上浮1m → 前进3s
      → 悬停3s → 右转60° → 前进触壁 → 上浮结束
"""

# ==================== 数据源 ====================
# 任务脚本只读共享内存，绝不开相机、不开串口（新增消费者只读共享内存是工程硬约束）
AUV_SHM_DIR = '/dev/shm'                    # 共享内存目录: 视觉与深度都从这里读
AUV_DET_FRONT = 'momo_det_front.json'       # 前视检测结果(front.py 写, 20~30Hz)
AUV_DET_BOTTOM = 'momo_det_bottom.json'     # 下视检测结果(bottom.py 写)
AUV_DEPTH_FILE = 'momo_depth.json'          # 融合深度(depth_kalman 写, 20Hz 原子写)
AUV_IMG_W = 640.0                           # 画面宽(采集分辨率): 归一化偏差的分母
AUV_IMG_H = 480.0                           # 画面高
AUV_CLIP_MARGIN_PX = 4.0                    # 贴边判定余量: bbox 触到边沿 ±N px 即算被裁切
AUV_DET_STALE_S = 0.50                      # 视觉新鲜度: 超期一律当"没数据", 绝不拿旧帧做控制
AUV_DEPTH_STALE_S = 1.00                    # 深度新鲜度(卡尔曼 20Hz, 留足余量)
# [2026-09-25 修正] 0.05 → 0.12。原值会和 depth_kalman 的 H_SIGMA_KNOWN=0.05 撞车：
#   H_MODE='known' 时 σ_D 的**稳态下界就是 H 的先验不确定度**，实测（sim_sigma_d.py，
#   无遥测 + B/C 高度计 + 池深 1.3m）稳态 σ_D p50=0.0506 / p95=0.0512 ——
#   门限定 0.05 等于永久判不可信，depth_if 会一直返回 ok=False，任务全程走降级路径。
# 0.12 的依据（同一仿真）：高度计断流后 σ_D +1s=0.090 / +2s=0.174 ⇒ 约 1.4 s 能发现掉线，
#   同时对稳态（0.051）有 2.3× 余量。
# ⚠ 等 H_M 实测标定、H_SIGMA_KNOWN 收紧后（比如 0.02 ⇒ σ_D 稳态 0.024），
#   这个门限可以同步降到 0.05 —— 但**必须一起改**，别只动一边。
AUV_SIGMA_D_MAX = 0.12                      # 卡尔曼 σ_D 超过它就判深度不可信(m)
AUV_MIN_SCORE = 0.50                        # 检测置信度门限: 低于它的目标直接丢弃
AUV_LOG_EVERY_S = 1.0                       # 0x09 日志节流秒数(阶段切换时立即打一条)

# ==================== 两路卡尔曼托管（src/kalman/ 下各一个子工程） ====================
# [2026-09-25] 卡尔曼不再需要单独的启停脚本 —— 由**上位机切进 AUV 模式**这件事驱动：
#   on_enter 自动起（已在跑就不重复起），on_exit 停掉"自己起的那个"。
# 起不来只记一条 WARN，任务照跑（读不到就走各自的降级路径，绝不影响主流程）。
# 板端目录（两路并列，各自保留自己的 config/ src/ tests/ run.sh）：
#   /userdata/GrandRDK/src/kalman/depth_kalman/    深度：高度计 + H_M → momo_depth.json
#   /userdata/GrandRDK/src/kalman/camera_kalman/   图像：前视门框   → momo_viskf.json
AUV_KALMAN_AUTOSTART = True                 # 进 AUV 时自动拉起【深度】卡尔曼
AUV_KALMAN_DIR = '/userdata/GrandRDK/src/kalman/depth_kalman'   # 工程根（含 config/ src/ logs/）
AUV_KALMAN_START_WAIT_S = 3.0               # 停止时等它优雅退出(SIGTERM)的秒数, 超时才 SIGKILL
AUV_KALMAN_FRESH_S = 1.5                    # momo_depth.json 比这更新 ⇒ 认为"已在跑"，不再另起一个
                                            # （防重复启动两个进程同时写 momo_depth.json）
AUV_VISKF_AUTOSTART = True                  # 进 AUV 时自动拉起【图像】卡尔曼（viskf）
AUV_VISKF_DIR = '/userdata/GrandRDK/src/kalman/camera_kalman'
AUV_VISKF_START_WAIT_S = 3.0                # 同上（两路各自独立，一路起不来不影响另一路）
AUV_VISKF_FRESH_S = 1.5                     # momo_viskf.json 新鲜度判据
AUV_VISKF_FILE = 'momo_viskf.json'          # 它写的文件名（在 AUV_SHM_DIR 下）
AUV_VISKF_STALE_S = 0.5                     # 滤波输出超期秒数（viskf 输出 20Hz，留 10 拍余量）
AUV_VISKF_TRUST_GATE = True                 # ★ 按 trust 位门禁（False = 退回只看 track，不推荐）
AUV_VISKF_TRUST_AGE_S = 0.25                # 兜底：trust 字段缺失时，age 超过它就判不可信
AUV_VISKF_MAX_SIG_X = 1.0                   # 兜底：trust 字段缺失时，sig_x 超过它就判不可信

# ==================== 过门闭环（用 viskf 的滤波量，关掉则退回原始像素伺服） ====================
# 三个自由度的分工（与 viskf 输出一一对应）：
#   e_x（横向偏差，按门宽归一化、无量纲）→ 偏航；de_x 是它白送的速度，直接当 D 项用
#   s_n（√(w·h)/画面宽，免标定）        → 前向推力：越近推力越小，靠 CROSS_TOL 收尾
#   el （水平系仰角，含俯仰补偿）        → 只做"到位确认"，不反修深度（遥测 0 帧时它不可信）
AUV_VISKF_USE = True                        # True = PASS_GATE 用滤波量；False = 完全用原始框
AUV_GATE_YAW_KP_DPS = 40.0                  # 🔴 e_x → 偏航速率的比例增益(°/s per 单位 e_x)
AUV_GATE_YAW_KD_DPS = 8.0                   # 🔴 de_x → 偏航速率的微分增益
AUV_GATE_YAW_RATE_MAX_DPS = 20.0            # 偏航速率限幅(°/s)
AUV_GATE_YAW_DEADBAND = 0.03                # |e_x| 死区（滤波后噪声小，比像素死区小一个量级）
AUV_GATE_S_STAR = 0.85                      # 🔴 到位尺度：s_n 到多少算"贴脸该穿了"（现场试）
AUV_GATE_CROSS_TOL = 0.05                   # ★ 到位容差：**不要设 0** —— 见下方说明
#   ⚠ CROSS 触发必须留容差，否则任务卡死：接近段越近推力越小，而 0x09 的 surge 是
#     int8×127（分辨率 1/254≈0.004），推力掉到 0.004 以下直接被量化成 0，
#     机器人停在 s_star 下方几个千分点处不动，硬阈值 s_n ≥ s_star 永远等不到 → 干等超时。
AUV_GATE_SURGE_K = 1.2                      # 🔴 前向推力律：surge = K·(s_star − s_n)
AUV_GATE_SURGE_MAX = 0.85                   # 前向推力上限
AUV_GATE_SURGE_MIN = 0.25                   # 前向推力下限（别让它被量化成 0）
AUV_GATE_SWAY_WITH_VISKF = False            # 用滤波偏航时是否还叠横向平移（默认不叠，避免打架）

# ==================== 目标类别 ====================
# 这里写 canonical 名; 模型原始名 → canonical 名的映射在 vision_if.CANON 表
# 🔴 新模型的球类别名出来后要改三处: 本段 + vision_if.CANON + quick_config.FRONT_TARGETS
AUV_LABEL_BALL = 'ball'                     # 撞球目标(前视): 现在模型里叫 red-ball
AUV_LABEL_GATE = 'gate'                     # 门(前视): 模型里叫 door
AUV_LABEL_PICK = 'ball'                     # 捡球目标(下视)

# ==================== 瞄准点(画面像素坐标) ====================
# dx/dy = 目标中心 − 瞄准点，带符号；控制律就是把 dx/dy 修到 0
AUV_AIM_RAM = (320, 240)                    # 撞球瞄准点: 画面中心 —— 开了垂向伺服就对准球心
                                            # （旧值 (320,300) 是"球略偏下好撞"，有伺服后不需要了）
AUV_AIM_GATE = (320, 240)                   # 过门瞄准点: 画面中心
AUV_AIM_PICK = (320, 240)                   # 捡球瞄准点: 画面中心

# ==================== 深度与时间 ====================
AUV_POOL_DEPTH_M = 1.30                     # 🔴 池深(m): 用户说"至少 1.3", 上机前实测
AUV_DIVE_DEPTH_M = 1.00                     # 巡航定深(m): ⚠ 1.3m 池里离底仅 ~30cm, 蹭底就降到 0.8
AUV_DEPTH_TOL_CM = 8.0                      # 定深到位判据(cm)
AUV_DEPTH_HOLD_S = 2.0                      # 到位需连续保持(s): 防在目标附近抖一下就算到位
AUV_DIVE_TIMEOUT_S = 30.0                   # 下潜保护(s): 到点还没到位也放行
AUV_DIVE_FALLBACK_S = 12.0                  # 无深度源时的定时放行(s): 不能判"到位"就别死等
AUV_SEEK_TIMEOUT_S = 30.0                   # ★ 三次搜索超时(s): 超时一律上浮中止
AUV_SEEK_SETTLE_S = 1.5                     # 搜索前稳定期(s): 防下潜水花/气泡误检
AUV_RAM_TIMEOUT_S = 20.0                    # 撞球保护(s)
AUV_GATE_TIMEOUT_S = 30.0                   # 过门保护(s)
AUV_ASCEND_TIMEOUT_S = 20.0                 # 坐底后上浮至 1m 的保护(s)
AUV_SURFACE_DEPTH_M = 0.0                   # 最终上浮目标(m): 0 = 水面
AUV_SURFACE_DONE_CM = 15.0                  # 判定已出水的深度(cm)
AUV_SURFACE_TIMEOUT_S = 30.0                # 上浮保护(s)
AUV_BOTTOM_HOLD_S = 3.0                     # ★ 坐底保持(s): 你要求的 3s
AUV_ASCEND_DEPTH_M = 1.0                    # ★ 坐底后上浮目标(m)
AUV_SURGE3_S = 3.0                          # ★ 上浮后前进 3s
AUV_HOVER_S = 3.0                           # ★ 悬停 3s

# ==================== 转向(开环定时) ====================
# 没有航向反馈 → 只能按标称角速度算时长: TURN_S = 角度 / AUV_YAW_RATE_DPS
AUV_TURN1_DEG = 120.0                       # ★ 撞球后右转角度
AUV_TURN2_DEG = 60.0                        # ★ 最后右转角度
AUV_YAW_RATE_DPS = 30.0                     # 🔴 转向速率(°/s): 必须水池标定, 它决定转多久
AUV_TURN_RIGHT_SIGN = 1.0                   # 🔴 "右转"符号: 水池实测定 +1 或 -1
AUV_TURN_SETTLE_S = 1.0                     # 转到位后的稳定(s): 让水阻尼把余速吃掉
AUV_TURN_TIMEOUT_S = 10.0                   # 转向保护(s)
AUV_YAW_TOL_DEG = 3.0                       # 有遥测时的转向到位判据(°): 无遥测时用不上

# ==================== 坐底 ====================
# 主判据是离底净空 clearance（绝对值，与池深/钳位/漂移都无关），不是"深度不再变化"
AUV_SIT_RATE_CMS = 15.0                     # 下沉速率(cm/s): 按 dt 累加, 不是每拍加一个定值
AUV_SIT_CLEAR_CM = 6.0                      # 🔴 离底净空阈值(cm): 小于它判坐底; 超声盲区典型 3~5cm
AUV_SIT_CLEAR_HOLD_S = 1.0                  # 净空判据需连续保持(s)
AUV_SIT_VZ = 0.02                           # 无高度计时的垂速判据(m/s)
AUV_SIT_STALL_CM = 2.0                      # 无高度计时的深度停滞判据(cm)
AUV_SIT_STALL_S = 2.0                       # 停滞需持续(s)
AUV_SIT_TIMEOUT_S = 25.0                    # 坐底保护(s): 到了就按"到底"处理, 不原地死等
AUV_CENTER_TIMEOUT_S = 15.0                 # 下视对中保护(s)

# ==================== 运动增益 ====================
AUV_SURGE_SEEK = 0.25                       # 搜索阶段前进(归一化推力)
AUV_SURGE_RAM = 0.40                        # 撞球冲刺
AUV_SURGE_GATE = 0.30                       # 穿门
AUV_SURGE_WALL = 0.35                       # 触壁前冲
AUV_KP_SWAY = 0.60                          # 横向对中 P 增益(归一化偏差 → sway)
AUV_SWAY_SIGN = 1.0                         # 🔴 横向对中符号: 目标在画面右(ex>0)时往哪边推; 反了会发散
AUV_SWAY_MAX = 0.50                         # sway 限幅
AUV_KP_SURGE_PICK = 0.40                    # 下视纵向对中(ey → surge)
AUV_SURGE_SIGN = 1.0                        # 🔴 下视纵向对中符号: 目标在画面下(ey>0)时该前进还是后退
AUV_SURGE_MAX_PICK = 0.30                   # 捡球对中限幅
AUV_MIN_THRUST = 0.10                       # int8 量化补偿: |推力| 小于它会被引脚归零, 抬到该值

# ==================== 视觉伺服（垂向 + 姿态） ====================
# ★ 球和门的高度未知，不再是"下潜到 1m 直冲"。看到目标后由前视画面闭环对准：
#     纵向偏差 dy → 改【目标深度】（heave 没有推力槽位，垂直只能走目标深度）
#     横向偏差 dx → 改【目标航向】（让机头正对目标，不只是平移对中）
# 🔴 四个符号/增益全部要水池实测；符号反了会发散（与 AUV_SWAY_SIGN 同理）。
AUV_VERT_SERVO = True                       # 垂向伺服总开关：用 dy 驱动目标深度
AUV_KP_DEPTH_CMS = 40.0                     # 🔴 垂向增益: ey=±1(画面上下沿)时的深度变化率 cm/s
AUV_DEPTH_SIGN = 1.0                        # 🔴 垂向符号: 目标在画面下方(ey>0)时 +1=下潜 / -1=上浮
                                            #    ★ 用户已定方向：在下方就下沉 → 保持 +1
                                            #      depth_cm 是绝对量(0=水面)，没有转向那种约定耦合
AUV_DEPTH_RATE_MAX_CMS = 30.0               # 垂向速率限幅(cm/s): 防止对着目标一头扎到底
AUV_VERT_DEADBAND_PX = 15.0                 # 纵向死区(px): |dy| 小于它就不动，防抖
AUV_DEPTH_MIN_CM = 10.0                     # 伺服能达到的最浅目标深度(cm): 留出水面余量
AUV_DEPTH_BOTTOM_MARGIN_CM = 10.0           # 离池底安全余量(cm): 伺服最深 = 池深 - 这个值
AUV_YAW_SERVO = True                        # 姿态伺服总开关: 用 dx 驱动目标航向（正对目标）
AUV_KP_YAW_DPS = 25.0                       # 🔴 航向增益: ex=±1(画面左右沿)时的角速度 °/s
AUV_YAW_SIGN = 1.0                          # 🔴 航向符号微调: 目标在画面右侧(ex>0)时 +1=右转 / -1=左转
                                            #    ★ 已跟随 AUV_TURN_RIGHT_SIGN：右转标定成 -1 时伺服自动跟着反,
                                            #      这里保持 +1 即可，只有实测发现仍反才动它
AUV_YAW_RATE_MAX_DPS = 20.0                 # 航向角速度限幅(°/s)
AUV_YAW_DEADBAND_PX = 15.0                  # 横向死区(px)

# ---- 搜索期的深度扫描：目标高度未知，光在当前深度平扫可能永远看不见 ----
AUV_SEEK_SCAN = True                        # 前视两个搜索阶段是否上下扫描深度
AUV_SCAN_RANGE_CM = 30.0                    # 扫描幅度(cm): 在"进入搜索时的深度"上下 ±这么多
AUV_SCAN_PERIOD_S = 12.0                    # 扫描周期(s): 一个完整上下扫的时间

# ==================== 状态回传（网线 → 上位机） ====================
# ★ 你要实地看自动运行效果，所以要有观测通道；但**观测绝不能拖累控制**：
#   上报在独立线程里做，tick() 只往队列塞一份快照（O(1)、队满丢旧保新），
#   连续失败 N 次就永久放弃，状态机照跑。
# ★ "与 ROV 一致"指的是通道与格式，不是重复造轮子：
#     图像 = web_server 的 http :5000 /cam1 /cam2（front/bottom 写共享内存 + YOLO 框），
#            由 web_server 常驻提供，**与运行模式无关**，AUV 下照样有画面，这里不重复开相机；
#     数据 = $TEL 41 字段（:8081）本就不断流（mode_auv 返回 None → 回落 tel_builder），
#            $AUV 帧是它的补充，装 41 字段塞不下的阶段/目标/观测。
AUV_REPORT_ENABLED = True                   # 状态回传总开关: False = 完全不建线程、不发一字节
AUV_REPORT_IP = '192.168.127.100'           # 上位机 IP（网线对端）
AUV_REPORT_PORT = 8085                      # 上报端口: 避开已占用的 8080/8081/8082/8084
                                            #   想复用 $TEL 那条的原始报文窗口可改成 8081
AUV_REPORT_HZ = 5.0                         # 上报节拍(Hz): 远低于 20Hz 主循环，省带宽也省 CPU
AUV_REPORT_MAX_FAIL = 3                     # ★ 连续发送失败几次后【永久放弃】回传（不重试、不刷屏）

# ==================== 判定阈值 ====================
AUV_PIX_TOL_PX = 25                         # 对中容忍像素: |dx|,|dy| 都小于它才算对上
AUV_CENTER_HOLD_S = 1.5                     # 对中需连续保持(s)
AUV_USE_IMU = False                         # 🔴 IMU 判据总开关: 遥测通 + 阈值标定后才置 True
AUV_IMPACT_ACC = 0.0                        # 🔴 撞击加速度阈值(待标定): 0 = 未启用
AUV_IMPACT_WINDOW_S = 0.3                   # 超阈值需持续(s): 防单帧尖峰误判
AUV_RAM_ARM_S = 0.8                         # 进入撞球后的武装延迟(s): 避开启动/转向自身的加速度
AUV_RAM_W_PX = 220.0                        # 球宽突增阈值(px): 撞球主判据, 贴脸 = 撞上
AUV_RAM_W_NEAR_PX = 150.0                   # "曾贴脸"阈值(px): 配合下面"消失"判据用
AUV_RAM_LOST_S = 0.8                        # 贴脸后目标消失需持续(s): 撞飞/撞沉/顶出画面
AUV_RAM_VZ = 0.15                           # 撞击引起的垂向速度判据(m/s)
AUV_RAM_DZ = 0.08                           # 撞击引起的深度跳变判据(m)
AUV_GATE_LOST_S = 0.5                       # 门宽超阈值后消失需持续(s) → 判定已穿过
AUV_GATE_PASS_W_RATIO = 0.80                # 门宽 / 画面宽 超过它 = 门已贴脸(下一步就是消失)
AUV_WALL_IMPACT_ACC = 0.0                   # 🔴 触壁加速度阈值(待标定)
AUV_WALL_TIMEOUT_S = 20.0                   # 触壁保护(s): ⚠ 无 IMU 时它就是实际判据

# ============================================================================
# ==================== 任务化重构 v7 新增（2026-10-04） ======================
# 配合 src/to32/move_test/auv_task/ 使用。旧键全部保留（旧 mission.py 仍在 move_test 里），
# 下面这些键只有新的任务化执行器会读。
# 约定沿用：长度 cm / 角度 deg / 时间 s / 推力归一化 [-1,1]；🔴 = 必须现场标定的占位值
# ============================================================================

# ==================== 水池实测尺寸（★ 现场第一件事就是把这几个量出来） ====================
# 赛事规则给的是"离底高度"，固件只认"目标深度"，换算在 servo.depth_of_height():
#     目标深度 = AUV_POOL_DEPTH_CM − 离底高度
AUV_POOL_DEPTH_CM = 130.0                   # 🔴 池深(cm): 规则只说"约 1.3m"，到场必须实测
AUV_MIN_DEPTH_CM = 10.0                     # 目标深度下限(cm): 留出水面余量（等同 AUV_DEPTH_MIN_CM）
AUV_BALL_HEIGHT_CM = 60.0                   # 撞球: 规则给"距底 60cm"        → 深度 70cm
AUV_GATE_LOW_HEIGHT_CM = 45.0               # 矮门: 中心距底 45cm            → 深度 85cm
AUV_GATE_HIGH_HEIGHT_CM = 65.0              # 高门: 中心距底 65cm            → 深度 65cm
AUV_RACK_HEIGHT_CM = 30.0                   # 待捡球: 规则给 20~40cm，取中   → 深度 100cm
AUV_BASKET_HEIGHT_CM = 45.0                 # 收集框: 规则给 20~60cm，取中
AUV_HOME_DEPTH_CM = 25.0                    # 回出发区时的巡航深度(cm)
AUV_RELEASE_HEIGHT_CM = 110.0               # 投放前悬停: 离底 110cm ≈ 水深 20cm（先浮起来再看）
AUV_SURGE_MAX = 1.0                         # surge 限幅（DepthTask/TimedTask 用）

# ==================== 门（现场 3~4 个，高矮交替） ====================
AUV_GATE_COUNT = 4                          # 门数量: 3 或 4（plan.py 按它生成 SEEK/PASS 段）
AUV_GATE_SEQ = 'low,high,low,high'          # 高矮顺序: 与现场实际一致；写错只是深度对不准，不会崩

# ==================== 测试模式（方案 §16b/§16c） ====================
# 总开关 False = 全流程作业（验收）；True = 测试模式，只跑 AUV_TEST_STAGES 里的段。
# ★ SURFACE / DONE 强制保留，任何配置都删不掉 —— 上浮不允许被关闭。
AUV_TEST_MODE = False                       # ★ 总开关：验收前必须置回 False
AUV_TEST_STAGES = []                        # 次级开关：要跑哪些段，支持 * ? 通配；[] = 全部
                                            #   例：['SEEK_BALL_F']  ['SEEK_GATE_*','PASS_GATE_*']
AUV_TEST_HOLD_MAX_S = 5.0                   # 测试模式下"判据需保持 N 秒"的上限（钳到 [1,30]）
AUV_TEST_LOOP = False                       # 单段循环调试：跑完最后一段是否回到第一段

# ==================== 收尾 / 预算（赛事硬约束） ====================
AUV_BUDGET_S = 900.0                        # ★ 全局预算(s): 规则给 15 分钟，到点强制上浮收尾
AUV_SURFACE_THEN_ROV = True                 # 上浮完成后请求切回【有线 ROV】
                                            # ★★ 走的是 dispatcher 模式 id（to32_config.MODE_ROV=0），
                                            #    不是 link_stm32.MODE_ROV(0x03, 无线 ROV, S100 禁发)。
                                            #    写成后者 0x04 一帧都发不出去，机器人会留在 AUV。

# ==================== 投放舵机（★ 未实装，先走 Stub） ====================
AUV_SERVO_IMPL = 'stub'                     # 'stub'(未实装, 打 WARN 跳过) / 'serial'(需注入 AUV_SERVO_SEND)
AUV_SERVO_RETRY = 3                         # serial 模式下最多尝试几次后放弃（放弃也不中止任务）

# ==================== 投放动作序列（方案 §16g：浮起来 → 走一段 → 悬停 → 投放） ====================
AUV_RELEASE_TURN_DEG = 90.0                 # 🔴 投放前转向角度(deg): 方向/距离待现场定
AUV_RELEASE_DIST_M = 2.0                    # 🔴 投放前推进距离(m): 先按 2m 试
AUV_RELEASE_HOVER_S = 3.0                   # 投放前悬停(s): 把余速吃掉，舵机投放才准

# ==================== 航位推算（替代引导线，方案 §16.5.1） ====================
# 比赛方引导线质量差 → 不用；改用高精度 IMU。IMU 来自 STM32 遥测应答，g 恒取 9.81。
AUV_G = 9.81                                # 重力加速度: 用户定，恒 9.81
AUV_SPEED_MPS = 0.25                        # 🔴 标称巡航速度(m/s): 必须水池标定（标定前只验结构）
AUV_SPEED_K_SURGE = 1.0                     # 🔴 推力→速度系数: surge 0.3 时的速度 = SPEED_MPS×K
AUV_DR_USE_IMU = False                      # 🔴 是否用 IMU 二次积分当位移主源（标定完成前**不要**开）
                                            #    无姿态补偿下几十秒就能漂出几米
AUV_DR_STILL_G = 0.05                       # "没在动"判据: |acc − g| 小于它持续 STUCK_S 秒 → 打 WARN
AUV_DR_STUCK_S = 2.0                        # 疑似被顶住的告警延时(s)
AUV_HOME_TURN_DEG = 90.0                    # 🔴 回出发区的转向角度(deg): 现场按出发区方位定

# ==================== 测试期监督通道（方案 §17） ====================
# AUV 模式不执行上位机杆位（on_cmd 只记日志），但流程要能被遥控 —— 走 $AUVCTL。
AUV_CMD_POLICY = 'supervise'                # ignore(全部拒绝) / supervise(默认) / passthrough(极端调试)
                                            # ⚠ 只在**测试模式**下生效；真实作业模式被下面的锁强制成 ignore
AUV_ACCEPTANCE_LOCK = True                  # ★ True = 真实作业(验收)模式**强制** policy=ignore：
                                            #   除"上位机切模式"外，拒绝一切干预（$AUVCTL 全拒）
                                            #   False = 真实模式也用 AUV_CMD_POLICY（不建议在比赛机上开）
AUV_TEST_FORCE_VIDEO = True                 # ★ True = 测试模式进 AUV 时自动打开图像回传（:5000 /cam1 /cam2）
                                            #   真实作业模式不动它（验收无缆，本来就没有回传通道）
# ⚠⚠ 2026-10-05 改为 False（原 True）：需求是"测试/作业中上位机一切回 ROV 就立即终止任务、
# 交还遥控"。锁住 mode 字段 = 上位机切不回来 = 水池里出事没人能接管，绝不允许。
AUV_LOCK_MODE = False                       # True = 忽略 $CMD 的 mode 字段（**与"随时可接管"冲突，别开**）

# ==================== ★ AUV 启动门控（2026-10-05 需求） ====================
# 无论**正式作业还是测试**，AUV 都只能由上位机发送指令（$CMD 带 mode=AUV）切入。
# 上电默认 / 命令行 --mode auv / 模式记忆文件 这三种来源一律回落到 IDLE 待命。
AUV_REQUIRE_PC_CMD = True                   # True = 必须由上位机指令切入（**默认开，别关**）
                                            # False = 允许上电自启进 AUV（仅"完全无人值守"时用）
# 退出 AUV（上位机切走）时是否补发一帧"零推力 + 停推"的 0x09，让下位机立刻不动
AUV_STOP_FRAME_ON_EXIT = True               # True = 发；急停闩锁期间不发（0x09 会退出急停）

# ==================== ★ 上位机终端状态提示（$MSG 帧，2026-10-05 需求） ====================
# 人在岸上看不到板子日志 → 单独开一条纯文本通道（与 $TEL 41 字段 / $AUV 16 字段互不干扰），
# 上位机按帧头 $MSG 分发，直接打到终端窗口：当前在测哪一段 / 警告 / 报错。
AUV_MSG_ENABLED = True                      # 总开关（无缆验收时发不出去，会自己降级放弃，不影响任务）
AUV_MSG_IP = '192.168.127.100'              # 上位机 IP（默认与 AUV_REPORT_IP 一致）
AUV_MSG_PORT = 8085                         # 端口（默认与 $AUV 同端口；帧头不同，老解析不受影响）
AUV_MSG_HZ = 2.0                            # INFO 级提示的发送节拍(Hz)（WARN/ERROR 立即发）
AUV_MSG_MIN_LEVEL = 'INFO'                  # 最低发送级别: INFO / WARN / ERROR
AUV_MSG_DEDUP_S = 2.0                       # 相同(级别,码,文本)在 N 秒内不重复发（防刷屏）
AUV_MSG_MAX_FAIL = 3                        # 连续失败几次后**永久放弃**（发不出去就别拖累主功能）
AUV_MSG_HEARTBEAT_S = 5.0                   # 心跳(s): 长时间没新提示时播报一次"还在跑 + 当前阶段"；0=关
