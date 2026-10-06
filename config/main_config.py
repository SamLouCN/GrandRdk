import os  # 路径拼接与绝对路径判断，用于推导项目各目录
import struct  # 计算帧头 struct 格式占用的字节数

import quick_config as QC  # 引入日常调参文件，作为主配置的真值来源


# ======================                                                          # 注释: 分组标题 - 项目路径
# 项目路径                                                                       # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
_HERE = os.path.dirname(os.path.abspath(__file__))  # 本文件所在目录，即 config/
PROJECT_ROOT = os.environ.get('GRDK_ROOT') or os.path.dirname(_HERE)  # 项目根；GRDK_ROOT=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）

PATH_CONFIG = os.path.join(PROJECT_ROOT, 'config')  # config/ 目录，存放配置模块
PATH_SRC    = os.path.join(PROJECT_ROOT, 'src')  # src/ 目录，存放各业务进程
PATH_WEB    = os.path.join(PROJECT_ROOT, 'web')  # web/ 目录，存放前端页面与 nginx 配置
PATH_MODELS = os.path.join(PROJECT_ROOT, 'models')  # models/ 目录，存放 BPU 的 .hbm 模型
PATH_LIBS   = os.path.join(PROJECT_ROOT, 'libs')  # libs/ 目录，存放 JPU 硬件解码动态库
PATH_LOGS   = os.path.join(PROJECT_ROOT, 'logs')  # logs/ 目录，存放运行日志

os.makedirs(PATH_LOGS, exist_ok=True)  # 启动时保底创建 logs/，避免下游写日志时崩溃

if os.path.isabs(QC.YOLO_MODEL):  # 判断模型名是绝对路径还是仅文件名
    YOLO_MODEL = QC.YOLO_MODEL  # 绝对路径直接采用，允许指向外部模型
else:  # 只有文件名的情况
    YOLO_MODEL = os.path.join(PATH_MODELS, QC.YOLO_MODEL)  # 拼到 models/ 下形成完整路径

JPU_LIB = os.path.join(PATH_LIBS, 'libmjpg_hw.so')  # JPU 硬件解码库绝对路径，MJPG 解码依赖它

# ======================                                                          # 注释: 分组标题 - 共享内存
# 共享内存                                                                            # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
SHM_DIR          = os.environ.get('GRDK_SHM_DIR') or '/dev/shm'  # 共享内存根(tmpfs)；GRDK_SHM_DIR=测试接缝，仅供无硬件测试重定向（见 hwless_tests/README_hwless_tests.md §八）
SHM_FRAME_FRONT  = f'{SHM_DIR}/momo_frame_front.bin'  # 前视 JPEG 帧文件，web_server 的 /cam1 读它
SHM_FRAME_BOTTOM = f'{SHM_DIR}/momo_frame_bottom.bin'  # 下视 JPEG 帧文件，web_server 的 /cam2 读它
SHM_DET_FRONT    = f'{SHM_DIR}/momo_det_front.json'  # 前视最近一帧检测结果 JSON(检测框列表)
SHM_DET_BOTTOM   = f'{SHM_DIR}/momo_det_bottom.json'  # 下视最近一帧检测结果 JSON
SHM_TELEM        = f'{SHM_DIR}/momo_telemetry.json'  # 汇总两路的统一遥测 JSON
SHM_FLOW_BOTTOM  = f'{SHM_DIR}/momo_flow_bottom.bin'  # 下视 NV12 原始帧，专供光流测速进程

SHM_STATS_FRONT  = f'{SHM_DIR}/momo_stats_front.json'  # 前视运行统计(fps/处理耗时/队列长度)
SHM_STATS_BOTTOM = f'{SHM_DIR}/momo_stats_bottom.json'  # 下视运行统计

HDR_MAGIC = b'MFS1'  # 共享帧头魔数，读端据此校验文件是否合法
HDR_FMT   = '<4sIIIQ'  # 帧头格式：魔数4s + 版本/宽/高 + uint64 微秒时间戳
HDR_SIZE  = struct.calcsize(HDR_FMT)  # 帧头固定字节数，读写双方按此偏移取数据

MAX_JPEG_BYTES = 2 * 1024 * 1024  # 单帧 JPEG 上限 2MB，超出即视为非法帧丢弃
MAX_JSON_BYTES = 64 * 1024  # 单个检测/统计 JSON 上限 64KB，超出即视为非法丢弃

# ======================                                                          # 注释: 分组标题 - 光流测速
# 光流测速 (src/flow_speed.py)                                                   # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
# 2026-09-17: 由 momo_pwmnet/config.ini 的 [sparse]/[motion]/[speed]/[visualization]/[calib]      # 注释: 历史变更
# 段融入本文件; flow_speed.py 只读这里的 FLOW_* 键, 原 config.ini 仅留给 momo_pwmnet 的演示程序。  # 注释: 说明
# [2026-10-04 光流停用] 光流测速整条链路已停用（计算 + 共享）：
#   ① 计算端 src/flow_speed.py 不再由 run.sh 拉起（处理逻辑保留未删，便于将来恢复）；
#   ② 共享端 bottom.py 不再写 momo_flow_bottom.bin（ENABLE_FLOW_SHARE_BOTTOM=False）。
#   下方 FLOW_* 参数仅为"将来恢复"而保留，当前无人读取。
#   ⚠ 图像回传不受影响：web_server 走 SHM_FRAME_BOTTOM(JPEG) -> /cam2。
# 数据源: bottom.py 写入的 SHM_FLOW_BOTTOM (NV12), 由 ENABLE_FLOW_SHARE_BOTTOM 控制  # 注释: 数据源说明
# 启动顺序: 先起 bottom, 后起 flow_speed; bottom 重启后 flow_speed 必须一并重启  # 注释: 启动顺序
#   (写入端重建会把 seq 归零, reader 遇到 seq <= last_seq 会静默丢帧)           # 注释: 后果警告
FLOW_ENABLE              = False  # [2026-10-04 光流停用] 原 True；run.sh 已不再据它拉起 flow_speed.py
FLOW_SHARE_NAME          = 'bottom'  # 光流读哪一路共享帧；front 默认不写，只能选 bottom
FLOW_FPS                 = 50  # 光流目标处理帧率，按此抽帧，实际受算力约束
FLOW_WEB_PORT            = 8000  # 光流调试用 MJPEG 推流端口
FLOW_STREAM_FPS          = 12.0  # Web 推流独立节流，与测速解耦以免 JPEG 编码抢 CPU

# ---- 尺度 (原 [speed]) ----                                                      # 注释: 段落标题
FLOW_PROCESS_SCALE       = 0.5  # 光流处理尺度 0~1，0.5 为半分辨率约快 4 倍但精度下降
FLOW_PX_TO_M             = 0.0  # 画面叠加用的像素到米近似常数(非标定)，0 表示不显示

# ---- 可视化 (原 [visualization]) ----                                            # 注释: 段落标题
FLOW_MJPEG_QUALITY       = 100  # 光流推流 JPEG 画质 1~100，越高越占带宽与 CPU

# ---- 物理标定 (原 [calib]): v(m/s) = px/帧 × (range_m/focal_px) / dt ----        # 注释: 标定公式
# 前提: 相机平移 ⊥ 光轴、场景近似为距离 range_m 的平面; 沿光轴前进/旋转不适用。  # 注释: 适用前提
# 下列为"常见值占位标定", 非真实标定; 真实标定后替换, 输出会持续标注。         # 注释: 现状说明
FLOW_CALIB_ENABLE        = True  # 是否启用物理标定，开启后输出真实 m/s
FLOW_CALIB_FOCAL_PX      = 554.0  # 等效焦距(px)，当前为占位值，需真实标定替换
FLOW_CALIB_RANGE_M       = 1.0  # 相机光心到场景平面距离(m)，占位值，线性影响速度结果
FLOW_CALIB_SOURCE        = 'placeholder_common_values(待真实标定替换)'  # 标定来源标注，输出中持续提示未标定
FLOW_CALIB_USE_NOMINAL_FPS = False  # False 用实测帧间隔 dt，能抗实际帧率抖动
FLOW_CALIB_NOMINAL_FPS   = 50.0  # 仅上一项为 True 时生效的名义帧率

# ======================                                                          # 注释: 分组标题 - Web 服务
# Web 服务                                                                        # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
WEB_HOST = '0.0.0.0'  # 必须是 0.0.0.0，上位机从网卡访问；改回 127.0.0.1 会连不上
WEB_PORT = 5000  # FastAPI Web 服务监听端口
WEB_MJPEG_QUALITY = 100  # MJPEG 推流 JPEG 画质，越高越占带宽与编码耗时
WEB_TELEM_HZ = 10  # 遥测推送频率(预留)，当前实际走 WebSocket

# ======================                                                          # 注释: 分组标题 - Web 后端
# Web 后端                                                                        # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
ENABLE_WEB_NEW      = True  # 启用新版 FastAPI web_server，替代 vp5 的内置 HTTP

DEFAULT_CONFIG = {  # 合成后的全局配置字典，供各业务进程直接取用

# ======================                                                          # 注释: 分组标题 - 全局开关
# 全局开关（来自 quick_config）                                                    # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
    'ENABLE_FRONT_CAM':  QC.ENABLE_FRONT,  # 前视相机总开关，False 则前视进程不起
    'ENABLE_BOTTOM_CAM': QC.ENABLE_BOTTOM,  # 下视相机总开关
    'ENABLE_FLOW_SHARE': False,  # 前视是否写共享帧，默认不写以免白占内存
    # [2026-10-04 光流停用] 原为 True（bottom 写 NV12 共享帧给 flow_speed）。
    #   现改为 False：不再向光流进程共享帧，避免白占内存。
    #   注意：图像回传链路是 SHM_FRAME_BOTTOM(JPEG)，与本键无关，不受影响。
    'ENABLE_FLOW_SHARE_BOTTOM': False,  # 下视是否写共享帧（光流已停用，置 False）

    'ENABLE_FRONT_YOLO':  QC.ENABLE_FRONT_YOLO,  # 前视是否跑 YOLO 检测
    'ENABLE_BOTTOM_HSV':  False,  # 下视 HSV 备用检测(预留)，默认关闭
    'ENABLE_BOTTOM_YOLO': QC.ENABLE_BOTTOM_YOLO,  # 下视是否跑 YOLO 检测

    'ENABLE_PREPROCESS_FRONT':  False,  # 前视图像增强总开关，关闭则整段预处理跳过
    'ENABLE_PREPROCESS_BOTTOM': False,  # 下视图像增强总开关

# ======================                                                          # 注释: 分组标题 - 前视相机
# 前视相机                                                                        # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
    'FRONT_CAMERA': {  # 前视相机采集参数段
        'device': QC.FRONT_DEVICE,  # 设备节点，由 camera_ports.device_for('cam1') 解析
        'index':  0,  # 相机节点 index 兜底值，实际以 device_for 结果为准
        'width':  1280,  # 采集宽度，需相机 MJPG 支持该分辨率否则开流失败
        'height': 720,  # 采集高度
        'fps':    QC.FRONT_FPS,  # 请求帧率，受 USB 带宽与后端能力限制
        'format': 'MJPG',  # 像素格式，MJPG 才能走 JPU 硬件解码
        'backend': 'auto',  # OpenCV 采集后端，auto 优先选 V4L2
        'hardware_decode': True,  # True 用 JPU_LIB 硬解，False 退回 CPU 软解
        'mark_point': [320, 240],  # 画面中心十字标注点，用于算目标相对偏移
    },

    'FRONT_PREPROCESS': {  # 前视预处理参数段
        'enable_color_correct': True,  # 启用水下色彩补偿，提升红通道
        'red_boost': 1.2,  # 红通道增益倍率，过大画面会偏红
        'enable_gaussian': False,  # 是否做高斯滤波去噪，开启会增加处理耗时
        'gaussian_kernel': 5,  # 高斯核大小，必须为奇数
        'enable_clahe': False,  # 是否做 CLAHE 局部对比度增强
        'clahe_clip': 2.0,  # CLAHE 对比度裁剪限制，越大增强越猛、噪声越多
        'clahe_tile': (8, 8),  # CLAHE 分块网格，块越小局部增强越强
    },

    'FRONT_YOLO': {  # 前视推理参数段
        'backend': 'hbm',  # BPU 推理后端，hbm 表示走 hbm_runtime
        'preprocess_mode': 'auto',  # 预处理模式，auto 按模型输入自动选择
        'model_path': YOLO_MODEL,  # .hbm 模型文件路径
        'score_thres': QC.SCORE_THRES,  # 置信度阈值，低于此值的检测框被丢弃
        'nms_thres':   QC.NMS_THRES,  # NMS 的 IoU 阈值，越大保留的框越多
        'strides': [8, 16, 32],  # YOLO 三个输出尺度，需与模型结构一致
        'priority': 0,  # 进程优先级，0 为默认
        'bpu_cores': QC.FRONT_BPU_CORES,  # 允许使用的 BPU 核列表，限定前视可用算力
        'class_names': QC.CLASS_NAMES,  # 模型类别名列表，顺序必须与训练一致
        'label_file': None,  # 可选的 labels.txt 覆盖类别名(预留)
        'target_class_names': QC.FRONT_TARGETS,  # 前视真正要筛出的类别名
        'target_class_ids': [],  # 目标类别 id，留空由 function.py 运行时换算
    },

# ======================                                                          # 注释: 分组标题 - 下视相机
# 下视相机                                                                        # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
    'BOTTOM_CAMERA': {  # 下视相机采集参数段
        'device': QC.BOTTOM_DEVICE,  # 设备节点，由 camera_ports.device_for('cam2') 解析
        'index':  2,  # 相机节点 index 兜底值
        'width':  1280,  # 采集宽度
        'height': 720,  # 采集高度
        'fps':    QC.BOTTOM_FPS,  # 请求帧率
        'format': 'MJPG',  # 像素格式
        'backend': 'auto',  # OpenCV 采集后端
        'hardware_decode': True,  # True 用 JPU 硬件解码
        'mark_point': [640, 360],  # 画面中心十字标注点
        'rotate_180': True,  # 下视相机物理装反，软件补偿 180° 旋转(转正画面与检测坐标)；硬件修好后置 False
    },

    'BOTTOM_PREPROCESS': {  # 下视预处理参数段，结构与前视相同
        'enable_color_correct': True,  # 启用水下色彩补偿
        'red_boost': 1.2,  # 红通道增益倍率
        'enable_gaussian': False,  # 是否做高斯滤波
        'gaussian_kernel': 5,  # 高斯核大小
        'enable_clahe': False,  # 是否做 CLAHE 增强
        'clahe_clip': 2.0,  # CLAHE 对比度裁剪限制
        'clahe_tile': (8, 8),  # CLAHE 分块网格
    },

    'BOTTOM_YOLO': {  # 下视推理参数段
        'backend': 'hbm',  # BPU 推理后端
        'preprocess_mode': 'auto',  # 预处理模式
        'model_path': YOLO_MODEL,  # 模型文件路径，两路共用同一个模型
        'score_thres': QC.SCORE_THRES,  # 置信度阈值
        'nms_thres':   QC.NMS_THRES,  # NMS 的 IoU 阈值
        'strides': [8, 16, 32],  # 输出三个尺度
        'priority': 0,  # 进程优先级
        'bpu_cores': QC.BOTTOM_BPU_CORES,  # 允许使用的 BPU 核列表
        'class_names': QC.CLASS_NAMES,  # 模型类别名列表
        'label_file': None,  # 可选 labels.txt 覆盖类别名
        'target_class_names': QC.BOTTOM_TARGETS,  # 下视真正要筛出的类别名
        'target_class_ids': [],  # 目标类别 id，留空运行时算
    },

# ======================                                                          # 注释: 分组标题 - 主循环 / 采集
# 主循环 / 采集                                                                   # 注释: 标题内容
# ======================                                                        # 注释: 分隔线
    'LOOP_SLEEP': 0.001,  # 主循环空闲休眠(秒)，避免空转占满 CPU
    'N_WORKERS': QC.N_WORKERS,  # 每进程的 worker 线程数，决定检测并行度
    'CAMERA_QUEUE_SIZE': QC.CAMERA_QUEUE_SIZE,  # 采集到 worker 的队列长度，队满则丢帧

    'ENABLE_TIMING': QC.ENABLE_TIMING,  # 是否打印各阶段时延统计
    'TIMING_INTERVAL': 100,  # 每 N 帧输出一次统计
    'SIMPLE_TIMING': QC.SIMPLE_TIMING,  # 简化统计输出，只打帧率

    'ENABLE_LOG': QC.ENABLE_LOG,  # 是否把逐帧结果写入 logs/
    'LOG_DIR': PATH_LOGS,  # 日志输出目录

    'SHOW': QC.SHOW,  # 是否 cv2.imshow，板端无桌面须保持 False
    'SHOW_INTERVAL': 1.0 / 60,  # imshow 帧率上限，约 60fps
}
