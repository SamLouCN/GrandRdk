# -*- coding: utf-8 -*-
"""下位机（STM32）链路 —— V2 二进制协议【骨架，预留】

本文件只搭"传输 + 组帧/收帧 + 遥测轮询"骨架；
「各模式要把什么下发下去」的映射逻辑写在各模式文件里（mode_rov.py / mode_auv.py）。

协议依据（以固件源码为准，勿凭记忆改）:
  《上位机对接说明_V2》 + 《V2_固件字段表_T0.1》
  - 帧:  CD | LEN | FUNC | DATA(n) | DC      LEN = n+3, 整帧 = LEN+1, ≤64B, 多字节小端
  - 只有 0x0C 遥测请求会回帧（1:1）；命令帧 0x01~0x0B 只执行、不回包
  - 0x09 摇杆综合运动 = 15B:
        [0..1] 目标Pitch int16×100   [2..3] 目标Yaw int16×100   [4..5] 目标Roll int16×100
        [6..7] 目标深度 uint16×100(cm)  [8] surge int8×2  [9] sway int8×2  [10] FLAG(bit0)
  - 0x04 模式（2026-10-03 多源控制协议换代号）:
        0x00 启动/恢复(START)   0x01 急停(STOP)      0x02 预编程(PREGRAM)
        0x03 无线ROV(ROV_WIRELESS)  0x04 测试(TEST)
        0x05 AUV                0x06 有线ROV(ROV_TETHERED)
    ⚠ S100 侧**禁发** 0x02(预编程) 与 0x03(无线ROV) —— 见《多源控制协议设计.md》§5.3；
      发送守卫见 frame_mode()。0x02 测试油门(FUNC_TEST_THROTTLE) 亦禁止实现（§12.3）。
  - 旧0x01组帧按0~3姿态/深度对接；v3.7新PID中继接受参考编号0~7并原字节转发，
    4~7在参考固件为补偿回路，业务映射以现役固件为准。
  - **0x09 没有 STANDBY 守卫**：急停后再发 0x09 会静默退出急停 → 急停锁存必须由中位机负责
"""
import threading  # 线程模块：轮询线程与停止事件
import time  # 时间模块：轮询节拍与请求时间戳

try:                      # 组帧缺省值取自 config.py（单一来源；无 config 时用内置缺省）
    import to32_config as C  # 导入板端配置模块，别名为 C
except Exception:         # 防御：正常部署下 config.py 必然存在
    C = None  # 导入失败时置空，后续 _cfg 走内置缺省值


def _cfg(name, default):  # 读取配置项：无 config 模块时用 default
    return getattr(C, name, default) if C is not None else default  # 有 config 就取其同名属性，否则退回缺省


HEAD = 0xCD  # 帧头 CD，收帧同步的唯一锚点
TAIL = 0xDC  # 帧尾 DC，用于确认整帧完整未错位
MAX_FRAME = 64  # 整帧最大字节数，超过即判为长度非法

# ---- 功能码 ----
FUNC_SET_PID        = 0x01  # 0x01 设置 PID 参数（通道 0~3）
FUNC_TEST_THROTTLE  = 0x02  # 0x02 测试油门（单桨试转）
FUNC_TARGET_ANGLE   = 0x03  # 0x03 下发目标姿态角
FUNC_MODE           = 0x04  # 0x04 模式命令（启动/急停/预编程/ROV/测试）
FUNC_SAVE           = 0x05  # 0x05 保存参数到固件
FUNC_READ_ANGLE     = 0x08  # 0x08 读取姿态角
FUNC_MOTION         = 0x09  # 0x09 摇杆综合运动（15B，无 STANDBY 守卫）
FUNC_DEPTH          = 0x0A  # 0x0A 下发目标深度
FUNC_READ_DEPTH     = 0x0B  # 0x0B 读取深度
FUNC_TELEMETRY      = 0x0C  # 0x0C 遥测请求，唯一会触发下位机回帧的功能码

# v3.7 PC PID 中继：参考协议编号0~7；业务名称由上位机/现役固件匹配。
PID_FRAME_LEN = 11
PID_REFERENCE_MAX_INDEX = 7

# ---- 模式命令 ----
MODE_START     = 0x00  # 启动/恢复运行
MODE_ESTOP     = 0x01  # 急停（固件侧停车）
MODE_PREPROGRAM = 0x02  # 预编程（自主航迹）模式
MODE_ROV       = 0x03  # [2026-10-03 重定义] 无线 ROV（LoRa 侧用；S100 禁发）
MODE_TEST      = 0x04  # 测试模式
MODE_AUV        = 0x05  # [2026-10-03 新增] AUV 自主航行
MODE_ROV_TETHERED = 0x06  # [2026-10-03 新增] 有线 ROV（S100 中继）
MODE_ROV_WIRELESS = MODE_ROV  # 语义别名，便于阅读
# S100 无权发送的模式码（《多源控制协议设计.md》§5.3）：物理挡在 frame_mode 里
S100_FORBIDDEN_MODES = (MODE_PREPROGRAM, MODE_ROV)

TELEM_FRAME_LEN = 48    # 0x0C 遥测回帧整帧 48B（LEN=0x2F）
# ==================== 组帧 ====================
def build_frame(func, data=b""):  # 通用组帧入口：所有下发帧都经此拼装
    """通用 V2 组帧: CD LEN FUNC DATA DC"""
    data = bytes(data)  # 统一转成字节序列，允许调用方传 list/bytearray
    return bytes([HEAD, len(data) + 3, func & 0xFF]) + data + bytes([TAIL])  # LEN=n+3 含 FUNC+DATA+字节数本身


def frame_telemetry_request():  # 构造 0x0C 遥测请求帧
    """0x0C 遥测请求（唯一会触发回帧的帧）"""
    return build_frame(FUNC_TELEMETRY)  # 无 DATA 的空负载帧


def frame_mode(mode):  # 构造 0x04 模式命令帧
    """0x04 模式命令
    [2026-10-03] 增加 S100 禁发守卫：0x02(预编程) / 0x03(无线ROV) 一律拒发并返回 None。
    《多源控制协议设计.md》§12.3 要求这两条通路"S100 侧不实现" —— 固件层不拦，
    所以必须在这里物理挡掉（返回 None，调用方 send 会跳过空帧）。
    返回 bytes；被拒时返回 None。
    """
    if (mode & 0xFF) in S100_FORBIDDEN_MODES:  # 命中禁发码
        _forbidden_mode_log(mode)  # 告警（每码一次，避免 20Hz 刷屏）
        return None  # 拒发：调用方须能处理 None
    return build_frame(FUNC_MODE, bytes([mode & 0xFF]))  # DATA 仅 1 字节模式值


def _forbidden_mode_log(mode):  # 禁发模式码告警
    """禁发模式码告警（每个码只提示一次，避免 20Hz 刷屏）"""
    global _FORBIDDEN_WARNED
    m = mode & 0xFF
    if m not in _FORBIDDEN_WARNED:
        _FORBIDDEN_WARNED.add(m)
        name = {MODE_PREPROGRAM: "0x02 预编程", MODE_ROV: "0x03 无线ROV"}.get(m, hex(m))
        import sys
        sys.stderr.write(
            "[link_stm32] 拒发禁发模式码 %s —— S100 无此权限（《多源控制协议设计.md》§5.3）\n" % name)
_FORBIDDEN_WARNED = set()


def frame_motion(pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0, depth_cm=0.0,  # 0x09 运动帧：前三项目标姿态角与深度
                 surge=0.0, sway=0.0, stick_stop=False,  # surge/sway 归一化推力，stick_stop 为停推标志
                 full_scale=None, depth_max_cm=None):  # 量程参数缺省由 config 提供
    """0x09 摇杆综合运动（供 ROV 模式调用，见 mode_rov.on_cmd）

    pitch/yaw/roll: 目标姿态角(度)；depth_cm: 目标深度(cm, 钳位 0~depth_max_cm)
    surge/sway: [-1,1] → int8(×full_scale)，固件内再 ×2 → 推力域 ±254
    stick_stop: FLAG bit0 = 停 surge/sway（保留姿态与深度）
    full_scale / depth_max_cm: 缺省取 config.SURGE_FULL_SCALE / config.DEPTH_MAX_CM
                               （2026-09-15：量程不再写死，改 config 即生效）
    """
    import struct  # 局部导入：打包 15B 二进制运动数据
    if full_scale is None:  # 未指定推力量程
        full_scale = _cfg("SURGE_FULL_SCALE", 127.0)  # 取 config 量程，缺省 127
    if depth_max_cm is None:  # 未指定深度上限
        depth_max_cm = _cfg("DEPTH_MAX_CM", 200.0)  # 取 config 深度上限，缺省 200cm
    p = int(round(max(-90.0, min(90.0, float(pitch_deg))) * 100))  # Pitch 钳位 ±90° 后转 int16×100
    y = int(round(float(yaw_deg) * 100))  # Yaw 不钳位，直接转 int16×100
    r = int(round(max(-90.0, min(90.0, float(roll_deg))) * 100))  # Roll 钳位 ±90° 后转 int16×100
    d = int(round(max(0.0, min(float(depth_max_cm), float(depth_cm))) * 100))  # 深度钳到 0~上限再 ×100
    sg = int(round(max(-1.0, min(1.0, float(surge))) * float(full_scale)))  # surge 钳位 ±1 后映射到 int8 量程
    sw = int(round(max(-1.0, min(1.0, float(sway))) * float(full_scale)))  # sway 同上
    flag = 0x01 if stick_stop else 0x00  # FLAG bit0：置 1 表示停 surge/sway、保留姿态深度
    data = struct.pack("<hhhHbbB", p, y, r, d, sg, sw, flag)  # 小端打包：3×int16 + uint16 + 2×int8 + uint8
    return build_frame(FUNC_MOTION, data)  # 套 V2 帧头帧尾输出


def frame_set_pid(ch, kp, ki, kd):  # 构造 0x01 PID 设置帧
    """0x01 设置 PID（固件仅认 ch 0~3；STANDBY 模式下整帧被忽略）

    量纲 int16×100 → |系数| ≤ 327.67。超范围一律钳位：否则 struct.pack 会抛
    struct.error 打断调用方（上位机输入框没有限幅，1e6 这类值会把中位机打挂）。
    """
    import struct  # 局部导入：打包通道号与三个系数
    limit = float(_cfg("PID_VALUE_LIMIT", 327.67))  # 系数上限，缺省 327.67

    def q(v):  # 内部量化函数：先把系数压进合法范围
        return max(-limit, min(limit, float(v))) * 100.0  # 双向钳位后再放大 100 倍

    return build_frame(FUNC_SET_PID, struct.pack(
        "<Bhhh", int(ch) & 0xFF, int(round(q(kp))), int(round(q(ki))), int(round(q(kd)))))  # 通道号 1B + 三系数各 int16LE


def frame_target_depth(meters):  # 构造 0x0A 目标深度帧
    """0x0A 目标深度（米 → uint16LE ×100 cm，钳位 0~2.00m）"""
    import struct  # 局部导入：打包 uint16 深度值
    cm = max(0.0, min(2.00, float(meters))) * 100.0  # 米钳位到 0~2.00m 并换算为厘米
    return build_frame(FUNC_DEPTH, struct.pack("<H", int(round(cm * 100))))  # 再 ×100 后打包为 uint16LE
def hex_str(data):  # 二进制转十六进制字符串，便于日志打印
    return " ".join("%02X" % b for b in bytes(data))  # 逐字节格式化为两位大写十六进制并以空格分隔


# ==================== 收帧 ====================
class FrameParser:  # CD/LEN/DC 定界状态机，负责从字节流里切出整帧
    """CD/LEN/DC 定界状态机 + 失步重同步；返回 [(func, data), ...]"""

    def __init__(self, max_frame=MAX_FRAME):  # 初始化解析器，可自定义最大帧长
        self.max_frame = max_frame  # 允许的最大整帧长度，用于长度合法性判断
        self._buf = bytearray()  # 累积未解析完的字节缓冲区
        self.bad_tail = 0  # 帧尾校验失败计数
        self.bad_len = 0  # 帧长非法计数
        self.resync_bytes = 0  # 重同步过程中被丢弃的字节数

    def feed(self, chunk):  # 喂入一段新数据，返回解出的帧列表
        out = []  # 本次解出的 (func, data) 列表
        if chunk:  # 有新数据才追加，避免空 bytes 触发无意义处理
            self._buf += chunk  # 并入尾部缓冲区，跨 TCP/串口分包也能拼帧
        while self._buf:  # 缓冲区还有数据就继续尝试解帧
            if self._buf[0] != HEAD:  # 首字节不是帧头 CD，说明已失步
                idx = self._buf.find(HEAD)  # 向后搜索最近的帧头位置
                if idx < 0:  # 整段都没有帧头
                    self.resync_bytes += len(self._buf)  # 计入丢弃统计
                    self._buf.clear()  # 整段丢弃，等待下一个帧头到来
                    break  # 无帧头可解，退出本轮
                self.resync_bytes += idx  # 记录跳过的垃圾字节数
                del self._buf[:idx]  # 丢弃帧头之前的无效数据，完成重同步
            if len(self._buf) < 2:  # 连 LEN 字节都还没收全
                break  # 等下一批数据补齐全后再解
            ln = self._buf[1]  # 取 LEN 字段（=DATA 长度 + 3）
            total = ln + 1  # 整帧长度 = LEN + 1（含 LEN 字节本身）
            if ln < 3 or total > self.max_frame:  # LEN 过小或有符号溢出般超长，判为非法
                self.bad_len += 1  # 累计长度异常次数
                del self._buf[0]  # 只丢首字节，可能是伪帧头需继续扫描
                continue  # 重新从下一字节尝试定界
            if len(self._buf) < total:  # 帧体尚未收全
                break  # 半帧等待，避免误判
            if self._buf[total - 1] != TAIL:  # 帧尾不是 DC，定界失败
                self.bad_tail += 1  # 累计帧尾错误次数
                del self._buf[0]  # 丢掉首字节重新找帧头
                continue  # 继续下一轮匹配
            func = self._buf[2]  # 提取 FUNC 功能码
            data = bytes(self._buf[3:total - 1])  # 提取 DATA 段（去掉头、LEN、FUNC 与帧尾）
            del self._buf[:total]  # 从缓冲区移除已消费的完整帧
            out.append((func, data))  # 收集结果给上层分发
        return out  # 返回本次解析出的全部帧


def parse_telemetry(data):  # 解析 0x0C 遥测回帧的 DATA 段
    """0x0C 遥测 DATA(44B) → dict；字段偏移见《V2_固件字段表_T0.1》§2"""
    import struct  # 局部导入：按小端解包遥测字段
    if len(data) < 44:  # DATA 必须满 44B，短帧说明不完整
        return None  # 返回 None 交上层忽略该包
    (tp, ty, tr, ap, ay, ar,  # 解包前三轴目标角：Pitch/Yaw/Roll
     gp, gy, gr, ax, ay2, az) = struct.unpack_from("<12h", data, 0)  # 偏移 0 起连续解 12 个 int16：目标/实际/陀螺/加速度各三轴
    motors = list(struct.unpack_from("<8h", data, 24))  # 偏移 24 起解 8 路电机输出值
    t_depth, a_depth = struct.unpack_from("<HH", data, 40)  # 偏移 40 起解目标深度与实际深度（uint16）
    return {
        "target_pitch": tp / 100.0, "target_yaw": ty / 100.0, "target_roll": tr / 100.0,  # 目标姿态角还原为浮点度
        "actual_pitch": ap / 100.0, "actual_yaw": ay / 100.0, "actual_roll": ar / 100.0,  # 实际姿态角还原
        "gyro_pitch": gp / 100.0, "gyro_yaw": gy / 100.0, "gyro_roll": gr / 100.0,  # 三轴角速度还原
        "acc_x": ax / 100.0, "acc_y": ay2 / 100.0, "acc_z": az / 100.0,  # 三轴加速度还原
        "motors": motors,  # 8 路电机原始输出，不做缩放
        "target_depth_cm": t_depth / 100.0, "actual_depth_cm": a_depth / 100.0,  # 深度还原为厘米浮点值
    }


# ==================== 链路 ====================
class Stm32Link:  # 下位机串口链路：负责开关串口、下发帧、轮询收帧
    """下位机链路。

    port_spec: "off" = 完全不启用；"sim" = 空跑（只打印将发送的帧，不开串口）；
               其它 = 串口设备路径（如 /dev/ttyCH9344USB5）
    """

    def __init__(self, cfg, log, port_spec="sim", on_telemetry=None):  # 构造链路对象，默认为空跑模式
        self.cfg = cfg  # 配置视图（波特率、轮询频率等）
        self.log = log  # 日志函数
        self.port_spec = port_spec  # 串口来源标识：设备路径 / sim / off
        self.on_telemetry = on_telemetry  # 遥测回调，供编排层注入
        self.parser = FrameParser()  # 独立收帧解析器
        self.ser = None  # 串口对象，打开成功后赋值
        self.enabled = (port_spec != "off")  # off 时整条链路禁用，不收不发
        self.sim = (port_spec == "sim")  # sim 时只打印不下串口
        self.stats = {"tx_frames": 0, "tx_errors": 0, "rx_frames": 0, "bad_tail": 0, "bad_len": 0}
        self._write_lock = threading.RLock()  # PID、运动、模式和兜底轮询共用整帧写锁。
        self._stop = threading.Event()  # 轮询线程的停止信号
        self._thread = None  # 轮询线程句柄
        self.last_req_ts = 0.0      # 最近一次发出 0x0C 请求的时间（模式层/兜底轮询共享）
        # [2026-10-04] IDLE 待命期抑制"兜底轮询"：编排层进/出 IDLE 时置位/清位。
        #   置 True 时 _poll_loop 不补发 0x0C（下位机完全安静，$TEL 用占位值）。
        self.poll_suspended = False  # True = 暂停兜底 0x0C 补发（IDLE 待命期）

    def open(self):  # 打开下位机链路（sim/off 时只做提示）
        if not self.enabled:  # 命令行指定 off，链路整体禁用
            self.log("[STM32] 已禁用（--stm32 off）")  # 提示未启用下位机
            return  # 直接返回，不打开串口
        if self.sim:  # 空跑模式
            self.log("[STM32] 空跑模式(sim)：不打开串口，只打印将下发的 V2 帧")  # 说明当前为协议仿真
            return  # 不建串口直接返回
        try:  # 尝试真正打开串口，失败则降级为空跑
            import serial  # 局部导入 pyserial，避免无硬件环境启动失败
            self.ser = serial.Serial(self.port_spec, self.cfg.STM32_BAUD, timeout=0.05)  # 按配置波特率打开串口，读超时 50ms
            self.log("[STM32] 已打开 %s @ %d" % (self.port_spec, self.cfg.STM32_BAUD))  # 打印已打开的设备与波特率
        except Exception as e:  # 串口不存在或被占
            self.log("[STM32] 打开失败: %s（转为空跑）" % e)  # 记录失败原因
            self.sim = True  # 降级为 sim，保证上层链路仍可跑协议状态机

    def start(self):  # 启动轮询收帧线程
        if not self.enabled or self.sim:  # 禁用或空跑时没有真实串口可读
            return  # 无需起线程
        self._thread = threading.Thread(target=self._poll_loop, name="Stm32Poll", daemon=True)  # 建守护线程跑兜底轮询
        self._thread.start()  # 启动该线程

    def stop(self):  # 停止轮询并关闭串口
        self._stop.set()  # 置位停止事件，让轮询循环退出
        try:  # 关闭串口可能抛异常（已关闭/设备拔出）
            if self.ser:  # 串口确实存在才关
                self.ser.close()  # 释放串口资源
        except Exception:  # 忽略关闭异常，退出流程不应被打断
            pass  # 静默处理

    def send(self, frame, note=""):  # 下发一帧到下位机
        """整帧互斥写入；完整写入/显式sim返回True，禁用/写失败返回False。"""
        if not self.enabled or not frame:  # 链路禁用或空帧则直接忽略
            return False
        frame = bytes(frame)
        with self._write_lock:
            if self.sim:
                self.log("[STM32·sim] TX(%s) %s" % (note, hex_str(frame)))
                self.stats["tx_frames"] += 1
                return True
            try:
                if self.ser is None:
                    raise IOError("串口尚未打开")
                written = self.ser.write(frame)
                if written != len(frame):
                    raise IOError("整帧写入不完整：%s/%d字节" % (written, len(frame)))
                self.stats["tx_frames"] += 1
                return True
            except Exception as e:
                self.stats["tx_errors"] += 1
                self.log("[STM32] 写失败(%s): %s" % (note, e))
                return False

    def send_telemetry_request(self, note="0x0C 遥测请求"):  # 发 0x0C 请求并记录时间戳
        """发一帧 0x0C 遥测请求（按协议，下位机收到即回一帧）。同时记时间戳供兜底轮询去重。"""
        sent = self.send(frame_telemetry_request(), note)
        if sent:
            self.last_req_ts = time.time()
        return sent
    def _poll_loop(self):  # 轮询线程主体：补发遥测请求并收帧分发
        """链路层兜底轮询：只在"最近没有模式层按节拍请求过"时才补发 0x0C。

        ROV 模式活跃时由 mode_rov.tick() 按同一节拍请求（mode_base.pump_telemetry），
        这里检测到刚请求过就跳过，避免重复（协议 1:1，重复会翻倍遥测率）；
        AUV 模式、以及急停锁存期间(tick 暂停) 则由本循环保底，保证 $TEL 不断。
        """
        period = 1.0 / max(0.5, self.cfg.STM32_POLL_HZ)  # 轮询周期（秒），频率下限 0.5Hz 防除零
        while not self._stop.is_set():  # 未收到停止信号就一直循环
            t0 = time.time()  # 记录本轮开始时间，用于扣减耗时后的精确睡眠
            try:  # 收发过程中任何异常都要吞掉，避免线程猝死
                # [2026-10-04 恢复] /userdata/To32 恢复 0x0C 遥测请求兜底补发
                #   （GrandRDK/src/to32 那份仍保持暂停）
                # [2026-10-04 IDLE] 待命期 poll_suspended=True：不补发 0x0C，下位机保持安静。
                if not self.poll_suspended and (time.time() - self.last_req_ts) >= period:  # 未暂停且已过周期
                    self.send_telemetry_request("0x0C 遥测请求(兜底)")
                chunk = self.ser.read(512)  # 读最多 512B（timeout=0.05 会阻塞到超时）
                if chunk:  # 读到数据才解析
                    for func, data in self.parser.feed(chunk):  # 交给状态机切帧
                        self.stats["rx_frames"] += 1  # 统计上行帧数
                        if func == FUNC_TELEMETRY and self.on_telemetry:  # 只把 0x0C 遥测帧交给回调
                            tel = parse_telemetry(data)  # 解析 44B 遥测数据
                            if tel:  # 解析成功才上报
                                self.on_telemetry(tel)  # 推送给编排层（最终转 $TEL 给上位机）
                        else:  # 非遥测帧或没有回调
                            self.log("[STM32] RX func=0x%02X data=%s" % (func, hex_str(data)))  # 打印未知/未处理的功能码
            except Exception as e:  # 串口异常（掉线、读失败）
                self.log("[STM32] 轮询异常: %s" % e)  # 记录异常详情
                self._stop.wait(0.5)  # 退避 0.5s，避免异常时刷屏式报错
            # 量测差值再睡：read(timeout=0.05) 会吃掉节拍，固定 sleep(period)
            # 会把兜底轮询拖到 ~6.7Hz；按剩余时间等待可稳定 ~POLL_HZ(10Hz)
            remain = period - (time.time() - t0)  # 计算本轮剩余应睡眠时间，抵消收发耗时
            if remain > 0:  # 还没超出周期才需要补睡
                self._stop.wait(remain)  # 用事件等待而非 sleep，收到停止信号可立即退出

    def snapshot(self):  # 返回链路运行快照，供状态汇报
        s = dict(self.stats)  # 复制收发统计，避免外部改动内部计数器
        s["parser_bad_tail"] = self.parser.bad_tail  # 附上解析器帧尾错误数
        s["parser_bad_len"] = self.parser.bad_len  # 附上解析器长度错误数
        s["port"] = ("sim" if self.sim else self.port_spec)  # 标注当前实际使用的串口来源
        return s  # 返回快照字典
