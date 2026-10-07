# ROV 控制站（上位机）

> Windows PyQt5 控制站 · 版本线 **v3.8（已并入 v3.6 板端融合值上屏）** · 最近更新 2026-10-07（PID 调参、参考二进制 PID、板端融合值）
> 配套：中位机 RDK S100（网线直连 `192.168.127.10`）+ STM32。
> 现成 EXE：**`ROV_ControlStation_v3.8.exe`**；源码运行 `python pc_main2.py`。目标深度目前为本地设定，发送接口待接入。
> 部署与合并说明见 **[文档/v3.8修改与部署说明.md](文档/v3.8修改与部署说明.md)**；v3.6 修改说明为历史文档，见 **[修改说明与操作方法](修改说明与操作方法.md)**。
> 历史改动记录见 **`CHANGELOG.md`**。

## 目录

| 章 | 内容 | 什么时候翻 |
|---|---|---|
| [一](#一三分钟上手) | 三分钟上手 | 第一次跑 / 忘了怎么开 |
| [二](#二文件与职责) | 文件与职责 | 不知道该改哪个文件 |
| [三](#三通讯协议速查) | 通讯协议速查 | 加字段 / 对帧 / 连不上 |
| [四](#四无线lora模式) | 无线 LoRa 模式 | 走无线链路时 |
| [五](#五板端-s100-与-hostlink-网关) | 板端 S100 与 hostlink | 板子上的进程怎么开 |
| [六](#六维护约定) | 维护约定 | 改 IP / 打包 EXE / 备份回滚 |
| [七](#七已知问题与待办) | 已知问题与待办 | 排障 / 接手开发 |

---

## 一、三分钟上手

### 1.1 PC 端

**方式 A：直接跑现成 EXE**（不用装 Python）

```
双击 ROV_ControlStation_v3.8.exe
```

**方式 B：源码运行**（改代码 / 调试用）

```bash
python -m pip install -r requirements_pc.txt
python pc_main2.py
```

> EXE 与源码功能一致，但 **EXE 是打包那一刻的快照**——改了 `pc_main2.py` / `protocol.py` 后必须重新打包（见 §六.4），否则 EXE 里跑的还是旧逻辑。

### 1.2 板端（按场景选一条，都在板子上敲）

```bash
cd /userdata/hostlink
python3 hostlink.py --no-alt --no-vid          # A 只要虚拟遥测，先验链路
python3 hostlink.py --channels B               # B 联调台：遥测 + 高度计
python3 hostlink.py --vid --channels B         # C 再加视频（需先跑 vp5A/run.sh）
python3 hostlink.py --channels B --serial /dev/ttyACM0   # D 实船：下行转发 STM32
```

停止：前台 `Ctrl+C`；后台 `pkill -f hostlink.py`。

> ⚠️ **上表的 hostlink 是 v3.3 时期的旧板端架构。现在现役是用 `GrandRDK/run.sh`**（它拉起
> `src/to32/main.py` 作为中位机）。用新架构时板端不需要 hostlink。见 §五。

### 1.3 界面速览（6 个选项卡）

| 页 | 内容 |
|---|---|
| **主界面** | 摇杆/推进器条形图、模式与链路切换、急停、LED、抓抛球、录像存储、快捷操作 |
| **可视窗口** | 视频墙 + 布局预设 + 三路帧率 + 位置/方向估算图 |
| **终端** | 事件流分级着色、原始 `$TEL`/`$CMD` 帧、5s 链路摘要、TX/RX/RTT |
| **函数图** | 每自由度目标/实际叠加曲线 + 顶部数据块多选（**曲线只在这一页**） |
| **全部** | 姿态/运动参数（含 **Kalman 融合深度**、**板端融合深度/离底净空**）+ 电池/环境 + 高度计 CH348 + 手柄快捷键 |
| **PID 调参** | 七项控制 PID（俯仰/偏航/横摇/深度 → STM32 编号 0~3；过门 → S100；撞球/捡球未接入），参考二进制帧下发，逐项确认后发送 |

可视窗口要点：画面严格 4:3 等比不变形；CAM3 默认待机，点「CAM3 取流」才拉流，且与 CAM2 同进同退。

**航向罗盘（v3.5）**：CAM1（主画面）**右上角**有一个圆形方向指示器，零度朝上、顺时针为正，
角度取 `$TEL` 的 `yaw` 字段。支持绝对/相对两种模式：

| 操作 | 效果 |
|---|---|
| 默认 | **绝对模式**（圆内显示蓝色 `ABS 绝对`），指针直接指板端 `yaw` 真值 |
| **单击盘面** | 把**此刻航向记为 0°**，转**相对模式**（橙色 `REL 相对`），指针立即归零朝上 |
| 相对模式下再单击 | 重置基准（指针仍朝上），用于连续「重新对齐」 |
| **双击 / 右键** | 切回绝对模式（跟随真值） |

圆内下方显示当前角度值（相对模式下即相对偏转量）。底盘半透明、指针橙红偏亮，不遮画面。

**录像画质（v3.5）**：录像已从 OpenCV 的 `XVID` 请求（**实际静默降级为低码率 FMP4，PSNR≈40dB**）
改为**自编码 MJPEG-AVI**（`cv2.imencode` 控质量 + 手写 AVI 容器），画质 **+约 5dB** 且质量可调。
三个顶部常量（`pc_main2.py`）：

| 常量 | 默认 | 说明 |
|---|---|---|
| `REC_FPS` | `30` | 录像帧率（原写死 15） |
| `REC_JPEG_QUALITY` | `90` | JPEG 质量；**90 是性价比最优点**（q80 更省体积、q95 更清晰但体积翻倍） |
| `REC_MAX_FILE_GB` | `1.0` | 单文件上限；达上限自动分段 `rec_camN_<时间>_p2.avi` |

实测（640×480）：q90 ≈ 23.9KB/帧 · 30fps ≈ 5.9Mbps · 1 小时 ≈ 2.6GB。嫌大可把
`REC_JPEG_QUALITY` 调到 80（≈3.8Mbps）。**仅改上位机，中位机回传不动**。

### 1.3.1 工作模式（v3.4）

主界面「快捷操作」区按**工作模式 / 遥控交还 / 安全 / 作业**四组排列：

| 按钮 | 含义 | 发什么 | S100 侧动作 |
|---|---|---|---|
| **有线遥控** | 经 S100 中继的手动遥控 | `$CMD,...,0,...` | 转 V2 `0x04 0x06`（ROV_TETHERED），杆位转发下位机 |
| **自主航行** | S100 自主跑任务 | `$CMD,...,1,...` | 转 V2 `0x04 0x05`（AUV），起任务状态机，杆位被忽略 |
| **交还自主** | 退出遥控、把控制权交回 S100 | `$CMD,...,1,...` | 同上（语义更明确的别名） |
| **急停** | 停推 + 进 STANDBY 并锁存 | 零杆位 `$CMD` + `$ESTOP#` | 转 V2 `0x04 0x01` |
| **温启动** | 从 STANDBY 恢复到急停前的模式 | `$ESTOP,0#` | 转 V2 `0x04 0x00`（START） |

模式标签：`有线遥控`(绿) / `自主航行`(蓝) / `待机`(红，急停后)。手柄 **LB** 在有线遥控 ↔ 自主航行之间切。

> **为什么没有"测试模式"按钮**：`$CMD.mode` 在 S100 侧**只认 0 和 1**（`to32_config.MODE_ROV=0` / `MODE_AUV=1`），
> 发别的值会被 `switch_mode` 拒绝。且《多源控制协议设计.md》§12.3 明令水下不得发送测试油门。
> 待 S100 侧将来支持后再补。

**急停后按模式按钮（v3.8 方案A）**：急停锁存中按「有线遥控/自主航行/交还自主」视为主动接管——
PC 会先补发 `$ESTOP,0#` 解除 S100 锁存（S100 下发 `CD 04 04 00 DC` START 恢复急停前模式），
再切到目标模式。此前仅清 UI 待机标志、S100 锁存不解除（0x09 一直抑制），会造成"PC 显示已切回、
板端仍锁存"的不一致。若只想恢复到急停前模式，仍可用「温启动」。

### 1.3.2 Kalman 融合深度（v3.3.3）

「全部」页 → 姿态/运动参数 →「深度 / 高度」组新增 **Kalman深度** 一行，是上位机**本地计算**的值（不是遥测原始字段）。

| 项 | 说明 |
|---|---|
| 状态量 | `[深度, 垂向速度]`（纯 Python 2×2 KF，无额外依赖） |
| 预测 | 匀速模型：`depth += vz·dt`，速度随机游走 |
| 观测① | `$TEL` 的 `depth`（压力深度，绝对值可靠但噪声大） |
| 观测② | `$TEL` 的 `vz`（垂向速度，提供变化率、减小滞后） |
| 观测③ | 高度计换算深度 `H − alt`，**默认关闭**（见下） |
| 断流处理 | 遥测中断 > 2 s 自动重置，不用陈旧状态预测 |

调参在 `pc_main2.py` 文件头的 `KF_*` 常量：`KF_Q_*` 越大越信观测（跟得紧但抖），`KF_R_*` 越小越信对应传感器。

> **启用高度计观测前先确认单位**：`$TEL` 里 `alt` 与 `depth` 是否同单位（m）。若 `alt` 是 mm，把 `KF_ALT_SCALE` 改成 `0.001`；确认后再把 `KF_USE_ALT` 改成 `True`。水深 `H = depth + alt` 由前 20 帧中位数定初值、之后慢速跟踪。

实测（20 Hz、噪声 ±0.15 m）：融合值 RMS 误差 0.041 vs 原始 0.144（噪声降至 ~28%），界面显示平滑度提升约 4.5 倍；5 m 阶跃约 1 s 内收敛。

### 1.4 手柄速查

```
左摇杆 前后 .. 前进/后退    右摇杆 前后 .. 上浮/下潜
左摇杆 左右 .. 左右横移     右摇杆 左右 .. 左转/右转
A/B ......... 抓球/抛球     LB ......... 有线遥控/自主航行 切换
X/Y ......... 录像/存储     DPad ↑↓←→ .. LED1 / LED2
```

RT/LT 扳机与 RB **未分配**。完整轴/键映射见 [3.2](#32-上行指令-cmd)。

### 1.5 排查速查

| 现象 | 原因 | 怎么办 |
|---|---|---|
| 顶部「未连接」红 | PING 都不通 | 板端 `ip -br a` 查网口；PC `ping 192.168.127.10` |
| 「板在线 (无遥测)」黄 | PING 通、无 `$TEL` | 板端没跑 hostlink / `telem_sender` |
| 高度计一直是 `--` | 口位不对或没接 | `--channels B` 指定实际口；看日志有无 `[ALT][B]` |
| 没画面 | 没跑 `run.sh`，或 hostlink 没加 `--vid` | 按 1.2 场景 C 顺序启动 |
| CAM3 十秒才出图 | `:5000` 无服务时 OpenCV 打开 URL 全局串行 | 正常现象，等一会 |
| 板端 `[PONG] :8081 已被占用` | 老 `telem_sender` 还在 | `pkill -f telem_sender` 再启动 |
| 板端 `[DOWN] 绑定 :8080 失败` | 端口被占 | `ss -ulnp \| grep 8080` 查占用并停掉 |
| Kalman深度 一直 `--` | 还没收到过 `depth` 遥测 | 先确认 Depth 行在动；滤波器首帧用原始深度初始化 |
| 融合深度比实际滞后 | 过程噪声偏小 / `vz` 不准 | 调大 `KF_Q_DEPTH`，或调小 `KF_R_DEPTH`（见 1.3.1） |
| 启动即退出 | 缺 pygame 等依赖 | `pip install -r requirements_pc.txt` |

---

## 二、文件与职责

| 文件 | 运行环境 | 说明 |
|---|---|---|
| `pc_main2.py` | Windows PC | GUI 主程序（手柄、遥测、视频墙、深度目标、高度计、终端页） |
| `protocol.py` | PC / 板端共用 | 协议定义：端口、IP、LoRa 参数、`$CMD/$TEL/$VID/$ESTOP/$ALT/$FLW` 编解码 + v3.7 参考二进制 PID（`build_pid`/`parse_pid`/`ReferenceTelemetryBuffer`） |
| `pid_panel.py` | Windows PC | 「PID 调参」页：七项控制面板（确认后发送、参数文件保存/加载） |
| `task_pid_wire.py` | PC / S100 共用 | 过门任务 PID 协议 `$TASKPID` / `$TASKPID_ACK` 构建与解析 |
| `x5_server.py` | RDK S100 | 旧版板端服务（Flask MJPEG + UDP 中继 + 串口转发）；现役用 `S100/vp/v5A/run.sh` |
| `stm32_rov.c` | STM32 | UART 帧解析 + 12 路电机混控（v3.2 起无需改动） |
| `requirements_pc.txt` | PC | PyQt5 / pygame / opencv-python / numpy / pyserial |
| `requirements_x5.txt` | 板端 | Flask / pyserial / opencv-python |
| `ROV_ControlStation_v3.8.exe` | Windows PC | 当前打包产物。**改了源码必须重打**，可运行 `build_exe.ps1` |
| `navigation.py` | Windows PC | 位置积分与两种方向视图 |
| `build_v34/verify_v34_ui.py` | Windows PC | v3.4 UI 自检（offscreen + mock socket 抓帧），本机 31/31 |
| `build_v34/verify_compass.py` | Windows PC | v3.5 罗盘控件自检（角度换算/绝对相对/鼠标交互/绘制），本机 26/26 |
| `build_v34/verify_compass_live.py` | Windows PC | v3.5 罗盘端到端自检（模拟 `$TEL` → `_on_tel` → 罗盘），本机 10/10 |
| `build_v34/verify_record_quality.py` | Windows PC | v3.5 录像画质/容器/分段/端到端自检，本机 18/18 |
| `build_v34/mjpeg_avi_writer.py` | Windows PC | MJPEG-AVI 容器独立模块（已内联进 `pc_main2.py`，此文件保留作参考） |
| `build_v34/verify_exe_version.py` | Windows PC | 从 EXE 里反查 `pc_main2` 字节码，核验打包版本与文案（可传 EXE 路径） |
| `recordings/` | PC | 录像 `rec_cam*.avi`、遥测 CSV、`pid_params.json` |
| `CHANGELOG.md` | — | 历次版本改动记录 |

---

## 三、通讯协议速查

### 3.1 端口总表

| 用途 | 方向 | 链路 | 帧格式 / 内容 | PC 端 | S100 端 |
|---|---|---|---|---|---|
| 遥控指令 | PC → S100 | UDP `:8080` | `$CMD,surge,sway,heave,yaw,led1,led2,mode,grab,store#` | `JoystickThread` 20Hz | hostlink / `x5_server.cmd_relay` → 串口转 STM32 |
| PID 参数（v3.7 参考二进制） | PC → S100 / 直连串口 | UDP `:8080` 或 STM32 串口 | `CD 0A 01 idx P_L P_H I_L I_H D_L D_H DC`（11B，P/I/D ×100 int16 小端） | PID 调参页（姿态/深度） | UDP 中位机原样透传二进制；或串口直连 STM32 |
| 过门任务 PID | PC → S100 | UDP `:8080` | `$TASKPID,<req>,gate,p,i,d#` → `$TASKPID_ACK,<req>,gate,OK/ERR,...#` | PID 调参页（过门） | S100 处理并回 ACK（2s 超时） |
| 视频开关 | PC → S100 | UDP `:8080` | `$VID,1#` / `$VID,0#` | `_set_video_stream()` | 板端拦截，不透传 STM32 |
| 急停 / 解除 | PC → S100 | UDP `:8080` | `$ESTOP#` / `$ESTOP,0#` | 「急停」「解除急停」按钮 | 板端拦截，进/退 STANDBY |
| 遥测 | S100 → PC | UDP `:8081` | `$TEL,` 41 字段 | `TelThread` 绑 `0.0.0.0:8081` | `telem_sender.py` 10Hz |
| 延迟 RTT | 双向 | UDP `:8081` | `PING` → `PONG` | `_measure_latency()` | `ping_responder` |
| 视频 CAM1/2 | S100 → PC | HTTP `:5000` | MJPEG `/cam1` `/cam2`（**带检测框**） | `VideoThread`（**直连 `192.168.127.10:5000`，不经 Nginx**） | `src/web_server.py`（读 `/dev/shm` 已画框 JPEG） |
| 第三路画面 | S100 → PC | HTTP `:8084` | MJPEG `/stream`（CAM3） | `VideoThread`(cam3) | `/userdata/show_cam.py` |
| 高度计 | S100 → PC | UDP `:8082` | `$ALT,<通道>,<mm>,<状态>#` | `AltThread` | `read_altimeter.py`（CH348 A–E） |
| 光流测速 | S100 → PC | UDP `:8083` | `$FLW,<m/s>,<ratio>,<tracked>,<状态>#` | 仅 v3.2 目录版本有 | `flow_speed.py` |
| **AUV 状态提示** | S100 → PC | UDP `:8085` | `$MSG,<ts>,<level>,<code>,<stage>,<text>#` | **`MsgThread`**（v3.5，终端页染色显示） | `auv_task/notify.py` |

### 3.2 上行指令 `$CMD`

帧：`$CMD,<surge>,<sway>,<heave>,<yaw>,<led1>,<led2>,<mode>,<grab>,<store>#\r\n`
节奏：有线 UDP `192.168.127.10:8080` @ **20Hz**；无线 LoRa 串口 9600 @ **5Hz**（约 37~39 B/帧）。

| # | 字段 | 取值 | 手柄 / 界面来源 |
|---|---|---|---|
| 1 | `surge` | −1.00~1.00 | 左摇杆 前后 `axis1`，前推为正（前进） |
| 2 | `sway` | −1.00~1.00 | 左摇杆 左右 `axis0`，右推为正（右移） |
| 3 | `heave` | −1.00~1.00 | 右摇杆 前后 `axis3`，前推为正（上浮） |
| 4 | `yaw` | −1.00~1.00 | 右摇杆 左右 `axis2`，右推为正（右转） |
| 5 | `led1` | 0~100 | D-pad 上/下，每帧 ±5；界面滑块可直接设 |
| 6 | `led2` | 0~100 | D-pad 左/右，每帧 ±5；界面滑块可直接设 |
| 7 | `mode` | 0=有线遥控 / 1=自主航行 | LB 键或界面模式按钮 |
| 8 | `grab` | 0/1/2 | A=1 抓球、B=2 抛球（**脉冲**，500ms 回 0） |
| 9 | `store` | 0/1 | Y 键或「存储」按钮（切换态），与本地 CSV 同步 |

- 死区：`abs(v) < 0.08` → 0，否则 `round(v, 2)`，满杆 ±1.00。
- 手柄未插入也按链路频率发零杆位 `$CMD`（`led/mode/grab/store` 照常携带）；拔出瞬间发急停。
- 自动急停：手柄拔出、链路切换、程序退出 → 发 `$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#`。
- **模式语义（v3.4）**：`mode=0` = 有线遥控（S100 转 V2 `0x04 0x06`），`mode=1` = 自主航行（S100 转 V2 `0x04 0x05`）。
  **`mode` 只发 0 或 1** —— S100 侧只注册了这两个模式（`to32_config.MODE_ROV=0` / `MODE_AUV=1`），
  发其他值会被 `switch_mode` 拒绝并计 `bad_mode`。详见 §3.5。
- **自主航行模式**：`surge/sway/heave/yaw` 强制 `0.00`，上位机只发模式信号，运动由 S100 自主完成；
  `led/grab/store/$VID` 不受影响。切回 `mode=0` 立即恢复。
  （实证：S100 的 `mode_auv.on_cmd` 本就忽略杆位，此处归零是为了报文干净 + 未来兼容。）
- X 键录像为纯本地动作，不发帧。

典型报文（37~39 B）：

| 操作 | 报文 |
|---|---|
| 静止 | `$CMD,0.00,0.00,0.00,0.00,0,0,0,0,0#` |
| 左摇杆前推到底 | `$CMD,1.00,0.00,0.00,0.00,0,0,0,0,0#` |
| 右摇杆上推到底 | `$CMD,0.00,0.00,1.00,0.00,0,0,0,0,0#` |
| 按 A 抓球 | `$CMD,0.00,0.00,0.00,0.00,0,0,0,1,0#` |
| 切 AUV | `$CMD,0.00,0.00,0.00,0.00,0,0,1,0,0#` |
| LED1=60 / LED2=80 / AUV | `$CMD,0.00,0.00,0.00,0.00,60,80,1,0,0#` |

### 3.3 参考二进制 PID 参数帧（v3.7，旧 `$PID` 文本已移除）

帧：`CD 0A 01 <PID_INDEX> P_L P_H I_L I_H D_L D_H DC`，共 **11 字节**；P/I/D ×100 转 `int16` 小端，
编号 0~7（`PID_REFERENCE_MAX_INDEX=7`）。构建/解析见 `protocol.build_pid()` / `parse_pid()`。
- 姿态/深度（俯仰 0 / 偏航 1 / 横摇 2 / 深度 3）经「PID 调参」页下发：可走 **S100 UDP 透传**
  （中位机必须原样透传二进制）或 **STM32 串口直连**；面板逐项确认后才发送，无参数回读。
- 过门航向参数走 `$TASKPID,<req>,gate,p,i,d#`（文本），S100 处理并回
  `$TASKPID_ACK,<req>,gate,OK,p,i,d#` 或 `ERR,<错误码>`；2s 超时判失败，参数回读不一致拒绝。
- 旧 `$PID,ch,p,i,d#` 文本帧（0~11 通道速度环）**已移除**，两版协议不可混用。

### 3.4 其它帧

| 帧 | 格式 | 说明 |
|---|---|---|
| `$TEL` | 41 字段 | 10 实际 + 10 目标 + 5 电池/温度 + 12 推进器 + `ax,ay,az,alt`；推进器在 index 25~36 |
| `$ALT` | `$ALT,<通道>,<mm>,<状态>#` | 状态 `OK / EMPTY / ERR / DOWN`，8082 |
| `$VID` | `$VID,1#` / `$VID,0#` | 板端拦截，停/启采集推流省 CPU 与带宽 |
| `$ESTOP` | `$ESTOP#` / `$ESTOP,0#` | 显式急停 / 温启动；S100 拦截转 V2 `0x04 0x01` / `0x04 0x00`，与「全零 `$CMD`」语义不同 |
| `$FLW` | `$FLW,<m/s>,<ratio>,<tracked>,<状态>#` | 光流，8083，当前仅 v3.2 目录版本含 |
| `$MSG` | `$MSG,<ts>,<level>,<code>,<stage>,<text>#` | **v3.5 新增**，8085（与 `$AUV` 共用端口，靠帧头区分）。AUV 在跑哪一段 / 降级 / 超时 / 中止原因 → 终端页按 `INFO/WARN/ERROR` 染色显示。解析见 `protocol.parse_msg()` |

#### `$MSG` 字段（v3.5）

| 字段 | 取值 |
|---|---|
| `ts` | `HH:MM:SS`（板端发送时刻） |
| `level` | `INFO`（白）/ `WARN`（黄）/ `ERROR`（红） |
| `code` | `STAGE` 阶段切换 · `ABORT` 中止上浮 · `TIMEOUT` 超时 · `SKIP` 跳过 · `DEGRADE` 降级 · `BUDGET` 预算用尽 · `TEST` 测试模式 · `KILLED` 被终止 · `PC_TAKEBACK` 上位机接管 · `MODE` 模式进出 · `SERVO` 舵机 · `VISION` 视觉 · `HB` 心跳 |
| `stage` | 当前 AUV 阶段名（`DIVE` / `SEEK_BALL_F` / `PASS_GATE_1` / `SURFACE` ...），无则 `-` |
| `text` | 中文一句话；内部逗号已转 `;`、`#` 已剔除、超 160 字截断 |

⚠ 解析时**按 5 段取**（去掉帧头 `$MSG,` 与帧尾 `#` 之后是 ts/level/code/stage/text）；
按 6 取会永远返回 None。⚠ 无缆验收时板端学不到上位机 IP → **收不到帧属正常**。

### 3.5 模式映射（v3.4 更新）

**链路**：上位机 `$CMD.mode` → S100 `mode_dispatcher.switch_mode()` → 中位机 `on_enter` 下发 V2 模式帧 → STM32。

| `$CMD.mode` | 上位机按钮 | S100 内部模式 | S100 发出的 V2 帧 | 说明 |
|---|---|---|---|---|
| 0 | 有线遥控 | `MODE_ROV` | `0x04 0x06`（ROV_TETHERED） | S100 中继有线遥控，杆位经 `0x09` 转发 |
| 1 | 自主航行 / 交还自主 | `MODE_AUV` | `0x04 0x05`（AUV） | S100 跑任务状态机，**忽略杆位** |

> **`$CMD.mode` 只有 0/1 两个合法值**。S100 的 `to32_config.py` 只注册了 `MODE_ROV=0` / `MODE_AUV=1`；
> `switch_mode()` 对未注册值会记 `stats["bad_mode"]` 并忽略（`mode_dispatcher.py:212-215`）。
> 上位机**不需要**知道 V2 码，那是 S100 在 `on_enter` 里映射的。

**急停 / 温启动**（不走 `mode` 字段，走独立的 `$ESTOP` 帧）：

| 帧 | 上位机按钮 | S100 动作 | V2 帧 |
|---|---|---|---|
| `$ESTOP#` | 急停 | `trigger_estop()` 锁存并抑制 `0x09` | `0x04 0x01` |
| `$ESTOP,0#` | 温启动 | `release_estop()` 恢复 Last_Con_Mode | `0x04 0x00`（START） |

调试可用板端 `/userdata/cmd_watch.py` 双向抓帧核对。

### 3.6 连接状态判定

- 绿 `已连接 (板+STM32)`：1.5s 内收到 `$TEL`；黄 `板在线 (无遥测)`：PING 通但无遥测；红 `未连接`：PING 也不通。
- 无线模式只显示 `LoRa: COMx 已连接`（纯下行，对端状态无法远程确认）。
- 注意：板端空跑假遥测也会显示绿色。

---

## 四、无线（LoRa）模式

1. 地面 LoRa USB 接 PC（透传模式 9600）；机载 LoRa 串口接 STM32（TX↔RX、共地），STM32 固件无需改动。
2. 上位机顶部「刷新串口」→ 选 COM → 点「链路」切无线。
3. 显示 `LoRa: COMx 已连接` 即可遥控；**遥测 / 视频在无线下不可用**；无回传时位置积分暂停。
4. 参数在 `protocol.py`：`LORA_BAUD = 9600`、`LORA_SEND_HZ = 5`（20Hz 会超空速）。
5. 切换链路瞬间自动发急停。

---

## 五、板端 S100 与 hostlink 网关

> ⚠️ **本节描述的是 v3.3 时期的旧板端架构（`vp5A/` + `hostlink` + `telem_sender.py` + `mjpeg_bridge.py`），现役板端工程 `/userdata/GrandRDK` 已完全取代它，本节保留仅作历史对照。**
> **现役对应关系（2026-10-02 核对）**：
> - 图像+遥测入口 = `GrandRDK/run.sh`（拉起 `front.py` / `bottom.py` / `web_server.py` / `to32/main.py` / `read_altimeter.py` / `flow_speed.py` / `show_cam.py` / Nginx）。
> - 视频桥 `mjpeg_bridge.py` → **`src/web_server.py`**（读 `/dev/shm` 已画检测框 JPEG，`:5000` `/cam1` `/cam2`）。
> - `telem_sender.py` / `hostlink` → **`src/to32/`**（中位机，UDP 8080/8081 + 串口 V2 到 STM32）。
> 见 §3.1 端口总表与 `GrandRDK/README.md`。

### 5.1 板端进程

- `S100/vp/v5A/run.sh` 是图像与遥测的唯一入口：一次拉起 `front.py`(CPU 0-2) / `bottom.py`(CPU 3-5) / `mjpeg_bridge.py` / `telem_sender.py`，`trap` 统一回收。
- `read_altimeter.py`、`hostlink` 为独立进程，需另行启动。
- `/userdata/To32`（现役中位机）把 `$CMD.mode` 映射为 V2 命令；`/userdata/To32_1` 为旧中转站。

### 5.2 hostlink 是什么

把原来散装的遥测、高度计、视频桥、下行指令、延迟应答**合成一个大管家**：只记一个命令、一套配置、一份日志。它**不**代替 vp5A 的采集与识别，只负责"把数据送上网线、把指令接回来"。

```
Windows 上位机 (192.168.127.100)
        │  ▲                ▲                ▲
        │  │ UDP 8081       │ UDP 8082       │ HTTP 5000 (/cam1 /cam2)
        ▼  │                │                │
   ┌─────────────────────────────────────────────────┐
   │                hostlink 网关 (板端)               │
   │   tel源  $TEL   ← 复用 vp5A 的参数生成器          │
   │   alt源  $ALT   ← 串口读 CH348 高度计             │
   │   vid守护      ← 拉起/看护 mjpeg_bridge(:5000)   │
   │   down 监听 :8080 收 $CMD/$PID/$VID             │
   │   pong 监听 :8081 回 PING→PONG                  │
   └─────────────────────────────────────────────────┘
        │（下行）串口转发：$CMD/$PID → STM32(实船才有)
```

| 文件 | 作用 | 关心程度 |
|---|---|---|
| `hostlink.py` | 主程序入口 | 是 |
| `config.py` | 全部默认参数（IP/端口/开关） | 偶尔改默认值 |
| `sources.py` | 各数据源实现 | 加新数据才需要 |
| `frames.py` | 协议帧小工具 | 不用管 |

### 5.3 参数表（按需查阅）

| 参数 | 默认 | 作用 |
|---|---|---|
| `--pc 192.168.127.100` | 同左 | 上位机 IP |
| `--tel-port 8081` | 8081 | 遥测上行端口 |
| `--alt-port 8082` | 8082 | 高度计上行端口 |
| `--channels B` | A,B,C,D,E | 读 CH348 哪些口 |
| `--alt-interval 0.2` | 0.2 | 高度计读取间隔（5Hz） |
| `--tel-hz 10` | 10 | 遥测频率 |
| `--vid` | 关 | 让 hostlink 管理视频桥 `:5000` |
| `--serial /dev/ttyACM0` | none | `$CMD`/`$PID` 转发串口；`none` = 只记录 |
| `--no-tel` / `--no-alt` / `--no-down` / `--no-pong` | — | 关掉对应源 |
| `--stats 10` | 10 | 每 10s 打印一次统计 |

### 5.4 怎么确认在正常工作

板端每 10s 一行统计：`[12:09:35] 统计 | TX ['18081:30', '18082:5'] | 错误 0 | 下行 2`
→ `TX` 数字在涨 = 正常；`错误 0` = 无发送失败；`下行 2` = 收到过 2 条指令。

上位机侧：主界面/「全部」页数值在滚动且延迟几十毫秒内；高度计 B 行显示「正常 / 5xx mm」；「可视窗口」页有画面。

### 5.5 加一种新数据（示例：水温计）

1. `sources.py` 加一个类（仿 `TelSource`，`link.send(字节, 端口)` 是统一出口）：

```python
class TempSource(Source):
    def __init__(self, port):
        super().__init__("temp", port)
    def run(self, stop, link):
        while not stop.is_set():
            link.send(b"$TEMP,1,25.3#\r\n", self.port)
            stop.wait(1.0)
```

2. `hostlink.py` 的「上行源」段加一行：`if not args.no_temp: sources.append(TempSource(8090))`
3. **上位机 PC 端同步加 `$TEMP` 解析**（协议两边必须一致）。

### 5.6 新旧对照

| 以前 | 现在 |
|---|---|
| `python3 vp5A/telem_sender.py --dst ...` | `python3 hostlink.py`（遥测默认开） |
| `python3 S100/USART/v1/read_altimeter.py` | 同上（`--channels` 选口） |
| `python3 vp5A/mjpeg_bridge.py` | `python3 hostlink.py --vid` |
| （`$VID` 无人处理） | hostlink 监听 8080 处理 `$VID,1/0#` |
| （`$CMD` 无人接收） | hostlink 收 8080，可转发 STM32（`--serial`） |

老脚本**都保留在原处**，需要时仍可单独用。

> **路径口径（以本文为准）**：图像+遥测 `S100/vp/v5A`、高度计 `S100/USART/v1/read_altimeter.py`、网关 `/userdata/hostlink`。旧的 `/userdata/vp/vp5.0`、`/userdata/USART/read_altimeter.py` 写法不再采用。

---

## 六、维护约定

1. **改 IP 只改一处**：`protocol.py` 的 `X5_IP`（默认 `192.168.127.10`，板端 eth1 网线直连），覆盖视频取流、遥测/PING、指令下行全链路。
2. **源码运行**：`python pc_main2.py`（用 `D:\Anaconda\python.exe`，托管 Python 3.13.12 没装 PyQt5/cv2/pygame）。**使用本包 v3.6 EXE**——走旧 IP/旧协议。
3. **依赖**：`python -m pip install -r requirements_pc.txt`（缺 pygame 会启动即退出）。
4. **打包 EXE**：当前版本运行 `powershell -ExecutionPolicy Bypass -File build_exe.ps1`，使用已安装的 Python/PyInstaller，输出 `ROV_ControlStation_v3.8.exe`（含 `pid_panel.py`/`task_pid_wire.py`）。修改 `pc_main2.py`、`navigation.py`、`protocol.py`、`pid_panel.py` 或 `task_pid_wire.py` 后均须重打。旧版本构建记录保存在 `CHANGELOG.md`。
5. **回滚**：改动前留 `.bak_*` 备份；`X5_IP` 改回 `192.168.5.11` 即回到经典 X5 拓扑。
6. 录像依赖视频流，视频传输关闭时录像不可用。
7. 有线「已连接」以 1.5s 内收到遥测为准。

---

## 七、已知问题与待办

| # | 事项 | 状态 |
|---|---|---|
| 1 | `v3.2_20260903/protocol.py` 光流段重复：`FLOW_PORT = 8083` 注释块与定义连续出现两次 | 待删重复段 |
| 2 | 光流（`$FLW` / `FlowThread` / 「全部」页光流区）未并入 v3.3，只在 v3.2 目录版本 | 待定合并方向 |
| 3 | PC 界面按钮点击、真实手柄轴序、CAM3 第三路画面未实机验证 | 需连板复核 |
| 4 | STM32 的 `pid_p/pid_i/pid_d` 已能接收，但控制回路仍是「油门直接映射 PWM」，**PID 未接入闭环** | 固件侧待接 |
| 5 | 参考二进制 PID 无回读确认，界面提示"已提交"≠ 固件已生效 | 面板已标注"未回读确认参数" |
| 14 | 参考二进制 PID 真实控制环生效未实机验证 | 需连板复核 |
| 15 | 过门参数依赖 S100 同版本补丁（`$TASKPID` 处理与 ACK）；撞球/捡球算法未接入 | 等 S100 侧 |
| 6 | STM32 `ROV_UART_RxHandler` 收到 `#` 后 `\r\n` 仍追加进 `rx_buf`，`parse_command()` 判尾 `rx_buf[rx_idx-1] != '#'` 可能整帧丢弃（`$CMD` 同样受影响） | 待实机确认，建议把 `\r`/`\n` 也作结束符 |
| 7 | **v3.4 UI 改动未实机验证**（按钮点击、模式标签、待机态） | 需连板复核；本机已过 `build_v34/verify_v34_ui.py` 31/31 |
| 8 | EXE 已重打为 `ROV_ControlStation_v3.5.exe`（旧 v3.4.1 已删） | ✅ 已解决（2026-10-04） |
| 9 | 「测试模式」按钮缺失（《多源控制协议设计.md》§12.4 要求有） | **等 S100 侧支持**：`$CMD.mode` 目前只认 0/1，加按钮也无落点 |
| 10 | 8 路电机目标转速无显示（§12.4 要求有） | **另立项**：该数据在 V2 `0x0C` 应答里，S100 的 `tel_builder` 未映射进 `$TEL`，需改 S100 侧代码 |
| 11 | 「全部」页 12 路推进器显示 vs V2 的 8 路电机布局**不一致**（历史遗留，非 v3.4 引入） | 待定；两套硬件布局不同 |
| 12 | **回传画面清晰度受源头限制**：相机 MJPG 640×480 + JPEG q80（`main_config.py` 的 `WEB_MJPEG_QUALITY`），web_server 只转发不重编码 | v3.4.1 已加「1:1 原始尺寸」开关 + 锐利放大 + 帧率 60→30；**根治需改板端**（q80→92、分辨率升级），见 `CHANGELOG.md` v3.4.1 |
| 13 | **航向罗盘未实机验证**（`yaw` 来自 `$TEL`，本机无真实遥测） | 需连板复核；本机已过 `build_v34/verify_compass.py` 26/26 + `verify_compass_live.py` 10/10 |

> 已解决的历史项（旧网段兜底、EXE 体积异常等）见 `CHANGELOG.md`。
