# 改动记录（CHANGELOG）

> 从原 `README.md` 第九章剥离（2026-09-21）。日常使用看 `README.md`，查历史改这看这里。
> 当前版本线 **v3.8**（2026-10-07 已并入 v3.6 板端融合值上屏与文档）。

---

## v3.8 合并 v3.6 板端融合值（2026-10-07）

- 在 v3.8（参考二进制 PID 七项调参）基础上并入 v3.6 的板端融合值上屏：
  - `protocol.py`：`parse_telemetry` 恢复 `$TEL` 帧尾 `fused_depth`(41)/`clearance`(42) 解析（纯增量、短帧缺省），保留 v3.7 参考二进制 PID。
  - `pc_main2.py`：「全部」页"深度/高度"组恢复 板端融合深度/离底净空 两行 + `act_map` + docstring 条目22。
- 旧 `$PID` 文本协议**不恢复**（v3.7 起为二进制 0x01 帧，两版不可混用）。
- 恢复 v3.6 文档（README/CHANGELOG/修改说明/目标深度接入说明/验证结果/文件校验）与板端模板
  （`x5_server.py`、`stm32_rov.c`、`requirements_x5.txt`）及 `build_v34/` 回归脚本。
- 新增 `文档/v3.8修改与部署说明.md` 与 `tests/test_v39.py`（协议级回归，替代缺失的 test_v36/test_v38）。
- 重新打包 `ROV_ControlStation_v3.8.exe`（含 `pid_panel.py`/`task_pid_wire.py`）。

---

## v3.8 急停后按模式按钮解除锁存（2026-10-07，方案A）

- PC `_set_mode`：急停待机态(`_frozen`)按 ROV/AUV/交还自主 时，先发 `$ESTOP,0#` 解除
  S100 急停锁存（S100 下发 `CD 04 04 00 DC` START 恢复 Last_Con_Mode），再切目标模式。
- 原因：S100 的 `estop_latch` 不会因 `$CMD` 自动清除（默认 `ESTOP_LATCH=True`），锁存期间
  0x09 一律抑制——旧行为"按 ROV 只清 UI 待机标志"导致 PC 显示已切回、板端仍锁存、
  固件停在 STANDBY。方案A 将"点模式按钮=主动接管"落实为真正解除板端锁存。
- S100 与 STM32 固件零改动；无线模式 `send_estop_release` 内部跳过（LoRa 不认 `$ESTOP`）。
- 更正 v3.4 过时注释（"S100 侧切模式与 STANDBY 互斥"与实际不符）。
- 重新打包 `ROV_ControlStation_v3.8.exe`。

---

## v3.6 定位浮层外观调整（2026-10-05）

- 改为直角矩形：背景透明度 80%，轨迹不透明度 70%，方向箭头不透明度 80%。
- 定位浮层贴齐视频画面左上角；切换布局、缩放窗口后由视频卡片布局保持对齐。
- 摄像头名称移到该卡片左下角，避免与透明定位浮层的标题叠字。
- 同步更新源码、EXE、截图、两个交付包和校验文件。

---

## v3.6 跨页模式、深度目标界面、叠加曲线和位置估算（2026-10-05）

- ROV/AUV 模式按钮移至常驻顶部状态栏，保持现有模式帧与手柄 LB 同步。
- 移除 PC 的 PID 面板与 PID 下发入口；共享旧协议保留兼容定义。
- 新增目标深度输入/本地设定/回传值显示，发送默认禁用，提供 `set_depth_sender` 接入点和通讯说明。
- 每个自由度叠加实际实线、回传目标虚线；深度另标本地设定；CSV 追加目标与位置列。
- 右摇杆 yaw 映射取反，显示和两条链路使用同一映射。
- 可视窗口新增与罗盘等大的位置图：当前位置居中，支持当前方向朝前/固定初始方向、零点重设、缩放和轨迹导出。
- vx/vy 速度积分估算，无有效速度不推算位移，超过两秒的遥测缺口不跨段积分；原模板速度为零时轨迹保持原点。
- 增加本地回归 `tests/test_v36.py`，模拟数据截图和 `build_exe.ps1` 构建脚本。实机通讯/运动尚未验证。

---

## v3.5 AUV 状态提示回传 `$MSG`（2026-10-05）

### 背景

水池测试时人在岸上，板子上只有日志。AUV 自主跑起来之后，**上位机终端看不到它在干嘛**：
现在跑到哪一段、是不是降级了、为什么超时/中止，全靠捞板端日志。

同时板端（`/userdata/GrandRDK/src/to32/move_test/auv_task/`）定了两条新规矩：
① AUV 只能由上位机 `$CMD` 切入（上电不自己跑）；② 上位机切回 ROV 时立即终止自主任务。

### 改动（只加不改，老协议零影响）

| 文件 | 改动 |
|---|---|
| `protocol.py` | 新增 `MSG_PORT = 8085`；新增 `parse_msg()`（解 `$MSG`）、`parse_auv()`（认 `$AUV`，暂不显示） |
| `pc_main2.py` | 新增 `MsgThread`（UDP 8085 收帧，`$MSG`/`$AUV` 按帧头分发）；新增 `_on_msg()` 槽 → 终端页按级别染色 |

帧格式（板端 `auv_task/notify.py` 发）：

```
$MSG,<ts>,<level>,<code>,<stage>,<text>#\r\n
  level  INFO / WARN / ERROR   ->  终端染色 白 / 黄 / 红
  code   STAGE / ABORT / TIMEOUT / SKIP / DEGRADE / BUDGET / TEST / KILLED /
         PC_TAKEBACK / MODE / SERVO / VISION / HB
  stage  当前 AUV 阶段名（DIVE / SEEK_BALL_F / PASS_GATE_1 / SURFACE ...）
  text   中文一句话（内部逗号已转 ';'、'#' 已剔除）
```

终端页显示效果：

```
[11:05:12] [INFO] [AUV][STAGE] SEEK_BALL_F: 进入 SEEK_BALL_F（前视找球）
[11:05:30] [WARN] [AUV][DEGRADE] SIT_BOTTOM: 深度源不可用; 定深走定时放行（判据只剩时间）
[11:06:02] [ERR ] [AUV][ABORT] SURFACE: 中止: 找不到球 -> 强制上浮
[11:06:30] [WARN] [AUV][KILLED] RAM_BALL: 自主任务已终止: PC_TAKEBACK（停止下发; 交还遥控）
```

### 坑（踩过一次）

`parse_msg` 里**不能按 6 段取**：去掉帧头 `$MSG,`（5 字符）和帧尾 `#` 之后只剩 **5 段**
（ts / level / code / stage / text）。写 `len(parts) < 6` 会永远返回 None，
表现是"终端一条提示都没有"，但板端明明在发 —— 排查时会白白怀疑网络。

### 验证

本机回环已验（板端 `AuvMsg` 真发 UDP → 上位机 `parse_msg` 解析，中文正常）：

```bash
python C:\Users\lenovo\WorkBuddy\RC\_scratch\verify_msg_link.py
# -> OK —— 板端帧格式与上位机解析一致
```

### 产物（2026-10-05 已重新打包）

```
ROV_ControlStation_v3.5.exe                    97,592,651 字节（93.06 MiB）  ← 含 $MSG
ROV_ControlStation_v3.5.exe.bak_premsg_*       97,590,744 字节             ← 打包前旧版（不含 $MSG）
```

打包（纯 ASCII 构建目录，避免 Qt 插件路径被中文破坏）：

```bash
pyinstaller --noconfirm --clean --onefile --windowed \
  --name ROV_ControlStation_v3.5 --paths=. \
  --hidden-import=protocol --hidden-import=serial.tools.list_ports --hidden-import=numpy \
  --exclude-module matplotlib --exclude-module scipy --exclude-module pandas \
  --exclude-module tkinter --exclude-module IPython --exclude-module notebook \
  pc_main2.py
```

静态核验（**每次重打包后必做**，确认打进去的不是旧二进制）：

```bash
python D:\RC\ROV控制站_v3.3\build_v34\verify_exe_version.py
# 期望："v3.5 AUV 状态提示 $MSG 回传核验" 11 项全部"存在"
```

⚠ 核验脚本的坑：`MsgThread` / `_on_msg` / `_msg_thread` 这类**类名/方法名/属性名
只出现在 `co_names`**，不在 `co_consts`。脚本原先只遍历 `co_consts`，会误报"缺失"。
已补收 `co_names` / `co_varnames` / `co_freevars`。
（另有一份不依赖 PyInstaller 的解析版：`build_v34/verify_exe_msg.py`，只用标准库
`zlib`+`struct` 解 CArchive，可在无打包环境时核验。）

### 配套（板端，非本次改动）

- 无缆验收时板端学不到上位机 IP → **一帧都不发**，不是故障；
- 端口 8085 与 `$AUV` 数值快照共用，靠帧头区分，老解析不受影响。

## v3.5 录像画质提升（2026-10-04）

### 背景与根因

用户反馈：**上位机保存的录像画质很差**。约束：**只改上位机，不动中位机回传代码**。

根因（本机实测定位）：

| 现象 | 实测证据 |
|---|---|
| 录像代码请求 `XVID` | `cv2.VideoWriter(..., "XVID")` |
| FFMPEG 后端**不支持 XVID** | `isOpened()` 仍返回 True（**静默降级**），实际编码器回落成 **FMP4**（mpeg4 默认参数） |
| 画质低 | 对照原始帧 PSNR 仅 **≈40.2dB**（帧内压缩后重建） |
| `VideoWriter` 无法设质量 | 无 quality/bitrate 参数，`XVID`/`MJPG`/`mp4v` 都不生效 |
| 帧率被写死 | 原为 **15fps**，低于回传实时帧率 |

补充实测（OpenCV 4.x FFMPEG 后端编码器矩阵）：

| fourcc | 结果 |
|---|---|
| `XVID` | 静默降级 **FMP4**（~40.5dB） |
| `mp4v` | 同样降级 FMP4 |
| `MJPG` | 真的是 MJPG，但 **质量写死 q≈75**（不可调） |
| `FFV1` | 无损，但体积爆炸（~100KB/帧） |
| `H264`/`avc1` | 打不开（缺 openh264 dll） |

### 方案 A（选定）

**自己 `cv2.imencode('.jpg', quality)` 编码 + 手写 MJPEG-AVI 容器** —— 因为 OpenCV 的
MJPG writer 质量不可调，只能自己控制 JPEG 质量。视频仍是帧内压缩（MJPG），体积可控、可调。

#### 新增 `MjpegAviWriter` 类（`pc_main2.py` §2.4，仅依赖 `struct`）

手写标准 MJPEG-AVI：RIFF/AVI → `LIST hdrl`（`avih` + `LIST strl`[`strh` + `strf`]）
→ `LIST movi`（`00dc` 帧块）→ `idx1` 索引。`close()` 回填 movi/avih/strh/RIFF 各 size 字段。

| 方法 | 说明 |
|---|---|
| `__init__(path, width, height, fps)` | 写头部占位，`_movi_list_pos` 记位置 |
| `append(jpeg_bytes)` | 追加一帧（`00dc` + size + data + padding），记 `(offset, size)` |
| `close()` | 写 `idx1` 索引 + 回填四类 size/frame 字段，返回体积守卫 dict |
| `frame_count` | 帧数属性 |

单 RIFF 块上限 ~1GB（未实现 OpenDML），超限由 `MjpegRecorder` 分段处理。

#### 新增 `MjpegRecorder` 类（§2.5，接口对齐 `cv2.VideoWriter`）

| 接口 | 说明 |
|---|---|
| `__init__(dir_path, stem, width, height, fps=REC_FPS, quality=REC_JPEG_QUALITY, max_bytes=None)` | `max_bytes` 默认 `REC_MAX_FILE_GB*(1024**3)` |
| `isOpened()` | 返回 `self._ok` |
| `write(frame)` | `cv2.imencode('.jpg', frame, [IMWRITE_JPEG_QUALITY, q])` → `append`；尺寸不符拒绝；达上限**先切段** |
| `release()` | 关闭 |
| `fname` / `bytes_written` | 属性：当前段路径 / 已写字节 |
| 自动分段 | 段名 `rec_camN_<stem>.avi` → 超限 → `rec_camN_<stem>_p2.avi` … |

#### 顶部常量

```python
REC_FPS = 30                 # 录像帧率（原写死 15 → 改 30）
REC_JPEG_QUALITY = 90        # JPEG 质量（可调；90 是性价比最优点）
REC_MAX_FILE_GB = 1.0        # 单文件上限（GB），达上限自动分段
```

### 改动清单（`pc_main2.py`）

| # | 位置 | 改动 |
|---|------|------|
| 1 | import | 补 `struct` |
| 2 | 顶部常量 | 新增 `REC_FPS=30` / `REC_JPEG_QUALITY=90` / `REC_MAX_FILE_GB=1.0` |
| 3 | 新增 `MjpegAviWriter` | §2.4，约 110 行（已从 `build_v34/mjpeg_avi_writer.py` 内联） |
| 4 | 新增 `MjpegRecorder` | §2.5，约 90 行 |
| 5 | `_show_frame` 录像段 | `cv2.VideoWriter(*"XVID")` → `MjpegRecorder(...)`；fps 15 → `REC_FPS`；写入门票日志含质量/帧率；分段时同步 fname + 日志 |
| 6 | `_toggle_record` | 释放循环改用 `writer.fname`/`bytes_written` 出日志 |
| 7 | 文件头 docstring | 新增第 21 条功能说明 |

### 画质/体积实测（640×480，真实水下帧）

| JPEG 质量 | 每帧体积 | PSNR |
|---|---|---|
| q80 | 15.3 KB | 44.66 dB |
| **q90** | **23.9 KB** | **46.07 dB** |
| q95 | 37.4 KB | 46.94 dB |
| q100 | 92.3 KB | 49.13 dB |

q90 为性价比最优点：30fps ≈ 5.9 Mbps，1 小时 ≈ 2.6 GB。

### 验证

- **`build_v34/verify_record_quality.py` 18/18 通过** ——
  ① 容器正确性（OpenCV 可开/fps=30/640×480/fourcc=MJPG/帧数精确）；
  ② idx1 索引随机寻址 6 点全 >35dB；
  ③ **画质对比：新 q90 = 45.33dB vs 旧 XVID→FMP4 = 40.24dB（+5.09dB）**；
  ④ 单文件上限自动分段（各段可开、不超限）；
  ⑤ 端到端（走 `MainWindow._show_frame`）：writer 创建/类型/停录释放/帧数/fps=30 全对。

### 已知取舍

- 体积比原来大（原 FMP4 约 1.33Mbps → 现约 5.9Mbps），换取 +5dB 画质；嫌大可调
  `REC_JPEG_QUALITY`（q80 约 3.8Mbps）。
- 仍为**帧内压缩**（MJPG），不做帧间预测。同码率下画质优于 FMP4 的默认参数。
- 1GB 单文件上限未实现 OpenDML，超限靠分段规避。

### 备份

`pc_main2.py.bak_20261004_185500_pre_v35_record`（罗盘后的基线，3175 行；本次 +约 265 行 → 3440 行）。

---

## v3.5 画面罗盘（2026-10-04）

### 背景

需求：在可视窗口加一个圆形方向指示器（罗盘），指示旋转角度。角度可读传感器绝对值，
也可切相对值 —— **点击盘面把"点击那一刻的航向"设为 0°**（因为鼠标点在盘面上任意位置
都等价于"以此刻航向为零点"，所以实现为单击盘面即归零，不区分点击坐标）。

用户给定口径（附图确认）：放在**最大画面（CAM1）的右上角**；底盘**半透明、低不透明度**；
指针**稍高**（比圆心略长）、**橙红色偏亮**；**上为 0°**；大小 **168px**（约下方日志框高度的 1.2 倍）。

### 新增控件 `HeadingCompass`（`pc_main2.py`）

纯 `QPainter` 自绘，无外部资源。零度朝上、顺时针为正（与 `yaw` 定义一致）。

| 特性 | 说明 |
|---|---|
| 数据源 | `$TEL` 的 `yaw` 字段（度），在 `_on_tel` 里 `self.compass.set_heading(t["yaw"])` |
| 绝对模式 | 指针直接指 `yaw` 真值；模式标记显示蓝色 `ABS 绝对` |
| 相对模式 | 指针指 `yaw - base`；模式标记显示橙色 `REL 相对` |
| 单击盘面 | 把当前 `yaw` 记为基准 `base` → 转相对模式，指针立即归零（朝上） |
| 相对下再单击 | 重置基准（仍朝上），便于连续"重新对齐" |
| 双击 / 右键 | 切回绝对模式（跟随真值） |
| 视觉 | 半透明圆底盘 `rgba(10,14,20,110)` + 外圈 + 每 15° 刻度（每 90° 加长）+ N/E/S/W；指针主体 `#ff6a3d`、箭头亮部 `#ff9e6e`、尾部对向短线；圆心帽 + 读数小底衬（角度值 + 模式） |

### 改动清单（`pc_main2.py`）

| # | 位置 | 改动 |
|---|------|------|
| 1 | 文件头 docstring | 新增第 20 条功能说明；版本号 v3.4 → v3.5 |
| 2 | L55-65 import | 补 `QSize/QPoint/QPointF/QRectF`、`QPainterPath/QPolygonF/QFontMetrics`、`math` |
| 3 | 新增 `HeadingCompass` 类 | 约 200 行，放在 `VideoTile` 之前（§4.4） |
| 4 | `VideoTile.__init__` | 新增 `self.compass = None` |
| 5 | `VideoTile.add_compass(size)` | 新增方法：`QGridLayout` 同格叠加 `AlignTop\|AlignRight` 贴右上角，返回罗盘 |
| 6 | `VideoTile.compass_widget()` | 新增取回方法 |
| 7 | `_build_view_page` | 创建 `self.compass = self.cam_tiles["cam1"].add_compass(168)`（**只给 CAM1**） |
| 8 | `_on_tel` | 新增 `if "yaw" in t and self.compass: self.compass.set_heading(t["yaw"])` |
| 9 | 窗口标题 | `ROV控制站 v3.4.1` → `ROV控制站 v3.5` |

### 验证

- **`build_v34/verify_compass.py` 26/26 通过** —— 控件创建/尺寸、绝对模式角度换算（含负数与 >360 取模）、
  相对模式基准归零与差值、鼠标交互（单击/双击/右键）、真实绘制非空、挂载到 VideoTile 右上角、
  鼠标事件不被穿透（角标仍穿透不受影响）。
- **`build_v34/verify_compass_live.py` 10/10 通过** —— 端到端：模拟 `$TEL` 帧 → `_on_tel` 喂 yaw → 罗盘更新；
  相对模式差值正确；缺 `yaw` 的帧不报错。
- **`build_v34/verify_v34_ui.py` 31/31 无回归**（按钮文案/分组/三态模式/协议帧）。

### 已知取舍

- **offscreen 平台无字体库**（`QFontDatabase().families()` 为空）⇒ 无头渲染时文字不出。
  已实测**真实平台**（357 个字族，含 Consolas）文字正常；代码里 `_mk_font()` 显式指定
  `Consolas` + `TypeWriter` 兜底、并用**像素尺寸**（非点数，避免 DPI 放大导致排布失控）。
- 指针朝左下/右下时会与读数底衬有轻微交叠 —— 底衬半透明，实测可读，未再调整。

### 备份

`pc_main2.py.bak_20261004_173149_pre_v35_compass`（**v3.4.1 基线**，2994 行；本次 +181 行 → 3175 行）。

---

## v3.5 EXE 重新打包（2026-10-04）

产物（项目根目录）：**`ROV_ControlStation_v3.5.exe`**（onefile、无控制台窗口，
**93.06 MiB** / **97,584,391 B**）；旧 `ROV_ControlStation_v3.4.1.exe` 已删除，避免误用。

打包环境 `C:\Users\lenovo\rovpack\venv`
（Python 3.13.9 · PyQt5 5.15.11(Qt 5.15.2) · numpy 2.5.3 · opencv-python 4.14.0 ·
pygame 2.6.1 · pyserial 3.5 · pyinstaller 6.22.3），耗时约 **2 分 5 秒**。

```bat
:: 1) 建纯 ASCII 构建目录(项目目录含中文, PyInstaller 解析 Qt 插件路径会出 ??? 报错)
mkdir D:\rov_pkg_v35\src
copy "D:\RC\ROV控制站_v3.3\pc_main2.py" D:\rov_pkg_v35\src\
copy "D:\RC\ROV控制站_v3.3\protocol.py"  D:\rov_pkg_v35\src\
cd /d D:\rov_pkg_v35\src

:: 2) 打包；--distpath/--workpath 必须用 Windows 风格路径, 写 /d/xxx 会被解析成 D:\d\xxx
C:\Users\lenovo\rovpack\venv\Scripts\pyinstaller.exe --noconfirm --clean --onefile --windowed ^
  --name ROV_ControlStation_v3.5 --paths=. ^
  --hidden-import=protocol --hidden-import=serial.tools.list_ports --hidden-import=numpy ^
  --exclude-module matplotlib --exclude-module scipy --exclude-module pandas ^
  --exclude-module tkinter --exclude-module IPython --exclude-module notebook ^
  --exclude-module pytest ^
  --distpath D:\rov_pkg_v35\dist --workpath D:\rov_pkg_v35\build --specpath D:\rov_pkg_v35 pc_main2.py

:: 3) 拷回项目目录后即可删掉 D:\rov_pkg_v35
copy D:\rov_pkg_v35\dist\ROV_ControlStation_v3.5.exe "D:\RC\ROV控制站_v3.3\"
```

**静态核验**：`build_v34/verify_exe_version.py`（传参指定 EXE 路径），提取 `pc_main2`
的 marshal 字节码（**入口脚本存 CArchive，不在 PYZ 里**），常量池 790 条：

- v3.5 罗盘文案全部命中：`HeadingCompass` / `航向罗盘` / `ABS 绝对` / `REL 相对` / `零度朝上` / `ROV控制站 v3.5`
- v3.4 / v3.4.1 文案全部命中（无回归）
- 旧标题 `ROV控制站 v3.3` 与 `ROV控制站 v3.4.1` **均已移除**
- 体积与 v3.4.1 差 **+6,369 B**（97,584,391 vs 97,578,022），符合"只加约 181 行源码"的预期

**UI 断言复核**：`verify_compass.py` **26/26** + `verify_compass_live.py` **10/10** +
`verify_v34_ui.py` **31/31**，全通过无回归。

> ⚠️ **双击启动验证未做**：需用户本机双击确认。

### 二次重打（并入录像画质改动，同日）

录像画质改动（见「v3.5 录像画质提升」）合入后**再次重打**，产物覆盖同名 `ROV_ControlStation_v3.5.exe`：

| 项 | 值 |
|---|---|
| 体积 | **97,590,744 B**（仅罗盘版 97,584,391 B，**+6,353 B**） |
| SHA256 | `e49b9a220570d944bfa3170b185b73e786d371c5c98385144718bb9eba882a58` |
| 构建方式 | 源码复制到 **纯 ASCII 目录** `D:\rov_pkg_v35\src\` 后打包（避免中文路径让 Qt 插件目录变 `???`） |
| 构建 venv | `C:\Users\lenovo\rovpack\venv`（Python 3.13.9 · PyInstaller 6.22.3 · cv2 4.14.0） |
| 静态核验 | v3.5 罗盘文案 + 录像常量（`MjpegAviWriter`/`MjpegRecorder`/`REC_JPEG_QUALITY`/`REC_FPS`/`REC_MAX_FILE_GB`）全命中；旧标题已移除 |

> ⚠️ 打包 `.bat` 里 `set VAR=<中文路径>` 会被 cmd 编码拆散 → 改为直接在 ASCII 目录调用 `python -m PyInstaller`。

---

## v3.4.1 画面清晰度（2026-10-04）

### 背景

上位机看板端回传画面"糊"。排查后确认**不是回传故障**，是**源头分辨率 + JPEG 质量**决定的：
相机 MJPG **640×480** → 板端 `front.py`/`bottom.py` 画框后 `cv2.imencode(JPEG, quality=80)`
（`main_config.py` 的 `WEB_MJPEG_QUALITY`）→ 共享内存 → `web_server.py:5000` **原样转发**
（不重编码）→ 上位机 `VideoThread` 直连取流。所以**改 web_server 无用**，只能改源头。

源头改造（板端，另案）涉及 JPEG 质量 80→92 与分辨率升级，本版先做**上位机侧能立刻见效的两件事**。

### 改动（`pc_main2.py`，9 处）

| # | 位置 | 改动 |
|---|------|------|
| 1 | L77-88 | `VID_DISPLAY_FPS` **60 → 30**（渲染压力减半，给清晰度让路）；新增 `VID_DISPLAY_FPS_MIN/MAX`；新增 `VID_SCALE_SHARP = True` |
| 2-4 | `VideoTile` | 新增 `self._fw/_fh` 记录最近一帧**真实像素**；`set_res()` 同步维护 |
| 5 | `VideoWall.fit_tiles` | 1:1 模式**放开卡片高度上限**（`maximumHeight=16777215`），否则帧被卡片压扁 |
| 6 | `MainWindow.__init__` | 新增 `self._rawsize_1to1 = False` |
| 7 | 控制栏 | 新增 **「显示: 自适应缩放 / 1:1 原始尺寸」**切换按钮（checkable） |
| 8 | `MainWindow.__init__` | `self.view_wall = wall`（供切换时重排调用） |
| 9 | `_on_rawsize_toggled` + `_show_frame` | 1:1 模式按帧真实像素抬 `QLabel` 最小尺寸并**不缩放**直接上屏；自适应模式用 `Qt.FastTransformation`（锐利邻域放大，检测框/边缘更清楚）。`_show_frame` 里 `QImage(...).copy()` 加固防花屏 |

- **窗口标题**：`ROV控制站 v3.3` → `ROV控制站 v3.4.1`
- **未改** `protocol.py`（一行未动），**未改**任何协议/网络逻辑

### 验证

`build_v34/verify_v34_ui.py` **31/31 通过**（按钮文案/分组、三态模式标签、`$CMD.mode` 只可能 0/1、
急停零杆位 + `$ESTOP#`、温启动 `$ESTOP,0#`、`_frozen` 置位/清除）。

### 备份

`pc_main2.py.bak_20261004_164500_pre_v341_quality`（**反向还原生成的 v3.4 真基线**，2838 行；
diff 干净：2838 → 2919 行，+81 行全是本次改动）。

### 未决（板端另案）

- JPEG 质量 80 → 92（`WEB_MJPEG_QUALITY`）
- 分辨率查相机支持档位（`v4l2-ctl --list-formats-ext`）后评估升 960×540 / 1280×720
- 带宽实测：640×480 q80 单帧 ~14 KB × 60 fps ≈ 6.9 Mbps，千兆链路有余量，升分辨率需重算

---

## v3.4.1 EXE 重新打包（2026-10-04）

产物（项目根目录）：**`ROV_ControlStation_v3.4.1.exe`**（onefile、无控制台窗口，
**93.06 MiB** / **97,578,022 B**）；旧 `ROV_ControlStation_v3.4.0.exe` 已删除，避免误用。

打包环境沿用 `C:\Users\lenovo\rovpack\venv`
（Python 3.13.9 · PyQt5 5.15.11(Qt 5.15.2) · numpy 2.5.3 · opencv-python 4.14.0 ·
pygame 2.6.1 · pyserial 3.5 · pyinstaller 6.22.3），**不用 Anaconda**。耗时约 **2 分 4 秒**
（比 v3.4.0 的 3 分 31 秒快，构建缓存命中）。

```bat
:: 1) 建纯 ASCII 构建目录(项目目录含中文, PyInstaller 解析 Qt 插件路径会出 ??? 报错)
mkdir D:\rov_pkg_v41\src
copy "D:\RC\ROV控制站_v3.3\pc_main2.py" D:\rov_pkg_v41\src\
copy "D:\RC\ROV控制站_v3.3\protocol.py"  D:\rov_pkg_v41\src\
cd /d D:\rov_pkg_v41\src

:: 2) 打包；--distpath/--workpath 必须用 Windows 风格路径, 写 /d/xxx 会被解析成 D:\d\xxx
C:\Users\lenovo\rovpack\venv\Scripts\pyinstaller.exe --noconfirm --clean --onefile --windowed ^
  --name ROV_ControlStation_v3.4.1 --paths=. ^
  --hidden-import=protocol --hidden-import=serial.tools.list_ports --hidden-import=numpy ^
  --exclude-module matplotlib --exclude-module scipy --exclude-module pandas ^
  --exclude-module tkinter --exclude-module IPython --exclude-module notebook ^
  --exclude-module pytest ^
  --distpath D:\rov_pkg_v41\dist --workpath D:\rov_pkg_v41\build --specpath D:\rov_pkg_v41 pc_main2.py

:: 3) 拷回项目目录后即可删掉 D:\rov_pkg_v41
copy D:\rov_pkg_v41\dist\ROV_ControlStation_v3.4.1.exe "D:\RC\ROV控制站_v3.3\"
```

**静态核验（2026-10-04）**：`build_v34/verify_exe_version.py`（已支持传参指定 EXE 路径），
从 EXE 的 CArchive 里提取 `pc_main2` 的 marshal 字节码（**入口脚本存 CArchive，不在 PYZ 里**），
反查常量池 771 条：

- v3.4 文案全部命中：`有线遥控` / `自主航行` / `交还自主` / `温启动` / `模式: 待机` / `工作模式` / `遥控交还`
- v3.4.1 文案全部命中：`显示: 1:1 原始尺寸` / `显示: 自适应缩放` / `自适应缩放` / `ROV控制站 v3.4.1`
- 旧标题 `ROV控制站 v3.3` **已移除**；旧文案（`遥控模式`/`AUV模式`/`解除急停`）仅残留于 3 处
  **注释/docstring**（第 32 行版本说明、329/2439 行函数文档），**无一处是 UI 显示文本**
- 体积与 v3.4.0 差 **+1,679 B**（97,578,022 vs 97,576,343），符合"只动少量源码"的预期，无 MKL 误入

> ⚠️ **双击启动验证未做**：onefile 首次启动解压约 3~5 s，且会先起一个 ~8.5 MB 的 bootloader 父进程（无窗口），
> 请用户本机双击确认。

---

## v3.4.0 多模式 UI（2026-10-03）

### 背景

依据 `XLB-AUH10-2026.10.3/多源控制协议设计.md`（2026-10-03 定稿）§12.4「S100 侧 UI 上位机」。
S100 中位机侧已先行改造完成（`refreence/To32_new/`，77/77 + 109/109 通过），本版做 PC 上位机侧对齐。

### 关键实证（读了 To32_new 源码后确认，非推测）

1. **`$CMD.mode` 在 S100 侧只认 0 和 1** —— `to32_config.py` 只注册 `MODE_ROV=0` / `MODE_AUV=1`；
   `mode_dispatcher.switch_mode()` 对未注册值计 `bad_mode` 并忽略。
   → 因此**不加**"测试模式"按钮（发了也无落点），**不加**禁发拦截（没有非法值可发）。
2. **模式切换只看 `$CMD.mode` 字段值**，没有"谁发运动帧就切模式"的逻辑。
   → 原以为的"AUV 下上位机发 `$CMD` 会把模式拽走"风险**不存在**。
3. **`AuvMode.on_cmd` 对杆位只记录、全忽略**（`mode_auv.py:116-121`）。
   → 现有 AUV 归零逻辑并非安全必需，保留它是为了报文干净 + 未来兼容，注释已改。
4. **上位机不需要知道 V2 码** —— 映射由 S100 在 `on_enter` 里做（0→`0x06`，1→`0x05`）。

### 改动

**主界面「快捷操作」区按功能分组重排**（原来是一锅 2×3 网格）：

```
┌─ 工作模式 ─────────────┐
│ [有线遥控] [自主航行]    │  互斥高亮，带 tooltip 说明发的帧与 S100 动作
├─ 遥控交还 ─────────────┤
│ [交还自主]              │  新增；§12.4 推荐的"甲"方案(显式退出遥控)
├─ 安全 ─────────────────┤
│ [急停]     [温启动]      │  常驻可见(§12.4 第 6 条)
├─ 作业 ─────────────────┤
│ [抓球 [A]] [抛球 [B]]    │
└────────────────────────┘
```

**文案重命名**（底层协议零改动）：

| 原 | 新 | 底层 |
|---|---|---|
| 遥控模式 | **有线遥控** | 仍是 `_set_mode(0)` → `$CMD.mode=0` |
| AUV模式 | **自主航行** | 仍是 `_set_mode(1)` → `$CMD.mode=1` |
| 解除急停 | **温启动** | 仍是 `$ESTOP,0#` → V2 `0x04 0x00`（START） |

**新增「交还自主」按钮**：与「自主航行」同发 `$CMD.mode=1`，但文案语义是"退出遥控"，
降低操作手在紧急交还时按错的概率。

**模式显示升级**（`_paint_mode()`）：
- `模式: 有线遥控`（绿 `#c8e6c9`）
- `模式: 自主航行`（蓝 `#bbdefb`）
- `模式: 待机`（红 `#ffcdd2`）—— **新增**。S100 无模式回传通道，由 UI 用 `_frozen` 标志自记：
  点「急停」置位，点「温启动」清除，点模式按钮也清除。
- 待机时两个模式按钮都不高亮。

**其他**：
- 顶部 `mode_lbl` 初始文案 `模式: ROV` → `模式: 有线遥控`。
- 终端页 5s 链路摘要的模式名同步改为中文（含"待机"）。
- 手柄 LB 日志文案改为 `自主航行/有线遥控`（**逻辑不变**：只有两态，`1 - _mode` 已正确）。
- 「手柄快捷键映射」面板 LB 行文案更新。
- AUV 归零处注释改为实证版（逻辑不动）。

### 未做（有意）

| 项 | 原因 |
|---|---|
| 6 态模式枚举 / 禁发拦截 | S100 只认 mode 0/1，扩了没用 |
| 测试模式按钮（§12.4 要求） | S100 侧无实现路径 + §12.3 水下禁发测试油门 |
| 8 路电机转速显示（§12.4 要求） | 数据在 V2 `0x0C`，不在 `$TEL`；需改 S100 侧，另立项 |
| EXE 重新打包 | 待用户决定；在此之前请源码运行 |

### 验证

- `py_compile` 通过。
- 新增 `build_v34/verify_v34_ui.py`（本机跑，`QT_QPA_PLATFORM=offscreen`，mock socket 抓帧）：
  **31/31 通过** —— 覆盖按钮文案/分组、三态模式标签、`$CMD.mode` 取值只可能 0/1、
  急停发零杆位 `$CMD` + `$ESTOP#`、温启动发 `$ESTOP,0#`、`_frozen` 置位/清除。
- **未实机验证**：真实手柄轴序、连板后的 S100 联动仍需复核。

### 改动量

`pc_main2.py` 约 +140 / 改 ~45 行；**`protocol.py` 一行未改**。
备份：`pc_main2.py.bak_20261003_143015` 等四份。

### v3.4.0 EXE 重新打包（2026-10-03）

产物（项目根目录）：`ROV_ControlStation_v3.4.0.exe`（onefile、无控制台窗口，
**93.1 MB** / **97,576,343 B**）；旧 `ROV_ControlStation_v3.3.3.exe` 已删除，避免误用。

打包环境沿用 `C:\Users\lenovo\rovpack\venv`（Python 3.13.9 · PyQt5 5.15.11(Qt 5.15.2) ·
numpy 2.5.3 · opencv-python 4.14.0 · pygame 2.6.1 · pyserial 3.5 · pyinstaller 6.22.3），
**不用 Anaconda**（其 numpy 带 MKL，体积会顶到 ~260 MB）。耗时约 3 分 31 秒。

```bat
:: 1) 建纯 ASCII 构建目录(项目目录含中文, PyInstaller 解析 Qt 插件路径会出 ??? 报错)
mkdir D:\rov_pkg_v34\src
copy "D:\RC\ROV控制站_v3.3\pc_main2.py" D:\rov_pkg_v34\src\
copy "D:\RC\ROV控制站_v3.3\protocol.py" D:\rov_pkg_v34\src\
cd /d D:\rov_pkg_v34\src

:: 2) 打包；--distpath/--workpath 必须用 Windows 风格路径, 写 /d/xxx 会被解析成 D:\d\xxx
C:\Users\lenovo\rovpack\venv\Scripts\pyinstaller.exe --noconfirm --clean --onefile --windowed ^
  --name ROV_ControlStation_v3.4.0 --paths=. ^
  --hidden-import=protocol --hidden-import=serial.tools.list_ports --hidden-import=numpy ^
  --exclude-module matplotlib --exclude-module scipy --exclude-module pandas ^
  --exclude-module tkinter --exclude-module IPython --exclude-module notebook ^
  --distpath D:\rov_pkg_v34\dist --workpath D:\rov_pkg_v34\build --specpath D:\rov_pkg_v34 pc_main2.py

:: 3) 拷回项目目录后即可删掉 D:\rov_pkg_v34
copy D:\rov_pkg_v34\dist\ROV_ControlStation_v3.4.0.exe "D:\RC\ROV控制站_v3.3\"
```

**静态核验（2026-10-03）**：写了 `build_v34/verify_exe_version.py`，从 EXE 的 CArchive 里提取
`pc_main2` 的 marshal 字节码（**入口脚本存 CArchive，不在 PYZ 里**），反查常量池：

- 新文案全部命中：`有线遥控` / `自主航行` / `交还自主` / `温启动` / `模式: 待机` / `工作模式` / `遥控交还`
- 旧文案仅残留于 4 处**注释/docstring**（版本说明、函数文档），**无一处是 UI 显示文本** —— 已逐行核对
- 体积与 v3.3.3 基本一致（97.58 MB vs 97.69 MB），无 MKL 误入

> ⚠️ **双击启动验证未做**：沙箱拦截了程序对 `192.168.127.10:5000` 的网络访问（EXE 启动后会去连板卡取视频流）。
> onefile 首次启动解压约 3~5 s，且会先起一个 ~8.5 MB 的 bootloader 父进程（无窗口），
> 真正带窗口的是它的子进程 —— 用 `MainWindowTitle` 判活要看子进程，别误判成"没起来"。
> **需用户在本机双击确认。**

---

## v3.3.3 Kalman 融合深度（2026-09-23）

### 新增

- 「全部」页 → 姿态/运动参数 →「深度 / 高度」组新增 **Kalman深度** 一行（目标列 `--`，实际列为融合值）。
- 新增 `DepthKalmanFilter` 类（`pc_main2.py` §3.6）：状态 `[depth, vz]`，匀速模型预测 + 逐个标量观测更新；纯 Python 2×2 矩阵手工展开，不依赖 numpy。
  - 观测① `depth`（压力深度）② `vz`（垂向速度）③ `H − alt`（高度计换算深度，**默认关闭**）。
  - 遥测中断 > 2 s（`KF_RESET_GAP`）自动重置，避免用陈旧状态预测。
- 融合值同步进函数图曲线（「深度/高度」数据块，通道名 `KF深度`）与 CSV（字段 `kdepth`，位于 `alt` 之后）。
- 文件头新增 6 个调参常量：`KF_Q_DEPTH / KF_Q_VEL / KF_R_DEPTH / KF_R_VEL / KF_R_ALT / KF_RESET_GAP`，另有 `KF_USE_ALT`（默认 `False`）与 `KF_ALT_SCALE`（默认 `1.0`）。

### 待确认（连板复核）

`$TEL` 里 `alt` 与 `depth` **是否同单位**未确认，故高度计观测默认关闭。确认后：若 `alt` 是 mm 则 `KF_ALT_SCALE = 0.001`，再把 `KF_USE_ALT` 改 `True`。

### 实测（20 Hz，深度噪声 ±0.15 m）

- 融合值对真值 RMS 误差 **0.041**，原始观测 **0.144**（噪声降至 ~28%）。
- GUI 侧 60 帧：融合 std **0.0263**，原始 std **0.1172**（平滑约 4.5 倍），显示 `3.02`（真值 3.00）。
- 5 m 阶跃：首帧 0.944 → 10 帧 4.341 → 30 帧 4.992（约 1 s 收敛）。
- 断流 3 s 后重新喂数：立即重置为新观测值（9.0），无陈旧预测。

### 重新打包

产物 `ROV_ControlStation_v3.3.3.exe`（93 MB）；旧 `ROV_ControlStation_v3.3.2.exe` 已删除，避免误用。

---

## v3.3.2 EXE 重新打包（2026-09-22）

产物（项目根目录）：`ROV_ControlStation_v3.3.2.exe`（onefile、无控制台窗口，**93.2 MB** / 97,694,330 B）。

打包环境沿用 `C:\Users\lenovo\rovpack\venv`（Python 3.13.9 · PyQt5 5.15.11 · numpy 2.5.3 · opencv-python 4.14.0 · pygame 2.6.1 · pyserial 3.5 · pyinstaller 6.22.3），**不用 Anaconda**（其 numpy 带 MKL，体积会顶到 ~260 MB）。

```bat
:: 1) 建纯 ASCII 构建目录(项目目录含中文, PyInstaller 解析 Qt 插件路径会出 ??? 报错)
mkdir D:\rov_pkg\src
copy pc_main2.py protocol.py D:\rov_pkg\src\
cd /d D:\rov_pkg\src

:: 2) 打包；--distpath/--workpath 必须用 Windows 风格路径, 写 /d/xxx 会被解析成 D:\d\xxx
C:\Users\lenovo\rovpack\venv\Scripts\pyinstaller.exe --noconfirm --clean --onefile --windowed ^
  --name ROV_ControlStation_v3.3.2 --paths=. ^
  --hidden-import=protocol --hidden-import=serial.tools.list_ports --hidden-import=numpy ^
  --exclude-module matplotlib --exclude-module scipy --exclude-module pandas ^
  --exclude-module tkinter --exclude-module IPython --exclude-module notebook ^
  --distpath D:\rov_pkg\dist --workpath D:\rov_pkg\build --specpath D:\rov_pkg pc_main2.py

:: 3) 拷回项目目录后即可删掉 D:\rov_pkg
```

验证（2026-09-22）：双击启动 20 s 后进程稳定存活，窗口标题 `ROV控制站 v3.3`，常驻内存 ~137 MB。
注意 onefile 会先起一个 ~8.5 MB 的 bootloader 父进程（无窗口），真正带窗口的是它的子进程 —— 用 `MainWindowTitle` 判活要看子进程，别误判成"没起来"。

---

## v3.3.2 「全部」页整理（2026-09-21）

### 移除重复曲线

- 「全部」页的实时曲线（`PlotPanel` 实例 `_plot_all`）**已删除**——与「函数图」页完全重复，统一在函数图页查看。
- 原位置留一行灰色提示「实时曲线已合并到『函数图』页」。
- 数据喂给只保留 `self._plot_board.feed(ch)`（函数图页），少一次绘图刷新；`PlotPanel` 类保留，其 `PALETTE` 仍被 `DataPlotBoard` 复用。
- 「全部」页现有内容：姿态/运动参数 + 电池/环境 + 高度计 CH348 + 完整 PID 调参 + 手柄快捷键映射。

### 高度计字号统一

- 原 `12px/13px/14px` 硬编码改为模块级常量 `STY_HEAD` / `STY_TEXT` / `STY_VAL`（10pt / 10pt / 11pt 粗体）。
- 原因：px 不随系统 DPI 缩放，而姿态/电池等区域继承全局字体（Microsoft YaHei 10pt）会缩放，导致缩放屏上高度计明显偏小。
- 姿态区小标题 `─ xxx ─` 也改用 `STY_HEAD`；`_on_alt()` 着色时沿用 `STY_TEXT`，不再写死 px。

---

## v3.3.1 可视窗口 → 视频墙（2026-09-20）

- 新增「可视窗口」页（2026-09-18）：CAM1/CAM2 从主界面迁入，并新增第三个画面 CAM3（预留）。主界面只留操控与调参。
- v3.3.1 重做为视频墙：新增 `VideoTile`（画面卡片：4:3 等比居中 + 画面名/分辨率/fps 角标 + 状态描边）与 `VideoWall`（深色容器，布局后按 4:3 收紧卡片高度）。
- 布局预设 5 档：CAM1 主 / CAM2 主 / 三画面等分 / 仅 CAM1 / CAM2 主+CAM3；CAM3 与 CAM2 同进同退。
- CAM3 取流端口 **8084**（`protocol.py` 的 `CAM3_PORT=8084`、`CAM3_PATH="/stream"`，默认走板端 `show_cam.py`）；未开启时卡片显示「未开启取流」。
- 录像同步支持第三路 `rec_cam3_*.avi`；切到其他选项卡时跳过渲染省 CPU（录像照常）。

### 文档合并（2026-09-20）

- `README.md` 与 `README_zxq.md` 合并为一份，`README_zxq.md` 已删除；章节号重排为十章。
- 链路总表补两行：第三路画面 `:8084 /stream`、`$ESTOP#`。
- `pc_main2.py` 中 `README_zxq §二.1 / §1.3` 的引用改为本文口径。

---

## v3.3 EXE 打包（2026-09-17）

产物：`dist/ROV_ControlStation_v3.3.exe`（onefile、无控制台窗口，**93 MB** / 97,477,818 B）。

打包环境（干净 venv，不带 Anaconda / MKL）：`C:\Users\lenovo\rovpack\venv`（Python 3.13.9）
依赖：PyQt5 5.15.11（Qt 5.15.2）、numpy 2.5.3、opencv-python 4.14.0、pygame 2.6.1、pyserial 3.5、pyinstaller 6.22.3

```bat
:: 在项目根执行
C:\Users\lenovo\rovpack\venv\Scripts\pyinstaller.exe --noconfirm --clean --onefile --windowed ^
  --name ROV_ControlStation_v3.3 --paths=. ^
  --hidden-import=protocol --hidden-import=serial.tools.list_ports ^
  --exclude-module matplotlib --exclude-module scipy --exclude-module pandas ^
  --exclude-module tkinter --exclude-module IPython --exclude-module notebook pc_main2.py
```

- **踩到的坑（重要）**：venv 若建在含中文的路径下（例如项目内 `build\venv-pack`），PyInstaller 解析 Qt 插件目录时会因编码问题把 `控制站` 变成 `???`，报 `Exception: Qt plugin directory 'D:/RC/ROV???_v3.3/...' does not exist!` 直接失败。→ **venv 必须放在纯 ASCII 路径**；`--paths=.` 指向含中文的项目目录本身不受影响。
- `opencv-python` 需锁 `<5`：5.x 已无 `cv2.VideoWriter_fourcc`（录像功能依赖它）。
- 验证：启动后 75 s 进程与窗口均稳定（标题 `ROV控制站 v3.3`），未再出现 Anaconda 版启动数十秒后自行退出的现象。

---

## 文档-代码一致性核对（v3.3，2026-09-17）

按 README 逐项核对 `pc_main2.py` / `protocol.py`，修正 3 处不符：

1. **`heave` 映射**：`JoystickThread.run()` 中 `heave` 由 RT/LT 扳机差值改为右摇杆前后（`axis3`，前推为正 `-dz`）；RT/LT 归一化逻辑保留、转为待定预留。
2. **`target_ip` 兜底**：`"192.168.5.11"` → `"192.168.127.10"`（与 `X5_IP` 一致）。
3. **手柄未插入时的下发**：原 `continue` 跳过发送，改为仍按当前链路频率（有线 20Hz / 无线 5Hz）以零杆位持续下发 `$CMD`；手柄热插拔重扫仍按 0.5s 节流。

核查后确认符合的项：`$CMD` 九字段与 20Hz/5Hz 节奏、`surge/sway/yaw` 轴映射与死区 `0.08` / `round(v,2)`、LED ±5（clamp 0~100）、`grab` 500ms 脉冲、`store` 切换态、X 录像不发帧、界面按钮补发零杆位帧、AUV 强制零杆位、`$ESTOP#`/`$ESTOP,0#` 按钮与字节、连接状态判定（遥测 1.5s / PING）、`X5_IP=192.168.127.10`。

- 验证：`py_compile` 通过；`build/verify_readme_conformance.py`（假手柄 + 本机 UDP 抓帧）**8/8 通过**，含 `axis3` 前推 → `$CMD,0.00,0.00,1.00,0.00,0,0,0,0,0#`、AUV 零杆位、无手柄 ≈20Hz 零杆位、`$ESTOP` 帧字节断言。
- **未实机验证**：PC 界面按钮点击、真实手柄轴序与对船联调仍需连板复核。
- 备份：`pc_main2.py.bak_20260917_124105`。

同一轮同步的 **GUI 文案**：

- 窗口标题与文件头版本号 `v3.2` → `v3.3`。
- 分组「右摇杆 / 扳机」→「右摇杆 (Yaw / Heave)」；标签改为 `Yaw (RStick X，右推为正)`、`Heave (RStick Y，前推上浮)`、`LT / RT 扳机 (预留)`。
- `YawBar` 增加 `label` 参数：三根横条分别为 `Yaw` / `Heave` / `Trig`；第三根显示 `rt - lt`。
- 「手柄快捷键映射」面板同步更新。
- 验证：`build/verify_ui_text.py`（构造真实 `MainWindow` 后读控件文本）**14/14 通过**。

---

## AUV 模式：上位机不再下发运动控制（v3.3，2026-09-17）

- `JoystickThread.run()`：组帧前判断 `self._mode == 1`，把 `surge / sway / heave / yaw` 强制 `0.00`，上位机在 AUV 下**只发模式信号**；运动交给 S100 自主完成。
- 切回遥控（`mode=0`）即恢复正常杆位下发，无需重启；`mode` 字段与 `$ESTOP` 机制不受影响。
- **待办（板端，未验证）**：`/userdata/To32` 需确认 AUV 期间忽略 `$CMD` 运动字段；本机无该代码副本，未实测。

---

## v3.3 显式急停帧 `$ESTOP#` / 解除帧 `$ESTOP,0#`（2026-09-15）

- **背景**：PC 的「急停」原来只有一招——发一帧全零 `$CMD`，它与「摇杆回中」在报文上完全同形，板端无法区分，只能做到零推力、进不了 STANDBY；且板端锁存后真实 UI 无法解除。
- `protocol.py`：新增 `build_estop()` → `$ESTOP#` 与 `build_estop_release()` → `$ESTOP,0#`（S100 板端拦截，不透传 STM32）；原 `build_emergency_stop()`（全零 `$CMD`）语义不变。
- `pc_main2.py`：「快捷操作」区新增 **急停** / **解除急停** 按钮；`JoystickThread` 新增 `send_estop()` / `send_estop_release()`（**只走有线 UDP 8080**；无线 LoRa 直连 STM32、不认该帧，自动跳过）；手柄断开时补发 `$ESTOP#`。
- **现场动作**：急停 = 零杆位 `$CMD`（先停推）+ `$ESTOP#`（进 STANDBY 并锁存）；恢复按「解除急停」发 `$ESTOP,0#`。
- 验证：`py_compile` 通过 + 帧字节断言（`$ESTOP#` = `24 45 53 54 4F 50 23 0D 0A`、`$ESTOP,0#` = `24 45 53 54 4F 50 2C 30 23 0D 0A`）；**PC 界面未实机点击验证**。

## 文档：hostlink 使用说明并入（2026-09-15）

- 板端 `hostlink_README.md` 全文并入原 README 第十章（10.1~10.10），现为 `README.md` 第五章。
- 路径与命名统一：图像+遥测 `S100/vp/v5A`、高度计 `S100/USART/v1/read_altimeter.py`、网关 `/userdata/hostlink`。

---

## v3.2.3 模式按钮（2026-09-11）

- 主界面「快捷操作」区新增 **遥控模式 / AUV模式** 两个互斥按钮（选中为绿 / 蓝高亮），与手柄 LB 共用 `_joy_thread._mode`；`mode` 随 `$CMD` 第 7 字段下发。
- 点击即补发一帧零杆位 `$CMD`，避免切换瞬间误动推进器；手柄未插入时按钮同样生效。
- `_paint_mode()` 作为唯一刷新入口，统一刷新左上角模式标签与按钮高亮（修复首版「标签不刷新」缺陷）。
- EXE 未重打：`ROV控制站v3.2.exe` 仍为 v3.2.2 包。

## v3.2.2 视频显示限帧（2026-09-13）

- 新增 `VID_DISPLAY_FPS = 60`（可调 30~60）；`VideoThread` 改为收/显双计数，仍读走每帧避免 TCP 反压，但按 `1/display_fps` 间隔出图，超出帧丢弃只保留最新帧。
- 终端与状态栏显示 `CAM1 60fps(收200)`，括号内为推流实际帧率。

## v3.2.2 高度计区域 3×6（2026-09-04）

- 「全部」页 CH348 区域由 6 行×3 列改为 **3 行 × 6 列**（左列标签：通道 / 状态 / 数值 (mm)，A–E 各占一列），纵向占用减半。
- 数据绑定不变：`_alt_labels`、`AltThread`、UDP 8082、`_on_alt` 着色逻辑均未改。

## v3.2.2 函数图与推进器曲线（2026-09-04）

- 「函数图」页改为 **顶部数据块多选 + 下方每自由度独立小图**（`DataPlotBoard`，整页可滚动）：姿态角 / 角速度 / 加速度 / 速度 / 深度·高度 / 推进器 六组，未勾选整行隐藏但后台继续收数。
- 12 路推进器独立为 **2 行 × 3 列** 六图（`ThrusterPlotGrid`，每图 2 路）；显示名 `Thr1–Thr12`，数据键仍为 `thr0..thr11`，奇数路浅蓝、偶数路橙。
- 纵轴 / 图例自适应：刻度格式按跨度切换、刻度区宽度随最长标签、图例自动换行。

## v3.2.1 UI 调整（2026-09-04）

- 删除「视频布局: 上下」按钮与 `_toggle_video_layout`，视频固定左右布局。（v3.3.1 起画面迁入「可视窗口」页，改为 5 档预设）
- 画面随窗口等比例缩放（保持 4:3），到图像区域高度上限即停；各自居中、未填满处黑底，最小 320×240。

## v3.2.1 高度计 CH348（2026-09-04）

- 新增 `$ALT,<通道>,<mm>,<状态>#` 帧、`ALT_PORT = 8082` 与 `build_alt` / `parse_alt`；「全部」页 PID 上方显示 A–E 五通道（正常绿 / 未接设备灰 / 异常红）。
- 板端 `read_altimeter.py` 增加 UDP 推送：`--pc-ip`（默认 `192.168.127.100`）、`--pc-port`（默认 8082）、`--no-udp`；实测 B 口 533~534 mm 正常显示。
