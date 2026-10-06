#!/usr/bin/env python3                                                          # Shebang: 让系统用 python3 解释器执行本文件
# -*- coding: utf-8 -*-                                                          # 编码声明: 源码按 UTF-8 解析, 中文不乱码
"""camera_ports.py — 按 USB 物理口固定各路相机的设备节点（勿用节点号写死）  # 模块文档字符串: 文件用途说明

问题: 设备节点号由内核枚举顺序决定, 上电顺序 / 插拔先后一变就会漂移, 节点号不作为配置真值。  # 注释: 解释为何不写死节点号
      本板 2026-09-23 实测: 节点号曾出现 "与角色互换" 错位（例如 cam1 节点拿到的是内窥镜），  # 注释: 实战案例
      触发原因是旧代码把节点号写死。换为按 USB 物理口绑定后即稳定。  # 注释: 修复策略

本板实测 USB 拓扑（2026-09-23, 三颗相机）:                                  # 注释: 实测硬件拓扑
    物理口 1-1  控制器 0000:06:00.0  ENDOSCOPE HDCAM (090c:f37d)  -> cam3 内窥镜  # 注释: USB 拓扑第 1 行
    物理口 1-2  控制器 0000:06:00.0  USB Camera      (0bda:5883)  -> cam2 下视  # 注释: USB 拓扑第 2 行
    物理口 3-2  控制器 0000:07:00.0  USB Camera      (0bda:5883)  -> cam1 前视  # 注释: USB 拓扑第 3 行
    两个控制器都是 PCIe xHCI: 06:00.0 走 pci 0000:03:06.0, 07:00.0 走 pci 0000:03:0e.0  # 注释: 控制器位置
    （节点号随枚举漂移，本表只记控制器 / 产品 / 物理口，不记具体设备节点号）  # 注释: 强调

通道分配（2026-09-23 定稿）:                                                  # 注释: 通道映射
    cam1 = 前视   = 物理口 3-2   (front.py  -> SHM_FRAME_FRONT  -> web :5000/cam1)  # 注释: 前视通道
    cam2 = 下视   = 物理口 1-2   (bottom.py -> SHM_FRAME_BOTTOM -> web :5000/cam2)  # 注释: 下视通道
    cam3 = 第三路 = 物理口 1-1   (show_cam.py -> :8084/stream)  # 注释: 第三路通道

三个必须知道的坑:                                                            # 注释: 坑列表标题
    1) 每颗 UVC 相机占两个 video 节点: index=0 是采集节点, index=1 是 metadata,  # 注释: 坑 1
       必须只认 index=0, 否则会挑到打不开的元数据节点。  # 注释: 坑 1 的后果
    2) 两颗 Realtek 的 serial 都是 20000001, 内窥镜的 serial 就是产品名  # 注释: 坑 2
       "ENDOSCOPE HDCAM" —— by-id 无法唯一区分, 所以只能按物理口(1-1/1-2/3-2)绑定。  # 注释: 坑 2 解决方案
    3) 找不到相机时本模块返回一个**不存在的路径**, 而不是 None:  # 注释: 坑 3
       front.py/bottom.py 里有 "节点号兜底"（按 index 拼路径的 fallback） , 一旦返回 None 就会  # 注释: 兜底代码存在
       静默退回旧编号, 又回到错位状态且没有任何报错。**device_for 返回不存在路径, 宁可让打开失败。**  # 注释: 设计意图
       兜底代码保留仅为防止 None 崩溃, 业务路径永远不该走到那条线。  # 注释: 兜底代码说明
"""                                                                            # 注释: 文档字符串结束
import glob                                                                    # 标准库: 用 shell 通配模式列目录, 用来扫 /sys/class/video4linux 下的所有 videoN 节点
import os                                                                      # 标准库: 路径拼接、判断文件存在、读 sysfs 等

# 通道 -> (USB 物理口 "bus-port", 采集节点 index)                              # 注释: 配置表说明
# 换插到别的物理口 / 加了 USB Hub（口名变成 1-1.2 这种）时, 改这里一行即可  # 注释: 维护提示
CAM_PORTS = {                                                                  # 通道 → (物理口, 采集节点 index) 的真值表
    'cam1': ('3-2', 0),                                                        # 前视: USB 物理口 3-2, 取采集节点 (index=0)
    'cam2': ('1-2', 0),                                                        # 下视: USB 物理口 1-2, 取采集节点 (index=0)
    'cam3': ('1-1', 0),                                                        # 第三路(内窥镜): USB 物理口 1-1, 取采集节点 (index=0)
}                                                                              # 注释: 配置表结束

# 语义别名, 方便按角色取                                                    # 注释: 别名表说明
ALIASES = {'front': 'cam1', 'bottom': 'cam2'}                                 # 别名 → 标准通道名映射, 允许 device_for('front') 当 cam1 用

_SYSFS_GLOB = '/sys/class/video4linux/video*'                                 # 扫描路径: linux 把 /dev/videoN 的设备信息都挂在这个目录下


def _port_of(devpath):                                                        # 内部函数: 从 sysfs 设备软链里抽出物理口
    """从 .../usb1/1-1/1-1:1.0/video4linux/video0 里取出物理口 '1-1'。"""     # 注释: 函数作用
    parts = devpath.rstrip('/').split('/')                                    # 把路径按 / 切分, 去掉尾部分隔符避免空字段
    if len(parts) < 3 or parts[-2] != 'video4linux':                          # 路径格式不对: 不是从 video4linux 父目录来的就拒
        return None                                                           # 返回 None: 让上层走"匹配不上"分支
    return parts[-3].split(':')[0]                                            # '1-1:1.0' -> '1-1' (冒号前才是物理口, 冒号后是 USB 配置号)


def _node_index(node):                                                        # 内部函数: 读 sysfs 的节点 index(0=采集, 1=metadata)
    """读 sysfs 的节点 index（0=采集, 1=metadata）。"""                       # 注释: 函数作用
    try:                                                                      # 兜住文件不存在/不是数字这两类异常
        with open(os.path.join(node, 'index')) as f:                          # 打开 sysfs 节点的 index 文件 (如 /sys/class/video4linux/video0/index)
            return int(f.read().strip())                                      # 读全部内容, 去空白, 转 int (0 或 1)
    except (OSError, ValueError):                                              # 文件打不开 / 内容不是数字
        return None                                                           # 返回 None 让上层跳过


def resolve(port, want_index=0):                                              # 内部函数: 找出某物理口上 index==want_index 的设备节点
    """返回该 USB 物理口上 index==want_index 的 cam 节点（不再以具体设备节点号为配置名）; 没有则 None。"""  # 注释: 返回值语义
    for node in sorted(glob.glob(_SYSFS_GLOB)):                               # 遍历所有 video4linux 节点, sorted 保证结果稳定
        if _port_of(os.path.realpath(node)) != port:                          # 物理口不匹配
            continue                                                          # 跳到下一个
        if _node_index(node) == want_index:                                   # 这个节点的 index 字段正好是我们要的
            return '/dev/' + os.path.basename(node)                           # 拼出 /dev/videoN 返回 (用真实节点号)
    return None                                                               # 没找到: 返回 None (调用方应据此回落到"不存在的占位路径", 见 device_for)


def device_for(name, verbose=True):                                           # 公开函数: 业务层统一入口
    """按通道名(cam1/cam2/cam3 或别名 front/bottom)解析设备节点。             # 注释: 函数作用

    解析失败时返回不存在的路径(故意), 让调用方 open 立刻失败并打印错误,     # 注释: 错误处理策略
    而不是静默退回具体设备节点号这类不稳定的旧编号（**节点号仅在历史日志里出现过, 配置层不再使用**）。  # 注释: 设计缘由
    """                                                                        # 注释: 文档字符串结束
    key = ALIASES.get(name, name)                                             # 先看是不是别名, 命中就用别名; 没命中就当标准通道名直接用
    if key not in CAM_PORTS:                                                  # 配置表里也没有这个通道
        raise KeyError('未知相机通道: %r (可用: %s)' % (name, ', '.join(CAM_PORTS)))  # 显式报错: 名字写错比静默错位好
    port, idx = CAM_PORTS[key]                                                # 拿出物理口和 index
    dev = resolve(port, idx)                                                  # 真去 sysfs 找
    if dev is not None:                                                       # 找到了
        if verbose:                                                           # 要不要打印解析过程 (默认要; 单元测试会关掉)
            print('[camera] %s: USB 口 %s (index=%d) -> %s' % (key, port, idx, dev), flush=True)  # 打印一行: cam1=USB 口 3-2(index=0) -> /dev/videoN
        return dev                                                            # 返回真实节点
    if verbose:                                                               # 没找到: 也要打印, 让用户知道是物理口/接线问题
        print('[camera] 错误: USB 口 %s 上没有 index=%d 的采集节点, %s 不可用' % (port, idx, key),
              flush=True)                                                      # 打印错误行, 让启动日志里能看到
    return '/dev/none-usb-%s-idx%d' % (port.replace('-', '.'), idx)           # 返回"故意不存在的路径", 让 open 立刻报错


def describe_all():                                                           # 公开函数: 给 status.sh / 自检用
    """返回 {通道: (物理口, 节点或 None)}，用于自检/诊断。"""                # 注释: 返回格式
    return {k: (v[0], resolve(v[0], v[1])) for k, v in CAM_PORTS.items()}     # 对每条配置解一次, 拼成 dict 返回


if __name__ == '__main__':                                                    # 当这个文件被直接执行 (不是被 import) 时进入自检模式
    print('USB 物理口 -> 设备节点 映射（实时解析）')                          # 标题
    for _k, (_port, _idx) in CAM_PORTS.items():                               # 遍历三条通道
        print('  %-5s USB 口 %-5s index=%d  ->  %s' % (_k, _port, _idx, resolve(_port, _idx)))  # 每条打印一行: camX / 物理口 / index / 解析结果
    print()                                                                   # 空一行隔开
    for _k in CAM_PORTS:                                                      # 再遍历一次
        print('  device_for(%-7r) = %s' % (_k, device_for(_k, verbose=False)))  # 走 device_for() 拿一遍结果 (不打印解析过程, 避免重复)