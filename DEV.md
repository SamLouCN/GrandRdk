# GrandRdk开发手册
本手册包含：  
- 项目文件结构
### 项目文件结构
```text
GrandRdk
├── To32/
├── USART/
│   ├── read_altimeter.py
│   └── README.md
├── momo_pwmnet/
│   ├── algos/							#光流算法
│   │   ├── __init__.py 				
│   │   ├── algo_common.py
│   │   ├── algo_dense.py
│   │   ├── algo_dis.py
│   │   ├── algo_sparse.py
│   │   └── algo_tvl1.py
│   ├── archive/
│   │   ├── algo_farneback.py
│   │   ├── algo_pyrlk.py
│   │   ├── flow_dis.py
│   │   ├── flow_farneback.py
│   │   ├── flow_pyrik.py
│   │   └── flow_viz.py
│   ├── depth/							#(aborted)测速演示用的深度
│   │   ├── model/
│   │   │   ├── depth_any.hbm
│   │   ├── calibrate_depth.py
│   │   ├── depth_anything.py
│   │   └── speed_fusion.py
│   ├── flow_speed.py 					#读取`vp5.1`的帧，输出平面速度
│   ├── gpu_flow_bench.py  				#测GPU光流到底快了多少
│   ├── optical_flow_demo.py 			#调度光流算法，对比帧率和质量
│   ├── speed_flow_demo.py 				#(aborted)测速演示
│   ├── t4_synthetic_writer.py 			#向`/dev/shm/`写入来自`vp5.1`的帧
│   ├── config.ini 						#光流参数配置文件
│   └── README.md
├── vp/
│   ├── utils/
│   ├── function.py
│   ├── hw_camera.py
│   ├── main_config.py
│   ├── main.py
│   ├── run.sh
│   └── test_nashe_640x640_nv12.hbm
└── cmd_watch.py
```
### 需要做的事
- 提高光流精度  
- 光流标定  
- 零散功能的总合