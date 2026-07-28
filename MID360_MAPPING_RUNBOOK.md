# MID360 实时建图运行记录与操作手册

日期：2026-07-28

本文档记录当前 MID360 激光雷达建图系统已经完成的配置、当前运行状态、常用启动命令、2D/3D 查看方式、地图保存方式和调参说明。

## 1. 当前成果

### 1.1 硬件与网络

- 激光雷达：Livox MID360
- 雷达 IP：`192.168.1.110`
- 主机有线网卡 IP：`192.168.1.5`
- 雷达驱动已经可以稳定收到数据。
- 原始点云话题：
  - `/livox/lidar`
  - 类型：`livox_ros_driver2/msg/CustomMsg`
  - 频率：约 `10 Hz`
  - 每帧点数：约 `20000`
- IMU 话题：
  - `/livox/imu`
  - 类型：`sensor_msgs/msg/Imu`
  - 频率：约 `200 Hz`

### 1.2 已完成的软件修复

已修复 Livox ROS2 驱动多网卡连接问题：

- 配置文件：
  - `ws_livox/src/livox_ros_driver2/config/MID360_config.json`
  - `ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`
- 已将 `host_net_info` 配成主机 IP `192.168.1.5` 对应雷达 IP `192.168.1.110`。

已修复 Livox ROS2 驱动收到数据但不发布话题的问题：

- 文件：`ws_livox/src/livox_ros_driver2/src/lds.cpp`
- 在 `Lds::StoragePointData()` 中补充了采样状态设置：

```cpp
lidars_[index].connect_state = kConnectStateSampling;
```

已修复 FAST-LIO 编译依赖：

- 初始化了 `FAST_LIO/include/ikd-Tree` 子模块。
- 在 FAST-LIO 中补充了 `tf2_ros` 编译依赖。
- 关闭 FAST-LIO 在线外参估计：

```yaml
extrinsic_est_en: false
```

该设置通常比在线估计更稳定，尤其是 MID360 这种 LiDAR/IMU 外参固定的设备。

### 1.3 雷达倒装显示

雷达是倒着安装的。当前只对 RViz 显示做反转，不改变 FAST-LIO 建图算法坐标。

已添加显示用 TF：

```text
fastlio_view_frame -> camera_init
roll = 180 deg
```

RViz 固定坐标系使用：

```text
fastlio_view_frame
```

这样 RViz 里看到的 3D/2D 地图会按倒装后的视角反过来显示。

### 1.4 3D 建图

FAST-LIO 已经可以实时输出：

- `/cloud_registered`：当前帧配准后的点云
- `/Laser_map`：累计三维地图点云
- `/Odometry`：雷达/IMU 当前位姿
- `/path`：运动轨迹

已保存过一次 3D PCD 地图：

```text
maps/mid360_fastlio_map.pcd
```

该文件约 `49 MB`，点数约 `3,180,679`。

### 1.5 实时 2D 占据栅格建图

已新增实时 2D 建图工具：

```text
tools/live_pointcloud_to_grid.py
```

该节点将 FAST-LIO 的 3D 点云实时转换成二维占据栅格：

- 输入点云：`/cloud_registered`
- 输入位姿：`/Odometry`
- 输出地图：`/map`
- 地图类型：`nav_msgs/msg/OccupancyGrid`

地图状态：

- `-1`：未知区域，未扫描到
- `0`：空闲区域，雷达射线经过
- `100`：占用区域，点云末端命中

这比单纯的点云投影更接近 SLAM 地图，因为可以区分“空闲”和“未知”。

当前 2D 建图参数：

```text
resolution = 0.03 m
min_z = -1.6 m
max_z = 0.1 m
raytrace = true
max_range = 50.0 m
margin_cells = 200
```

当前 RViz 2D 显示内容：

- `Live2DMap`：实时二维地图 `/map`
- `LidarPose`：雷达当前位置箭头 `/Odometry`
- `LidarPath`：雷达运动轨迹 `/path`

## 2. 当前运行状态

最近确认的 ROS 节点：

```text
/fastlio_view_tf
/laser_mapping
/live_pointcloud_to_grid
/livox_lidar_publisher
/rviz
```

当前链路：

```text
MID360 -> livox_ros_driver2 -> FAST-LIO -> live_pointcloud_to_grid -> RViz
```

数据流：

```text
/livox/lidar + /livox/imu
  -> FAST-LIO
  -> /cloud_registered + /Odometry
  -> live_pointcloud_to_grid
  -> /map
  -> RViz
```

## 3. 环境准备

每个新终端先执行：

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
```

## 4. 启动实时 2D 建图

建议使用三个终端。

### 4.1 终端 1：启动 MID360 驱动

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

检查雷达输入：

```bash
ros2 topic echo --once /livox/lidar --field point_num
timeout 5 ros2 topic hz /livox/lidar
```

正常情况：

- `point_num` 约 `20000`
- 频率约 `10 Hz`

### 4.2 终端 2：启动 FAST-LIO

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/home/hu/Desktop/bxi/mid360/ws_livox/src/FAST_LIO/config \
  config_file:=mid360.yaml \
  rviz:=false
```

启动后先让雷达静止几秒，等待日志出现：

```text
IMU Initial Done
Initialize the map kdtree
```

然后再缓慢移动雷达。

检查 FAST-LIO 输出：

```bash
ros2 topic echo --once /cloud_registered --field width
ros2 topic echo --once /Odometry --field pose.pose.position
```

### 4.3 终端 3：启动实时 2D 占据栅格

当前推荐参数：

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
python3 tools/live_pointcloud_to_grid.py \
  --ros-args \
  -p input_topic:=/cloud_registered \
  -p odom_topic:=/Odometry \
  -p map_topic:=/map \
  -p frame_id:=camera_init \
  -p resolution:=0.03 \
  -p min_z:=-1.6 \
  -p max_z:=0.1 \
  -p publish_rate:=2.0 \
  -p raytrace:=true \
  -p max_range:=50.0 \
  -p margin_cells:=200
```

检查 2D 地图：

```bash
ros2 topic echo --once /map --field info
```

### 4.4 终端 4：打开 2D RViz

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 run rviz2 rviz2 -d /home/hu/Desktop/bxi/mid360/ws_livox/src/FAST_LIO/rviz/fastlio_2d.rviz
```

RViz 中显示：

- `Live2DMap`：实时 2D 地图
- `LidarPose`：雷达当前位置
- `LidarPath`：轨迹

## 5. 启动 3D 查看

如果要看三维点云地图，启动雷达驱动和 FAST-LIO 后执行：

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 run rviz2 rviz2 -d /home/hu/Desktop/bxi/mid360/ws_livox/src/FAST_LIO/rviz/fastlio.rviz
```

3D RViz 显示：

- `CloudRegistered`：当前帧点云 `/cloud_registered`
- `CloudMap`：累计 3D 地图 `/Laser_map`
- `Odometry`：雷达位姿 `/Odometry`
- `Path`：轨迹 `/path`

## 6. 常用操作

### 6.1 重启 2D 地图

只停止 `tools/live_pointcloud_to_grid.py` 对应终端，然后重新执行 2D 建图命令即可。

效果：

- 清空当前二维累计地图
- 雷达驱动不重启
- FAST-LIO 不重启
- RViz 不必重启

### 6.2 调整高度范围

高度单位是米。

高度标准是 FAST-LIO 的 `camera_init` 坐标系中的 `z` 值：

- `z = 0` 大致是 FAST-LIO 启动时雷达/IMU 所在高度平面
- 不是自动识别的地面高度
- RViz 的倒装显示不影响高度判断

示例：最低 `-1.6m`，最高 `0.1m`：

```bash
-p min_z:=-1.6
-p max_z:=0.1
```

修改高度后，需要重启 `tools/live_pointcloud_to_grid.py` 节点。

### 6.3 调整地图分辨率

当前：

```bash
-p resolution:=0.03
```

含义：每个栅格 `3 cm`。

常用建议：

- `0.03`：细，显示更精细，但更吃 CPU/RViz
- `0.05`：稳，适合大范围建图
- `0.10`：粗，适合快速看整体结构

### 6.4 调整地图范围

当前使用动态地图边界，外扩：

```bash
-p margin_cells:=200
```

在 `0.03m` 分辨率下，四周多出：

```text
200 * 0.03 = 6 m
```

如果想地图更大，可以继续增大 `margin_cells`，但 RViz 和 CPU 压力会增大。

### 6.5 保存 3D 点云地图

推荐从当前 `/Laser_map` 抓一帧保存：

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
python3 tools/save_pointcloud2_pcd.py /Laser_map /home/hu/Desktop/bxi/mid360/maps/mid360_fastlio_map.pcd
```

保存路径：

```text
maps/mid360_fastlio_map.pcd
```

### 6.6 查看已保存的 3D PCD

发布保存的 PCD 到 `/Laser_map`：

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
python3 tools/publish_pcd_map.py maps/mid360_fastlio_map.pcd /Laser_map camera_init
```

另开终端发布反转显示 TF：

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 3.141592653589793 \
  --pitch 0 --yaw 0 \
  --frame-id fastlio_view_frame \
  --child-frame-id camera_init
```

再打开 3D RViz：

```bash
ros2 run rviz2 rviz2 -d /home/hu/Desktop/bxi/mid360/ws_livox/src/FAST_LIO/rviz/fastlio.rviz
```

## 7. 重要文件

Livox 驱动配置：

```text
ws_livox/src/livox_ros_driver2/config/MID360_config.json
ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

Livox 驱动源码修复：

```text
ws_livox/src/livox_ros_driver2/src/lds.cpp
```

FAST-LIO 配置：

```text
ws_livox/src/FAST_LIO/config/mid360.yaml
ws_livox/install/fast_lio/share/fast_lio/config/mid360.yaml
```

FAST-LIO 启动文件：

```text
ws_livox/src/FAST_LIO/launch/mapping.launch.py
ws_livox/install/fast_lio/share/fast_lio/launch/mapping.launch.py
```

RViz 配置：

```text
ws_livox/src/FAST_LIO/rviz/fastlio.rviz
ws_livox/src/FAST_LIO/rviz/fastlio_2d.rviz
```

自定义工具：

```text
tools/live_pointcloud_to_grid.py
tools/save_pointcloud2_pcd.py
tools/publish_pcd_map.py
```

地图目录：

```text
maps/
```

## 8. 漂移与建图注意事项

当前已做的稳定性处理：

- `extrinsic_est_en` 已关闭，避免在线外参估计带来不稳定。
- 建图启动后先静止几秒，让 IMU 初始化完成。
- RViz 倒装只影响显示，不影响 FAST-LIO 坐标计算。

操作建议：

- 启动 FAST-LIO 后先静止 3 到 5 秒。
- 移动时尽量慢，避免突然大幅旋转。
- 尽量减少快速经过的人、门、车等动态物体。
- 如果漂移严重，重新启动 FAST-LIO，从静止状态开始。
- 回到起点附近闭合一圈，通常地图整体效果更好。

## 9. 已知问题

### 9.1 RViz Map 插件 OpenGL 警告

RViz 有时会输出类似：

```text
active samplers with a different type refer to the same texture image unit
```

这是当前虚拟机/OpenGL 环境下 RViz Map shader 的警告。只要 RViz 能显示 `/map`，一般可以忽略。

### 9.2 Livox 驱动退出时报 `exit code -11`

停止 Livox 驱动时，日志里可能出现 SDK deinit 后 `exit code -11`。目前观察到雷达 SDK 已经完成释放，不影响建图和保存的地图文件。

### 9.3 当前 2D 地图还不是完整 Nav2 导航地图

当前 `/map` 已经是 OccupancyGrid，并且有未知/空闲/占用三态。

但如果要给 Nav2 做导航，还建议继续增加：

- 地图保存为 `pgm + yaml`
- 障碍膨胀
- 小噪点滤除
- 可通行区域清理
- 机器人 footprint/costmap 参数

## 10. 快速恢复当前 2D 建图状态

按顺序运行：

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/home/hu/Desktop/bxi/mid360/ws_livox/src/FAST_LIO/config \
  config_file:=mid360.yaml \
  rviz:=false
```

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
python3 tools/live_pointcloud_to_grid.py \
  --ros-args \
  -p input_topic:=/cloud_registered \
  -p odom_topic:=/Odometry \
  -p map_topic:=/map \
  -p frame_id:=camera_init \
  -p resolution:=0.03 \
  -p min_z:=-1.6 \
  -p max_z:=0.1 \
  -p publish_rate:=2.0 \
  -p raytrace:=true \
  -p max_range:=50.0 \
  -p margin_cells:=200
```

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 run rviz2 rviz2 -d /home/hu/Desktop/bxi/mid360/ws_livox/src/FAST_LIO/rviz/fastlio_2d.rviz
```
