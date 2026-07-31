# MID360 实时建图运行记录与操作手册

日期：2026-07-28

本文档记录当前 MID360 激光雷达建图系统已经完成的配置、当前运行状态、常用启动命令、2D/3D 查看方式、地图保存方式和调参说明。

## 1. 当前成果

### 1.1 硬件与网络

- 激光雷达：Livox MID360
- 雷达 IP：`192.168.1.110`
- 主机有线网卡 IP：`192.168.1.51`
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
- 已将 `host_net_info` 配成主机 IP `192.168.1.51` 对应雷达 IP `192.168.1.110`。

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
min_z = -0.8 m
max_z = 0.8 m
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
  -p min_z:=-0.8 \
  -p max_z:=0.8 \
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

示例：最低 `-0.8m`，最高 `0.8m`：

```bash
-p min_z:=-0.8
-p max_z:=0.8
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

## 9. FAST-LIO + pointcloud_to_laserscan + slam_toolbox 方案

这套方案已在项目里单独做成 ROS2 包：

```text
ws_livox/src/mid360_slam_toolbox
```

链路是：

```text
MID360
  -> livox_ros_driver2
  -> /livox/lidar + /livox/imu
  -> FAST-LIO
  -> /cloud_registered_body
  -> pointcloud_to_laserscan
  -> /scan
  -> slam_toolbox
  -> /map
```

这套方案使用 FAST-LIO 提供位姿和去畸变后的 body-frame 点云，不使用 `tools/live_pointcloud_to_grid.py`。

### 9.1 安装依赖

当前系统如果还没有安装这两个包，需要先执行：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-pointcloud-to-laserscan ros-humble-slam-toolbox
```

### 9.2 编译配置包

```bash
cd /home/hu/Desktop/bxi/mid360/ws_livox
source /opt/ros/humble/setup.bash
colcon build --packages-select mid360_slam_toolbox --symlink-install
source install/setup.bash
```

### 9.3 启动

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch mid360_slam_toolbox mid360_slam_toolbox.launch.py
```

默认参数：

```text
输入点云: /cloud_registered_body
输出激光: /scan
输出地图: /map
FAST-LIO odom TF: camera_init -> body
slam_toolbox TF: map -> camera_init
base_frame: body
odom_frame: camera_init
高度范围: -0.2 m 到 1.2 m
range_min: 0.3 m
range_max: 25.0 m
angle_increment: 0.5 deg
```

如果已经单独启动了 Livox 驱动或 FAST-LIO，可以关闭对应启动项：

```bash
ros2 launch mid360_slam_toolbox mid360_slam_toolbox.launch.py start_livox:=false
ros2 launch mid360_slam_toolbox mid360_slam_toolbox.launch.py start_fast_lio:=false
```

后续接入真实或仿真的底盘后，应改成：

```text
base_frame = base_link
odom_frame = odom
```

并提供 `odom -> base_link` 的 TF。

## 10. 已知问题

### 10.1 RViz Map 插件 OpenGL 警告

RViz 有时会输出类似：

```text
active samplers with a different type refer to the same texture image unit
```

这是当前虚拟机/OpenGL 环境下 RViz Map shader 的警告。只要 RViz 能显示 `/map`，一般可以忽略。

### 10.2 Livox 驱动退出时报 `exit code -11`

停止 Livox 驱动时，日志里可能出现 SDK deinit 后 `exit code -11`。目前观察到雷达 SDK 已经完成释放，不影响建图和保存的地图文件。

### 10.3 2D OccupancyGrid 还需要导航侧处理

当前 `/map` 已经是 OccupancyGrid，并且有未知/空闲/占用三态。

但如果要给 Nav2 做导航，还建议继续增加：

- 地图保存为 `pgm + yaml`
- 障碍膨胀
- 小噪点滤除
- 可通行区域清理
- 机器人 footprint/costmap 参数

## 11. 快速恢复当前 2D 建图状态

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
ros2 launch mid360_slam_toolbox mid360_slam_toolbox.launch.py start_livox:=false
```

启动后先让 MID360 静止 3 到 5 秒，等 FAST-LIO 完成 IMU 初始化后再缓慢移动。

## 12. 2026-07-31 雷达连接与 2D 建图排障记录

### 12.1 网络问题与最终配置

本次不能连接雷达的根因不是 ROS、RViz、UDP 端口或防火墙，而是主机地址与雷达保存的目标地址不一致。

- 旧记录中的主机地址：`192.168.1.5`
- 雷达地址：`192.168.1.110`
- 抓包发现雷达持续发送 ARP：`who-has 192.168.1.51 tell 192.168.1.110`
- 最终主机地址：`192.168.1.51/24`
- 活动连接：`Wired connection 2`
- 雷达网卡：`enx00e04c0c6cc8`

NetworkManager 配置：

```bash
nmcli connection modify 'Wired connection 2' \
  ipv4.method manual \
  ipv4.addresses 192.168.1.51/24 \
  ipv4.gateway '' \
  ipv4.dns '' \
  ipv4.never-default yes
nmcli connection up 'Wired connection 2'
```

驱动的源码配置和安装配置均已同步为：

```text
host_ip: 192.168.1.51
lidar_ip: 192.168.1.110
```

网络恢复后的验证结果：雷达 ping 无丢包，Livox 驱动可以同时发布 `/livox/lidar` 和 `/livox/imu`。

### 12.2 完整 2D 建图链路

最终使用统一启动文件，避免 PointCloud2 模式与 FAST-LIO 所需的 Livox CustomMsg 类型冲突：

```bash
cd /home/hu/Desktop/bxi/mid360
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
ros2 launch mid360_slam_toolbox mid360_slam_toolbox.launch.py
```

实际链路：

```text
MID360
  -> livox_ros_driver2 (CustomMsg)
  -> FAST-LIO
  -> /cloud_registered_body
  -> pointcloud_to_laserscan
  -> /scan
  -> slam_toolbox
  -> /map
```

验证结果：

- `/scan`：约 `10 Hz`
- `/map`：约 `2 Hz`
- 地图分辨率：`0.05 m`
- FAST-LIO 可以完成 `IMU Initial Done`
- RViz 可以显示 LaserScan、射线清空和 OccupancyGrid

### 12.3 当前过滤与 slam_toolbox 参数

点云转 LaserScan：

```text
target_frame: body
min_height: -0.2 m
max_height: 1.2 m
range_min: 0.3 m
range_max: 25.0 m
angle_increment: 0.5 deg
```

为兼顾障碍实时更新与漂移抑制，slam_toolbox 当前使用：

```text
minimum_time_interval: 0.2 s
minimum_travel_distance: 0.0 m
minimum_travel_heading: 0.0 rad
scan_buffer_size: 30
map_update_interval: 0.5 s
do_loop_closing: true
```

曾尝试将最小运动门限设置为 `0.05 m / 1 deg`。该配置可以减少静止噪声，但会导致雷达静止或小幅移动时不处理新扫描，表现为障碍和射线不更新，因此已撤销，改用 `0.2 s` 的时间间隔限制处理频率。

### 12.4 漂移与显示说明

- FAST-LIO 本身没有回环检测，长时间运行会有位置和航向漂移。
- slam_toolbox 通过 `map -> camera_init` 修正全局地图定位，但不会反向修正 FAST-LIO 的原始 `/Odometry`。
- 长期定位应使用 `map -> body`，不要把 `/Odometry` 当作全局无漂移位姿。
- 建图开始后先让雷达静止 3 到 5 秒，再缓慢移动并尽量回到起点形成闭环。
- `active samplers with a different type` 是当前虚拟机 OpenGL 的 RViz 警告，不代表地图数据停止。
- `/map_updates` 没有发布者不代表地图不更新；当前 slam_toolbox 发布完整 `/map`，实测约 `2 Hz`。

### 12.5 尚存问题

- FAST-LIO 偶发输出 `No point, skip this scan` 或 `Too few input point cloud`，点云切片稀疏时更明显。
- slam_toolbox 偶发 `Message Filter ... queue is full`，表示处理速度短时落后于输入速度。
- Livox 驱动停止时可能以 `exit code -11` 退出；确认 SDK 已打印 `Deinit completely` 后，未发现残留进程。
- slam_toolbox 停止时偶发 Karto 异常退出，但运行期间生成的 ROS 话题和地图不受影响。
- 本次最后一轮地图没有单独保存。

### 12.6 停止状态与后续目录

本次结束时，Livox、FAST-LIO、pointcloud_to_laserscan、slam_toolbox 和 RViz 均已停止，无残留建图进程。雷达仍保持网络在线。

后续工作目录已切换到：

```text
/home/hu/Desktop/bxi/bxi_rc_slam
```
