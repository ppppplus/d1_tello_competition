# D1 + Tello 比赛开发环境（ROS 2）

学生可以直接 clone 本分支，在本机编译、启动仿真并开发参赛算法。当前文件树不包含完整参考解法、自动任务路线或颜色识别示例。

| ROS 2 包 | 目录 | 内容 |
| --- | --- | --- |
| `d1_tello_competition` | [d1_tello_competition](d1_tello_competition/README.md) | Gazebo、模型、冻结步态、无人机底层控制、随机场景、裁判真值与判定 |
| `d1_competition_client` | [d1_competition_client](d1_competition_client/README.md) | ROS 2 接口、零速度节点骨架、规则与场地图 |

比赛要求统一见 [比赛规则](d1_competition_client/docs/COMPETITION_RULES.md)。

## 安装与编译

环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress。先安装 ROS 2 Humble 和 Gazebo Fortress 软件源，再安装依赖：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions python3-numpy \
  ignition-fortress libignition-gazebo6-dev libeigen3-dev nlohmann-json3-dev \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-rosgraph-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-launch-ros ros-humble-ament-index-python
mkdir -p ~/d1_ws/src
cd ~/d1_ws/src
git clone --single-branch --depth 1 -b competition/contestant-organizer-split \
  https://github.com/ppppplus/d1_tello_competition.git
cd ~/d1_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

模型与冻结权重随包安装，不需要 PyTorch、CUDA 或在线下载模型。

## 启动仿真

终端 1：

```bash
source ~/d1_ws/install/setup.bash
ros2 launch d1_tello_competition competition.launch.py \
  cameras:=true camera_views:=uav,uav_front,fpv
```

两台机器人初始等待选手命令。无人机不会自动穿窗、巡航或报告目标；机器狗不会自动进门、上楼或作业。无人机保留速度跟踪和姿态稳定，机器狗保留冻结的底层步态。

终端 2：

```bash
source ~/d1_ws/install/setup.bash
ros2 run d1_competition_client contestant_node.py --ros-args -p use_sim_time:=true
```

骨架只发布两台机器人的零速度，并订阅里程计和状态。学生自行加入识别、飞行控制、任务规划、机器狗导航和作业逻辑。接口说明见选手包 README。

无界面物理启动检查：

```bash
ros2 launch d1_tello_competition competition.launch.py \
  gui:=false cameras:=false max_sim_time:=10
```

## 开发与验收

`seed:=42` 可复现场景；默认每次随机。`output:=/绝对路径/新目录` 指定结果目录，目录必须尚不存在。每次重启重置比赛；修改程序后重新编译并加载工作空间。

不再提供 `mode:=demo`、`scout_mode:=reference` 或一键完成比赛入口。当前本机开发版同时包含裁判配置和真值，规则禁止参赛算法读取这些真值文件；本地文件不是安全隔离。正式比赛应由组织方隔离裁判进程和真值接口。

本分支当前文件已移除参考解法，但旧提交和其他分支仍可能含有历史参考程序；浅克隆只减少默认下载的历史，不能撤回此前公开的代码。

验证范围及结果见 [验证记录](docs/SPLIT_VALIDATION.md)。
