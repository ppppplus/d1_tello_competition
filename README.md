# D1 + Tello 空地协同教学仿真（ROS 2）

包含 D1 EDU 机器狗、发射器、投放仓、无人机、两层地图和预训练步态。
机器狗可平地行走、原地转向并连续上 8 级楼梯；演示完成 1 次打靶、2 次投放及集合，无人机最后返回降落。
**无需训练、PyTorch 或 CUDA。** 源码、地图、模型与推理权重均在本包内，运行时不下载模型。

## 1. 安装与编译

适用 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress（ign gazebo / Gazebo 6）**。
先按 [ROS 2 官方说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装 Humble，打开未加载 ROS 1 的终端：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions \
  ignition-fortress libignition-gazebo6-dev libeigen3-dev nlohmann-json3-dev \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-rosgraph-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-launch-ros ros-humble-ament-index-python

mkdir -p ~/d1_ws/src
# 将收到的压缩包放入当前目录；解压后为 src/d1_tello_competition
tar -xzf d1_tello_competition_student_20261019.tar.gz -C ~/d1_ws/src
cd ~/d1_ws
colcon build --packages-select d1_tello_competition --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果找不到 Fortress 安装包，按 [Gazebo 官方安装说明](https://gazebosim.org/docs/fortress/install_ubuntu/) 添加软件源后重试。
Humble 与 Fortress 的配套关系见 [官方兼容说明](https://gazebosim.org/docs/fortress/ros_installation/)。本包不适用于 Gazebo Classic 11 或直接替换为 Harmonic。

## 2. 一键演示

```bash
source ~/d1_ws/install/setup.bash
ros2 launch d1_tello_competition competition.launch.py
```

自动播放完整比赛，约 5 分钟仿真时间。窗口中可拖动、缩放查看场景；Ctrl+C 退出。
场地为 12 × 10.4 m，二层高 0.8 m，楼梯高 0.1 m、踏面 0.3 m。
六个候选位置中固定启用 1 号红靶、4/6 号绿投放点；此版本不随机抽取目标。

无窗口快速验收：

```bash
ros2 launch d1_tello_competition competition.launch.py \
  gui:=false rate:=0 exit_on_finish:=true
```

每次启动都会打印独立结果目录，默认在 `~/.ros/d1_tello_competition/runs/`。
`result.json` 的 `strict_pass: true` 表示完整任务通过；同时保留运行日志和轨迹。
可用 `output:=/你选择的/新目录` 指定位置，该目录必须尚不存在。重新启动即重置比赛。

## 3. 学生控制接口

终端 1 启动学生模式，机器狗站立等待外部命令，无人机仍自动巡航：

```bash
ros2 launch d1_tello_competition competition.launch.py mode:=student
```

终端 2 加载工作空间，然后运行最小示例（前进、转向、停止）：

```bash
source ~/d1_ws/install/setup.bash
ros2 run d1_tello_competition student_walk.py --ros-args -p use_sim_time:=true
```

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/d1/cmd_vel` | `geometry_msgs/msg/Twist` | 发布机身坐标速度：linear.x 前进、linear.y 左移、angular.z 左转 |
| `/d1/odom` | `nav_msgs/msg/Odometry` | 仿真真值位姿，world 坐标；速度为机身坐标 |
| `/d1/joint_states` | `sensor_msgs/msg/JointState` | 12 个关节的角度、速度、施加力矩 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟，学生节点设置 use_sim_time:=true |
| `/d1/fire` | `std_srvs/srv/Trigger` | 向同层 2 m 内最近的未完成红靶发射 |
| `/d1/drop` | `std_srvs/srv/Trigger` | 向同层 2 m 内最近的未完成绿目标投放，初始 3 件 |
| `/competition/state` | `std_msgs/msg/String` | JSON 状态：目标、巡航报告、任务结果、事件等 |

速度建议 20–50 Hz 持续发布；超过 **0.4 秒仿真时间**收不到新命令会停止。
已验证范围：平地前进 ≤ 0.30 m/s、原地转向 ≤ 0.50 rad/s、上楼前进 ≤ 0.22 m/s。
建议先停稳再原地转向，上楼时沿楼梯中心正向前进。侧移、后退、下楼和更高台阶不在已验收范围。

```bash
# 持续前进；Ctrl+C 停止发布后由超时保护停下
ros2 topic pub -r 20 /d1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'

# 到达作业点并对准后调用；终端输出 success 仅表示受理
ros2 service call /d1/fire std_srvs/srv/Trigger '{}'
ros2 service call /d1/drop std_srvs/srv/Trigger '{}'
ros2 topic echo /competition/state
```

动作受理后等待 5 秒仿真时间，再从状态确认是否成功；忙碌时拒绝重复动作。
打靶需要球体真实碰撞并使靶板倒下 ≥ 1 rad；投放需要物资接触指定楼层地面且落点误差 ≤ 0.2 m。
完成 3 个目标后，D1 到二层集合点 `(5, 6.6)`，停在 0.15 m 范围内，等待无人机降落，生成比赛结果；学生模式限时 900 秒仿真时间。

## 4. 可选相机与修改入口

```bash
ros2 launch d1_tello_competition competition.launch.py mode:=student cameras:=true
```

启用 `/competition/{overview,dog,uav,fpv}/image` 四路 ROS 2 图像，分别为总览、跟拍、无人机俯视、机器狗前视，20 Hz。
需要可用的图形环境与驱动；图形较慢时关闭相机，或 `gui:=false cameras:=false` 只运行物理仿真。
切勿同时启动两个同域仿真实例；多组同机实验可分别设置不同的 `ROS_DOMAIN_ID` 和 `IGN_PARTITION`。

- 学生算法入口：`examples/student_walk.py`，可自行新建 ROS 2 节点接入上述接口。
- 任务与演示航点：`config/mission.json`；几何地图：`worlds/competition.sdf.in`。
- 步态已冻结：`policy/actor.bin`，50 Hz 推理、1000 Hz 关节 PD；修改地图高度时须同步 `config/terrain.json` 的高度观测。
- 修改源码或资源后重新 `colcon build`，再加载 `install/setup.bash`；构建和安装目录不需手工编辑。

教学边界：载荷装置只保留外观，不增加机身质量、惯量和碰撞；发射后的球与投放后的物资有独立物理动力学。
步态使用仿真状态和已知地图高度；无人机使用预设航点与力控制，不含 PX4、SLAM 或图像识别算法。
无人机在分发版采用基础几何体外观，动力学保留。来源与许可见 [THIRD_PARTY.md](THIRD_PARTY.md)。
