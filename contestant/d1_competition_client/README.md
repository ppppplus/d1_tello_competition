# D1 + Tello 比赛选手包

本包仅提供 ROS 2 通信骨架和接口说明，不含视觉识别、导航路线、进门、任务规划或完整比赛参考解法。
比赛流程与判定统一见 [比赛规则](docs/COMPETITION_RULES.md)。

## 安装与启动

环境：Ubuntu 22.04、ROS 2 Humble。仿真与冻结步态由组织方服务器运行，本包不提供独立仿真器，不需要 GPU 或训练框架。

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions ros-humble-ament-cmake \
  ros-humble-rclpy ros-humble-geometry-msgs ros-humble-nav-msgs \
  ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-std-srvs
mkdir -p ~/d1_ws/src
# 将收到的 d1_competition_client 目录放入 ~/d1_ws/src/
cd ~/d1_ws
colcon build --packages-select d1_competition_client
source install/setup.bash
# 按组织方给定的域号设置；同网段且 DDS 网络可达
export ROS_DOMAIN_ID=171
ros2 run d1_competition_client contestant_node.py --ros-args -p use_sim_time:=true
```

收到里程计和比赛状态后，节点分别打印连接成功。默认持续发布零速度，不会自主运动或完成任务。选手自行实现 `scripts/contestant_node.py`，也可使用其他 ROS 2 节点。

## 接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟，节点使用 `use_sim_time:=true` |
| `/d1/cmd_vel` | `geometry_msgs/msg/Twist` | 机身坐标：x 前进、y 左移、angular.z 左转 |
| `/d1/odom` | `nav_msgs/msg/Odometry` | world 位姿，机身坐标速度 |
| `/d1/joint_states` | `sensor_msgs/msg/JointState` | 关节状态 |
| `/uav/odom` | `nav_msgs/msg/Odometry` | 无人机世界位姿 |
| `/competition/uav/image` | `sensor_msgs/msg/Image` | 无人机俯视 RGB |
| `/competition/uav_front/image` | `sensor_msgs/msg/Image` | 无人机前视 RGB |
| `/competition/fpv/image` | `sensor_msgs/msg/Image` | 机器狗前视 RGB |
| `/uav/target_report` | `std_msgs/msg/String` | JSON，例如 `{"slot":3,"marker_color":"green"}`；slot 为 0–5，颜色为 red/green/gray |
| `/d1/fire`、`/d1/drop` | `std_srvs/srv/Trigger` | 异步动作请求；success 表示受理，结果见比赛状态 |
| `/competition/state` | `std_msgs/msg/String` | JSON，公开几何、已收到的报告和动作事件 |

图像订阅使用 `rclpy.qos.qos_profile_sensor_data`（Best Effort），以匹配相机发布端。

`entry` 和 `stations` 为公开场地几何；`targets` 为收到的报告；`uav_entered`、`d1_entered`、`uav_landed` 表示通行和降落状态；`action_busy` 表示动作执行中。

速度建议以 20–50 Hz 发布，0.4 秒仿真时间未收到新命令则停止。已验证控制范围：前进 ≤ 0.30 m/s、转向 ≤ 0.50 rad/s、上楼前进 ≤ 0.22 m/s。建议先停稳再原地转向；侧移、后退和下楼不在已验收范围。这些是底层控制能力说明，比赛要求见规则文档。

## 当前比赛接口范围

冻结步态在组织方仿真进程内执行，选手通过速度命令控制机器狗，不需要获得或训练底层权重。无人机穿窗巡航目前仍由组织方预设控制器执行，尚未提供选手无人机飞行指令接口；选手负责基于图像报告任务，并实现机器狗规划与作业。请勿将本版理解为已经支持选手自主控制两台机器人的飞行与行走。
