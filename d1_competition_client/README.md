# D1 + Tello 比赛选手包

本包仅提供 ROS 2 通信骨架和接口说明，不含视觉识别、导航路线、进门、任务规划或完整比赛参考解法。
比赛流程与判定统一见 [比赛规则](docs/COMPETITION_RULES.md)。

## 安装与启动

两个包都在本仓库，学生可在本机运行完整仿真。克隆、依赖和编译方法见 [仓库 README](../README.md)。

启动仿真后运行：

```bash
source ~/d1_ws/install/setup.bash
ros2 run d1_competition_client contestant_node.py --ros-args -p use_sim_time:=true
```

默认只发布两台机器人的零速度，不会自主执行任务。选手自行实现 `scripts/contestant_node.py`，也可使用其他 ROS 2 节点。

## 接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟，节点使用 `use_sim_time:=true` |
| `/d1/cmd_vel` | `geometry_msgs/msg/Twist` | 机身坐标：x 前进、y 左移、angular.z 左转 |
| `/d1/odom` | `nav_msgs/msg/Odometry` | world 位姿，机身坐标速度 |
| `/d1/joint_states` | `sensor_msgs/msg/JointState` | 关节状态 |
| `/uav/cmd_vel` | `geometry_msgs/msg/Twist` | 世界坐标速度、偏航角速度 |
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

## 无人机控制约定

`/uav/cmd_vel` 为 `geometry_msgs/msg/Twist`：`linear.x/y/z` 是 **world 坐标系**速度（m/s），`angular.z` 是偏航角速度（rad/s）。初始朝向为 +Y。水平各轴限幅 ±0.8 m/s，垂直 ±0.45 m/s，偏航 ±0.8 rad/s；超过 0.4 秒仿真时间未收到指令则目标速度和偏航角速度归零。零速度表示速度阻尼悬停，不锁定位置。靠近地面下降会产生真实接触，裁判依规则判断降落。

无人机不会自动起飞或巡航，学生需要自行控制穿窗与搜索。完成任务后应自行飞往集合区下降并停稳。底层稳定器和冻结步态属于仿真能力，不包含任务解法。
