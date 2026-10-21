# D1 + Tello 空地协同教学仿真（ROS 2）

包含 D1 EDU 机器狗、发射器、投放仓、无人机、两层地图和预训练步态。
无人机从楼外起飞，穿过红圆着火标记对应的窗户进入；首报目标后，机器狗从独立地面门进入。
机器狗完成平地行走、转向、8 级楼梯、1 个打靶任务、2 个投放任务（共 3 件物资）；两机最终在二层左上角的上下排列的小区域集合，无人机在蓝色区降落。
**无需训练、PyTorch 或 CUDA。** 源码、地图、模型与推理权重均在本包内，运行时不下载模型。

![楼外入口：黄色门框与两排六扇小窗，红圆标出本轮着火窗](docs/entry_facade.png)

[查看地图俯视图：半幅二层、一层及右侧楼梯](docs/map_top.png)

## 1. 安装与编译

适用 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress（ign gazebo / Gazebo 6）**。
先按 [ROS 2 官方说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装 Humble，打开未加载 ROS 1 的终端：

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
# 将收到的压缩包放入当前目录；解压后为 src/d1_tello_competition
tar -xzf d1_tello_competition_student_20261021_v2.0.1.tar.gz -C ~/d1_ws/src
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

自动播放完整比赛，约 5–7 分钟仿真时间，取决于随机任务布局。窗口中可拖动、缩放查看场景；Ctrl+C 退出。
灰色地面底板为 10.36 × 9.26 m（含门外起点区），上方和右侧与外墙外缘齐平；建筑内侧净尺寸约 10.12 × 7.52 m，上方及右侧外墙内面贴齐二层外缘。二层高 0.8 m、进深 4.0 m；楼梯嵌入二层右侧缺口，低端入口朝俯视图下侧，机器狗朝图中上方上楼。
二层范围为 Y=5.1–9.1 m；梯脚在 Y=5.1 m，高端转向平台在 Y=7.5–9.1 m。
8 级楼梯每级高 0.1 m、踏面 0.3 m、宽 1.4 m，保留上层转向平台。目标列横坐标为 2.5、4.7、6.9 m。
1–3 号目标移至 Y=4.3 m，4–6 号目标位于二层下侧 Y=6.5 m，开口朝图中上方。
1 号目标左边界 X=1.8225 m，在门洞右边界 X=1.7 m 以右；最右窗框边界 X=7.95 m，在楼梯左侧 X=8.8 m 以内。
六个候选位置每局随机启用三个，保证覆盖两层：一个红色打靶点、一个绿色单件投放点、一个蓝色双件投放点，其余三个为灰色无需处理。六点的侧面、靶板和投放区完全相同，只有朝上的标识面颜色不同。任务色使用相机可见性分层，仅无人机俯视和教师总览显示；机器狗相机始终看到灰色标牌，包括从二层俯看一层时。数字不再印在标识上，1–6 号按一楼从左到右、二楼从左到右的位置约定。
外立面位于 `Y=1.5 m`，机器狗与无人机起点均在楼外。黄色门框的门宽 1.4 m、高 1.05 m，无门槛；
六扇窗分为上下两排，每排三扇，中心横坐标均为 `X=3.2/5.4/7.6 m`。
下排 1–3 号窗中心高 `Z=1.3 m`，上排 4–6 号窗中心高 `Z=2.5 m`，净开口均为 **0.6 × 0.6 m**。
六扇窗的尺寸、窗框外形和蓝色材质完全相同，仅用红圆标识着火窗。机器狗门框为黄色，地板外围边线为黑色。
默认随机选择一个窗户并显示红圆着火标记，可用 `fire_window:=1` 至 `fire_window:=6` 切换。
模型尺寸保持原样：无人机含护圈碰撞外廓约 17.8 × 19.3 cm；缩窗后窗宽约为机宽的 3.1 倍。
选择其他着火窗会同时改变红圆位置、入楼航点和通行判定：

```bash
ros2 launch d1_tello_competition competition.launch.py fire_window:=1
```

仅从正确窗户穿入且机体完全进入才算无人机入楼；从错误窗、地面门、墙顶或其他外墙进入会判失败。
机器狗必须等待首个楼内目标报告，再通过地面门；擦碰外立面也会判失败。

无窗口快速验收：

```bash
ros2 launch d1_tello_competition competition.launch.py \
  gui:=false rate:=0 exit_on_finish:=true
```

每次启动都会打印独立结果目录，默认在 `~/.ros/d1_tello_competition/runs/`。
`result.json` 的 `strict_pass: true` 表示完整任务通过；同时保留运行日志和轨迹。
可用 `output:=/你选择的/新目录` 指定位置，该目录必须尚不存在。重新启动即重置比赛。

## 3. 学生控制接口

终端 1 启动学生模式，机器狗等待外部命令，无人机自动穿窗巡航。默认 `scout_mode:=external`，需要学生节点从无人机俯视图识别并报告任务；未收到报告时机器狗不能进门。只练机器狗控制可使用 `scout_mode:=reference`，由教师模拟俯视传感器在到达目标上方后报告。

```bash
ros2 launch d1_tello_competition competition.launch.py mode:=student scout_mode:=reference
```

终端 2 加载工作空间，然后运行最小示例（前进、转向、停止）：

```bash
source ~/d1_ws/install/setup.bash
ros2 run d1_tello_competition student_walk.py --ros-args -p use_sim_time:=true
```

完整进门衔接示例：在**新启动的学生模式**中用下面程序替代 `student_walk.py`。
程序等待无人机穿窗和首报，然后转向、穿门，停在门内；后续任务由学生自行接入。

```bash
ros2 run d1_tello_competition student_enter.py --ros-args -p use_sim_time:=true
```

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/d1/cmd_vel` | `geometry_msgs/msg/Twist` | 发布机身坐标速度：linear.x 前进、linear.y 左移、angular.z 左转 |
| `/d1/odom` | `nav_msgs/msg/Odometry` | 仿真真值位姿，world 坐标；速度为机身坐标 |
| `/d1/joint_states` | `sensor_msgs/msg/JointState` | 12 个关节的角度、速度、施加力矩 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟，学生节点设置 use_sim_time:=true |
| `/d1/fire` | `std_srvs/srv/Trigger` | 对同层 2 m 内最近点申请发射；须先有无人机报告 |
| `/d1/drop` | `std_srvs/srv/Trigger` | 对同层 2 m 内最近点申请投放；初始 3 件，蓝色点需成功投放两次 |
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
未完成进门时不受理发射或投放。状态 JSON 的 `entry` 给出门窗公开几何；正确窗号在无人机入楼后才公开。`stations` 是六个位置的公开几何，`targets` 仅包含已经收到的无人机报告，`reported` 是按位置排列的六个报告标志；
`uav_entered`、`d1_entered` 表示机体已完全进入，`uav_phase` 表示无人机当前阶段。
`uav_exited` 是兼容旧版的离楼记录；本版二楼集合时应为 false，`uav_landed` 为降落状态。
打靶需要球体真实碰撞并使靶板倒下 ≥ 1 rad；每件投放需要物资接触指定楼层地面且落点误差 ≤ 0.2 m。绿色点一件成功即完成，蓝色点必须两件分别成功才完成；重复上报不会补充物资或重复计分。
完成 3 个目标后，D1 到二层左上角黄色集合点 `(1.0, 8.0)`（俯视图上方，0.9 × 0.8 m），停在 0.15 m 范围内；
Tello 在下方蓝色集合点 `(1.0, 6.85)`（0.7 × 0.7 m） 降落，支撑面高 0.8 m。无人机必须实际接触指定平台并停稳，才记录降落；两机集合后生成比赛结果。
本版结束时保持在楼内，不要求穿窗返回楼外；学生模式限时 900 秒仿真时间。


## 随机规则与视觉报告（v2.0）

- 顶部红色：打靶；绿色：投放 1 件；蓝色：投放 2 件；灰色：不处理。
- 默认每次启动重新随机；`seed:=42` 可复现任务和窗号，`fire_window:=1` 可覆盖窗号用于调试。
- 窗户大小、楼梯、步态、作业精度不变；最新紧凑外墙、黑色地板边框、黄色门框及左侧上下集合区均已包含。
- `/uav/target_report` 接收 `std_msgs/msg/String` JSON，例如 `{"slot": 3, "marker_color": "blue"}`。slot 为 0–5；颜色为 red/green/blue/gray。必须已入楼、处于该标识上方水平 0.6 m 内且高于标识至少 0.6 m，报告才会被接收。
- `/uav/odom` 提供无人机世界位姿；`/competition/uav/image` 是俯视 RGB。参考颜色节点只读这两个接口及公开几何，不读任务真值。
- `targets` 中颜色是收到的识别结果，不代表裁判已确认识别正确；实际任务以物理动作结果判定。机载状态不会预先发布隐藏任务颜色。

视觉报告示例，终端 1：

```bash
ros2 launch d1_tello_competition competition.launch.py mode:=student cameras:=true camera_views:=uav seed:=42
```

终端 2：

```bash
ros2 run d1_tello_competition uav_color_report.py --ros-args -p use_sim_time:=true
```

终端 3 可运行 `student_enter.py` 或自己的机器狗控制器。颜色示例仅在无人机稳定飞到标识正上方时分类，适用于此课程固定地图，非通用视觉定位算法。随机窗号已实现，但无人机穿窗导航目前仍由教师控制器依据裁判配置执行；尚未实现自主着火窗视觉导航。

本地运行目录中的 `config.json`、`world.sdf`、`result.json` 是教师/裁判文件，含真值与随机种子，不得供参赛算法读取。当前是教学包，不提供操作系统或 Gazebo 真值接口的安全隔离；正式比赛需由裁判进程隔离这些文件和接口。

## 4. 可选相机与修改入口

```bash
ros2 launch d1_tello_competition competition.launch.py mode:=student cameras:=true
```

启用 `/competition/{overview,dog,uav,fpv,uav_front,entry}/image` 六路 ROS 2 图像，分别为总览、跟拍、无人机俯视、机器狗前视、无人机前视、楼外入口观察，默认 20 Hz。
可用 `camera_views:=entry,uav_front` 只开所需视角，`camera_fps:=30` 设置相机目标采样帧率。
需要可用的图形环境与驱动；图形较慢时关闭相机，或 `gui:=false cameras:=false` 只运行物理仿真。
切勿同时启动两个同域仿真实例；多组同机实验可分别设置不同的 `ROS_DOMAIN_ID` 和 `IGN_PARTITION`。

- 学生算法入口：`examples/student_walk.py` 和 `examples/student_enter.py`，可自行新建 ROS 2 节点接入上述接口。
- 起点与集合参数：`config/mission.json`；随机任务及对应演示路线：`scripts/scenario.py`；入口参数：`config/entry.json`，立面与进出航点由 `scripts/entry_scene.py` 同步构建；基础地图：`worlds/competition.sdf.in`。
- 步态已冻结：`policy/actor.bin`，50 Hz 推理、1000 Hz 关节 PD；修改地图高度时须同步 `config/terrain.json` 的高度观测。
- 修改源码或资源后重新 `colcon build`，再加载 `install/setup.bash`；构建和安装目录不需手工编辑。

教学边界：载荷装置只保留外观，不增加机身质量、惯量和碰撞；发射后的球与投放后的物资有独立物理动力学。
步态使用仿真状态和已知地图高度；无人机飞行使用预设航点与力控制，不含 PX4 或 SLAM。`reference` 报告使用受位置约束的模拟俯视传感器；`external` 报告由学生节点提供，附带一个简单 RGB 颜色识别示例。
红圆与所选窗户由场景配置关联，演示不声称从图像自主识别着火窗；新增前视图可供学生实现识别。
外墙有真实碰撞，3.9 m 处有仅碰撞的天花板，俯视教学画面保持可见；二层仅覆盖后侧部分区域，右侧留出楼梯缺口。
无人机在分发版采用基础几何体外观，动力学保留。来源与许可见 [THIRD_PARTY.md](THIRD_PARTY.md)。
本次布局与路线的验收见 [地图调整验收](docs/MAP_VALIDATION.md)。旧版穿窗返航流程的历史记录见 [v1.2 入楼验收](docs/ENTRY_VALIDATION.md)。
