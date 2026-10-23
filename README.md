# D1 + Tello 比赛：选手端与组织方分离

本分支将原教学包拆成两个可独立构建的 ROS 2 包，比赛规则见 [规则文档](contestant/d1_competition_client/docs/COMPETITION_RULES.md)。

| 目录 | 发放对象 | 内容 |
| --- | --- | --- |
| [contestant/d1_competition_client](contestant/d1_competition_client/README.md) | 选手 | ROS 2 接口、零速度节点骨架、规则与场地图 |
| [organizer/d1_tello_competition](organizer/d1_tello_competition/README.md) | 仅组织方 | 仿真、模型、冻结步态、裁判真值与判定、完整参考路线和示例 |

## 发放选手包

```bash
python3 tools/export_contestant.py --output /tmp/d1_contestant.tar.gz
```

只发放生成的压缩包，不向选手发放整个仓库或本分支。导出使用明确文件清单，不包含 `.git`、历史记录、参考解法、仿真配置或权重。该分支仍保留历史完整代码；此前公开的代码无法通过这次拆分撤回。

## 组织方构建与运行

```bash
source /opt/ros/humble/setup.bash
# 在本仓库根目录执行；系统依赖见组织方包 README
colcon build --base-paths organizer --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=171
ros2 launch d1_tello_competition competition.launch.py \
  mode:=student scout_mode:=external cameras:=true camera_views:=uav,uav_front,fpv
```

组织方进程持有仿真和冻结步态，选手按选手包 README 连接同一 ROS 域。启动默认是学生模式、外部视觉报告，不会驱动机器狗执行完整参考任务。无人机预设巡航保持原版行为，目前没有选手无人机飞行命令接口。

组织方内部完整回归：

```bash
ros2 launch d1_tello_competition competition.launch.py \
  mode:=demo scout_mode:=reference gui:=false rate:=0 seed:=0 exit_on_finish:=true
```

## 部署边界

目录拆分和发包裁剪不等同于运行时安全隔离。正式部署需将组织方文件、Gazebo Transport 与教师相机隔离在组织方机器或容器中，只开放允许的 ROS 2 接口；同一个未限制的 DDS 域不能阻止选手访问其他发布话题或冒充发布者。当前分支未实现 SROS2 权限策略或网络网关。

本次拆分的验收记录见 [拆分验证](docs/SPLIT_VALIDATION.md)。
