# 仿真与裁判包

本包为学生本机开发提供 Gazebo 仿真、D1 官方外观、载荷装置、冻结步态、无人机速度与姿态稳定控制、随机任务及裁判判定。
安装、双包构建与运行方法见 [仓库 README](../README.md)，比赛要求见 [规则文档](../d1_competition_client/docs/COMPETITION_RULES.md)。

本包不包含自动比赛解法：没有机器狗参考路线、无人机穿窗/巡航航点、模拟颜色报告或完整作业示例。控制输入由选手包发送；不传命令时等待，不会自动完成比赛。

## 仿真选项

- `gui:=false` 关闭界面；`cameras:=true` 开启相机。
- `camera_views:=uav,uav_front,fpv` 仅桥接选手机载图像；`camera_fps:=20` 设置采样率。
- `seed:=42` 固定任务随机种子；`fire_window:=1` 至 `6` 用于单窗调试。
- `max_sim_time:=10` 在指定仿真时间停止；停止启动检查不表示比赛通过。
- `exit_on_finish:=true` 在裁判写入结果后退出；完整通过看 `result.json` 的 `strict_pass`。
- `rate:=0` 尽可能快地运行，默认 `rate:=1`。

默认结果保存在 `~/.ros/d1_tello_competition/runs/`。目录中的配置、世界文件和结果含裁判真值，不能作为选手算法输入。

## 底层控制与模型

D1 使用冻结策略 `policy/actor.bin`，50 Hz 推理、1000 Hz 关节 PD。输入 `/d1/cmd_vel` 为机身坐标速度；不提供自动路径规划。外观采用官方 STEP 派生网格，物理碰撞、质量和惯量保留已验证模型。载荷装置仅显示，发射后的球和投放后的物资有独立物理动力学。

无人机 `/uav/cmd_vel` 接收世界坐标速度及偏航角速度，底层执行速度跟踪、重力补偿和姿态稳定；无自动起飞航线、识别或巡航。无指令时目标速度为零，不是位置锁定控制。

来源许可见 [THIRD_PARTY.md](THIRD_PARTY.md)。
