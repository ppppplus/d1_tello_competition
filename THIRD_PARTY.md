# 来源与许可

本包新增的 ROS 2 控制、场景与教学代码使用 Apache-2.0，见 LICENSE。
第三方资源保留各自许可；下表不将其统一重新授权。

| 资源 | 来源及版本 | 许可与通知 |
| --- | --- | --- |
| D1 EDU 外观与动力学描述 | [rl_sar_zoo](https://github.com/fan-ziqi/rl_sar_zoo)，提交 `7dd30bdc7806898950b354260655d5a7f0ce844e`；显示网格经过减面，动力学保留 | 上游 d1_description 的 package.xml 声明 Apache-2.0，副本见 licenses/d1_description.package.xml |
| 控制约定参考 | [rl_sar](https://github.com/fan-ziqi/rl_sar)，提交 `376d42c9b128f963ab08579762d5a216a976ce39` | licenses/Apache-2.0.txt；随包 actor.bin 是课程另行训练的策略，不是上游原权重 |
| 发射器 CAD 派生显示网格 | [autonomous-foam-launcher](https://github.com/noahfitzgerald/autonomous-foam-launcher)，提交 `853fafbf0a58a74d14dda0a3bc2a52c311350220`；Noah Fitzgerald、Kevin Jung、Aidan Schwing / Cal Poly ME405 | MIT，licenses/launcher-MIT.txt |
| 输送机 CAD 裁剪及减面网格 | [conveyor_demo](https://github.com/rokokoo/conveyor_demo)，提交 `59d8d987080ceb917de0d25737459291234419c5` | GPL-3.0，licenses/conveyor-GPL-3.0.txt；派生网格为 assets/meshes/943fd6720567_conveyor_head.stl |

无人机使用基础几何体外观，不分发许可未明确的历史 Tello 网格。
ROS 2 / Gazebo / Eigen 等通过系统包安装，不打包其二进制依赖。
地图由课程场景迁移；平台高度、楼梯、起点、任务位置与演示路线沿用已验证版本。
品牌名称用于说明仿真对象，本包不是厂商官方 SDK，也不用于实机控制。
