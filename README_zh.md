
# LeRobot-Kinematics: 为Lerobot SO100机械臂提供的简洁精确正逆运动学示例

<p align="center">
  <a href="README.md">English</a> •
  <a href="README_zh.md">中文</a> 
</p>

## 声明

本项目在开发过程中参考并借鉴了以下开源项目，对其核心设计与实现有重要参考价值：
 - [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
 - [huggingface/lerobot](https://github.com/huggingface/lerobot)
 - [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
 - [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco)
 
我们衷心感谢这些项目对机器人技术社区做出的贡献。

## 最新动态

- **2025年8月14日**: 增加了用于 so101 新校准方式的运动学工具
- **2025年4月19日**: 新增中文文档

## A. 安装指南

推荐使用conda创建**python=3.10**环境以保持与[lerobot](https://github.com/huggingface/lerobot)的一致性

```bash
  # 若未创建lerobot虚拟环境
  # conda create -y -n lerobot python=3.10
  conda activate lerobot

  git clone https://github.com/box2ai-robotics/lerobot-kinematics.git
  cd lerobot-kinematics
  pip install -e .
```

## B. 仿真环境示例

启动后建议先用鼠标点击终端窗口再按键操作，避免[mujoco](https://github.com/google-deepmind/mujoco)场景配置被意外修改

#### (1) 关节角度控制

通过键盘控制机械臂各关节角度变化的示例，启动后将显示Mujoco可视化界面

```shell
python examples/lerobot_keycon_qpos.py
```

- ``1, 2, 3, 4, 5, 6`` 增加对应关节角度
- ``q, w, e, r, t, y`` 减少对应关节角度
- 长按'0'键复位

若遇到错误"GLFWError: (65543) b'GLX: Failed to create context: BadValue..."且系统为ubuntu 21.04，可能是默认使用集成显卡导致，请切换至独立显卡：

```shell
sudo prime-select nvidia
sudo reboot
```

#### (2) 末端位姿控制

通过键盘控制机械臂末端位姿变化的Mujoco示例

```shell
python examples/lerobot_keycon_gpos.py
```

| 按键 | 正向动作          | 按键 | 反向动作          |
|-----|------------------|-----|------------------|
| `w` | 前进              | `s` | 后退             |
| `a` | 右移              | `d` | 左移             |
| `r` | 上升              | `f` | 下降             |
| `e` | 横滚角增加         | `q` | 横滚角减少        |
| `t` | 俯仰角增加         | `g` | 俯仰角减少        |
| `z` | 夹爪打开          | `c` | 夹爪闭合          |

长按'0'键复位

#### (3) 手柄控制

使用游戏手柄控制Mujoco中机械臂的示例，需先安装[joycon-robotics](https://github.com/box2ai-robotics/joycon-robotics)

```shell
python examples/lerobot_joycon_gpos.py
```

## C. 实体机械臂示例

**重要提示**：使用实体机械臂前，需通过[lerobot-joycon](https://github.com/box2ai-robotics/lerobot-joycon)仓库完成端口绑定与校准，校准文件需复制至`/examples`目录（例如`lerobot-joycon/.cache/calibration/so100/main_follower.json`）

#### (1) 实体键盘控制

通过键盘同步控制仿真环境与实体Lerobot机械臂末端位姿

```shell
python examples/lerobot_keycon_gpos_real.py
```

此示例也可用于键盘控制的数据采集

#### (2) 实体手柄控制

```shell
python examples/lerobot_joycon_gpos_real.py
```

## D. 更多信息
1. 欢迎关注[B站视频账号-盒子桥](https://space.bilibili.com/122291348)
2. 加入QQ交流群：948755626

如果这个仓库对您有帮助，请给我们一颗小星星，祝您使用愉快！⭐ ⭐ ⭐ ⭐ ⭐