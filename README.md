
# LeRobot-Kinematics: Simple and Accurate Forward and Inverse Kinematics Examples for the Lerobot SO100 ARM

<p align="center">
  <a href="README.md">English</a> •
  <a href="README_zh.md">中文</a> 
</p>

## Declaration

This project is developed with great respect and reference to the following open-source projects, which have significantly influenced its core design and implementation:

 - [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
 - [huggingface/lerobot](https://github.com/huggingface/lerobot)
 - [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
 - [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco)

We sincerely appreciate the contributions of these projects to the robotics community.

## What's New

- **August 14, 2025**: Added kinematic tools for new calibration of so101 robot
- **April 19, 2025**: Added Chinese readme

## A. Installation

We recommend using conda for **python=3.10** installations to be consistent with [lerobot](https://github.com/huggingface/lerobot).

```bash
  # if no lerobot conda env
  # conda create -y -n lerobot python=3.10
  conda activate lerobot

  git clone https://github.com/box2ai-robotics/lerobot-kinematics.git
  cd lerobot-kinematics
  pip install -e .
```

## B. Examples in simulation

We recommended to click on the terminal window with the mouse after startup and then enter the keys, to avoid that the keys in the departure [mujoco](https://github.com/google-deepmind/mujoco) change the configuration of the scene.


#### (1) qpos control

Example of joint angle control, when opened the Mucojo visualization will appear and you can use the keyboard to control the corresponding angle change of the robot arm.

```shell
python examples/lerobot_keycon_qpos.py
```

- ``1, 2, 3, 4, 5, 6`` Increase the angle.

- ``q, w, e, r, t, y`` Decrease Angle.

Press and hold '0' to return to position

If you encounter error "GLFWError: (65543) b'GLX: Failed to create context: BadValue (integer paraneter out of range for operation)'
warnings.warn(nessage,GLFWError) the Mu ERROR: could not create window" and are using ubuntu 21.04, it may be because your computer is using integrated graphics by default and does not support mujoco visualization, please run the following command to switch to discrete graphics.

```shell
sudo prime-select nvidia
sudo reboot
```

#### (2) gpos Control

Example of Gripper Posture (gpos) control, where you can use the keyboard to control the end-posture changes of Lerobot in mucojo.

```shell
python examples/lerobot_keycon_gpos.py

# or for SO101 new calibration
python examples/lerobot_keycon_gpos_101.py
```

| Key | Action +            | Key | Action -            |
|-----|---------------------|-----|---------------------|
| `w` | Move Forward        | `s` | Move Backward       |
| `a` | Move Right          | `d` | Move Left           |
| `r` | Move Up             | `f` | Move Down           |
| `e` | Roll +              | `q` | Roll -              |
| `t` | Pitch +             | `g` | Pitch -             |
| `z` | Gripper Open        | `c` | Gripper Close       |

Press and hold '0' to return to position


#### (3) Joycon Control

This is an example of using joycon to control Lerobot in mucojo, if you want to use it, please install [joycon-robotics
](https://github.com/box2ai-robotics/joycon-robotics) repository first!

```shell
python examples/lerobot_joycon_gpos.py
```

<!-- #### (4) Genesis IK Control

Example of Gripper Posture (gpos) control based on the Genesis positive inverse kinematics library.

First, if you want try this, you need install the genesis repo:
```shell
pip install genesis-world
```

```shell
python examples/lerobot_genesis.py
``` -->

## C. Examples in Real

**NOTE:** Before using the robotic arm, you must first bind and calibrate the robotic arm ports using the [lerobot-joycon](https://github.com/box2ai-robotics/lerobot-joycon) repository. After completing the calibration, copy the resulting calibration file to the `/examples` directory. For example, you can find the calibration file at `lerobot-joycon/.cache/calibration/so100/main_follower.json`.


#### (1) Keyboard Control in Real

Example of gripper posture (gpos) control, where you can use the keyboard to control the Lerobot's end posture changes in mucojo while going from simulation to physical control of a real Lerobot arm.

```shell
python examples/lerobot_keycon_gpos_real.py
```

If you're interested in this, you can try using the keyboard to collect data.

#### (2) Joycon Control in Real

```shell
python examples/lerobot_joycon_gpos_real.py
```

## C. More Info
1. You can follow the [bilibili video account - Box2AI](https://space.bilibili.com/122291348)
2. Join the discussion on the QQ group: 948755626

If this repository was helpful to you, please give us a little star and have a great time! ⭐ ⭐ ⭐ ⭐ ⭐