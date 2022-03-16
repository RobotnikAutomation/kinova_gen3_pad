# kinova_gen3_pad

Package containing [robotnik_pad](http://www.github.com/RobotnikAutomation/robotnik_pad) plugins for teleoperating Kinova Gen3 arms.

## Installation

The package depends on some Robotnik packages:

- robotnik_pad [🔗](https://github.com/RobotnikAutomation/robotnik_pad/)
```bash
git clone https://github.com/RobotnikAutomation/rcomponent
```

This package also depends on kortex driver:

- ros_kortex [🔗](https://github.com/RobotnikAutomation/ros_kortex)
```bash
git clone https://github.com/RobotnikAutomation/ros_kortex.git
```

## Parameters

This an example of a config file loading the kinova gen3 plugin:

```yaml
KinovaArmGen3:
  type: kinova_gen3_pad_plugin/KinovaArmGen3
  arm_control_topic_name: /robot/arm/in/cartesian_velocity
  gripper_topic_name: /robot/arm/robotiq_2f_85_gripper_controller/gripper_cmd/goal
  set_home_service_name: /robot/arm/in/home_arm
  axis_topic_name: /robot/arm/in/joint_velocity
  joint_state_topic_name: /robot/arm/base_feedback/joint_state
  

  max_linear_speed: 2.0
  max_angular_speed: 3.0
  config:
    button_deadman: 4
    button_movement_deadman: 5 # this plugin will not work if movement deadman button is pressed
    button_home_arm: 12

    axis_open_gripper: 4
    axis_close_gripper: 3

    axis_linear_x_ee: 1
    axis_linear_y_ee: 0
    axis_linear_z_ee: 5

    axis_angular_x_ee: 1
    axis_angular_y_ee: 0
    axis_angular_z_ee: 2

    button_speed_up: 3
    button_speed_down: 1

    axis_speed: 10

    button_switch_mode: 8 
    axis_inc_axis: 9
```


