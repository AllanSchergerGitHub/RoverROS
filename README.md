# RoverROS

A repository containing the source code of a Rover with a 5DOF manipulator.

My user name in github 'mturktest123' is a carryover from my days working on crowdsourcing via AWS Mechanical Turk.

This project includes the development of the robot to be simulated in _Gazebo_.

It uses `MoveIT` to trajectory control the manipulator.

## Dependancies:

You will need to install the following:

  ROS Melodic
  
  `sudo apt-get install ros-melodic-ackermann-msgs`

  `sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`


## A detailed description of various packages is as follows.

## rover_arm_resources_pkg

This package contains the xacro/urdf used by the _rover_ and _arm_ descriptions for materials, links & joints macros, and some common constants used in defining the robot models.

## rover_desc_pkg

This package contains the urdf/xacro description of the __rover__. The robot model includes the _rover chassis_, _steers_, and _wheels_. The model is deliberately kept simple to have a lower computation load in simulation.

The following command shall be used to spawn the __Rover__ model in _RViz_.

  `roslaunch rover_desc_pkg rover_rviz.launch`

This command launches _RViz_ with __joint_state_publisher__ that allows us to play with various _steers_ and _wheels_ joints.

To understand how the command works, take a look at this [video on youtube.](https://www.youtube.com/watch?v=HbfHDPkXFZw)

## rover_arm_desc_pkg

This package contains the urdf/xacro description of the __5DOF manipulator arm__. The _arm_ model includes 5 _revolure_ joints.

The following command shall be used to spawn the  __Arm__ in _RViz_.

  `roslaunch rover_arm_desc_pkg arm_rviz.launch`

This command launches _RViz_ with __joint_state_publisher__ that allows us to play with various joints of the _arm_.

## rover_with_arm_desc_pkg

This package combines the description of _rover_ and _arm_ to allow the spawning of complete _rover_ assembly.

The following command shall be used to spawn the __rover_with_arm__ in _RViz_.

  `roslaunch rover_with_arm_desc_pkg rover_arm_rviz.launch`

This command launches _RViz_ with __joint_state_publisher__ that allows us to play with various joints of the _rover_ and the _arm_.

To understand how the command works, take a look at this [video on youtube.](https://www.youtube.com/watch?v=YDhvmKc58bs&t=5s)

## rover_gazebo_control

This package simulates the _rover_ in the _gazebo_ simulation world. It spawns only the _rover_ in _gazebo_.

Following commands shall be used to control the _rover_ in simulation as described under:

  1. Use this command to spawn the _rover_ in empty _gazebo_ world:

      `roslaunch rover_gazebo_control rover_gazebo_control.launch`

      Following command launches a __rqt_gui__ to control _rover_ joints:

        `rosrun rqt_gui rqt_gui`

      Watch this [video on youtube](https://www.youtube.com/watch?v=vkbC5o6LF9M&t=44s) to understand how _rover_ joints can be played.

  2. Use this command to spawn the _rover_ in empty _gazebo_ world and be controlled using keyboard commands:

      `roslaunch rover_gazebo_control rover_gazebo_keyboard_ctrl.launch`

      This command also brings up a node to take in the keyboard commands and publish velocity commands to the joints.

      The `up` and `down` arrow keys are used to run the _rover_ in `forward` and `reverse` directions respectively.

      The `left` and `right` arrow keys are used for _steering control_ of the _rover_.

      This [youtube video](https://www.youtube.com/watch?v=CBOU8EzkTOA&t=23s) shows how the _rover_ is controlled using keyboard inputs.

## rover_arm_gazebo_control

This package spawns the _arm_ in the _gazebo_ simulation world. It spawns only the _arm_ in _gazebo_.

The following command shall be used to spawn the _arm_.

  `roslaunch rover_arm_gazebo_control rover_arm_gazebo_control.launch`

## rover_with_arm_gazebo_control

This package spawns the _rover_ and the _arm_ in the _gazebo_ simulation world.

The following commands shall be used to have various controls of the _rover_ and _arm_ in the _gazebo_ simulation world.

  1. Use this command to simply launch the _rover_ and the _arm_ in _gazebo_.

      `roslaunch rover_with_arm_gazebo_control rover_with_arm_gazebo_control.launch`

  2. Use this command to launch the _rover_ and the _arm_ in _gazebo_ simulation and to control __only__ the _rover_ using keyboard inputs.

      `roslaunch rover_with_arm_gazebo_control rover_with_arm_gazebo_keyboard_ctrl.launch`

  3. Use this command to launch the _rover_ and the _arm_ in _gazebo_ simulation and to control __only__ the _arm_ using `MoveIT` trajectory control.

      `roslaunch rover_with_arm_gazebo_control rover_with_arm_gazebo_moveit_control.launch`

      Please refer to the description under the next package to launch the `MoveIT` control of the arm.

      You may watch this [video on youtube](https://www.youtube.com/watch?v=0F9IiBc68Lo&t=6s) to understand how the command can be used with `MoveIT` package.

  4. Use this command to launch the _rover_ and the _arm_ in _gazebo_ simulation with `nodes` to control the _rover_ using keyboard inputs and the _arm_ using `MoveIT` trajectory control.

      `roslaunch rover_with_arm_gazebo_control rover_with_arm_gazebo_moveit_keyboard_control.launch`

      Please refer to the description under the next package to launch the `MoveIT` control of the arm.

## rover_with_arm_moveit_config

This package defines the `MoveIT` configuration of the _arm_ joints. The joints are controlled using __joint_trajectory_controller__. Some random `cartesian` positions are defined for the _arm_.

Follow these commands to test or move the _arm_ in `cartesian` control.

  1. This command launches the _arm_ in _RViz_ to `plan` and `execute` fake controllers for the testing of the configuration.

      `roslaunch rover_with_arm_moveit_config demo.launch`

  2. Use this command to control the _arm_ in simulation mode such as in _gazebo_ as mentioned in the above package description.

      `roslaunch rover_with_arm_moveit_config moveit_planning_execution.launch`

      This [youtube video](https://www.youtube.com/watch?v=0F9IiBc68Lo&t=6s) explains how this command can be used to control the _arm_ in _gazebo_ environment.
