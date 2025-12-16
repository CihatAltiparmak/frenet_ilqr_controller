# Frenet Iterative Linear Quadratic Regulator Controller

> [!IMPORTANT]  
> When in doubt, flat out.
> ~ Colin McRae

[Screencast from 09-21-2025 10:10:19 PM.webm](https://gist.github.com/user-attachments/assets/3724fa44-ab6b-4c8b-8fb6-a4a16812625e)

# Overview

This is local trajectory planner that generates trajectories using the method in paper named [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://ieeexplore.ieee.org/document/5509799) and tracks this generated trajectory using Iterative Linear Quadratic Regulator Method.

In its structure, it has some policy implementations such as `ObstaclePolicy`, which eliminates unfeasible trajectories right before selecting best trajectory, 
and cost implementations such as Lateral Distance Cost Checker and Longtitutal Velocity Cost Checker, 
which is used for selecting best trajectory to be given for Iterative Linear Quadratic Regulator Trajectory Tracker to find optimal inputs. This local planner consists of 2 modules, which is `ilqr_trajectory_tracker`, `frenet_trajectory_planner`.

For trajectory generation phase, `frenet_trajectory_planner` is used for generating optimal trajectory. This trajectory generation is done by planning longtitutal velocity and lateral distance separately. 
For longtitutal velocity planning, Quartic Polynomial Trajectory Planning generates some velocity trajectory plans in Frenet frame, which the user also can set how many velocity plan can be produced.
For lateral distance planning, Quintic Polynomial Trajectory Planning generates some lateral distance plans in Frenet Frame, which the user also can set how many lateral distance plan can be produced. 

Then using cartesian product of two set, one of them is the set of lateral distance plans and other one is the set of longtitutal velocity plans, trajectory in Frenet frame, which gives where robot is in type of arclength of line and the distance from curve in that point, is produced and 
this trajectory in Frenet frame can be converted to cartesian trajectory, which consists of x, y cartesian points and its first and second derivatives, later. After generating all trajectories, the best frenet trajectory inside these trajectories is selected using `policies` and `costcheckers` and converted to Cartesian trajectory.
Finally this generated best trajectory is tracked by Iterative Linear Quadratic Regulator.

# How to run

Clone repository
```sh
git clone https://github.com/CihatAltiparmak/frenet_ilqr_controller.git -b humble
```

Build dockerfile by going to the directory of cloned repository.
```sh
cd frenet_ilqr_controller
docker build -t frenet_ilqr_controller_demo .
```

Run below command not to fail gui works.
```sh
xhost +
```

Open two terminals and start gazebo simulation from the first terminal like below and navigation2 from the second terminal. (you should continue from turtlebot3_navigation2 package because of navigation2's turtlebot3 sdf file is outdated and causes odom not handling cmd_vel commands well, who knows, maybe this repo's bug or navigation2's bug :/)
```sh
docker run -it --gpus=all -e NVIDIA_DRIVER_CAPABILITIES=all --runtime=nvidia --rm --net=host --name frenet_ilqr_controller_demo --privileged --volume="${XAUTHORITY}:/root/.Xauthority" --env="DISPLAY=$DISPLAY" -v="/tmp/.gazebo/:/root/.gazebo/" -v /tmp/.X11-unix:/tmp/.X11-unix:rw --shm-size=1000mb ros-navigation/nav2_docker:humble
(inside docker) source /opt/ros/humble/setup.bash
(inside docker) source install/setup.bash
(inside docker) ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```sh
docker exec -it frenet_ilqr_controller_demo bash
(inside docker) source /opt/ros/humble/setup.bash
(inside docker) source install/setup.bash
(inside docker) ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True params_file:=/root/nav2_ws/src/navigation2/nav2_bringup/params/nav2_param_frenet_ilqr_controller_demo.yaml
```

# Configurations

### Controller

| Parameter                  | Type         | Definition                                                                              |
| ---------------------      | ------------ | --------------------------------------------------------------------------------------- |
| time_discretization        | double       | Default: 0.05 (s). time interval (s) between the states in ILQR and Frenet trajectory planning. Used for euler discretization|
| cost_checker_plugins       | string array | Default: None. Cost Checker (plugins) names                                             |
| policy_plugins             | string array | Default: None. Policy (plugins) names                                                    |

 #### Frenet Trajectory Planner

| Parameter                  | Type   | Definition                                                                              |
| ---------------------      | -------| --------------------------------------------------------------------------------------- |
| min_lateral_distance       | double | Default: -0.5. Minimum lateral distance along interpolated curve. |
| max_lateral_distance       | double | Default: +0.5. Maximum lateral distance along interpolated curve. |
| step_lateral_distance      | double | Default: +0.05.  Increasing rate for producing lateral distance trajectories in Frenet Frame |
| min_longitital_velocity    | double | Default:  0.0. Minimum longitutal velocity along interpolated curve. |
| max_longitital_velocity    | double | Default: +0.5. Maximum longitutal velocity along interpolated curve. |
| step_lateral_distance      | double | Default: +0.05.  Increasing rate for producing longtitutal velocity trajectories in Frenet Frame |
| time_interval              | double | Default: 1.5.  time (s) required to achieve corresponding frenet state. Used by polynomial trajectory planning for both velocity planning and lateral distance planning, (e.g time (s) required to increase speed from 1.0 m/s to 2 m/s)|
| max_state_in_trajectory    | int | Default: 40.  the number of how many state the generated trajectory can have at most.  |

#### Iterative Linear Quadratic Regulator

| Parameter                  | Type         | Definition                                                                              |
| ---------------------      | -------------| --------------------------------------------------------------------------------------- |
| iteration_number           | int          | Default: 20. Maximum iteration number of newton optimizer. |
| alpha                      | double       | Default: 1.0. Line search relavant parameter |
| input_limits_min           | double array | Default: [-2.5, -1.0]. Minimum values the input vectors should be during optimization. The default parameters are set for a differential drive robot, which means a robot's minimum linear acceleration will be -2.5 and its angular velocity will be -1.0|
| input_limits_max           | double array | Default: [2.5, 1.5]. Maximum values the input vectors should be during optimization. The default parameters are set for a differential drive robot, which means a robot's maximum linear acceleration will be +2.5 and its maximum angular velocity will be +1.0|

#### Lateral Distance Cost Checker
| Parameter                  | Type   | Definition |
| ---------------------      | -------| ------------------------------- |
| K_lateral_distance         | double | Default: 5.0. Cost Coefficient for punishing lateral distance along curve |

#### Longtitutal Velocity Cost Checker
| Parameter                  | Type   | Definition |
| ---------------------      | -------| ------------------------------- |
| desired velocity           | double | Default: 0.5. desired velocity for velocity keeping during trajectory execution |
| K_longtitutal_velocity    | double | Default: 10.0. Cost Coefficient in order to keep longitutal velocity in desired longitutal velocity along curve |

#### Obstacle Policy

Eliminates the frenet trajectories that collides any obstacle using costmap.

### XML configuration example

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 40.0
    FollowPath:
      plugin: "nav2_frenet_ilqr_controller::FrenetILQRController"
      time_discretization: 0.05
      frenet_trajectory_planner:
        min_lateral_distance: -0.5
        max_lateral_distance: 0.5
        step_lateral_distance: 0.05
        min_longtitutal_velocity: 0.0
        max_longtitutal_velocity: 0.5
        step_longtitutal_velocity: 0.05
        time_interval: 1.5
        max_state_in_trajectory: 40
      ilqr_trajectory_tracker:
        iteration_number: 20
        alpha: 1.0
        input_limits_min: [-2.5, -1.0]
        input_limits_max: [2.5, 1.0]
      cost_checker_plugins: ["LateralDistanceCostChecker", "LongtitutalVelocityCostChecker"]
      LateralDistanceCostChecker:
        plugin: "nav2_frenet_ilqr_controller::costs::LateralDistanceCost"
        K_lateral_distance: 5.0
      LongtitutalVelocityCostChecker:
        plugin: "nav2_frenet_ilqr_controller::costs::LongtitutalVelocityCost"
        K_longtitutal_velocity: 5.0
        desired_velocity: 0.5

      policy_plugins: ["ObstaclePolicy"]
      ObstaclePolicy:
        plugin: "nav2_frenet_ilqr_controller::policies::ObstaclePolicy"
```

# Citation

```bibtex
@INPROCEEDINGS{5509799,
  author={Werling, Moritz and Ziegler, Julius and Kammel, Sören and Thrun, Sebastian},
  booktitle={2010 IEEE International Conference on Robotics and Automation}, 
  title={Optimal trajectory generation for dynamic street scenarios in a Frenét Frame}, 
  year={2010},
  volume={},
  number={},
  pages={987-993},
  keywords={Trajectory;Vehicle dynamics;Remotely operated vehicles;Aerodynamics;Road vehicles;Mobile robots;Merging;Road transportation;Traffic control;Robotics and automation},
  doi={10.1109/ROBOT.2010.5509799}}
```

```bibtex
@article{article,
author = {Kunz, Tobias and Stilman, Mike},
year = {2012},
month = {07},
pages = {},
title = {Time-Optimal Trajectory Generation for Path Following with Bounded Acceleration and Velocity},
doi = {10.15607/RSS.2012.VIII.027}
}
```

```bibtex
@inproceedings{inproceedings,
author = {Li, Weiwei and Todorov, Emanuel},
year = {2004},
month = {01},
pages = {222-229},
title = {Iterative Linear Quadratic Regulator Design for Nonlinear Biological Movement Systems.},
volume = {1},
journal = {Proceedings of the 1st International Conference on Informatics in Control, Automation and Robotics, (ICINCO 2004)}
}
```
