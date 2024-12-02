# Frenet Iterative Linear Quadratic Regulator Controller

[Screencast from 12-02-2024 11:48:19 PM.webm](https://github.com/user-attachments/assets/7f1043e1-162a-4b1c-9775-ae54953393cb)

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

TODO

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
