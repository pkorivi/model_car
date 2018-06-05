# Motion Planner - Trajectory planner software for model car

The package implements trajectory planner module for the model car. This package is a middleware between route planner for high level reference path and controller for low level control of the car.

### Dependencies:
The motion planner depends on

#### External libraries
* ecl_lib - for creating splines
* Eigen - for solving equations
* networkx - for creating graph from rndf and calculating shortest path

#### Model car Packages
* route_planner -  for high level path
* fub_controller - for converting the x,y,t coordinates into corresponding speed and steering controls
* autonomos_obstacle_msgs - for obstacle information
* fub_trajectory_msgs - for transmitting the path to fub_controller package.

### Launching the Planner
* compile the above mentioned dependencies and the motion planner packages.
* `roslaunch motion_planner planner.launch` launches all the required packages - route_planner, motion_planner and controller. They can be launched individually also.

#### Tip
It may not possible to install networkx on model car, in this situation please install on your local PC and launch `route_planner` from local PC and `motion_planner and fub_controller` on modelcar. 

### Executing the plan
* route_planner reads the rndf information from the provided rndf file and constructs the graph.
* Use 2D-Nav goal to publish a final pose or publish the destination pose on `/move_base_simple/goal`.
* Motion planner receives the sub path (the portion of the route path which is on same segment as the ego) and starts calculating trajectory.
* To change the lane publish /clicked_point either using Rviz or anything else.
* Adjust the parameters in `ControllerMig.cfg` of `fub_controller` to tune the behaviour.


### Improvements needed
* Optimize the code - There are several computationally heavy operations which could be reduced by restructuring.
* Improve the conversion between frenet and cartisian frames.
* Improve Collision detection - especially for lateral moving objects and create different mechanism for handling intersections.
* Improve heuristics in assigning pre-costs to the planner.
* Improve the velocity planning, and handling in sharp curves.
* Improve the controller to create smooth control of the vehicle.
