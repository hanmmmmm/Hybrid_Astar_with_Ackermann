# Hybrid_Astar_with_Ackermann

This project is my implementation of a navigation package, including the Hybrid A* planner and pure pursuit controller. 

It also has a basic simulator for an Ackermann robot (using the bicycle model). This simulator has only the calculation parts. The visualization happens in RVIZ. 

My implementation of Hybrid A* has the incremental searching and Reeds-Shepp curve searching. This generates acceptable results but it certainly can be improved more:
- avoid some very short path segments that are too hard to steer
- avoid apex points that are too close to obstacles
- add a numerical optimization module so that the curve will be adjusted to match the dynamics of the robot. E.g. acceleration and steer rate. 

Every code in this project was developed by myself using standard C++ and ROS tools. 

It was developed and tested in **ROS Noetic** on the original Ubuntu 20 Desktop and WSL in Win11. 

-----------

### compile:
    catkin_make

### start the demo: 
    roslaunch hawa_pathplan hawa_demo_manual_target.launch

The launch file above will start **RVIZ**. In RVIZ, from the buttons on the top, use **2D Nav Goal** button to set the target position. 

The whole path is rendered in red color. The green part is the current segment that the robot is following at the moment. The big red dot (in front of the robot) on
the path is the target point selected by the pure-pursuit controller. 


### demo clips:

(If they are not properly displayed here on the web, they can be found and downloaded from the folder of demo_images.)

<a id="demogif1" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p1.gif">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p1.gif" alt="gif 1" title="case 1" width="600"/>
</a>

<a id="demogif2" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p2.gif">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p2.gif" alt="gif 2" title="case 2" width="600"/>
</a>

<a id="demogif3" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p3.gif">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p3.gif" alt="gif 3" title="case 3" width="600"/>
</a>

----

### Drive the simulation by keyboard:
    python3 src/hawa_ackermann_sim/src/keyboard_to_ackermannDriveStamped.py

Use this code in case you want to move the robot manually.

----
### TODO:
- Finish refactoring the remaining parts in the reeds shepp curves code. (Currently, only CSC and CCC types are used)
- Move the remaining parameters into JSON configuration files.
- Finish the path validation feature so that the planner module can respond to dynamic obstacles and environment.
- Improving the scoring method in RS curves so that it will prioritize the curves with fewer reversing motions. The current solution only considers the total length.
- Add feature: while searching for the path, hold multiple valid paths before terminating, so that it can have several options to choose from.
- And the points mentioned in the Hybrid A* section at the beginning. 
