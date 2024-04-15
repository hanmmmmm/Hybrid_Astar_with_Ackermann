# Hybrid_Astar_with_Ackermann

This project is my implementation of a navigation package, including the Hybrid A* planner and pure pursuit and LQR controller. 

It also has a basic simulator for an Ackermann robot (using the bicycle model). This simulator has only the calculation parts. The visualization happens in RVIZ. 

My implementation of Hybrid A* has the incremental searching and Reeds-Shepp curve searching. This generates acceptable results but it certainly can be improved more:
- avoid some very short path segments that are too hard to steer
- add additional cost for the paths closer to obstacles 

Every code in this project was developed by myself using standard C++ and ROS tools. 

It was developed and tested in **ROS Humble** on Ubuntu 22, on both x86 and RaspberryPi 4. 

As shown in the last demo below, it was tested on a mini-ackermann robot platform, made of 
- a RaspberryPi 4,
- a DC motor controller board,
- a LeiShen lsn10 2D lidar
- 2 DC motor for rear wheels
- a servo motor control the steering wheels 


This branch **hawa_ros2** is the continued work in branch **main**. 

I moved from ros1 to ros2, and finished some items in the to-do list:
- path validator for dynamic obstacles.
- Linear quadratic regulator controller, that controls the speed and steering fo the robot.
- re-organize the project fr better simplicity. 

### compile:
    colcon build

-----------

### Demo in simulation:

The target pose is selected in RVIZ, using the **2D Nav Goal** button. 

The whole path is rendered in red color. 

The green part is the current segment that the robot is following at the moment. 

The big red dot (in front of the robot) on the path is the target point selected by the pure-pursuit controller. 


<a id="demogif1" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/hawa_ros2/demo_images/p1.gif">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p1.gif" alt="gif 1" title="case 1" width="600"/>
</a>


<a id="demogif2" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/hawa_ros2/demo_images/p2.gif">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p2.gif" alt="gif 2" title="case 2" width="600"/>
</a>


<a id="demogif4" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/hawa_ros2/demo_images/p4.gif">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p4.gif" alt="gif 3" title="case 3" width="600"/>
</a>

----

### Demo on mini-ackermann robot:

This is a test running indoor.  

For the controller, I was using LQR.

The SLAM module was GMapping.

Note the RVIZ window was very slow and laggy because the WiFi signal was terribly bad in that room.  

The RVIZ process was running on a laptop, while everything else is running on robot's Raspberrypi 4. 

The processes on robot was running smoothly.

<a id="demogif5" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/hawa_ros2/demo_images/real_demo1.gif">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/hawa_ros2/demo_images/real_demo1.gif" alt="gif 5" title="on robot" width="600"/>
</a>

----

### the package: map_fusion

Just to quickly explain what does the this package do: 

It receives the occupancy grid provided by the SLAM module, then add inflation into it. And publish the result so the path planner can plan the path with considering sufficient clearance from the obstacles.

<a id="demopic1" href="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/hawa_ros2/demo_images/map_fusion_example.png">
    <img src="https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/hawa_ros2/demo_images/map_fusion_example.png" alt="pic 1" title="map_fusion" width="600"/>
</a>

----
### TODO:
- Finish refactoring the remaining parts in the reeds shepp curves code. (Currently, only CSC and CCC types are used)
- Improving the scoring method in RS curves so that it will prioritize the curves with fewer reversing motions. The current solution only considers the total length.
- Add feature: while searching for the path, hold multiple valid paths before terminating, so that it can have several options to choose from.

