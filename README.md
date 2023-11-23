# Hybrid_Astar_with_Ackermann

This project is my implementation of a navigation package, including the Hybrid A* planner and pure pursuit controller. 

It also has a basic simulator for an ackermann robot (using bicycle model). This simulator has only the calculation parts. The visulization happens in RVIZ. 

My implementation of Hybrid A* has the incremental searching and Reeds Shepp curve searching. This generates acceptable reaults but it certainly can be improved more:
- avoid some very short path segments that are too hard to steer
- avoid apex points that are too close to obstacles
- add numerical optimiztion module to so that the curve will be adjusted to match the dynamics of the robot. E.g. acceleration and steer rate. 

Every code in this project was developed by myself using standard C++ and ROS tools. 

It was developed and tested in **ROS Noetic** on original Ubuntu 20 Desktop and WSL in Win11. 

-----------

### compile:
    catkin_make

### start the demo: 
    roslaunch hawa_pathplan hawa_demo_manual_target.launch

The launch file above will start **RVIZ**. In RVIZ, from the buttons on the top, use **2D Nav Goal** button to set the target position. 

The red path is the whole path. The green part is the segment that the robot is following at the moment. The red dot on
the path is the target point selected by the purepursuit controller. 


### demo clips:


https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/4ce2bd41-bde0-40b6-8ffd-014cb2a9bcab


https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/eb60ac7e-02db-48ab-8b33-a22223d483ac


https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/1bf8b5ec-82de-4ee3-9591-4bf0c4789b31


https://github.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/3a0cf466-5b9a-4048-82b7-246840751ca8



----

### drive the simulation by keyboard:
    python3 src/hawa_ackermann_sim/src/keyboard_to_ackermannDriveStamped.py

Use this code in case you want to move the robot manually.

----
### TODO:
- Finish refactoring the remaining parts in the reeds shepp curves code. (Currently only CSC and CCC types are used)
- Move the remaining parameters into json configuration files.
- Finish the path validation feature so that the planner module can reponse to dynamic obstacles and environment.
- The points mentioned in the Hybrid A* section at the beginning. 