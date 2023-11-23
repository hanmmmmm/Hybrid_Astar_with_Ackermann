# Hybrid_Astar_with_Ackermann

This project is my implementation of a navigation package, including the Hybrid A* planner and pure pursuit controller. 

It also has a basic simulator for an ackermann robot (using bicycle model). This simulator has only the calculation parts. The visulization happens in RVIZ. 

My implementation of Hybrid A* has the incremental searching and Reeds Shepp curve searching. This generates acceptable reaults but it certainly can be improved more. 

- avoid some very short path segments that are too hard to steer
- avoid apex points that are too close to obstacles
- add numerical optimiztion module to so that the curve  

Every code in this project was developed by myself using standard C++ and ROS tools. 

It was developed and tested in **ROS Noetic**. 

-----------

### compile:
    catkin_make

### start the demo: 
    roslaunch hawa_pathplan hawa_demo_manual_target.launch

In RVIZ, from the buttons on the top, use **2D Nav Goal** button to set the target position. 


Here are some demo clips:





