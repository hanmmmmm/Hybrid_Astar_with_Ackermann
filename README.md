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


<!--https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/4ce2bd41-bde0-40b6-8ffd-014cb2a9bcab


https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/eb60ac7e-02db-48ab-8b33-a22223d483ac


https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/1bf8b5ec-82de-4ee3-9591-4bf0c4789b31


https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/3a0cf466-5b9a-4048-82b7-246840751ca8
-->

<!--![image]( https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p1.gif)-->

<!--
<a id="demogif1" href="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p1.gif">
    <img src="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p1.gif" alt="gif 1" title="case 1" width="600"/>
</a>

<a id="demogif2" href="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p2.gif">
    <img src="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p2.gif" alt="gif 2" title="case 2" width="600"/>
</a>

<a id="demogif3" href="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p3.gif">
    <img src="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p3.gif" alt="gif 3" title="case 3" width="600"/>
</a>
-->

<a id="demogif3" href="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p3.gif">
    <img src="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/blob/main/demo_images/p3.gif" alt="gif 3" title="case 3" width="600"/>
</a>


<!-- <img src="https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/853f1b1a-396f-4698-a910-576b7fe15fe5" alt="gif 3" title="case 3" width="600"/> -->

<!--
![p1](https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/853f1b1a-396f-4698-a910-576b7fe15fe5)
![p2](https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/f9e0c32d-889f-493f-b04e-29dbca94573a)
![p4](https://githubfast.com/hanmmmmm/Hybrid_Astar_with_Ackermann/assets/35117797/9e07ed57-c49f-42ae-aa8a-17492401ca55)
-->

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
