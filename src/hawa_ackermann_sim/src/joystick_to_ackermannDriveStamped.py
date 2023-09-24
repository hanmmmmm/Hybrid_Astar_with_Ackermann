#!/usr/bin/env python3

import pygame 
import time 
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

FLAG_speed_cmd_output_enable = False
TIMESTAMP_output_enable_changed = 0.0 

pygame.joystick.init()

# print(f"num {pygame.joystick.get_count()}")

joysticks = [ ]
for x in range(pygame.joystick.get_count()):
    joysticks.append(pygame.joystick.Joystick(x))
    print(joysticks)

print()
print("Press Triangle button to toggle enable/disable manual control.")
print(f"Manual control enable: {FLAG_speed_cmd_output_enable}")
print()

# joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

pygame.init()

rospy.init_node("node_joy_cmdvel")

# puber = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
# msg = Twist()

puber = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=10)
msg = AckermannDriveStamped()

puber_obstacle_move = rospy.Publisher("/obstacle_motion", Float32MultiArray, queue_size=10)
obstacle_motion_cmd = Float32MultiArray()


while not rospy.is_shutdown():
    pygame.event.get()

    # left joystick
    a1 = round(pygame.joystick.Joystick(0).get_axis(0), 2)  #(-1.0, 1.0)
    a2 = round(pygame.joystick.Joystick(0).get_axis(1), 2)

    # right joystick
    a3 = round(pygame.joystick.Joystick(0).get_axis(4), 2)
    a4 = round(pygame.joystick.Joystick(0).get_axis(3), 2)

    # print([a1, a2, a3, a4])

    button_cross = pygame.joystick.Joystick(0).get_button(0)  # button: blue cross
    button_circle = pygame.joystick.Joystick(0).get_button(1)  # button: blue cross
    button_square = pygame.joystick.Joystick(0).get_button(2)  # button: blue cross
    button_triangle = pygame.joystick.Joystick(0).get_button(3)  # button: blue cross

    # print(button_cross, button_square, button_triangle, button_circle)
    # print(button_triangle)

    # pygame.joystick.Joystick(0).get_button(4)  # left shoulder button
    # pygame.joystick.Joystick(0).get_button(7)  # right trigger button
    # pygame.joystick.Joystick(0).get_button(8)  # Share button
    # pygame.joystick.Joystick(0).get_button(11)  # left joytisck click
    # button_cross_up = pygame.joystick.Joystick(0).get_button(13)
    # button_cross_down = pygame.joystick.Joystick(0).get_button(14)
    # button_cross_left = pygame.joystick.Joystick(0).get_button(15)
    # button_cross_right = pygame.joystick.Joystick(0).get_button(16)
    # print(button_cross_up, button_cross_down, button_cross_left)

    if time.time() - TIMESTAMP_output_enable_changed > 1.0:
        if button_triangle:
            FLAG_speed_cmd_output_enable = not FLAG_speed_cmd_output_enable
            TIMESTAMP_output_enable_changed = time.time()
            print(f"Manual control enable: {FLAG_speed_cmd_output_enable}")

    if abs(a1) < 0.02:
        a1 = 0

    if abs(a2) < 0.02:
        a2 = 0

    # if a2 < 0.0:
    #     a1 *= -1.0
    
    msg.header.stamp = rospy.Time.now()

    msg.drive.steering_angle = -1 * a1 * 0.4 
    msg.drive.speed = -1 * a2 
    if FLAG_speed_cmd_output_enable:
        puber.publish(msg)

    
    if abs(a3) < 0.02:
        a3 = 0

    if abs(a4) < 0.02:
        a4 = 0
    
    # obstacle_motion_cmd.data.clear()
    # obstacle_motion_cmd.data.append(a3)
    # obstacle_motion_cmd.data.append(a4)

    # if button_cross_up and not button_cross_down:
    #     obstacle_motion_cmd.data.append(1)
    # elif button_cross_down and not button_cross_up:
    #     obstacle_motion_cmd.data.append(-1)
    # else:
    #     obstacle_motion_cmd.data.append(0)
    
    # if button_cross_left and not button_cross_right:
    #     obstacle_motion_cmd.data.append(1)
    # elif button_cross_right and not button_cross_left:
    #     obstacle_motion_cmd.data.append(-1)
    # else:
    #     obstacle_motion_cmd.data.append(0)

    # puber_obstacle_move.publish(obstacle_motion_cmd)


    time.sleep(0.01)
