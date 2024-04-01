#!/usr/bin/env python3
import threading
import time 
import sys
from select import select

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

import termios
import tty

'''
This file is a process for maunally driving the simulated ackermann robot in this ros package, by keyboard. 

run this directly in a terminal:
python3 keyboard_to_ackermannDriveStamped.py

Then press the keyboard keys to move the robot:
i = forward
, = backward
u = forward left turn
o = forward right turn
m = backward left turn
. = backward right turn
k = stop the robot (optional because the robot will automatically stop when no command received)
spacebar = terminate this process

Note: Ctrl+C cannot terminate this process. 
'''


key_timeout = 0.5 
key_timestamp = 0.0


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def getKey(settings, timeout):
    global key_timestamp
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        key_timestamp = time.time()
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


settings = saveTerminalSettings()

# class ClassAckermannMsgKeyboard:
#     def __init__(self) -> None:
        
#         rospy.init_node("node_akmsim_keyboard")

#         self.puber = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=10)
#         self.msg = AckermannDriveStamped()
#         self.FLAG_publish_sim_cmd = True
#         self.end_process = False

#         print()
#         print("Ending keyboard control by pressing SpaceBar on keyboard. Ctrl+C cannot stop this process.")
#         print()

#         while not rospy.is_shutdown():
#             key = getKey(settings, key_timeout)
#             valid = self.process_key_inputs(key=key)
#             if valid:
#                 self.puber.publish(self.msg)
#             rospy.sleep(0.01)

#             if self.end_process:
#                 break

#         restoreTerminalSettings(settings)



class ClassCmdPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_cmd_publisher')
        self.cmd_publisher = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        self.msg = AckermannDriveStamped()
        self.FLAG_publish_sim_cmd = True
        self.end_process = False

        timer_period = 0.01  #sec
        self.timer = self.create_timer(timer_period, self.timerCallback)

        print()
        print("Ending keyboard control by pressing SpaceBar on keyboard. Ctrl+C cannot stop this process.")
        print()


    def timerCallback(self):
        key = getKey(settings, key_timeout)
        valid = self.process_key_inputs(key=key)
        if valid:
            self.cmd_publisher.publish(self.msg)


    def process_key_inputs(self, key) -> bool:
        if len(key) <= 0:
            return False
        
        if time.time() - key_timestamp > key_timeout:
            return False
        
        key = key.lower().strip()

        # print(f">{key}<")
        # print(type(key))

        if "k" in key:
            self.reset_akm_msg()
            return True
        
        if "" == key:
            print("Ending keyboard control.")
            self.end_process = True
            raise SystemExit
            return True
        
        speed_forward = 0.4
        speed_backward = -0.4
        speed_stop = 0.0

        steer_left = 0.6
        steer_right = -0.6
        steer_neutral = 0.0

        if "i" == key:
            self.msg.drive.speed = speed_forward
            self.msg.drive.steering_angle = steer_neutral
            return True
        elif "," == key:
            self.msg.drive.speed = speed_backward
            self.msg.drive.steering_angle = steer_neutral
            return True
        elif "j" == key:
            self.msg.drive.speed = speed_stop
            self.msg.drive.steering_angle = steer_left
            return True
        elif "l" == key:
            self.msg.drive.speed = speed_stop
            self.msg.drive.steering_angle = steer_right
            return True
        elif "u" == key:
            self.msg.drive.speed = speed_forward
            self.msg.drive.steering_angle = steer_left
            return True
        elif "o" == key:
            self.msg.drive.speed = speed_forward
            self.msg.drive.steering_angle = steer_right
            return True
        elif "m" == key:
            self.msg.drive.speed = speed_backward
            self.msg.drive.steering_angle = steer_left
            return True
        elif "." == key:
            self.msg.drive.speed = speed_backward
            self.msg.drive.steering_angle = steer_right
            return True
        
        # print(key)


    def reset_akm_msg(self):
        self.msg = AckermannDriveStamped()


def main(args=None):
    rclpy.init(args=args)

    publisher = ClassCmdPublisher()

    try:
        rclpy.spin(publisher)
    except SystemExit:
        rclpy.logging.get_logger("ClassCmdPublisher").info('Done')

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


