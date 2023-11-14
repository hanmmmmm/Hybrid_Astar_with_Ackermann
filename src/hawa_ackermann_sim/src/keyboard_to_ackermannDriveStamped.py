#!/usr/bin/env python3
import threading
import time 
import sys
from select import select

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

import termios
import tty


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

class ClassAckermannMsgKeyboard:
    def __init__(self) -> None:
        
        rospy.init_node("node_akmsim_keyboard")

        self.puber = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=10)
        self.msg = AckermannDriveStamped()
        self.FLAG_publish_sim_cmd = True
        self.end_process = False

        print()
        print("Ending keyboard control by pressing SpaceBar on keyboard. Ctrl+C cannot stop this process.")
        print()

        while not rospy.is_shutdown():
            key = getKey(settings, key_timeout)
            valid = self.process_key_inputs(key=key)
            if valid:
                self.puber.publish(self.msg)
            rospy.sleep(0.01)

            if self.end_process:
                break

        restoreTerminalSettings(settings)


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


if __name__ == "__main__":
    kb = ClassAckermannMsgKeyboard()


