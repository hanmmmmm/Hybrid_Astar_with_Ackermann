#!/usr/bin/env python3

import rospy
import math
import tf
import random

from time import time 

br = tf.TransformBroadcaster()


if __name__ == '__main__':
    rospy.init_node('tf_puber')

    last_obs_time = time()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        br.sendTransform((0.5 , -0.5 , 0),
                    tf.transformations.quaternion_from_euler(0, 0, 1.5),
                    rospy.Time.now(),
                    "base_link",
                    "map")

        # if time() - last_obs_time > 4.0:
        #     last_obs_time = time()

        #     br.sendTransform((0.5 + random.random(), -0.5 + random.random(), 0),
        #             tf.transformations.quaternion_from_euler(0, 0, 1.5+ random.random()),
        #             rospy.Time.now(),
        #             "base_link",
        #             "map")


        rate.sleep()