
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped




rospy.init_node("test_path_node")
puber = rospy.Publisher("/path", Path, queue_size=1)

rospy.sleep(0.5)


msg = Path()
msg.header.frame_id = "map"

with open("paths/1.txt", 'r') as ff:
    line = ff.readline().strip()
    while line != "":
        # print(line)
        try:
            xyyaw = line.split(",")
            x = float(xyyaw[0])
            y = float(xyyaw[1])
            yaw = float(xyyaw[2])
            print(x,y,yaw)
            a_pose = PoseStamped()
            a_pose.pose.position.x = x
            a_pose.pose.position.y = y
            msg.poses.append(a_pose)
            line = ff.readline().strip()
        except:
            break
    
    puber.publish(msg)
    print("published.")

print(msg)

while not rospy.is_shutdown():
    puber.publish(msg)
    rospy.sleep(1)


# a_pose.pose.position.x = 0
# a_pose.pose.position.y = 0
# msg.poses.append(a_pose)

# a_pose.pose.position.x = 0
# a_pose.pose.position.y = 0
# msg.poses.append(a_pose)

# a_pose.pose.position.x = 0
# a_pose.pose.position.y = 0
# msg.poses.append(a_pose)

# a_pose.pose.position.x = 0
# a_pose.pose.position.y = 0
# msg.poses.append(a_pose)

# a_pose.pose.position.x = 0
# a_pose.pose.position.y = 0
# msg.poses.append(a_pose)

# a_pose.pose.position.x = 0
# a_pose.pose.position.y = 0
# msg.poses.append(a_pose)

# a_pose.pose.position.x = 0
# a_pose.pose.position.y = 0
# msg.poses.append(a_pose)
