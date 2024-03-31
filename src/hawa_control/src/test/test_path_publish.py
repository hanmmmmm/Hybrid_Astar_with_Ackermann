


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from random import choice




def load_path(path_file):
    print(f"load_path({path_file})")

    with open(path_file, 'r') as f:
        content = f.readlines()

    for i in range(len(content)):
        content[i] = content[i].strip()

    # print(content)

    path_msg = Path()
    path_msg.header.frame_id = "map"

    points_array_msg = MarkerArray()

    ct = 0
    for line in content:
        if line.startswith('#'):
            continue
        x = float(line.split(',')[0])
        y = float(line.split(',')[1])
        pos = PoseStamped()
        pos.pose.position.x = x
        pos.pose.position.y = y
        path_msg.poses.append(pos)

        mk = Marker()
        mk.header.frame_id = "map"
        mk.id = ct
        mk.action = mk.ADD
        mk.type = mk.SPHERE
        mk.pose.position.x = x
        mk.pose.position.y = y 
        mk.pose.orientation.w = 1.0
        mk.scale.x = 0.05
        mk.scale.y = 0.05
        mk.scale.z = 0.05
        mk.color.a = 1.0
        mk.color.b = 0.0
        mk.color.g = 1.0
        mk.color.r = 1.0
        

        points_array_msg.markers.append(mk)

        ct += 1
    return path_msg, points_array_msg



class ClassPathPublisher(Node):
    def __init__(self):
        super().__init__('node_test_path_pub')
        self.path_publisher = self.create_publisher(Path, "/path", 10)
        self.point_publisher = self.create_publisher(MarkerArray, "/visualization_marker_array", 1)
        
        self.path_msg, self.points_array_msg = load_path('path_wave.txt')

        timer_period = 1  #sec
        self.timer = self.create_timer(timer_period, self.publish_all)

    def publish_all(self):
        self.path_publisher.publish(self.path_msg)
        self.point_publisher.publish(self.points_array_msg)



def cb_change_path(msg:String):
    global path_msg, points_array_msg
    paths = ['path_long_circle.txt',
             'path_long_circle_and_slope.txt',
             'path_wave.txt']
    print("")
    path_msg, points_array_msg = load_path(choice(paths))



if __name__ == "__main__":

    # rospy.init_node("node_test_path_pub")

    # rospy.Subscriber("/change_path", String, cb_change_path)
    # puber = rospy.Publisher("/path", Path, queue_size=1)
    # points_puber = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size= 1)

    # path_msg, points_array_msg = load_path('path_long_circle.txt')

    rclpy.init()

    publisher = ClassPathPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

    # while not rospy.is_shutdown():
    #     puber.publish(path_msg)
    #     points_puber.publish(points_array_msg)

    #     rospy.sleep(1)












