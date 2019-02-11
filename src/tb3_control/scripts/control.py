import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


"""Callback function for odometry msg, used for odom subscriber
"""
def callback(odom_data):
    """Upon "hearing" odom msg, retrieve its position and orientation (yaw) information
    """
    global roll, pitch, yaw, Odom_v, Odom_w
    orientation_q = odom_data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    Odom_v = odom_data.twist.twist.linear.x
    Odom_w = odom_data.twist.twist.angular.z
    rospy.loginfo("Yaw: %f", yaw)

"""Display function that draws a circular lane in Rviz
"""
def displayLane():
    path = Marker()
    #this should be "paired" to the fixed frame id entry in Rviz, the default setting in Rviz for tb3-fake is "odom". Keep this line as is if you don't have an issue
    path.header.frame_id = "odom"
    path.type = path.LINE_STRIP
    path.header.stamp = rospy.Time()
    path.ns = "odom"
    path.id = 0;
    path.action = path.ADD#use line marker
    path.lifetime = rospy.Duration()
    #path line strip is blue
    path.color.b = 1.0
    path.color.a = 1.0
    path.scale.x = 0.02
    path.pose.orientation.w = 1.0
    num_slice2 = 50#divide a circle into segments
    slice_index2 = 0
    M_PI = 3.14159
    num = 0

    while len(path.points) <= num_slice2:
        p = Point()
        angle = slice_index2*2*M_PI/num_slice2
        slice_index2 += 1
        p.x = 4*math.cos(angle)-0.5 #some random circular trajectory, with radius 4, and offset (-0.5, 1, 0)
        p.y = 4*math.sin(angle)+1.0
        p.z = 0

        path.points.append(p) #for drawing path, which is line strip type
        num += 1

    marker_pub.publish(path)

"""This function publishes control message and display trajectories
"""
def control():
    rospy.init_node("control")
    rate = rospy.Rate(10) #ros spins 10 frames per second
    m_listener = tf.TransformListener()
    tw_msg = geometry_msgs.msg.Twist()#we use geometry_msgs::twist to specify linear and angular speeds (v, w) which also denote our control inputs to pass to turtlebot
    while not rospy.is_shutdown():
        """YOUR CONTROL STRETEGY HERE
	Insert your code
	Insert your code
	Insert your code
	Insert your code
        """
        #for linear speed, we only use the first component of 3D linear velocity "linear.x" to represent the speed "v"
        tw_msg.linear.x = 0.3
        #for angular speed, we only use the third component of 3D angular velocity "angular.z" to represent the speed "w" (in radian)
        tw_msg.angular.z = 0.25
        displayLane()
        #publish this message to the robot
        cmd_vel_pub.publish(tw_msg)
        rate.sleep()

if __name__ == '__main__':
    marker_pub = rospy.Publisher('visualization_marker',Marker, queue_size=1)
    cmd_vel_pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    odom_sub = rospy.Subscriber("odom",Odometry,callback)
    try:
        control()
    except rospy.ROSInterruptException:
        pass
