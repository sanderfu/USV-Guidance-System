#!/usr/bin/env python
from audioop import cross
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from usv_msgs.msg import reinit
from std_msgs.msg import Float32, Bool
import queue
from tf.transformations import euler_from_quaternion
from geographiclib.geodesic import Geodesic


class TurnRadiusEnforcer:
    """
        LOS controller for the USV
    """
    def __init__(self) -> None:
        self.corrected_setpoint_pub = rospy.Publisher("los/corrected_setpoint",Twist, queue_size=1,tcp_nodelay=1)
        self.waypoint_reached_pub = rospy.Publisher("los/waypoint_reached",Pose,queue_size=1,tcp_nodelay=True)
        self.reference_pub = rospy.Publisher("los/desired_yaw",Float32,queue_size=1,tcp_nodelay=True)

        
        first_odom:Odometry = rospy.wait_for_message("odom", Odometry)
        _,_,yaw = euler_from_quaternion([first_odom.pose.pose.orientation.x,first_odom.pose.pose.orientation.y,first_odom.pose.pose.orientation.z,first_odom.pose.pose.orientation.w])
        self.vessel_yaw = yaw
        self.desired_speed = 10
        self.yaw_turn = np.pi/2

        self.reference_publish_timer = rospy.Timer(rospy.Duration(0.1),self.publish_reference_cb)

        #Register subscribers
        rospy.Subscriber("odom",Odometry,self.odom_cb,queue_size=1,tcp_nodelay=True)

    def odom_cb(self,msg: Odometry) -> None:
        _,_,self.vessel_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

    def publish_reference_cb(self,timer):
        msg_corrected = Twist()
        msg_corrected.angular.z = self.vessel_yaw+self.yaw_turn

        msg = Twist()
        msg_corrected.linear.x = self.desired_speed

        #self.reference_pub.publish(Float32(msg.angular.z))
        self.corrected_setpoint_pub.publish(msg_corrected)


