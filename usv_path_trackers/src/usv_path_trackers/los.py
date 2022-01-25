#!/usr/bin/env python
from socket import MsgFlag
from matplotlib.pyplot import switch_backend
from numpy.core.numeric import cross
from sklearn.metrics import euclidean_distances
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float32, Bool
import queue


class LOS:
    """
        LOS controller for the USV
    """
    def __init__(self) -> None:
        rospy.Subscriber("/usv/odom",Odometry,self.odom_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("/usv_planner/waypoint",Pose,self.waypoint_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("/external_reset/USV_LOS",Bool, self.reset,queue_size=1,tcp_nodelay=True)
        rospy.Timer(rospy.Duration(0.1),self.publish_reference_cb)

        self.debug_crosstrack = rospy.Publisher("/los/crosstrack_error",Float32,queue_size=1,tcp_nodelay=True)
        self.debug_alongtrack = rospy.Publisher("los/alongtrack",Float32,queue_size=1,tcp_nodelay=True)
        self.debug_cmd = rospy.Publisher("/los/setpoint",Twist, queue_size=1,tcp_nodelay=1)

        self.reference_pub = rospy.Publisher("/los/desired_yaw",Float32,queue_size=1,tcp_nodelay=True)

        self.pose = None
        self.waypoint_queue = queue.Queue()
        self.current_waypoint = Pose()
        self.last_waypoint = Pose()

        self.tangential_transform = np.eye(3)
        self.spline_coord_center = np.array([0,0])
        self.pi_p = None

        self.first_wpt_set = False
        self.stop = True

        self.desired_yaw = 0
        self.desired_speed = 0

        #Configuration
        self.los_distance = 8

    
    def reset(self, msg:Bool):
        print("LOS reset")
        self.pose = None
        self.current_waypoint = Pose()
        self.last_waypoint = Pose()
        self.tangential_transform = np.eye(3)
        self.spline_coord_center = np.array([0,0])
        self.pi_p = None

    def odom_cb(self,msg: Odometry) -> None:
        self.pose = msg.pose.pose

        if self.current_waypoint==Pose():
            print("Current waypoint not set, setting using odometry")
            self.current_waypoint.position = msg.pose.pose.position
            print(msg.pose.pose)

        #Check if should switch waypoint
        if self.euclidean_distance(msg.pose.pose,self.current_waypoint)<10:
            print("Within circle of acceptance, switching waypoint")
            self.switch_waypoint()

        if self.stop==True:
            self.desired_speed = 0
            return

        #Calculate crosstrack
        crosstrack_error = self.calculate_crosstrack_error()
        #print("Crosstrack error is: ",crosstrack_error)

        #Calculate desired yaw
        self.desired_yaw = self.pi_p - math.atan2(crosstrack_error,self.los_distance)
        #print("Desired yaw: ", self.desired_yaw)

    def euclidean_distance(self,point_a:Pose,point_b:Pose)->float:
        return np.sqrt(pow(point_a.position.x-point_b.position.x,2)+pow(point_a.position.y-point_b.position.y,2))

    def waypoint_cb(self,msg:Pose) -> None:
        print("Waypoint received, adding to queue")
        self.waypoint_queue.put(msg)

    def switch_waypoint(self) -> None:
        if self.waypoint_queue.qsize()==0:
            print("No waypoint in queue, stopping and waiting for new")
            self.stop = True
            return
        else:
            self.stop=False

        self.last_waypoint.position.x = self.current_waypoint.position.x
        self.last_waypoint.position.y = self.current_waypoint.position.y
        self.last_waypoint.orientation.x = self.current_waypoint.orientation.x
        self.last_waypoint.orientation.y = self.current_waypoint.orientation.y
        self.last_waypoint.orientation.z = self.current_waypoint.orientation.z
        self.last_waypoint.orientation.w = self.current_waypoint.orientation.w

        self.current_waypoint = self.waypoint_queue.get()

        #Determine current tangential frame of spline between last and current
        self.spline_coord_center[0] = self.last_waypoint.position.x
        self.spline_coord_center[1] = self.last_waypoint.position.y
        rotation_angle = math.atan2((self.current_waypoint.position.y-self.last_waypoint.position.y),(self.current_waypoint.position.x-self.last_waypoint.position.x))
        R_ab = np.array([[math.cos(rotation_angle),-math.sin(rotation_angle)],[math.sin(rotation_angle),math.cos(rotation_angle)]])
        r_ab_a = self.spline_coord_center.reshape((2,1))

        #Note: If sets this together to transformation matrix we get T_ab which takes point in tangential frame and throws to global frame.
        # We must thus take inverse, which is not transpose for the entire T_ab. Notation from p.223 in Modelling and SImulation for Automatic Control. 
        R_ba = R_ab.T
        r_ba_b = -R_ba@r_ab_a

        self.tangential_transform[:2,:2] = R_ba
        self.tangential_transform[:2,2] = r_ba_b.flatten()

        self.pi_p = rotation_angle
        self.has_received_wp = True

        self.desired_speed = 3

    def publish_reference_cb(self,timer):
        if self.pose==None:
            print("publishReferenceCb but no pose yet received, thus ignore")
            return
        self.reference_pub.publish(Float32(self.desired_yaw))
        msg = Twist()
        msg.linear.x = self.desired_speed
        msg.angular.y = self.desired_yaw
        self.debug_cmd.publish(msg)

    def calculate_crosstrack_error(self):
        tangential_pose_homogenous = self.tangential_transform@np.array([self.pose.position.x,self.pose.position.y,1]).reshape((3,1))
        tangential_pose = tangential_pose_homogenous[:2]
        self.debug_crosstrack.publish(Float32(tangential_pose[1]))
        self.debug_alongtrack.publish(Float32(tangential_pose[0]))
        return tangential_pose[1]