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
from usv_map.geotf_client import GeodeticConverterClient
from geographiclib.geodesic import Geodesic


class LOS:
    """
        LOS controller for the USV
    """
    def __init__(self) -> None:
        self.debug_crosstrack = rospy.Publisher("los/crosstrack_error",Float32,queue_size=1,tcp_nodelay=True)
        self.debug_alongtrack = rospy.Publisher("los/alongtrack",Float32,queue_size=1,tcp_nodelay=True)
        self.setpoint_pub = rospy.Publisher("los/setpoint",Twist, queue_size=1,tcp_nodelay=1)
        self.corrected_setpoint_pub = rospy.Publisher("los/corrected_setpoint",Twist, queue_size=1,tcp_nodelay=1)
        self.waypoint_reached_pub = rospy.Publisher("los/waypoint_reached",Pose,queue_size=1,tcp_nodelay=True)

        self.reference_pub = rospy.Publisher("los/desired_yaw",Float32,queue_size=1,tcp_nodelay=True)

        self.pose = None
        self.waypoint_queue = queue.Queue()
        self.current_waypoint = Pose()
        self.last_waypoint = Pose()

        self.tangential_transform = np.eye(3)
        self.spline_coord_center = np.array([0,0])
        self.pi_p = None

        self.first_wpt_set = False
        self.stop = True

        self.has_correction = False

        #Coordinate conversion client
        self.converter_client = GeodeticConverterClient()
        first_odom:Odometry = rospy.wait_for_message("odom", Odometry)
        _,_,yaw = euler_from_quaternion([first_odom.pose.pose.orientation.x,first_odom.pose.pose.orientation.y,first_odom.pose.pose.orientation.z,first_odom.pose.pose.orientation.w])
        self.desired_yaw = yaw
        self.desired_speed = 0
        self.correction = Twist()
        self.correction.linear.x = 1

        self.converter_client.add_frame("global_enu",first_odom.pose.pose.position.x,first_odom.pose.pose.position.y,0,True)

        #Configuration
        self.los_distance = 40
        self.reference_publish_timer = rospy.Timer(rospy.Duration(0.1),self.publish_reference_cb)

        #Register subscribers
        rospy.Subscriber("odom",Odometry,self.odom_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("mission_planner/desired_speed",Twist,self.speed_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("mission_planner/geo_waypoint",Pose,self.geo_waypoint_cb,queue_size=10,tcp_nodelay=True)
        rospy.Subscriber("mc/system_reinit",reinit, self.reset,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("colav/correction",Twist,self.correction_cb,queue_size=1,tcp_nodelay=True)
        

    def reset(self, msg:Bool):
        print("LOS reset start")
        self.pose = None
        self.current_waypoint = None
        self.waypoint_queue = queue.Queue()
        self.last_waypoint = Pose()

        self.tangential_transform = np.eye(3)
        self.spline_coord_center = np.array([0,0])
        self.pi_p = None

        self.stop = True
        self.has_correction = False

        first_odom:Odometry = rospy.wait_for_message("odom", Odometry)
        _,_,yaw = euler_from_quaternion([first_odom.pose.pose.orientation.x,first_odom.pose.pose.orientation.y,first_odom.pose.pose.orientation.z,first_odom.pose.pose.orientation.w])
        self.converter_client.add_frame("global_enu",first_odom.pose.pose.position.x,first_odom.pose.pose.position.y,0,True)
        self.desired_yaw = yaw
        self.desired_speed = 0
        self.correction = Twist()
        self.correction.linear.x = 1

        

    def odom_cb(self,msg: Odometry) -> None:
        self.pose = msg.pose.pose

        if self.current_waypoint is None:
            #rospy.logwarn_throttle(1,"Current waypoint is none")
            return

        #Check if should switch waypoint
        if abs(Geodesic.WGS84.Inverse(self.pose.position.y,self.pose.position.x,self.current_waypoint.position.y,self.current_waypoint.position.x)["s12"])<20:
            #print("Within circle of acceptance, switching waypoint")
            self.waypoint_reached_pub.publish(self.current_waypoint)
            self.switch_waypoint()

        #Calculate crosstrack
        crosstrack_error = self.calculate_crosstrack_error()
        if crosstrack_error is None:
            #rospy.logerr("[LOS] Failed to calculate crosstrack due to frame conversion, skipping iteration")
            return

        #Calculate desired yaw
        if self.pi_p is None:
            #rospy.logerr("[LOS] pi_p is none, skipping iteration")
            return

        self.desired_yaw = self.pi_p - math.atan2(crosstrack_error,self.los_distance)
        


    def euclidean_distance(self,point_a:Pose,point_b:Pose)->float:
        return np.sqrt(pow(point_a.position.x-point_b.position.x,2)+pow(point_a.position.y-point_b.position.y,2))

    def geo_waypoint_cb(self,msg:Pose) -> None:
        #Convert to cartesian coordinates
        wpt = Pose(Point(msg.position.x,msg.position.y,msg.position.z),Quaternion(0,0,0,1))
        if self.current_waypoint is None:
            self.current_waypoint = wpt

            #Do the calculation normally done in waypoint switching, using current position as last waypoint
            last_wpt_cart = self.converter_client.convert("WGS84",[self.pose.position.x,self.pose.position.y,self.current_waypoint.position.z],"global_enu")
            self.last_waypoint.position = self.pose.position

            self.last_waypoint.orientation.x = self.pose.orientation.x
            self.last_waypoint.orientation.y = self.pose.orientation.y
            self.last_waypoint.orientation.z = self.pose.orientation.z
            self.last_waypoint.orientation.w = self.pose.orientation.w

            current_waypoint_cart = self.converter_client.convert("WGS84",[self.current_waypoint.position.x,self.current_waypoint.position.y,self.current_waypoint.position.z],"global_enu")

            #Determine current tangential frame of spline between last and current
            self.spline_coord_center[0] = last_wpt_cart[0]
            self.spline_coord_center[1] = last_wpt_cart[1]
            rotation_angle = math.atan2((current_waypoint_cart[1]-last_wpt_cart[1]),(current_waypoint_cart[0]-last_wpt_cart[0]))
            R_ab = np.array([[math.cos(rotation_angle),-math.sin(rotation_angle)],[math.sin(rotation_angle),math.cos(rotation_angle)]])
            r_ab_a = self.spline_coord_center.reshape((2,1))

            #Note: If sets this together to transformation matrix we get T_ab which takes point in tangential frame and throws to global frame.
            # We must thus take inverse, which is not transpose for the entire T_ab. Notation from p.223 in Modelling and SImulation for Automatic Control. 
            R_ba = R_ab.T
            r_ba_b = -R_ba@r_ab_a

            self.tangential_transform[:2,:2] = R_ba
            self.tangential_transform[:2,2] = r_ba_b.flatten()

            self.pi_p = rotation_angle
            self.stop = False

        else:
            self.waypoint_queue.put(wpt)
    
    def speed_cb(self,msg:Twist) -> None:
        self.desired_speed = msg.linear.x

    def switch_waypoint(self) -> None:
        if self.waypoint_queue.qsize()==0:
            #print("No waypoint in queue, stopping and waiting for new")
            self.stop = True
            return
        else:
            self.stop=False
        self.converter_client.add_frame("global_enu",self.current_waypoint.position.x,self.current_waypoint.position.y,0,True)

        last_wpt_cart = self.converter_client.convert("WGS84",[self.current_waypoint.position.x,self.current_waypoint.position.y,self.current_waypoint.position.z],"global_enu")
        self.last_waypoint.position = self.current_waypoint.position

        self.last_waypoint.orientation.x = self.current_waypoint.orientation.x
        self.last_waypoint.orientation.y = self.current_waypoint.orientation.y
        self.last_waypoint.orientation.z = self.current_waypoint.orientation.z
        self.last_waypoint.orientation.w = self.current_waypoint.orientation.w

        self.current_waypoint = self.waypoint_queue.get()
        current_waypoint_cart = self.converter_client.convert("WGS84",[self.current_waypoint.position.x,self.current_waypoint.position.y,self.current_waypoint.position.z],"global_enu")

        #Determine current tangential frame of spline between last and current
        self.spline_coord_center[0] = last_wpt_cart[0]
        self.spline_coord_center[1] = last_wpt_cart[1]
        rotation_angle = math.atan2((current_waypoint_cart[1]-last_wpt_cart[1]),(current_waypoint_cart[0]-last_wpt_cart[0]))
        R_ab = np.array([[math.cos(rotation_angle),-math.sin(rotation_angle)],[math.sin(rotation_angle),math.cos(rotation_angle)]])
        r_ab_a = self.spline_coord_center.reshape((2,1))

        #Note: If sets this together to transformation matrix we get T_ab which takes point in tangential frame and throws to global frame.
        # We must thus take inverse, which is not transpose for the entire T_ab. Notation from p.223 in Modelling and SImulation for Automatic Control. 
        R_ba = R_ab.T
        r_ba_b = -R_ba@r_ab_a

        self.tangential_transform[:2,:2] = R_ba
        self.tangential_transform[:2,2] = r_ba_b.flatten()

        self.pi_p = rotation_angle

    def publish_reference_cb(self,timer):
        if self.pose==None or self.current_waypoint==None:
            #rospy.logerr_throttle(1,f"PublishReference requested but pose is none: {self.pose==None} and waypoint is none:  {self.current_waypoint==None}")
            return


        msg_corrected = Twist()
        msg_corrected.angular.z = self.desired_yaw+self.correction.angular.z

        msg = Twist()
        if(self.stop):
            msg_corrected.linear.x = 0
        else:
            msg_corrected.linear.x = self.desired_speed*self.correction.linear.x
        msg.linear.x = self.desired_speed
        msg.angular.z = self.desired_yaw

        #self.reference_pub.publish(Float32(msg.angular.z))
        self.setpoint_pub.publish(msg)
        if self.has_correction:
            self.corrected_setpoint_pub.publish(msg_corrected)

    def correction_cb(self,msg:Twist)->None:
        self.correction = msg
        self.has_correction=True

    def calculate_crosstrack_error(self):
        position_enu = self.converter_client.convert("WGS84",[self.pose.position.x,self.pose.position.y,self.pose.position.z],"global_enu")
        if position_enu is None:
            return None
        tangential_pose_homogenous = self.tangential_transform@np.array([position_enu[0],position_enu[1],1]).reshape((3,1))
        tangential_pose = tangential_pose_homogenous[:2]
        self.debug_crosstrack.publish(Float32(tangential_pose[1]))
        self.debug_alongtrack.publish(Float32(tangential_pose[0]))
        return tangential_pose[1]

