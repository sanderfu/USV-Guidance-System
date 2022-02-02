#!/usr/bin/env python
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker
import queue
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from usv_map.geotf_ros_python import GeodeticConverterClient


class LOS:
    """
        LOS controller for the USV
    """
    def __init__(self) -> None:
        rospy.Subscriber("odom",Odometry,self.odom_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("mission_planner/desired_speed",Twist,self.speed_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("mission_planner/geo_waypoint",Pose,self.geo_waypoint_cb,queue_size=10,tcp_nodelay=True)
        rospy.Subscriber("external_reset/los",Bool, self.reset,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber("colav/correction",Twist,self.correction_cb,queue_size=1,tcp_nodelay=True)
        rospy.Timer(rospy.Duration(0.1),self.publish_reference_cb)

        self.debug_crosstrack = rospy.Publisher("los/crosstrack_error",Float32,queue_size=1,tcp_nodelay=True)
        self.debug_alongtrack = rospy.Publisher("los/alongtrack",Float32,queue_size=1,tcp_nodelay=True)
        self.setpoint_pub = rospy.Publisher("los/setpoint",Twist, queue_size=1,tcp_nodelay=1)
        self.corrected_setpoint_pub = rospy.Publisher("los/corrected_setpoint",Twist, queue_size=1,tcp_nodelay=1)

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

        self.desired_yaw = 0
        self.desired_speed = 0
        self.correction = Twist()
        self.correction.linear.x = 1

        #Configuration
        self.los_distance = 40

        #Coordinate conversion client
        self.converter_client = GeodeticConverterClient()

        #Visualization
        self.first_viz = True
        self.waypoints_viz = Marker()
        self.los_vector_viz = Marker()
        self.waypoints_pub = rospy.Publisher("los/visualize_waypoints",Marker,queue_size=1,tcp_nodelay=True)
        self.los_vector_pub = rospy.Publisher("los/visualize_vector",Marker,queue_size=1,tcp_nodelay=True)
        self.visualize_timer = rospy.Timer(rospy.Duration(0.1),self.visualize_los_vector)
        self.initialize_visualization()

    def reset(self, msg:Bool):
        print("LOS reset")
        self.current_waypoint = Pose()
        self.last_waypoint = Pose()
        self.waypoint_queue.queue.clear()
        self.tangential_transform = np.eye(3)
        self.spline_coord_center = np.array([0,0])
        self.pi_p = None

        #Reset visualization
        self.waypoints_viz.points.clear()
        self.visualize_waypoints()


    def odom_cb(self,msg: Odometry) -> None:
        self.pose = msg.pose.pose

        if self.current_waypoint==Pose():
            #print("Current waypoint not set, setting using odometry")
            self.current_waypoint.position = msg.pose.pose.position

        #Check if should switch waypoint
        if self.euclidean_distance(msg.pose.pose,self.current_waypoint)<10:
            #print("Within circle of acceptance, switching waypoint")
            self.switch_waypoint()

        if self.stop==True:
            self.desired_speed = 0
            return

        #Calculate crosstrack
        crosstrack_error = self.calculate_crosstrack_error()

        #Calculate desired yaw
        self.desired_yaw = self.pi_p - math.atan2(crosstrack_error,self.los_distance)

    def euclidean_distance(self,point_a:Pose,point_b:Pose)->float:
        return np.sqrt(pow(point_a.position.x-point_b.position.x,2)+pow(point_a.position.y-point_b.position.y,2))

    def geo_waypoint_cb(self,msg:Pose) -> None:
        #Convert to cartesian coordinates
        wpt_cart = self.converter_client.convert("WGS84",[msg.position.x,msg.position.y,msg.position.z],"global_enu")
        wpt = Pose(Point(wpt_cart[0],wpt_cart[1],wpt_cart[2]),Quaternion(0,0,0,1))
        self.waypoint_queue.put(wpt)
        self.waypoints_viz.points.append(wpt.position)
        self.visualize_waypoints()
    
    def speed_cb(self,msg:Twist) -> None:
        self.desired_speed = msg.linear.x

    def switch_waypoint(self) -> None:
        if self.waypoint_queue.qsize()==0:
            #print("No waypoint in queue, stopping and waiting for new")
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

    def publish_reference_cb(self,timer):
        if self.pose==None:
            #print("publishReferenceCb but no pose yet received, thus ignore")
            return
        msg_corrected = Twist()
        msg_corrected.linear.x = self.desired_speed*self.correction.linear.x
        msg_corrected.angular.z = self.desired_yaw+self.correction.angular.z

        msg = Twist()
        msg.linear.x = self.desired_speed
        msg.angular.z = self.desired_yaw

        #self.reference_pub.publish(Float32(msg.angular.z))
        self.setpoint_pub.publish(msg)
        self.corrected_setpoint_pub.publish(msg_corrected)

    def correction_cb(self,msg:Twist)->None:
        self.correction = msg

    def calculate_crosstrack_error(self):
        tangential_pose_homogenous = self.tangential_transform@np.array([self.pose.position.x,self.pose.position.y,1]).reshape((3,1))
        tangential_pose = tangential_pose_homogenous[:2]
        self.debug_crosstrack.publish(Float32(tangential_pose[1]))
        self.debug_alongtrack.publish(Float32(tangential_pose[0]))
        return tangential_pose[1]

    def initialize_visualization(self):
        self.waypoints_viz.header.frame_id="map"
        self.waypoints_viz.header.stamp = rospy.Time.now()
        self.waypoints_viz.ns = "waypoints"
        self.waypoints_viz.id = 0
        self.waypoints_viz.type = Marker.SPHERE_LIST
        self.waypoints_viz.scale.x = 10
        self.waypoints_viz.scale.y = 10
        self.waypoints_viz.scale.z = 10
        self.waypoints_viz.action = Marker.ADD
        self.waypoints_viz.color.a = 1.0
        self.waypoints_viz.color.r = 1.0
        self.waypoints_viz.pose.orientation.w = 1.0

        self.los_vector_viz.header.frame_id="map"
        self.waypoints_viz.header.stamp = rospy.Time.now()
        self.los_vector_viz.ns= "los_vector"
        self.los_vector_viz.type = Marker.ARROW
        self.los_vector_viz.scale.x = 10*self.desired_speed
        self.los_vector_viz.scale.y = 1
        self.los_vector_viz.scale.z = 1
        self.los_vector_viz.color.a = 1.0
        self.los_vector_viz.color.b = 1.0

    def visualize_waypoints(self):
        self.waypoints_pub.publish(self.waypoints_viz)
    
    def visualize_los_vector(self,timer):
        try: 
            self.los_vector_viz.pose.position = self.pose.position
            q = quaternion_from_euler(0,0,self.desired_yaw)
            self.los_vector_viz.scale.x = 10*self.desired_speed
            self.los_vector_viz.pose.orientation.x = q[0]
            self.los_vector_viz.pose.orientation.y = q[1]
            self.los_vector_viz.pose.orientation.z = q[2]
            self.los_vector_viz.pose.orientation.w = q[3]

            self.los_vector_pub.publish(self.los_vector_viz)
        except AttributeError as e:
            return

