#!/usr/bin/env python3
# Author: Sander Furre
import rospy
import numpy as np
import rospkg
import os
from geometry_msgs.msg import Pose, Twist, Point,Quaternion, Vector3
def main():
    rospy.init_node('static_mission_planner_node')
    rospy.sleep(2)
    wpt_pub = rospy.Publisher("mission_planner/geo_waypoint",Pose,queue_size=1,tcp_nodelay=True)
    speed_pub = rospy.Publisher("mission_planner/desired_speed",Twist,queue_size=1,tcp_nodelay=True)
    pkg_path = rospkg.RosPack().get_path("usv_mission_planner")
    csv_path = os.path.join(pkg_path,"data/",rospy.get_param("waypoint_file"))
    geo_wpts = np.loadtxt(csv_path,delimiter=",")
    for wpt in geo_wpts:
        wpt_pub.publish(Pose(Point(wpt[0],wpt[1],0),Quaternion(0,0,0,1)))
        rospy.sleep(0.01)
    speed_pub.publish(Twist(Vector3(5,0,0),Vector3(0,0,0)))

    while(not rospy.is_shutdown()):
        rospy.sleep(1)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass