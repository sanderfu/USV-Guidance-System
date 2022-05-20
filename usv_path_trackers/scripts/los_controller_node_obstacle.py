#!/usr/bin/env python3
# Author: Sander Furre
import rospy

from usv_path_trackers.los_obstacle import LOS

def main():
    rospy.init_node('los_guidance_node')
    los = LOS()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass