#!/usr/bin/env python3
# Author: Sander Furre
import rospy

from usv_path_trackers.turn_radius_enforcer import TurnRadiusEnforcer

def main():
    rospy.init_node('los_guidance_node')
    los = TurnRadiusEnforcer()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass