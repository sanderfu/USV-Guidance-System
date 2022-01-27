#!/usr/bin/env python3
# Author: Sander Furre
import rospy
import rosnode
from usv_map.geotf_ros_python import GeodeticConverterClient

def main():
    rospy.init_node('test_conv_node')
    converter_client:GeodeticConverterClient = GeodeticConverterClient()
    converter_client.add_frame(name="enu1",lon=-73.9761,lat=40.5612,alt=0)

    result = converter_client.convert("WGS84",[-73.9761,40.5612,0],"enu1")
    print(f"Converted to: {result[0]}, {result[1]}, {result[2]}")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass