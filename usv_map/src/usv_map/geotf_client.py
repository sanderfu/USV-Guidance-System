import rospy
from usv_map.srv import add_frame,frame_conversion
from geometry_msgs.msg import Point

class GeodeticConverterClient:
    def __init__(self) -> None:
        self.add_frame_client_ = rospy.ServiceProxy('/map/add_frame',add_frame)
        self.convert_point_client_ = rospy.ServiceProxy('/map/frame_conversion',frame_conversion)
    def add_frame(self, name:str, lon:float, lat:float, alt:float)->None:
        try:
            resp = self.add_frame_client_(frame_name=name,latitude=lat,longitude=lon,altitude=alt)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to add frame!")
    def convert(self,from_frame:str,from_vec:list,to_frame:str)->list:
        try:
            from_point:Point = Point(from_vec[0],from_vec[1],from_vec[2])
            resp = self.convert_point_client_(from_frame=from_frame,from_point=from_point,to_frame=to_frame)
            return [resp.to_point.x,resp.to_point.y,resp.to_point.z]  
        except rospy.ServiceException as e:
            rospy.logerr("Failed to convert coordinates!")
            return None
    