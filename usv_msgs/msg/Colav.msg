#Message for visualization from COLAV system
time time
nav_msgs/Odometry ownship_odom
geometry_msgs/Point ownship_pose_local
nav_msgs/Odometry[] obstacles_odom
geometry_msgs/Point[] obstacle_pose_local
float64[] obstacle_desync

float64 speed_correction
float64 course_correction
float64 cost
usv_msgs/ColavPath path

usv_msgs/ColavPath[] path_options
float64[] cost_options

#Benchmark data
float64 getBestControlOffset_time
float64[] evaluateCandidate_time
float64[] simulateOwnship_time
float64[] costFnc_time
float64[] checkForCollision_time


