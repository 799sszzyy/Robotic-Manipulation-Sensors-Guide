# TactileData.msg (ROS2 interface definition)
std_msgs/Header header
int32 num_taxels
float64[] capacitance_values
float64[] pressure_values
geometry_msgs/Point[] positions
bool[] contact_detected
# MultiModalState.msg
std_msgs/Header header
geometry_msgs/Pose object_pose
geometry_msgs/Wrench contact_wrench
TactileData tactile_data
sensor_msgs/Image visual_features
float64 grasp_quality
