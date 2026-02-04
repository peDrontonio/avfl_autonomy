# BaseDetection.msg - YOLO base detection information
std_msgs/Header header
bool detected
float32 confidence
int32 x_center
int32 y_center
int32 bbox_width
int32 bbox_height
