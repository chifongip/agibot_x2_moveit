# State of the manipulation server's independent RoboJuDo posture publisher.
# target_active only means this server is continuously publishing its target;
# RoboJuDo may retain its last accepted target after the publisher is released.
bool enabled
bool execution_enabled
bool target_active
float64 target_height
float64 target_waist_yaw
float64 feedback_window_timeout_sec
string endpoint
string detail
