# High-level gait/body request. This is never a raw actuator command.

uint8 GAIT_STAND=0
uint8 GAIT_SIT=1
uint8 GAIT_TRIPOD=2
uint8 GAIT_RIPPLE=3
uint8 GAIT_WAVE=4
uint8 GAIT_CRAWL=5

std_msgs/Header header
uint32 sequence
builtin_interfaces/Duration valid_for
uint8 gait
float32 body_height_m
float32 stride_length_m
float32 step_height_m
float32 duty_factor
float32 speed_scale
float32 normalized_vx
float32 normalized_vy
float32 normalized_wz
geometry_msgs/Vector3 body_translation_m
geometry_msgs/Vector3 body_rpy_rad