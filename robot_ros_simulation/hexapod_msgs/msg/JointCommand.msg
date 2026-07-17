# Traceable ControllerCore result. This is output-only and not an actuator API.

std_msgs/Header header
uint32 source_time_ms
uint32 config_revision
uint8 safety_state
uint8 fault_reason
uint8 command_source
bool motion_gate
bool goal_valid
string[] name
uint16[] goal_tick
float64[] position_rad
bool[] clamped