# Controller safety and authority result for SIL observation.

std_msgs/Header header
uint32 source_time_ms
uint32 config_revision
uint8 safety_state
uint8 fault_reason
uint8 command_source
bool motion_authorized
bool motion_gate
bool allow_dxl_power
bool allow_torque
bool goal_valid
bool simulation_output_published