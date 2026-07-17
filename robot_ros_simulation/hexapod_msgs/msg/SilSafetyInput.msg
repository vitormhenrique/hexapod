# SIL-only health and RC authority snapshot. It is not a hardware control API.

std_msgs/Header header
bool rc_seen
bool rc_kill
bool rc_armed
bool rc_autonomy_enabled
bool host_estop
bool battery_valid
uint16 battery_millivolts
bool watchdog_fault
bool dxl_hard_fault
bool foot_contact_enabled
bool terrain_leveling_enabled