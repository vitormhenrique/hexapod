# Simulation-only fused foot-contact snapshot. `source_time_ms` uses the
# ControllerCore monotonic time domain; `header.stamp` is ROS publication time.
# The six entries are keyed by `leg_index`, not their array order. A clear bit
# in `present_mask` means the adapter must expose that foot as unavailable.

std_msgs/Header header

uint8 VALIDITY_UNKNOWN=0
uint8 VALIDITY_FRESH=1
uint8 VALIDITY_STALE=2
uint8 VALIDITY_FAULT=3

uint32 source_time_ms
uint8 validity
uint8 present_mask
FootContact[6] feet