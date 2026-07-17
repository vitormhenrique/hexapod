# One fused foot-contact observation for the simulation-only ControllerCore
# input boundary. It preserves the portable firmware contact semantics rather
# than modeling raw I2C devices or sensor physics.

uint8 STATE_AIR=0
uint8 STATE_NEAR=1
uint8 STATE_TOUCH=2
uint8 STATE_LOADED=3
uint8 STATE_RELEASE=4
uint8 STATE_STALE=5
uint8 STATE_FAULT=6

uint8 leg_index
uint8 state
uint8 confidence
uint16 proximity_raw
int32 pressure_raw
int32 pressure_baseline
int32 pressure_delta