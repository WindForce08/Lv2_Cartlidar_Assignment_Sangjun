# manipulator_node 안전 래치 (v1.2 §4) — /arm/safety_state
uint8 NORMAL=0
uint8 HOLD=1
uint8 ESTOP=2
std_msgs/Header header
uint8 state
string reason
