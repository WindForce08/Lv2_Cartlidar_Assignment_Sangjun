# manipulator_node 스킬 실행 FSM 상태 (v1.2 §3) — /arm/skill_state
uint8 IDLE=0
uint8 PRECHECK=1
uint8 RUNNING=2
uint8 VERIFY=3
uint8 HOLD=4
uint8 DONE=5
uint8 FAILED=6
uint8 REJECTED=7
std_msgs/Header header
string skill              # pose:<name> / pick
uint8 state
string reason
