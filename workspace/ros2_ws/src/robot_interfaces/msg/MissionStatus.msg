# mission_node → 모니터링 (rviz · 노트북), 2 Hz + 변경 시
builtin_interfaces/Time stamp
int32 tag_id          # -1 = 아직 못 읽음
uint16 step_index
string step_name
uint8 IDLE=0
uint8 RUNNING=1
uint8 PAUSED=2
uint8 DONE=3
uint8 FAILED=4
uint8 state
string reason         # 실패 · 정지 이유
