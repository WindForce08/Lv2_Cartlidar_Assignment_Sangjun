# perception_node → mission · drive · manipulator  (Reliable, depth 1 · 받는 쪽은 stamp 300 ms 지나면 버림)
# 동결 — 변경은 docs/interfaces.md PR + Controller 팀장 승인
std_msgs/Header header                  # stamp = 촬영 시각 (ROS 시계) — 300 ms 규칙 기준
uint8 mode                              # 현재 켜진 모드 비트 (SetPerceptionMode 상수). 0 = OFF 알림(받는 쪽 초기화)
bool object_found
geometry_msgs/PoseStamped object_pose   # base_link 기준 (SEARCH_FRONT · SEARCH_REAR 결과)
bool offset_valid
geometry_msgs/Vector3 arm_offset        # ARM_ALIGN: x · y 화면 중심 대비 (-1~1, 오른쪽 · 아래 +), z 박스 폭 / 화면 폭
float32 confidence
bool tag_found                          # READ_TAG 결과
int32 tag_id
geometry_msgs/PoseStamped tag_pose      # arm_camera_link 기준
