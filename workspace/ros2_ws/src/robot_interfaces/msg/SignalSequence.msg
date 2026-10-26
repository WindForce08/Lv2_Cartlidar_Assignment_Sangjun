# perception_node → mission  (Reliable, depth 1). 신호등 점멸 순서 (READ_SIGNAL)
#   한 묶음 = 점멸 12회, 묶음 사이 텀을 두고 반복. 같은 묶음이 두 번 보이면 CONFIRMED (reset 은 set_mode 의 READ_SIGNAL 끔)
std_msgs/Header header      # stamp = 마지막으로 쓴 영상 촬영 시각, frame_id = base_link
uint8 SEARCHING=0           # 신호등 위치를 아직 모름 (같은 자리에서 켜졌다 꺼지는 등을 찾는 중)
uint8 COUNTING=1            # 신호등을 찾았고 점멸을 세는 중
uint8 CANDIDATE=2           # 완성 묶음 1개 (sequence = 그 순서, 아직 확정 아님)
uint8 CONFIRMED=3           # 같은 묶음 2개 → 확정
uint8 MISMATCH=4            # 완성 묶음 2개 이상이 서로 다름 (sequence = 마지막 묶음)
uint8 status
string sequence             # 예: "RGRRGRGGYGYG" (R 빨강 · Y 노랑 · G 초록)
uint8 red                   # sequence 의 색별 횟수
uint8 yellow
uint8 green
uint8 cycles                # 본 완성 묶음 수 (점멸 수가 맞는 것만)
uint8 LIGHT_OFF=0
uint8 LIGHT_RED=1
uint8 LIGHT_YELLOW=2
uint8 LIGHT_GREEN=3
uint8 light                 # 마지막 영상의 신호등 상태 (light_visible 일 때만 의미)
bool light_visible          # 마지막 처리 주기에 신호등 자리가 카메라 화면 안에 있었음
bool position_valid
geometry_msgs/Point position   # base_link 기준 신호등 위치 (position_valid 일 때)
float32 t_on                # 잰 켜짐 길이 중앙값 [s]
float32 t_off               # 잰 점멸 사이 꺼짐 길이 중앙값 [s]
float32 t_term              # 잰 묶음 사이 텀 [s]: 점멸 수가 맞는 두 묶음 사이 꺼짐의 중앙값 (모르면 0)
uint8 camera                # 마지막으로 신호등을 본 카메라 (Puck.CAMERA_*)
