# perception_node → mission · manipulator  (Reliable, depth 1)
std_msgs/Header header      # stamp = 결과에 쓴 영상 중 가장 오래된 촬영 시각, frame_id = base_link
Puck[] pucks                # base_link 원점에서 수평 거리가 가까운 것부터
int32 target_index          # 집을 후보: 제외 상자 밖이고 (번호를 읽었거나 confidence ≥ 기준) 중 가장 가까운 것. 없으면 -1
