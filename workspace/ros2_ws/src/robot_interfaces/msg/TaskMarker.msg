# perception_node → mission  (Reliable, depth 1). 장소 태그 L1~L3 (tag36h11 ID 12~14, 48 mm)
std_msgs/Header header      # stamp = 촬영 시각, frame_id = base_link
bool found                  # false = 보고 있는 카메라에 마커 없음
int32 marker_id
geometry_msgs/Pose pose     # base_link 기준 마커 중심. orientation 의 z 축 = 마커 면에서 밖으로
float32 distance            # 카메라에서 마커 중심까지 [m]
uint8 camera                # Puck.CAMERA_* 와 같은 값
