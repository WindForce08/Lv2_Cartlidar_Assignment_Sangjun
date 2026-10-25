# /perception/pucks 의 퍽 하나
int32 tag_id                # AprilTag 번호, 읽지 못했으면 -1
uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_GREEN=2
uint8 COLOR_BLUE=3
uint8 color
uint8 SHAPE_UNKNOWN=0
uint8 SHAPE_CYLINDER=1
uint8 SHAPE_CUBOID=2
uint8 shape
geometry_msgs/Pose pose     # base_link 기준 퍽 중심. orientation 의 z 축 = 퍽 긴 축 (orientation_valid 일 때만)
bool orientation_valid
bool standing               # 긴 축이 위로 서 있음 (orientation_valid 일 때만)
float32 confidence          # 0~1. 태그로 읽은 퍽: 색 띠 화소 중 고른 색 비율, D435 퍽: 색 검출 점수
uint8 CAMERA_FRONT=0
uint8 CAMERA_REAR=1
uint8 CAMERA_ARM=2
uint8 camera
