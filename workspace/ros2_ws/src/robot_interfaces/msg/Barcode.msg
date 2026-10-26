# perception_node → mission · manipulator  (Reliable, depth 1). 바코드 라벨 (READ_BARCODE)
#   같은 토픽으로 두 가지 결과를 낸다(source 로 구분).
#   LOCATE: 앞 D435 컬러 + 깊이 depth_frames 장 평균으로 라벨 자리를 찾음. 로봇이 멈춘 동안 평균하고,
#           평균 중 차체가 움직이면(odom) 다시 모은다. 한 묶음마다 한 번 낸다
#   READ  : 팔 카메라 영상 한 장에서 CODE128 문자열을 읽음. 서로 다른 문자열이 둘 이상이면 못 읽음
std_msgs/Header header          # stamp = 쓴 영상 촬영 시각 (LOCATE 는 평균에 쓴 마지막 컬러), frame_id = base_link
uint8 LOCATE=1
uint8 READ=2
uint8 source
# LOCATE
geometry_msgs/Point[] positions # 라벨 가운데 (base_link), 점수 높은 순. 비면 못 찾음
float32[] scores                # 라벨 그림과의 정규 상관 (문턱 barcode.thr 이상)
uint8 SURFACE_WALL=0
uint8 SURFACE_TOP=1
uint8[] surfaces                # 라벨이 붙은 면 (벽판 · 선반 윗면), positions 와 같은 순서
uint8 depth_frames              # 평균한 깊이 장수
# READ
bool read_ok                    # 문자열 하나를 읽음
string text                     # read_ok 일 때 읽은 문자열 (형식 검사 없음)
uint8 symbols                   # quality 거르기(barcode.read_min_quality)를 통과한 CODE128 기호 수 (같은 문자열 중복 포함)
