# B0 퍽 등록부 노드

인지 관측을 누적하는 독립 ROS 2 패키지.

- 입력: `/perception/pucks` (`robot_interfaces/PuckArray`), 메시지 촬영 시각의 world_frame←header.frame_id TF. 기본 지도 frame=`map`, 지도 좌표가 발제 원점에 정렬돼 있어야 한다. 모의 odom은 파라미터로 지정.
- 출력: `/perception/puck_survey` (`std_msgs/String`, reliable·transient_local·depth1), B0의 boxes6개·slots12개 JSON. 좌표 mm, 초기 박스 중심은 발제 고정값. `complete`가 false인 결과는 할당에 쓰지 않는다.
- 시작: `/perception/survey/start` (`std_srvs/Trigger`)가 초기화 후 수집 허용. 초기 상태는 수집 안 함.
- 완료: `/perception/survey/finish`는 색·모양·자세·ID가 각각 독립 시각/카메라 관측 최소2개에서 충돌 없이 일치하고, ID0~11이 한 번씩 있을 때만 성공. output_path로 원자적 저장 후 고정·수집 중지. 저장할 경로가 없거나 실패하면 성공 응답하지 않음.
- 초기화: `/perception/survey/reset`가 수집 중지·12칸 초기화. 각 조사에는 새 survey_id가 붙으며 이전 완료 파일은 성공한 다음 완료 때까지 유지됨. mission은 start→complete 토픽→finish 성공 응답의 순서를 지켜야 함.
- 관측 이력: output_path 옆 `puck_survey.observations.jsonl`에 survey_id·실제 촬영 stamp_ns·입력 frame·world frame·map/odom 좌표(m)·ID·색·모양·자세·카메라를 기록. 동일 슬롯의 동일 stamp·camera는 n_obs에 중복 가산하지 않음.
- 파라미터: `world_frame=map`, `merge_gate_m=0.035`(80mm 간격의 절반보다 작아야 함), `min_observations=2`, `max_age_s=0.5`, `output_path`(완료 저장 전에 필수). ROS 기본 use_sim_time 지원.
- box yaw는 이동 전 ±40mm 배치의 두 관측 중심에서 추정하며 `yaw_source=puck_pair`로 표시한다. 박스 테두리를 직접 검출했다는 뜻이 아니다. 조사가 끝난 뒤 놓인 퍽은 다른 재검증 관측 결과로 다루며 시작 박스 등록부를 덮어쓰지 않는다.
- Puck 메시지에는 confidence 하나만 있다. 색·모양에 검출 confidence≥0.6을 쓰고, 유효한 태그 ID에 색 confidence를 대신 적용하지 않는다. 이는 실제 태그 읽기 신뢰도를 실측했다는 뜻이 아니다. `conf`는 현재 입력의 근거이며 확률 보정값이 아니다.
- 태그 ID를 못 읽은 D435 관측을 위치로 같은 슬롯에 연결한다. 모르는 ID·색·모양·자세를 채워 넣지 않는다. 충돌은 다수결로 덮지 않으며 조사 재시작이 필요하다. 다음 개선은 슬롯별 재관측 초기화 서비스다.

기존 perception 노드/robot_interfaces/mission을 수정하지 않는다. 실제 카메라·YOLO·태그의 인식률이나 조사 경로 완주를 검증한 패키지가 아니라, 인지 관측을 누적·전달하는 노드다. 초기 박스가 발제 위치에서 35mm 이상 이동하거나 ±30° 가정을 넘으면 별도 박스 관측이 필요하다.

- `tagged_min_confidence`(기본0.6): 태그 ID가 있고 camera=arm인 관측의 색·형상 점수 문턱. D435는 기존0.6을 유지한다. Puck.confidence는 태그의 경우 정사각 색띠 점유율이므로 원기둥의 곡면에서는 올바른 색도0.6 미만일 수 있다. 기존 R6 방식과 연결한 모의 구성에서는0.4를 명시해 검증한다. 이는 실물 최적값·확률 보정이 아니며 낮은 점수·충돌·미관측·최소관측 조건은 계속 거부한다.
- 실행: `perception/launch/perception.launch.py`가 `use_survey:=true`(기본)일 때 설정 `perception/config/puck_survey.yaml`로 띄운다. 완료 파일은 `survey_output` 인자(기본 `~/.ros/puck_survey/puck_survey.json`). 조사는 `/perception/survey/start`·`finish`·`reset`(std_srvs/Trigger).
