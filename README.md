# ros2_topic_logger

ROS2 토픽을 CSV로 저장하는 로깅 모듈 예제입니다.

## 핵심 기능
- 시간 단위 폴더 저장: `root/YYYY/MM/DD/HH/...`
- 토픽별 CSV 저장
- 개별 파일 크기 제한 초과 시 rotate
- 전체 저장소 크기 제한 / 보관 기간 관리
- 큰 데이터(`LaserScan`)는 이벤트 발생 시점 전후 구간만 raw CSV 저장
- 일반 수치 토픽은 `always`, `periodic` 모드 지원

## 현재 지원 타입
- `sensor_msgs/msg/Imu`
- `nav_msgs/msg/Odometry`
- `geometry_msgs/msg/PoseStamped`
- `sensor_msgs/msg/LaserScan` (`event_window` 전용)

## 빌드
```bash
cd ~/ros2_ws
cp -r /path/to/ros2_topic_logger src/
colcon build --packages-select ros2_topic_logger
source install/setup.bash
```

## 실행
```bash
ros2 run ros2_topic_logger topic_logger_node --ros-args -p config_path:=/absolute/path/to/logger.yaml
```

## LaserScan event-window 동작
- 평소에는 최근 `pre_buffer_sec` 구간만 메모리에 유지
- `min_valid_range <= trigger_distance_threshold` 이 되면 이벤트 시작
- 이벤트 시작 시 pre-buffer를 먼저 dump
- 이후 `post_buffer_sec` 동안 raw CSV 저장
- 이벤트 중 다시 trigger 되면 종료 시각을 연장

## 다음 단계 추천
- 비동기 writer thread 추가
- PointCloud2 지원
- 조건식 일반화
- 시각화 모듈에서 event CSV와 일반 CSV를 동시에 로드하는 기능 추가
