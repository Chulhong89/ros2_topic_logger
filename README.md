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

### 직접 실행
```bash
ros2 run ros2_topic_logger topic_logger_node --ros-args -p config_path:=/absolute/path/to/logger.yaml
```

### Launch 파일로 실행 (프로덕션)
```bash
# 기본 config/logger.yaml 사용
ros2 launch ros2_topic_logger logger.launch.py

# 커스텀 설정 파일 지정
ros2 launch ros2_topic_logger logger.launch.py config_path:=/absolute/path/to/logger.yaml
```

### Launch 파일로 실행 (테스트)
```bash
ros2 launch ros2_topic_logger test_logger.launch.py
```

`test_logger.launch.py`는 로거 노드(`logger_test.yaml` 설정)와 아래 더미 퍼블리셔를 함께 시작합니다.

| 토픽 | 타입 | 모드 | 발행 주기 |
|---|---|---|---|
| `/test/imu` | `sensor_msgs/msg/Imu` | always | 20 Hz |
| `/test/odom` | `nav_msgs/msg/Odometry` | periodic (100 ms) | 10 Hz |
| `/test/pose` | `geometry_msgs/msg/PoseStamped` | always | 1 Hz |
| `/test/scan` | `sensor_msgs/msg/LaserScan` | event_window | 5 Hz (2초 후 시작) |

LaserScan은 `ranges = [0.5 m, ...]`로 발행되므로 `trigger_distance_threshold: 1.0` 조건을 즉시 충족합니다.

CSV 출력 위치: `/tmp/ros2_logger_integration_test/`

```bash
# 출력 파일 확인
find /tmp/ros2_logger_integration_test -name "*.csv"

# 내용 미리 보기
head /tmp/ros2_logger_integration_test/*/test_imu*.csv
```

## LaserScan event-window 동작
- 평소에는 최근 `pre_buffer_sec` 구간만 메모리에 유지
- `min_valid_range <= trigger_distance_threshold` 이 되면 이벤트 시작
- 이벤트 시작 시 pre-buffer를 먼저 dump
- 이후 `post_buffer_sec` 동안 raw CSV 저장
- 이벤트 중 다시 trigger 되면 종료 시각을 연장

## 테스트

### 사전 요구사항

- Docker 설치 및 실행 중

### 빌드 + 전체 테스트 (단일 명령)

```bash
cd /path/to/ros2_topic_logger
docker build -t ros2_topic_logger_test .
```

Dockerfile이 패키지 빌드와 테스트 실행을 모두 수행합니다.  
빌드가 성공하면 모든 테스트가 통과한 것입니다.  
테스트가 실패하면 빌드가 오류와 함께 종료되고 실패 내용이 출력됩니다.

### 컨테이너 내부에서 대화형 실행

```bash
# 빌드된 이미지로 컨테이너 시작
docker run --rm -it ros2_topic_logger_test bash

# 컨테이너 내부 — 전체 테스트 실행
source /opt/ros/foxy/setup.bash
source /ws/install/setup.bash
cd /ws
colcon test --packages-select ros2_topic_logger --event-handlers console_direct+

# 결과 요약 출력
colcon test-result --verbose
```

### 특정 테스트 바이너리만 실행

```bash
docker run --rm -it ros2_topic_logger_test bash

# 컨테이너 내부
/ws/build/ros2_topic_logger/test_topic_config
/ws/build/ros2_topic_logger/test_event_buffer
/ws/build/ros2_topic_logger/test_message_serializers
/ws/build/ros2_topic_logger/test_csv_writer
/ws/build/ros2_topic_logger/test_retention_manager
/ws/build/ros2_topic_logger/test_topic_logger_node
```

특정 테스트 케이스만 실행:

```bash
/ws/build/ros2_topic_logger/test_csv_writer \
  --gtest_filter="CsvWriterTest.FileRotatesWhenSizeLimitReached"
```

### 정상 출력 (전체 통과)

```
100% tests passed, 0 tests failed out of 6

Summary: 60 tests, 0 errors, 0 failures, 0 skipped
All tests passed
```

| 테스트 바이너리 | 테스트 수 | 검증 내용 |
|---|---|---|
| `test_topic_config` | 7 | `sanitize_topic_name` 엣지 케이스 |
| `test_event_buffer` | 7 | `EventBuffer<T>` push/prune/snapshot |
| `test_message_serializers` | 18 | CSV 헤더, 행 형식, LaserScan 트리거 |
| `test_csv_writer` | 9 | 파일 생성, rotation, 디렉토리 구조 |
| `test_retention_manager` | 8 | 나이·크기 기반 파일 보존 정책 |
| `test_topic_logger_node` | 5 | 노드 전체: pub→sub→CSV (4가지 메시지 타입) |

## 다음 단계 추천
- 비동기 writer thread 추가
- PointCloud2 지원
- 조건식 일반화
- 시각화 모듈에서 event CSV와 일반 CSV를 동시에 로드하는 기능 추가
