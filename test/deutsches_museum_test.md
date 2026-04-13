# Deutsches Museum Dataset Test

## Dataset

- **Source bag (ROS1):** `/home/kch/work/datasets/deutsches_museum.bag`
- **Converted bag (ROS2):** `/home/kch/work/datasets/deutsches_museum_ros2/`
- **Duration:** ~31 minutes
- **Total messages:** 617,965

### Topics in bag

| Topic | Type | Message Count |
|---|---|---|
| `/imu` | `sensor_msgs/msg/Imu` | 478,244 |
| `/horizontal_laser_2d` | `sensor_msgs/msg/MultiEchoLaserScan` | 70,358 |
| `/vertical_laser_2d` | `sensor_msgs/msg/MultiEchoLaserScan` | 69,363 |

> Note: `MultiEchoLaserScan` topics are not currently supported by the logger. Only `/imu` is logged.

## Conversion (ROS1 → ROS2)

```bash
pip3 install rosbags --break-system-packages
rosbags-convert --src /home/kch/work/datasets/deutsches_museum.bag \
                --dst /home/kch/work/datasets/deutsches_museum_ros2
```

## Logger Config

Config file: `config/deutsches_museum.yaml`

```yaml
root_dir: /tmp/ros2_logger_deutsches_museum
max_total_size_mb: 5120
max_age_days: 7

topics:
  - topic: /imu
    type: sensor_msgs/msg/Imu
    mode: always
    max_file_size_mb: 200
```

## Running the Test

**Terminal 1 — start the logger:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/kch/work/install/setup.bash
ros2 run ros2_topic_logger topic_logger_node \
  --ros-args -p config_path:=/home/kch/work/src/ros2_topic_logger/config/deutsches_museum.yaml
```

**Terminal 2 — play the bag:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play /home/kch/work/datasets/deutsches_museum_ros2 --rate 1.0
```

## Checking Output

```bash
find /tmp/ros2_logger_deutsches_museum -name "*.csv"
head /tmp/ros2_logger_deutsches_museum/*/*/imu*.csv
```
