# RoboRacer Fast-LIO2 + PGO MulRan Mapping Test

이 문서는 Jetson Ubuntu 24.04, ROS2 Jazzy 환경에서 Livox MID360 드라이버가 이미 설치되어 있다는 전제하에, MulRan 데이터셋으로 Fast-LIO2와 PGO 기반 최적화 맵 생성이 정상 동작하는지 확인하는 절차를 정리한다.

핵심 흐름은 다음과 같다.

```text
MulRan raw dataset
  -> MOLA MulRan ROS2 replay
  -> Fast-LIO2 ROS2
  -> /Odometry + /cloud_registered_body
  -> PGO node
  -> map.pcd + keyframe patches
```

주의할 점은 MulRan의 LiDAR가 Livox MID360이 아니라 Ouster OS1-64라는 것이다. 따라서 이 절차는 MID360 하드웨어 검증이 아니라, Fast-LIO2와 PGO 맵 생성 파이프라인 검증용이다. MID360 실주행 데이터로 넘어갈 때는 Fast-LIO2 설정을 `mid360.yaml` 계열로 바꾸고 `/livox/lidar`, `/livox/imu`를 사용한다.

## 선택한 구성

ROS2 Jazzy에서 바로 실험하기 위해 아래 조합을 권장한다.

```text
Dataset replay:
  MOLA mola_input_mulran_dataset

Fast-LIO2 frontend:
  hku-mars/FAST_LIO ROS2 branch

PGO backend:
  liangheming/FASTLIO2_ROS2 의 pgo 패키지만 사용
```

`liangheming/FASTLIO2_ROS2`의 기본 `pgo_launch.py`는 Livox `livox_ros_driver2/msg/CustomMsg`를 구독하는 자체 LIO 노드도 같이 실행한다. MulRan은 Ouster PointCloud2 데이터이므로 이 launch를 그대로 쓰지 말고, `pgo_node`만 따로 실행해서 hku Fast-LIO2의 출력 토픽을 입력으로 넣는 방식이 안전하다.

원본 Scan Context 기반 SC-PGO를 정확히 쓰고 싶다면 `gisbi-kim/FAST_LIO_SLAM`을 봐야 한다. 이 저장소는 MulRan용 launch가 준비되어 있지만 ROS1/catkin 기반이라 Ubuntu 24.04 + ROS2 Jazzy에서 바로 쓰는 흐름은 아니다. 정확한 SC-PGO 재현은 ROS1 Noetic Docker 또는 별도 포팅이 필요하다.

## 사전 조건

Jetson 또는 테스트 PC에 ROS2 Jazzy가 설치되어 있고, Livox 드라이버 workspace가 이미 빌드되어 있다고 가정한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
```

기본 의존성:

```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential \
  python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libpcl-dev libboost-all-dev \
  libgtsam-dev libyaml-cpp-dev libomp-dev \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-rviz2 ros-jazzy-tf2-ros
```

MulRan raw 파일을 ROS2 topic으로 publish하기 위해 MOLA 패키지를 설치한다.

```bash
sudo apt install -y \
  ros-jazzy-mola \
  ros-jazzy-mola-demos \
  ros-jazzy-mola-viz \
  ros-jazzy-mola-academic-datasets
```

패키지명이 배포 상태에 따라 다르면 아래로 확인한다.

```bash
apt-cache search ros-jazzy-mola
```

## MulRan 데이터셋 준비

MulRan 공식 사이트에서 원하는 sequence를 내려받는다. Fast-LIO2 + PGO 검증에는 loop가 있는 sequence가 좋다. 처음에는 `KAIST03` 또는 `Riverside02`처럼 기존 예제가 많은 sequence를 추천한다.

MOLA가 기대하는 구조는 대략 아래와 같다.

```text
~/datasets/MulRan/
  KAIST03/
    Ouster/
      156...bin
      ...
    data_stamp.csv
    global_pose.csv
    gps.csv
    ouster_front_stamp.csv
    xsens_imu.csv
```

MulRan 공식 안내처럼 2020-11-19 이후 sequence는 LiDAR만 두면 player가 제대로 동작하지 않을 수 있다. IMU와 GPS를 사용하지 않더라도 `xsens_imu.csv`, `gps.csv`가 같은 sequence 폴더에 있어야 한다.

환경 변수 설정:

```bash
export MULRAN_BASE_DIR=$HOME/datasets/MulRan
```

## Workspace 빌드

새 workspace를 만든다.

```bash
mkdir -p ~/fastlio_mulran_ws/src
cd ~/fastlio_mulran_ws/src
```

Fast-LIO2 ROS2 branch를 받는다.

```bash
git clone -b ROS2 --recursive https://github.com/hku-mars/FAST_LIO.git
```

PGO node만 쓰기 위해 `FASTLIO2_ROS2`도 받는다.

```bash
git clone https://github.com/liangheming/FASTLIO2_ROS2.git fastlio2_pgo
```

빌드한다. Livox 드라이버가 이미 설치되어 있어도 Fast-LIO2 빌드 전에 반드시 source한다.

```bash
cd ~/fastlio_mulran_ws
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select fast_lio interface pgo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Jetson에서 메모리가 부족하면 worker 수를 줄인다.

```bash
colcon build --symlink-install \
  --packages-select fast_lio interface pgo \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## MulRan replay 확인

Terminal 1에서 MulRan sequence를 ROS2 topic으로 publish한다.

```bash
source /opt/ros/jazzy/setup.bash
export MULRAN_BASE_DIR=$HOME/datasets/MulRan
ros2 launch mola_demos ros-mulran-play.launch.py mulran_sequence:=KAIST03
```

Terminal 2에서 topic을 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic list | grep -E "lidar|ouster|imu|gps|ground"
```

MOLA 버전에 따라 topic 이름이 달라질 수 있으므로, 실제 LiDAR와 IMU topic 이름을 여기서 확인해서 Fast-LIO2 config에 넣는다. LiDAR topic은 `sensor_msgs/msg/PointCloud2`, IMU topic은 `sensor_msgs/msg/Imu`여야 한다.

```bash
ros2 topic info /YOUR_LIDAR_TOPIC
ros2 topic info /YOUR_IMU_TOPIC
ros2 topic hz /YOUR_LIDAR_TOPIC
ros2 topic hz /YOUR_IMU_TOPIC
```

PointCloud2 field도 확인한다.

```bash
ros2 topic echo /YOUR_LIDAR_TOPIC --once
```

Fast-LIO2의 Ouster 모드는 PointCloud2 안에 최소한 `x`, `y`, `z`, `intensity`, `ring`, `t` 또는 호환 가능한 per-point time field가 있어야 안정적이다. `t`/`time` 계열 field가 없으면 deskew가 깨지고 맵이 두꺼워지거나 시작 직후 경고가 나온다.

## Fast-LIO2 MulRan 설정

config 폴더를 따로 만들고 Ouster 설정을 복사한다.

```bash
mkdir -p ~/fastlio_mulran_ws/config
cp ~/fastlio_mulran_ws/src/FAST_LIO/config/ouster64.yaml \
  ~/fastlio_mulran_ws/config/mulran_ouster64.yaml
```

`~/fastlio_mulran_ws/config/mulran_ouster64.yaml`에서 아래 항목을 실제 topic에 맞춘다.

```yaml
/**:
  ros__parameters:
    feature_extract_enable: false
    point_filter_num: 3
    max_iteration: 3
    filter_size_surf: 0.5
    filter_size_map: 0.5
    cube_side_length: 1000.0
    runtime_pos_log_enable: false
    map_file_path: "./test.pcd"

    common:
      lid_topic: "/YOUR_LIDAR_TOPIC"
      imu_topic: "/YOUR_IMU_TOPIC"
      time_sync_en: false
      time_offset_lidar_to_imu: 0.0

    preprocess:
      lidar_type: 3
      scan_line: 64
      timestamp_unit: 0
      blind: 4.0

    mapping:
      acc_cov: 0.1
      gyr_cov: 0.1
      b_acc_cov: 0.0001
      b_gyr_cov: 0.0001
      fov_degree: 360.0
      det_range: 150.0
      extrinsic_est_en: true
      extrinsic_T: [0.0, 0.0, 0.0]
      extrinsic_R: [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]

    publish:
      path_en: true
      scan_publish_en: true
      dense_publish_en: true
      scan_bodyframe_pub_en: true

    pcd_save:
      pcd_save_en: true
      interval: -1
```

`timestamp_unit`은 point field 단위에 맞춘다.

```text
0: second
1: millisecond
2: microsecond
3: nanosecond
```

MOLA의 MulRan 문서는 per-point time을 초 단위 `T` 값으로 설명하므로 우선 `timestamp_unit: 0`으로 시작한다. Ouster ROS driver가 publish한 `/ouster/points`를 직접 쓰는 경우에는 보통 `t`가 nanosecond 단위라 `timestamp_unit: 3`이 맞다.

## Fast-LIO2 실행

Terminal 1: MulRan replay.

```bash
source /opt/ros/jazzy/setup.bash
export MULRAN_BASE_DIR=$HOME/datasets/MulRan
ros2 launch mola_demos ros-mulran-play.launch.py mulran_sequence:=KAIST03
```

Terminal 2: Fast-LIO2.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/fastlio_mulran_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py \
  config_path:=$HOME/fastlio_mulran_ws/config \
  config_file:=mulran_ouster64.yaml \
  rviz:=false
```

Terminal 3: 출력 확인.

```bash
source /opt/ros/jazzy/setup.bash
source ~/fastlio_mulran_ws/install/setup.bash

ros2 topic hz /Odometry
ros2 topic hz /cloud_registered_body
ros2 topic hz /cloud_registered
```

RViz에서 확인할 때는 Fixed Frame을 `camera_init`으로 둔다.

```bash
rviz2
```

정상이라면 `/Odometry`, `/path`, `/cloud_registered`, `/cloud_registered_body`가 지속적으로 publish된다.

## PGO 설정

PGO는 Fast-LIO2의 body-frame scan과 odometry를 동기화해서 keyframe을 만들고, loop candidate를 찾은 뒤 GTSAM 기반 pose graph optimization을 수행한다.

PGO config를 만든다.

```bash
nano ~/fastlio_mulran_ws/config/pgo_mulran.yaml
```

내용:

```yaml
cloud_topic: /cloud_registered_body
odom_topic: /Odometry
map_frame: camera_init
local_frame: body

key_pose_delta_deg: 10
key_pose_delta_trans: 0.5

loop_search_radius: 5.0
loop_time_tresh: 60.0
loop_score_tresh: 0.15
loop_submap_half_range: 5
submap_resolution: 0.1
min_loop_detect_duration: 5.0
```

처음에는 `loop_search_radius`를 `5.0` 정도로 시작한다. loop가 잘 안 잡히면 `10.0` 또는 `15.0`까지 키워본다. 너무 크게 잡으면 잘못된 loop 후보가 늘 수 있다.

## PGO 실행과 맵 저장

Terminal 1: MulRan replay.

```bash
source /opt/ros/jazzy/setup.bash
export MULRAN_BASE_DIR=$HOME/datasets/MulRan
ros2 launch mola_demos ros-mulran-play.launch.py mulran_sequence:=KAIST03
```

Terminal 2: Fast-LIO2.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash
source ~/fastlio_mulran_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py \
  config_path:=$HOME/fastlio_mulran_ws/config \
  config_file:=mulran_ouster64.yaml \
  rviz:=false
```

Terminal 3: PGO node만 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/fastlio_mulran_ws/install/setup.bash

ros2 run pgo pgo_node --ros-args \
  -p config_path:=$HOME/fastlio_mulran_ws/config/pgo_mulran.yaml
```

Terminal 4: topic과 service 확인.

```bash
ros2 topic hz /pgo/loop_markers
ros2 service list | grep save_maps
```

맵 저장:

```bash
mkdir -p ~/maps/mulran_kaist03_pgo
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$HOME/maps/mulran_kaist03_pgo', save_patches: true}"
```

성공하면 아래 파일이 생긴다.

```text
~/maps/mulran_kaist03_pgo/
  map.pcd
  poses.txt
  patches/
    0.pcd
    1.pcd
    ...
```

PCD 확인:

```bash
pcl_viewer ~/maps/mulran_kaist03_pgo/map.pcd
```

## 정상 동작 기준

Fast-LIO2만 켰을 때:

```text
/Odometry가 10 Hz 근처로 publish된다.
/cloud_registered_body가 지속적으로 publish된다.
RViz Fixed Frame camera_init에서 trajectory와 point cloud가 이동 방향대로 쌓인다.
벽이나 건물이 심하게 두 겹으로 보이지 않는다.
```

PGO까지 켰을 때:

```text
/pgo/save_maps 서비스가 보인다.
loop가 있는 구간에서 /pgo/loop_markers가 publish된다.
map.pcd가 저장된다.
patches/와 poses.txt가 같이 저장된다.
```

PGO가 항상 큰 보정을 만드는 것은 아니다. sequence에 충분한 loop가 없거나 loop threshold가 보수적이면, PGO는 keyframe map 저장 역할만 하고 trajectory 변화가 작을 수 있다.

## 자주 나는 문제

### PointCloud2 field 오류

Fast-LIO2 로그에 아래와 비슷한 메시지가 나오면 point cloud field가 맞지 않는 것이다.

```text
Failed to find match for field 't'
Failed to find match for field 'time'
Failed to find match for field 'ring'
```

해결:

```text
1. ros2 topic echo /YOUR_LIDAR_TOPIC --once 로 fields를 확인한다.
2. ring field가 없으면 Ouster 64 설정을 그대로 쓰기 어렵다.
3. per-point time field 단위에 맞춰 timestamp_unit을 바꾼다.
4. field 이름이 다르면 중간 converter node로 x/y/z/intensity/ring/t 형태로 변환한다.
```

### 맵이 두껍거나 겹친다

가능성이 높은 원인:

```text
per-point timestamp 단위가 틀림
LiDAR와 IMU 시간이 맞지 않음
extrinsic이 부정확함
처음 초기화할 때 차량이 움직였음
```

처음에는 `extrinsic_est_en: true`로 시작해서 동작을 확인하고, 나중에 MulRan의 Ouster-IMU extrinsic을 반영해 `extrinsic_est_en: false`로 고정한다.

### PGO가 아무것도 저장하지 않는다

`NO POSES`가 나오면 PGO가 입력을 못 받은 것이다.

```bash
ros2 topic hz /cloud_registered_body
ros2 topic hz /Odometry
```

두 topic의 timestamp가 크게 어긋나면 message filter가 동기화하지 못한다. Fast-LIO2 출력 topic을 사용하고 있는지, PGO config의 topic 이름이 맞는지 확인한다.

### PGO loop가 안 잡힌다

도심 sequence에서 drift가 있으면 기본 `loop_search_radius: 1.0`은 너무 좁을 수 있다. `5.0`, `10.0`, `15.0` 순서로 키워본다. 단, 잘못된 loop가 생기면 맵이 더 나빠질 수 있으므로 `loop_score_tresh`와 RViz marker를 같이 확인한다.

### Jetson 성능 문제

Jetson에서 raw dataset replay, Fast-LIO2, PGO, RViz를 동시에 돌리면 버거울 수 있다.

```text
RViz는 다른 PC에서 실행한다.
Fast-LIO2 launch는 rviz:=false로 둔다.
MulRan 데이터는 NVMe SSD에 둔다.
colcon build는 --parallel-workers 2를 쓴다.
처음 검증은 짧은 구간이나 작은 sequence로 한다.
```

## MID360 실제 데이터로 넘어갈 때

MulRan으로 알고리즘 흐름을 확인한 뒤 MID360 실데이터에서는 아래처럼 바뀐다.

```text
MulRan replay 제거
Livox driver 실행
Fast-LIO2 config를 mid360.yaml 기반으로 변경
lid_topic: /livox/lidar
imu_topic: /livox/imu
preprocess.lidar_type: 1
scan_line: 4
```

Livox는 반드시 custom message launch를 쓰는 것이 좋다.

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

`rviz_MID360_launch.py`처럼 PointCloud2 시각화용 launch만 쓰면 per-point timestamp가 Fast-LIO2가 기대하는 형태와 다를 수 있다.

MID360에서 PGO를 붙일 때도 방식은 같다.

```text
Fast-LIO2 output:
  /Odometry
  /cloud_registered_body

PGO input:
  cloud_topic: /cloud_registered_body
  odom_topic: /Odometry
```

## 정확한 SC-PGO를 재현하고 싶을 때

원본 `gisbi-kim/FAST_LIO_SLAM`은 다음 구조다.

```text
FAST-LIO2 node
  -> odometry + point cloud
SC-PGO node
  -> Scan Context loop detection
  -> GTSAM pose graph optimization
  -> optimized map
```

하지만 원본 실행 예시는 `catkin_make`, `roslaunch`, `file_player_mulran` 기반이다. Ubuntu 24.04 + ROS2 Jazzy에서 그대로 실행하는 대상은 아니다.

재현 선택지는 두 가지다.

```text
1. ROS1 Noetic Docker에서 원본 FAST_LIO_SLAM을 실행한다.
2. SC-PGO만 ROS2로 포팅해서 hku Fast-LIO2 ROS2 출력 topic을 받게 한다.
```

빠른 검증 목적이면 이 README의 ROS2 PGO 경로를 먼저 추천한다. 논문/원본 결과와 맞춰야 하면 ROS1 Docker 재현부터 하는 편이 덜 위험하다.

## 참고 링크

```text
MulRan download:
https://sites.google.com/view/mulran-pr/download

MOLA MulRan ROS2 replay:
https://docs.mola-slam.org/latest/tutorial-mulran-replay-to-ros2.html

MOLA MulRan dataset structure:
https://docs.mola-slam.org/latest/class_mola_MulranDataset.html

hku-mars FAST_LIO ROS2:
https://github.com/hku-mars/FAST_LIO/tree/ROS2

Original FAST_LIO_SLAM with SC-PGO:
https://github.com/gisbi-kim/FAST_LIO_SLAM

ROS2 PGO package:
https://github.com/liangheming/FASTLIO2_ROS2

Livox ROS Driver 2:
https://github.com/Livox-SDK/livox_ros_driver2
```
