# ROS API

全ての API を網羅してはおらず，主要なものについてのみ記載しています．
詳しくは実行時に`$ ros2 topic list`等でご確認ください．
`remote_interface`名前空間内の API を使うと，PC 内のノードから FC と通信することもできます．

## トピック

<!-- tobas_constants/constants.hppの内容 -->
<!-- tobas_gazebo_plugins/common/constants.hppの内容 -->

---

### センサデータ

#### battery (tobas_msgs/Battery)

バッテリーの状態．

```txt
std_msgs/Header header
float64 voltage  # [V]
float64 current  # [A]
```

#### cpu (tobas_msgs/Cpu)

CPU の状態．

```txt
std_msgs/Header header
float64 temperature  # [celsius]
uint64 frequency     # [Hz]
float64 load         # [-]
```

#### rc_input (tobas_msgs/RCInput)

RC レシーバーから取得した RC 入力．

```txt
std_msgs/Header header
float64 roll    # CH1: [-1, 1]
float64 pitch   # CH2: [-1, 1]
float64 thrust  # CH3: [-1, 1]
float64 yaw     # CH4: [-1, 1]
uint8 mode      # CH5: Flight Mode
bool e_stop     # CH7: Emergency Stop
bool gpsw       # CH8: General Purpose Switch

uint8 MODE_PROGRAM = 0
uint8 MODE_STABILIZE = 1
uint8 MODE_ACROBAT = 2
```

#### imu (tobas_msgs/Imu)

6 軸 IMU (ジャイロ + 加速度)．

```txt
std_msgs/Header header
tobas_kdl_msgs/Vector gyro                  # [rad/s]
tobas_kdl_msgs/Vector accel                 # [m/s^2]
tobas_eigen_msgs/Matrix3d gyro_covariance   # [rad^2/s^2]
tobas_eigen_msgs/Matrix3d accel_covariance  # [m^2/s^4]
```

#### magnetic_field (tobas_msgs/MagneticField)

3 軸の地磁気．

```txt
std_msgs/Header header
tobas_kdl_msgs/Vector magnetic_field  # [-]
tobas_eigen_msgs/Matrix3d covariance  # [-]
```

#### air_pressure (sensor_msgs/FluidPressure)

大気圧．

```txt
std_msgs/Header header
float64 fluid_pressure  # [Pa]
float64 variance        # [Pa^2]
```

#### gps (tobas_msgs/Gps)

GNSS から取得した位置と速度．

```txt
std_msgs/Header header

# Fix Type: gpsFix (UBX-STATUS), fixType (UBX-PVT)
uint8 fix_type
uint8 NO_FIX = 0
uint8 DEAD_RECHONING_ONLY = 1
uint8 FIX_2D = 2
uint8 FIX_3D = 3
uint8 GPS_DEAD_RECHONING_COMBINED = 4
uint8 TIME_ONLY_FIX = 5

# Position
float64 latitude                 # [deg]
float64 longitude                # [deg]
float64 altitude                 # [m]
float64[9] position_covariance   # [m^2]

# Velocity
tobas_kdl_msgs/Vector ground_speed  # [m/s]
float64[9] velocity_covariance      # [m^2/s^2]
```

#### point_cloud (sensor_msgs/PointCloud)

LiDAR から取得した点群データ

```txt
std_msgs/Header header
geometry_msgs/Point32[] points
sensor_msgs/ChannelFloat32[] channels
```

#### rotor_speeds (tobas_msgs/RotorSpeeds)

各ロータの回転数．

```txt
std_msgs/Header header
float64[] speeds  # [rad/s]
```

#### joint_states (sensor_msgs/JointState)

カスタムジョイントの状態．

```txt
std_msgs/Header header
string[] name
float64[] position  # [m or rad]
float64[] velocity  # [m/s or rad/s]
float64[] effort    # [N or Nm]
```

### 状態推定

#### odom (tobas_msgs/Odometry)

状態推定器によって推定された，起動位置に対する位置，速度，加速度．

```txt
std_msgs/Header header

tobas_kdl_msgs/Frame frame
tobas_kdl_msgs/Twist twist
tobas_kdl_msgs/Accel accel

tobas_eigen_msgs/Matrix3d position_covariance              # [m^2]
tobas_eigen_msgs/Matrix3d orientation_covariance           # [rad^2]
tobas_eigen_msgs/Matrix3d linear_velocity_covariance       # [m^2/s^2]
tobas_eigen_msgs/Matrix3d angular_velocity_covariance      # [rad^2/s^2]
tobas_eigen_msgs/Matrix3d linear_acceleration_covariance   # [m^2/s^2/4]
tobas_eigen_msgs/Matrix3d angular_acceleration_covariance  # [rad^2/s^4]

int8 status
int8 NO_ERROR = 0
int8 POSITION_LOST = -1
```

#### latency (tobas_msgs/Latency)

IMU 取得からロータコマンドまでのレイテンシ．

```txt
std_msgs/Header header
builtin_interfaces/Duration data
```

#### arming (std_msgs/Bool)

ロータのアーム状態．

```
bool data
```

### コマンド

ユーザはこれらのトピックを発行することでドローンを操作することができます．

#### command/throttles (tobas_msgs/ThrottleArray)

各ロータのスロットル (0 ~ 1)．

```txt
std_msgs/Header header
tobas_msgs/Throttle[] throttles
```

#### command/rotor_speeds (tobas_msgs/RotorSpeeds)

各ロータの回転数 (非負)．

```txt
std_msgs/Header header
float64[] speeds  # [rad/s]
```

#### command/deflections (tobas_msgs/ControlSurfaceDeflections)

固定翼の舵角．

```txt
std_msgs/Header header
float64[] deflections  # [deg]
```

#### command/pos_vel_acc_yaw (tobas_msgs/PosVelAccYaw)

```txt
tobas_msgs/CommandLevel level
tobas_msgs/FrameId frame_id  # The frame in whch velocity and acceleration are expressed
tobas_kdl_msgs/Vector pos    # [m]
tobas_kdl_msgs/Vector vel    # [m/s]
tobas_kdl_msgs/Vector acc    # [m/s^2]
float64 yaw                  # [rad]
```

#### command/rpy_throttle (tobas_msgs/RollPitchYawThrottle)

```txt
tobas_msgs/CommandLevel level
tobas_kdl_msgs/Euler rpy  # [rad]
float64 throttle          # [0, 1]
```

#### command/pose_twist_accel (tobas_msgs/PoseTwistAccelCommand)

```txt
tobas_msgs/CommandLevel level
tobas_msgs/FrameId frame_id  # The frame in whch velocity and acceleration are expressed

tobas_kdl_msgs/Vector pos    # Target position [m]
tobas_kdl_msgs/Vector vel    # Target linear velocity [m/s]
tobas_kdl_msgs/Vector acc    # Target linear acceleration (feedforward) [m/s^2]

tobas_kdl_msgs/Euler rpy     # Target orientation [rad]
tobas_kdl_msgs/Vector gyro   # Target angular velocity wrt. the local coordinates [rad/s]
tobas_kdl_msgs/Vector dgyro  # Target angular acceleration wrt. the local coordinates (feedforward) [rad/s^2]
```

#### command/speed_roll_delta_pitch (tobas_msgs/SpeedRollDeltaPitch)

```txt
float64 speed        # [m/s]
float64 roll         # [rad]
float64 delta_pitch  # [rad]
```

#### command/joint_positions (tobas_msgs/JointCommandArray)

カスタムジョイントに対する位置指令．

```txt
std_msgs/Header header
tobas_msgs/JointCommand[] commands
```

#### command/joint_velocities (tobas_msgs/JointCommandArray)

カスタムジョイントに対する速度指令．

```txt
std_msgs/Header header
tobas_msgs/JointCommand[] commands
```

#### command/joint_efforts (tobas_msgs/JointCommandArray)

カスタムジョイントに対する力指令．

```txt
std_msgs/Header header
tobas_msgs/JointCommand[] commands
```

### Gazebo

#### battery (tobas_msgs/Battery)

バッテリーの状態の真値．

```txt
std_msgs/Header header
float64 voltage  # [V]
float64 current  # [A]
```

#### gazebo/ground_truth/odom (tobas_msgs/Odometry)

起動位置に対する位置，速度，加速度の真値．

```txt
std_msgs/Header header

tobas_kdl_msgs/Frame frame
tobas_kdl_msgs/Twist twist
tobas_kdl_msgs/Accel accel

tobas_eigen_msgs/Matrix3d position_covariance              # [m^2]
tobas_eigen_msgs/Matrix3d orientation_covariance           # [rad^2]
tobas_eigen_msgs/Matrix3d linear_velocity_covariance       # [m^2/s^2]
tobas_eigen_msgs/Matrix3d angular_velocity_covariance      # [rad^2/s^2]
tobas_eigen_msgs/Matrix3d linear_acceleration_covariance   # [m^2/s^2/4]
tobas_eigen_msgs/Matrix3d angular_acceleration_covariance  # [rad^2/s^4]

int8 status
int8 NO_ERROR = 0
int8 POSITION_LOST = -1
```

#### gazebo/ground_truth/wind (tobas_msgs/Wind)

グローバル座標系における風速の真値．

```txt
std_msgs/Header header
tobas_kdl_msgs/Vector vel  # [m/s]
```

#### gazebo/ground_truth/rotor_state (tobas_msgs/msg/RotorState)

```txt
std_msgs/Header header
float64 speed    # [rad/s]
float64 current  # [A]

```

## サービス

---

### Common

#### set_arm (tobas_msgs/SetArm)

ロータのアーム状態を変更する．

```txt
bool arming
bool ignore_pre_arm_check
---
bool success
string message
```

### Gazebo

#### gazebo/charge_battery (std_srvs/Empty)

バッテリーをフルチャージする．

```txt
---
```

#### gazebo/get_wind_parameters (tobas_gazebo_msgs/GetWindParams)

シミュレーション中の風を生成するパラメータを取得する．

```txt
---
tobas_gazebo_msgs/WindParams params
```

#### gazebo/set_wind_parameters (tobas_gazebo_msgs/SetWindParams)

シミュレーション中の風を生成するパラメータを設定する．

```txt
tobas_gazebo_msgs/WindParams params
---
bool success
tobas_gazebo_msgs/WindParams params
```

#### gazebo/get_tether_parameters (tobas_gazebo_msgs/GetTetherParams)

テザーステーションに関するパラメータを取得する．

```txt
---
tobas_gazebo_msgs/TetherParams params
```

#### gazebo/set_tether_parameters (tobas_gazebo_msgs/SetTetherParams)

テザーステーションに関するパラメータを設定する．

```txt
tobas_gazebo_msgs/TetherParams params
---
bool success
tobas_gazebo_msgs/TetherParams params
```

## アクション

---

#### takeoff_action (tobas_msgs/Takeoff)

離陸する．

```txt
# Goal
tobas_msgs/CommandLevel level
float64 target_altitude     # [m]
float64 altitude_tolerance  # [m]
float64 duration            # [s]
float64 timeout             # [s] Timeout after command duration. By default, timeout is infinite.

---

# Result
string message

---

# Feedback
```

#### land_action (tobas_msgs/Land)

着陸する．

```txt
# Goal
tobas_msgs/CommandLevel level

---

# Result
string message

---

# Feedback
```

#### move_action (tobas_msgs/Move)

指定した位置に移動する．

```txt
# Goal
tobas_msgs/CommandLevel level
float64 target_latitude    # [m]
float64 target_longitude   # [m]
float64 target_altitude    # [m]
float64 acceptance_radius  # [m]
float64 duration           # [s]
float64 timeout            # [s] Timeout after command duration. By default, timeout is infinite.

---

# Result
string message

---

# Feedback
geometry_msgs/Vector3 target_position   # [m]
geometry_msgs/Vector3 current_position  # [m]
geometry_msgs/Vector3 position_error    # [m]
```
