# ROS API

全ての API を網羅してはおらず，主要なものについてのみ記載しています．
詳しくは実行時に`$ rostopic list`等でご確認ください．

## トピック

<!-- tobas_tools/constants.hppの内容 -->
<!-- tobas_gazebo_plugins/common.hppの内容 -->

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

# Delay
duration delay  # The communication delay. It is only available when connected to the Internet.
```

#### point_cloud (sensor_msgs/PointCloud)

LiDAR から取得した点群データ

```txt
std_msgs/Header header
geometry_msgs/Point32[] points
sensor_msgs/ChannelFloat32[] channels
```

#### external_odometry (nav_msgs/Odometry)

VIO (Visual Inertial Odometry) などの外部の位置推定機器から取得したオドメトリ．

```txt
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

#### rotor_speeds (tobas_msgs/RotorSpeeds)

各モータの回転数．

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

#### euler (tobas_kdl_msgs/Euler)

推定された機体姿勢をオイラー角に変換したもの．

```txt
float64 roll   # [rad]
float64 pitch  # [rad]
float64 yaw    # [rad]
```

#### wind (tobas_msgs/Wind)

推定された風速．

```txt
std_msgs/Header header
tobas_kdl_msgs/Vector vel  # [m/s]
```

### コマンド

ユーザはこれらのトピックを発行することでドローンを操作することができます．

#### command/throttles (tobas_msgs/ThrottleArray)

各モータのスロットル (0 ~ 1)．

```txt
std_msgs/Header header
tobas_msgs/Throttle[] throttles
```

#### command/rotor_speeds (tobas_msgs/RotorSpeeds)

各モータの回転数 (非負)．

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

#### command/position_yaw (tobas_msgs/PositionYaw)

```txt
tobas_msgs/CommandLevel level
tobas_kdl_msgs/Vector pos  # [m]
float64 yaw                # [rad]
```

#### command/rpy_thrust (tobas_msgs/RollPitchYawThrust)

```txt
tobas_msgs/CommandLevel level
tobas_kdl_msgs/Euler rpy  # [rad]
float64 thrust            # [N]
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

#### command/joint_positions (tobas_msgs/JointPositions)

カスタムジョイントに対する位置指令．

```txt
string[] name
float64[] data  # [m or rad]
```

#### command/joint_velocities (tobas_msgs/JointVelocities)

カスタムジョイントに対する速度指令．

```txt
string[] name
float64[] data  # [m/s or rad/s]
```

#### command/joint_efforts (tobas_msgs/JointEfforts)

カスタムジョイントに対する力指令．

```txt
string[] name
float64[] data  # [N or Nm]
```

### Gazebo

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

## サービス

---

### Common

#### get_arm (tobas_msgs/GetArm)

モータのアーム状態を取得する．

```txt
---
bool arming
```

#### set_arm (tobas_msgs/SetArm)

モータのアーム状態を取得する．

```txt
bool arming
bool ignore_pre_arm_check
---
bool success
string message
```

#### pre_arm_check (std_srvs/Trigger)

アーム可能かどうかの確認．

```txt
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
float64 target_altitude  # [m]
float64 duration         # [s]
float64 timeout          # [s] By default, timeout is infinite.

---

# Result
int8 error_code
int8 NO_ERROR = 0
int8 NOT_READY = -1
int8 INVALID_GOAL = -2
int8 PREEMPTED = -3
int8 TIMEOUT = -4
int8 UNKNOWN_ERROR = -5

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
int8 error_code
int8 NO_ERROR = 0
int8 NOT_READY = -1
int8 INVALID_GOAL = -2
int8 PREEMPTED = -3
int8 UNKNOWN_ERROR = -4

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

---

# Feedback
tobas_kdl_msgs/Vector target_position   # [m]
tobas_kdl_msgs/Vector current_position  # [m]
tobas_kdl_msgs/Vector position_error    # [m]
```
