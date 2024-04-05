# ROS API

全ての API を網羅してはおらず，主要なものについてのみ記載しています．
詳しくは実行時に`$ rostopic list`等でご確認ください．

## トピック

<!-- tobas_tools/constant.hppの内容 -->
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
float64 thrust  # CH3: [0, 1]
float64 yaw     # CH4: [-1, 1]
uint8 mode      # CH5: Flight Mode
bool e_stop     # CH7: Emergency Stop
bool gpsw       # CH8: General Purpose Switch
```

#### imu (sensor_msgs/Imu)

6 軸 IMU (ジャイロ + 加速度)．

```txt
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance          # [rad^2]
geometry_msgs/Vector3 angular_velocity     # [rad/s]
float64[9] angular_velocity_covariance     # [rad^2/s^2]
geometry_msgs/Vector3 linear_acceleration  # [m/s^2]
float64[9] linear_acceleration_covariance  # [m^2/s^4]
```

#### magnetic_field (sensor_msgs/MagneticField)

3 軸の地磁気．

```txt
std_msgs/Header header
geometry_msgs/Vector3 magnetic_field
float64[9] magnetic_field_covariance
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

float64[9] position_covariance              # [m^2]
float64[9] orientation_covariance           # [rad^2]
float64[9] linear_velocity_covariance       # [m^2/s^2]
float64[9] angular_velocity_covariance      # [rad^2/s^2]
float64[9] linear_acceleration_covariance   # [m^2/s^2/4]
float64[9] angular_acceleration_covariance  # [rad^2/s^4]

int8 status
int8 NO_ERROR = 0
int8 POSITION_LOST = -1
```

#### wind (tobas_msgs/Wind)

推定された風速．

```txt
std_msgs/Header header
tobas_kdl_msgs/Vector vel  # [m/s]
```

### コマンド

ユーザはこれらのトピックを発行することでドローンを操作することができます．

#### command/throttles (tobas_msgs/Throttles)

各 ESC に指令されるスロットル．

```txt
std_msgs/Header header
float64[] data  # [0, 1]
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

#### ground_truth/odom (tobas_msgs/Odometry)

起動位置に対する位置，速度，加速度の真値．

```txt
std_msgs/Header header

tobas_kdl_msgs/Frame frame
tobas_kdl_msgs/Twist twist
tobas_kdl_msgs/Accel accel

float64[9] position_covariance              # [m^2]
float64[9] orientation_covariance           # [rad^2]
float64[9] linear_velocity_covariance       # [m^2/s^2]
float64[9] angular_velocity_covariance      # [rad^2/s^2]
float64[9] linear_acceleration_covariance   # [m^2/s^2/4]
float64[9] angular_acceleration_covariance  # [rad^2/s^4]

int8 status
int8 NO_ERROR = 0
int8 POSITION_LOST = -1
```

#### ground_truth/wind (tobas_msgs/Wind)

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

### Navio2

#### enable_pwm (tobas_msgs/EnablePwm)

PWM を有効化する．

```txt
uint64 channel
bool enable
---
bool success
```

### Gazebo

#### gazebo/charge_battery (std_srvs/Empty)

バッテリーをフルチャージする．

```txt
---
```

#### gazebo/set_wind_parameters (tobas_gazebo_plugins/SetWindParameters)

シミュレーション中の風を生成するパラメータを設定する．

```txt
# Request
float64 mean_speed         # [m/s] 地面からの高度20ftで測った平均風速
float64 direction          # [rad] 風向 (ヨー角)
float64 gust_speed_factor  # [-] 定常風速に対する突風成分の風速の比率
float64 gust_duration      # [s] 突風の発生時間
float64 gust_interval      # [s] 突風が過ぎ去ってから次の突風が来るまでの時間

---

# Response
bool success

# 設定された値
float64 mean_speed
float64 direction
float64 gust_speed_factor
float64 gust_duration
float64 gust_interval
```

## アクション

---

#### takeoff_action (tobas_msgs/Takeoff)

離陸する．

```txt
# Goal
tobas_msgs/CommandLevel level
float64 target_altitude  # [m]
float64 target_duration  # [s]
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

#### landing_action (tobas_msgs/Land)

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
