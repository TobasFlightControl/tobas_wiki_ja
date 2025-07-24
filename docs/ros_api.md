# ROS API

全ての API を網羅してはおらず，主要なものについてのみ記載しています．
詳しくは実行時に`$ ros2 topic list`等でご確認ください．
`remote_interface`名前空間内の API を使うと，外部端末から FC と通信することもできます．

## トピック

<!-- tobas_constants/constants.hppの内容 -->
<!-- tobas_gazebo_common/constants.hppの内容 -->

---

### センサデータ

#### battery (tobas_msgs/Battery)

バッテリーの状態．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 voltage  # [V]
float64 current  # [A]
```

### engine_state (tobas_msgs/EngineState)

エンジンの状態．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 speed            # [rad/s]
float64 fuel_quantity    # [L]
float64 oil_temperature  # [degC]
```

#### cpu (tobas_msgs/Cpu)

CPU の状態．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 temperature  # [degC]
uint64 frequency     # [Hz]
float64 load         # [-]
```

#### rc_input (tobas_msgs/RCInput)

RC レシーバーから取得した RC 入力．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 roll      # CH1: [-1, 1]
float64 pitch     # CH2: [-1, 1]
float64 throttle  # CH3: [-1, 1]
float64 yaw       # CH4: [-1, 1]
bool enable       # CH5: Enable Radio Control
bool kill         # CH6: Kill Switch
uint8 mode        # CH7: Flight Mode
bool sub_mode     # CH8: Sub Flight Mode
bool[8] gpsw      # CH9-16: General Purpose Switch
```

#### imu_raw (tobas_msgs/Imu)

フィルタリング前の IMU．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector accel  # [m/s^2]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector gyro   # [rad/s]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector dgyro  # [rad/s^2]
	float64 x
	float64 y
	float64 z
```

#### imu_raw (tobas_msgs/Imu)

フィルタリング後の IMU．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector accel  # [m/s^2]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector gyro   # [rad/s]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector dgyro  # [rad/s^2]
	float64 x
	float64 y
	float64 z
```

#### magnetic_field (tobas_msgs/MagneticField)

3 軸の地磁気．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector mag  # [-]
	float64 x
	float64 y
	float64 z
```

#### air_pressure (tobas_msgs/FluidPressure)

大気圧．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 pressure  # [Pa]
```

#### gnss (tobas_msgs/Gnss)

GNSS から取得した位置と速度．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Fix Type: gpsFix (UBX-STATUS), fixType (UBX-PVT)
uint8 fix_type
uint8 NO_FIX = 0
uint8 DEAD_RECHONING_ONLY = 1
uint8 FIX_2D = 2
uint8 FIX_3D = 3
uint8 GPS_DEAD_RECHONING_COMBINED = 4
uint8 TIME_ONLY_FIX = 5

# Position
float64 latitude                                # [deg]
float64 longitude                               # [deg]
float64 altitude                                # [m]
tobas_eigen_msgs/Matrix3d position_covariance   # [m^2]
	float64[9] data

# Velocity
tobas_kdl_msgs/Vector ground_speed                 # [m/s]
	float64 x
	float64 y
	float64 z
tobas_eigen_msgs/Matrix3d velocity_covariance      # [m^2/s^2]
	float64[9] data
```

#### rotor_speeds (tobas_msgs/RotorStateArray)

各ロータの状態．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/RotorState[] states
	string link_name
	float64 speed   # [rad/s]
	float64 thrust  # [N]
	uint8 status
	uint8 NO_ERROR = 0
	uint8 COMMUNICATION_FAILURE = 1
```

#### joint_states_2 (tobas_msgs/JointStateArray)

可動ジョイントの状態．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointState[] states
	string name
	float64 position
	float64 velocity
	float64 effort
```

### 状態推定

#### odom (tobas_msgs/Odometry)

状態推定器によって推定された，起動位置に対する位置，速度，加速度．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

tobas_kdl_msgs/Frame frame
	tobas_kdl_msgs/Vector trans
		float64 x
		float64 y
		float64 z
	tobas_kdl_msgs/Rotation rot
		float64[9] data
tobas_kdl_msgs/Twist twist
	tobas_kdl_msgs/Vector linear
		float64 x
		float64 y
		float64 z
	tobas_kdl_msgs/Vector angular
		float64 x
		float64 y
		float64 z
tobas_kdl_msgs/Accel accel
	tobas_kdl_msgs/Vector linear
		float64 x
		float64 y
		float64 z
	tobas_kdl_msgs/Vector angular
		float64 x
		float64 y
		float64 z

tobas_eigen_msgs/Matrix3d position_covariance     # [m^2]
	float64[9] data
tobas_eigen_msgs/Matrix3d orientation_covariance  # [rad^2]
	float64[9] data
tobas_eigen_msgs/Matrix3d velocity_covariance     # [m^2/s^2]
	float64[9] data
tobas_eigen_msgs/Matrix3d gyro_covariance         # [rad^2/s^2]
	float64[9] data

int8 status
int8 NO_ERROR = 0
int8 POSITION_LOST = -1
```

#### arming (tobas_msgs/Arming)

全てのロータがアームされているか否か．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
bool data
```

### コマンド

ユーザは FC 内部からこれらのトピックを発行することでドローンを操作することができます．
受け付けるコマンドは機体フレームの型と飛行モードによって決まるため，ROS 2 の CLI でご確認ください．

#### command/rate (tobas_command_msgs/Rate)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector rate  # Target angular velocity wrt. the local coordinates [rad/s]
	float64 x
	float64 y
	float64 z
```

#### command/rate_throttle (tobas_command_msgs/RateThrottle)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector rate  # Target angular velocity wrt. the local coordinates [rad/s]
	float64 x
	float64 y
	float64 z
float64 throttle            # Target throttle [0, 1]
```

#### command/angle (tobas_command_msgs/Angle)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Euler angle  # Target euler angles wrt. the global coordinates [rad]
	float64 roll   # [rad]
	float64 pitch  # [rad]
	float64 yaw    # [rad]
```

#### command/angle_throttle (tobas_command_msgs/AngleThrottle)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Euler angle  # Target euler angles wrt. the global coordinates [rad]
	float64 roll   # [rad]
	float64 pitch  # [rad]
	float64 yaw    # [rad]
float64 throttle            # Target throttle [0, 1]
```

#### command/accel (tobas_command_msgs/Accel)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector accel  # Target linear acceleration wrt. the global coordinates [m/s^2]
	float64 x
	float64 y
	float64 z
```

#### command/accel_yaw (tobas_command_msgs/AccelYaw)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector accel  # Target linear acceleration wrt. the global coordinates [m/s^2]
	float64 x
	float64 y
	float64 z
float64 yaw                  # Target yaw angle [rad]
```

#### command/pos_vel (tobas_command_msgs/PosVel)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector pos  # Target position wrt. the global coordinates [m]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector vel  # Target linear velocity wrt. the global coordinates [m/s]
	float64 x
	float64 y
	float64 z
```

#### command/pos_vel_yaw (tobas_command_msgs/PosVelYaw)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

tobas_kdl_msgs/Vector pos  # Target position wrt. the global coordinates [m]
	float64 x
	float64 y
	float64 z
tobas_kdl_msgs/Vector vel  # Target linear velocity wrt. the global coordinates [m/s]
	float64 x
	float64 y
	float64 z
float64 yaw                # Target yaw angle [rad]
```

#### command/speed_roll_delta_pitch (tobas_command_msgs/SpeedRollDeltaPitch)

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

float64 speed        # [m/s]
float64 roll         # [rad]
float64 delta_pitch  # [rad]
```

#### command/joint_positions (tobas_msgs/JointCommandArray)

カスタムジョイントに対する位置指令．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointCommand[] commands
	string name
	float64 data
```

#### command/joint_velocities (tobas_msgs/JointCommandArray)

カスタムジョイントに対する速度指令．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointCommand[] commands
	string name
	float64 data
```

#### command/joint_efforts (tobas_msgs/JointCommandArray)

カスタムジョイントに対する力指令．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_msgs/JointCommand[] commands
	string name
	float64 data
```

### Gazebo

#### gazebo/ground_truth/battery (tobas_msgs/Battery)

バッテリーの状態の真値．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float64 voltage  # [V]
float64 current  # [A]
```

#### gazebo/ground_truth/odom (tobas_msgs/Odometry)

起動位置に対する位置，速度，加速度の真値．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

tobas_kdl_msgs/Frame frame
	tobas_kdl_msgs/Vector trans
		float64 x
		float64 y
		float64 z
	tobas_kdl_msgs/Rotation rot
		float64[9] data
tobas_kdl_msgs/Twist twist
	tobas_kdl_msgs/Vector linear
		float64 x
		float64 y
		float64 z
	tobas_kdl_msgs/Vector angular
		float64 x
		float64 y
		float64 z
tobas_kdl_msgs/Accel accel
	tobas_kdl_msgs/Vector linear
		float64 x
		float64 y
		float64 z
	tobas_kdl_msgs/Vector angular
		float64 x
		float64 y
		float64 z

tobas_eigen_msgs/Matrix3d position_covariance     # [m^2]
	float64[9] data
tobas_eigen_msgs/Matrix3d orientation_covariance  # [rad^2]
	float64[9] data
tobas_eigen_msgs/Matrix3d velocity_covariance     # [m^2/s^2]
	float64[9] data
tobas_eigen_msgs/Matrix3d gyro_covariance         # [rad^2/s^2]
	float64[9] data

int8 status
int8 NO_ERROR = 0
int8 POSITION_LOST = -1
```

#### gazebo/ground_truth/wind (tobas_msgs/Wind)

グローバル座標系における風速の真値．

```txt
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
tobas_kdl_msgs/Vector vel  # [m/s]
	float64 x
	float64 y
	float64 z
```

## サービス

---

### Common

#### set_arm (tobas_msgs/SetArm)

全てのロータのアーム状態を変更する．

```txt
bool arming
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
	float64 mean_speed         # [m/s]
	float64 direction          # [rad]
	float64 gust_speed_factor  # [-]
	float64 gust_duration      # [s]
	float64 gust_interval      # [s]
```

#### gazebo/set_wind_parameters (tobas_gazebo_msgs/SetWindParams)

シミュレーション中の風を生成するパラメータを設定する．

```txt
tobas_gazebo_msgs/WindParams params
	float64 mean_speed         # [m/s]
	float64 direction          # [rad]
	float64 gust_speed_factor  # [-]
	float64 gust_duration      # [s]
	float64 gust_interval      # [s]
---
bool success
tobas_gazebo_msgs/WindParams params
	float64 mean_speed         # [m/s]
	float64 direction          # [rad]
	float64 gust_speed_factor  # [-]
	float64 gust_duration      # [s]
	float64 gust_interval      # [s]
```

#### gazebo/get_tether_parameters (tobas_gazebo_msgs/GetTetherParams)

テザーステーションに関するパラメータを取得する．

```txt
---
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
```

#### gazebo/set_tether_parameters (tobas_gazebo_msgs/SetTetherParams)

テザーステーションに関するパラメータを設定する．

```txt
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
---
bool success
tobas_gazebo_msgs/TetherParams params
	float64 tension         # [N]
	float64 maximum_length  # [m]
```

## アクション

---

#### takeoff_action (tobas_mission_msgs/Takeoff)

離陸する．

```txt
# Goal
tobas_command_msgs/CommandLevel level
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

#### land_action (tobas_mission_msgs/Land)

着陸する．

```txt
# Goal
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2

---

# Result
string message

---

# Feedback
```

#### move_action (tobas_mission_msgs/Move)

指定した位置に移動する．

```txt
# Goal
tobas_command_msgs/CommandLevel level
	uint8 data
	uint8 NORMAL = 0
	uint8 DEFENSIVE = 1
	uint8 MANUAL = 2
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
	float64 x
	float64 y
	float64 z
geometry_msgs/Vector3 current_position  # [m]
	float64 x
	float64 y
	float64 z
geometry_msgs/Vector3 position_error    # [m]
	float64 x
	float64 y
	float64 z
```
