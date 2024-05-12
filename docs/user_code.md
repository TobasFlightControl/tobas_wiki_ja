# User Code

<!-- TODO -->
<!-- ## ROS API で操作

---

ROS API 用いてドローンに指令を送ることもできます．
ユーザのプログラムからドローンの情報にアクセスできるため，アプリケーションを作成する際に有用です．
詳しくは[ROS API](ros_api.md)をご覧ください．

まず，ドローンを操作するスクリプトを配置するための ROS パッケージを作成します．

```bash
$ cd ~/catkin_ws/src/
$ catkin_create_pkg my_tobas_example
```

一度ビルドとソースを行い，パッケージをシェルに読み込ませます．

```bash
$ catkin build my_tobas_example
$ source ~/catkin_ws/devel/setup.bash
```

次に，スクリプトを ROS パッケージ内に作成します．
以下は`takeoff_action`アクションで離陸し，`command/pos_vel_acc_yaw`トピックで位置指令を行う Python スクリプトの例です．
これを`my_tobas_example/scripts/`以下に配置してください．

```python
#!/usr/bin/env python3

import rospy
import actionlib

from tobas_msgs.msg import TakeoffAction, TakeoffGoal, TakeoffResult, PosVelAccYaw

ALTITUDE = 3.0  # [m]
SIDE_LENGTH = 5.0  # [m]
INTERVAL = 5.0  # [s]


if __name__ == "__main__":
    # ROSノードの初期化
    rospy.init_node("command_square_trajectory")

    # 離陸アクションクライアントの作成
    takeoff_client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)

    # アクションサーバーが起動するのを待つ
    takeoff_client.wait_for_server()

    # アクションゴールを作成
    takeoff_goal = TakeoffGoal()
    takeoff_goal.target_altitude = ALTITUDE
    takeoff_goal.duration = INTERVAL

    # アクションゴールを送信
    takeoff_client.send_goal_and_wait(takeoff_goal)

    # アクションの結果を取得
    if takeoff_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
        rospy.logerr("Takeoff action failed.")
        rospy.signal_shutdown()

    # コマンドのパブリッシャーを作成
    command_pub = rospy.Publisher("command/pos_vel_acc_yaw", PosVelAccYaw, queue_size=1)

    # 正方形の頂点を指令し続ける
    while not rospy.is_shutdown():
        # 頂点1
        command = PosVelAccYaw()
        command.pos.x = SIDE_LENGTH / 2
        command.pos.y = SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点2
        command = PosVelAccYaw()
        command.pos.x = -SIDE_LENGTH / 2
        command.pos.y = SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点3
        command = PosVelAccYaw()
        command.pos.x = -SIDE_LENGTH / 2
        command.pos.y = -SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点4
        command = PosVelAccYaw()
        command.pos.x = SIDE_LENGTH / 2
        command.pos.y = -SIDE_LENGTH / 2
        command.pos.z = ALTITUDE
        command_pub.publish(command)
        rospy.sleep(INTERVAL)
```

スクリプトに実行権限を与えます．

```bash
$ chmod u+x ~/catkin_ws/src/my_tobas_example/scripts/command_square_trajectory_node.py
```

スクリプトを実行すると，ドローンが離陸後に正方形の辺上を移動し続けます．
トピックはドローンの名前空間 (URDF 作成時に設定した`Robot Name`) 内に存在するため，名前空間`__ns`を指定します．

```bash
$ rosrun my_tobas_example command_square_trajectory_node.py __ns:=f450
```

## パラメータチューニング

---

必要であれば飛行中にオンラインでパラメータを調整することができます．
以下のコマンドで調整用の GUI を立ち上げます:

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

![rqt_reconfigure](resources/gazebo_simulation/rqt_reconfigure.png)

オンラインで調整可能な全てのパラメータが表示され，水平バー，エディタ等で値を調整することができます．
パラメータ名にカーソルを重ねると，パラメータの説明文が表示されます．
詳しくは<a href=https://wiki.ros.org/rqt_reconfigure>rqt_reconfigure | ROS</a>をご覧ください． -->
