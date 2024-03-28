# Gazebo Simulation

ドローンの物理シミュレーションを行います．
Tobas パッケージの作成とハードウェアセットアップは既に済んでいることを前提とします．

## Gazebo シミュレーションの起動

---

1. ラズパイのアクセスポイントに接続します．SSID は`raspberry_wifi`，パスワードは`raspberry`です．
1. Applications から Tobas を起動します．
1. 左上の選択リストから`Simulation`を選択します．
1. 右上の`Load`ボタンを押し，Setup Assistant で作成した`tobas_f450_config`を選択します．すると，`Start`ボタンが有効になります．
1. ラズパイへの書き込みが済んでいなければ，`Send`ボタンを押して設定をラズパイに書き込みます．これには数分かかります．

![simulation_page](resources/gazebo_simulation/simulation_page.png)

`Start`ボタンを押すと Gazebo が起動し，ドローンが原点位置に配置されます．これには数十秒かかります．

![gazebo](resources/gazebo_simulation/gazebo.png)

## プロポでの操縦

---

1. プロポの電源を入れます．
1. スロットルレバーを一番下まで下げます．
1. `E-Stop`トグルを一度オンにしてからオフにすると，数秒後にモータがアームします．
1. プロペラが回転し始めたら各レバーで操縦することができます．

<span style="color: red;"><strong>警告: アーム後に E_STOP をオンにすると，全てのモータが緊急停止します．</strong></span>

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
    takeoff_goal.target_duration = INTERVAL

    # アクションゴールを送信
    takeoff_client.send_goal_and_wait(takeoff_goal)

    # アクションの結果を取得
    takeoff_result: TakeoffResult = takeoff_client.get_result()
    if takeoff_result.error_code < 0:
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
