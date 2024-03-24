# Hardware in the Loop (HIL)

Hardware in the Loop (HIL) とは，実際のハードウェア (フライトコントローラやプロポなど) を使用しながらシミュレーションを行うことです．
これにより，リアルタイムでシステムの挙動をテストすることができ，実機試験の際のリスクを軽減することができます．

まず，ラズパイで roscore を立ち上げます:

```bash
pi@navio $ roscore
```

次に，外部 PC で Gazebo を立ち上げます．
ROS マスターはラズパイ側にあるため，環境変数の設定が必要です．

```bash
user@pc $ export ROS_MASTER_URI=http://(ラズパイのIPアドレス):11311
user@pc $ roslaunch tobas_f450_config gazebo.launch
```

ラズパイで HIL に必要なソフトウェアを立ち上げます．

<span style="color: red;"><strong>警告: プロペラがモータから取り外されていることを確認してください．</strong></span>

```bash
pi@navio $ su
root@navio $ roslaunch tobas_f450_config hil.launch
```

プロポの E_STOP (CH5) を一度オンにしてからオフにすると，プロポから Gazebo 中のドローンが操作できます．
<span style="color: red;"><strong>E_STOP をもう一度オンにすると緊急停止となり，全てのモータが停止するため注意してください．</strong></span>
プロポのピッチレバーは南北 (X 軸) ，ロールレバーは東西 (Y 軸) ，スロットルレバーは上下 (Z 軸) の速度にそれぞれ対応しています．
モータの回転数の変化がシミュレーションと実機で一致していることを確認してください．
