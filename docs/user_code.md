# ユーザプログラム

Setup Assistant で作成した Tobas パッケージ (例: tobas_f450.TBS) に含まれる ROS パッケージのうち，
ユーザパッケージ (例: tobas_f450_user) はユーザが自由に編集できるパッケージです．
ユーザパッケージには 3 つの launch ファイルが含まれます．

- `common.launch`: 実機とシミュレーションの両方で起動されます．
- `gazebo.launch`: シミュレーション時のみ起動されます．
- `real.launch`: 実機でのみ起動されます．

以下は GNSS の状態を確認し，測位できているか否かを Tobas メッセージで発行する Python スクリプトです．
これを`scripts/gnss_state_checker_node.py`として保存し，`chmod`で実行権限を与えてください．

```python
#!/usr/bin/env python3

import rospy
from tobas_msgs.msg import Message, Gps


class GnssStateChecker:
    def __init__(self) -> None:
        self._message_pub = rospy.Publisher("message", Message, queue_size=1)
        self._gps_sub = rospy.Subscriber("gps", Gps, self._gps_callback, queue_size=1)

    def _gps_callback(self, gps: Gps) -> None:
        message = Message()
        message.header.stamp = gps.header.stamp
        message.name = rospy.get_name()

        if gps.fix_type == Gps.FIX_3D:
            message.level = Message.INFO
            message.message = "GNSS Fix"
        else:
            message.level = Message.WARN
            message.message = "GNSS No Fix"

        self._message_pub.publish(message)


if __name__ == "__main__":
    rospy.init_node("gnss_state_checker")
    node = GnssStateChecker()
    rospy.spin()
```

作成したスクリプトが自動で起動されるようにします．
`common.launch`を以下のように編集してください．

```xml
<!-- Do not delete or rename this file because it is executed in tobas_f450_config/bringup.launch. -->

<launch>

  <!-- Please launch the nodes that run on both the actual machine and the Gazebo simulation. -->
  <node pkg="tobas_f450_user" type="gnss_state_checker_node.py" name="gnss_state_checker"/>

</launch>
```

GUI からシミュレーションを起動すると，`Console`にメッセージが表示されます．

![console](resources/user_code/console.png)
