# ユーザプログラム

Setup Assistant で作成した Tobas パッケージ (例: tobas_f450.TBS) に含まれる ROS パッケージのうち，
ユーザパッケージ (例: tobas_f450_user\_\*) はユーザが自由に編集できるパッケージです．
C++と Python の 2 つのパッケージが生成され，それぞれ以下の 3 つの launch ファイルが含まれます．

- `common.launch.py`: 実機とシミュレーションの両方で起動されます．
- `gazebo.launch.py`: シミュレーション時のみ起動されます．
- `real.launch.py`: 実機でのみ起動されます．

自動生成される`tobas_f450_user_cpp/nodes/user_node.cpp`と`tobas_f450_user_py/tobas_f450_user_py/user_node.py`は
予め launch ファイルに記述されているため，それらは編集するだけで機能します．

試しに GNSS の状態を確認し，測位できているか否かを Tobas メッセージで発行する Python ノードを作成してみます．
`tobas_f450_user_py/tobas_f450_user_py/user_node.py`を以下のように編集してください．

```python
import rclpy
import rclpy.node
import rclpy.qos

from tobas_std_msgs.msg import Message
from tobas_msgs.msg import Gps


class GnssStateCheckerNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("gnss_state_checker")

        qos = rclpy.qos.QoSProfile(depth=1)
        qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        qos.durability = rclpy.qos.DurabilityPolicy.VOLATILE

        self._message_pub = self.create_publisher(Message, "message", qos)
        self._gps_sub = self.create_subscription(Gps, "gps", self._gps_callback, qos)

    def _gps_callback(self, gps: Gps) -> None:
        message = Message()
        message.stamp = gps.header.stamp
        message.name = self.get_name()

        if gps.fix_type == Gps.FIX_3D:
            message.level = Message.LEVEL_INFO
            message.message = "GNSS Fix"
        else:
            message.level = Message.LEVEL_WARN
            message.message = "GNSS No Fix"

        self._message_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssStateCheckerNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

```

GUI からシミュレーションを起動すると，`Control System`のコンソールにメッセージが表示されます．

![console](resources/user_code/console.png)
