# パラメータ

## パラメータとは?

ROSではパラメータというものを設定することが出来ます。
これはプログラムを実行する前に値を任意に変更することが出来て、ソースコードを変えることなく自由に値を変えられます。

今回は以下のようなノードをもつ`hello_param`パッケージを作ってみましょう

- `multiply_node.py`
    - `std_msgs/Uint64`型の`/number`トピックからサブスクライブした数を`m_number`パラメータの値を掛けて`/multiplied_number`トピックにパブリッシュする
    - `m_number`パラメータはデフォルトで`int`型の`2`

## パッケージの作成

```none
ros2 pkg create --build-type ament_python hello_param
```

## multiply_node.pyのコード

`hello_param/hello_param/multiply_node.py`を以下の内容で書き込む。

```py
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt64

class Multiply(Node):
    def __init__(self):
        super().__init__("multiply_node")
        self.number_sub = self.create_subscription(UInt64, "number", self.number_callback, 10)
        self.double_number_pub = self.create_publisher(UInt64, "multiplied_number", 10)
        self.declare_parameter("m_number", 2)
        self.m = self.get_parameter("m_number").get_parameter_value().integer_value

    def number_callback(self, sub_msg):
        self.get_logger().info("Subscribed {}".format(sub_msg.data))

        pub_msg = UInt64()
        pub_msg.data = sub_msg.data * self.m
        self.double_number_pub.publish(pub_msg)
        self.get_logger().info("Publishing {}".format(pub_msg.data))

def main(args=None):
    rclpy.init(args=args)

    node = Multiply()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## ビルドと実行

```none
cd ~/my_ws
colcon build --symlink-install
source install/setup.bash
```

```none
ros2 run hello_topic iteration_node
ros2 run hello_param multiply_node
```

次に作った`m_number`パラメータを`3`に変更してノードを実行してみましょう。

```none
ros2 run hello_topic iteration_node
ros2 run hello_param multiply_node --ros-args -p m_number:=3
```

`multiply_node`が`/number`トピックの3倍した数を`multiplied_number`トピックにパブリッシュしていれば成功!

## 参照

- [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [https://www.theconstruct.ai/how-to-manipulate-parameters-at-runtime-ros2-humble-python-tutorial/](https://www.theconstruct.ai/how-to-manipulate-parameters-at-runtime-ros2-humble-python-tutorial/)
