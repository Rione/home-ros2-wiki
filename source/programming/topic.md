# トピック通信

## トピック通信とは?

トピック通信ではデータを送る**パブリッシャ**とデータを受け取る**サブスクライバ**が存在します。
パブリッシャは任意のトピックにデータを連続的に送り続け、サブスクライバは任意のトピックに送られたデータを受け取ることができます。
UDP通信をイメージすると分かりやすいと思います。

### 今回の目標

今回は以下のようなノードをもつ`hello_topic`パッケージを作ってみましょう

- `iteration_node.py`
    - `std_msgs/UInt64`型の数字を一秒毎に`/number`トピックにパブリッシュする
    - 送る数は一秒毎に1足される
- `double_node.py`
    - `std_msgs/UInt64`型の`/number`トピックをサブスクライブする
    - この時、受け取った数を2倍にして`/double_number`トピックにパブリッシュする

## パッケージの作成

```bash
ros2 pkg create --build-type ament_python hello_topic
```

## iteration_node.pyのコード

`hello_topic/hello_topic/iteration_node.py`を以下の内容で書き込む。

```py
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt64

class Iteration(Node):
    def __init__(self):
        super().__init__("iteration_node")
        self.number_pub = self.create_publisher(UInt64, "number", 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = UInt64()
        msg.data = self.i
        self.number_pub.publish(msg)

        self.get_logger().info(f"Publishing {msg.data}")

        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    node = Iteration()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## double_node.pyのコード

`hello_topic/hello_topic/double_node.py`を以下の内容で書き込む。

```py
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt64

class Double(Node):
    def __init__(self):
        super().__init__("double_node")
        self.number_sub = self.create_subscription(UInt64, "number", self.number_callback, 10)
        self.double_number_pub = self.create_publisher(UInt64, "double_number", 10)

    def number_callback(self, sub_msg):
        self.get_logger().info(f"Subscribed {sub_msg.data}")

        pub_msg = UInt64()
        pub_msg.data = sub_msg.data * 2
        self.double_number_pub.publish(pub_msg)

        self.get_logger().info(f"Publishing {pub_msg.data}")

def main(args=None):
    rclpy.init(args=args)

    node = Double()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## setup.pyの編集

```py
from setuptools import find_packages, setup

package_name = 'hello_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ri-one',
    maintainer_email='ri-one@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # この2行を追加
            'iteration_node = hello_topic.iteration_node:main',
            'double_node = hello_topic.double_node:main',
        ],
    },
)
```

## ビルドと実行

```bash
cd ~/my_ws
colcon build --symlink-install
source install/setup.bash
```

```bash
ros2 run hello_topic iteration_node
ros2 run hello_topic double_node
```

```{figure} topic-terminal-output.png
端末での実行画面
```

## 課題

前回の
{doc}`/programming/custom-message`
で独自に作った`hello_msgs`パッケージの`hello_msgs/Person`型のメッセージをパブリッシュ、サブスクライブするノードを書いてみましょう。

- `person_pub_node.py`
    - `hello_msgs/Person`型のメッセージを`/person`トピックにパブリッシュする
    - メッセージは1Hz間隔で、自分の名前と年齢を送る
- `person_sub_node.py`
    - `hello_msgs/Person`型の`/person`トピックをサブスクライブする
    - メッセージを受け取ったら、その内容を画面に出力する

## 参照

- [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
