# アクション通信

アクション通信は一言で言うと、前回学んだ{doc}`/programming/topic`と{doc}`/programming/service`の両方を組み合わせたようなものです。
リクエスを送る**クライアント**と、受け取ったリクエスに応じてレスポンスを返す**サーバ**が存在します。
アクション通信の特徴として、サーバはレスポンスを返すまで、途中経過の値をクライアントに送り続けることが出来ます。

```{note}
アクション通信は使う頻度が少ない(?)ので飛ばしても大丈夫です。
必要になったら勉強してください。
```

## 今回の目標

今回は以下のようなノードをもつ`hello_action`パッケージを作ってみましょう。

- `server_node.py`
    - `hello_msgs/Sum`型の`/sum`アクションからリクエスをを受け取り、クライアントにレスポンスを返す
    - `goal`をクライアントから受け取り、1から`goal`までの総和をクライアントに送る
        - 目標の値(`goal`)をgoalとして受け取る
        - 総和の途中経過の値(`tmp_sum`)をfeedbackとして送る
        - 総和(`sum`)をresultとして送る
- `client_node.py`
    - `hello_msgs/Sum`型の`/sum`アクションにリクエストを送って、サーバからレスポンスを受け取る
    - `goal`をサーバに送り、1から`goal`までの総和をサーバから受け取る
        - 目標の値(`goal`)をgoalとして送る
        - 総和の途中経過の値(`tmp_sum`)をfeedbackとして受け取る
        - 総和(`sum`)をresultとして受け取る

## パッケージの作成

```bash
ros2 pkg create --build-type ament_python hello_action
```

## server_node.pyのコード

`hello_action/server_node.py`を以下の内容で書き込む。

```py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from hello_msgs.action import Sum
import time

class Server(Node):
    def __init__(self):
        super().__init__("server_node")
        self.sum_server = ActionServer(self, Sum, "sum", self.sum_callback)

    def sum_callback(self, goal_handle):
        goal = goal_handle.request.goal
        self.get_logger().info(f"Recieved goal: {goal}")

        num = 0
        feedback_msg = Sum.Feedback()
        feedback_msg.tmp_sum = num

        for i in range(1, goal):
            feedback_msg.tmp_sum += i
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Sending feedback: {feedback_msg.tmp_sum}")
            time.sleep(0.5)

        goal_handle.succeed()

        result = Sum.Result()
        result.sum = feedback_msg.tmp_sum
        self.get_logger().info(f"Sending result: {result.sum}")
        return result

def main(args=None):
    rclpy.init(args=args)

    node = Server()
    rclpy.spin(node)
```

## client_node.pyのコード

`hello_action/client_node.py`を以下の内容で書き込む。

```py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hello_msgs.action import Sum

class Client(Node):
    def __init__(self):
        super().__init__("client_node")
        self.sum_client = ActionClient(self, Sum, "sum")

    def send_goal(self, goal):
        self.get_logger().info(f"Sending goal: {goal}")
        goal_msg = Sum.Goal()
        goal_msg.goal = goal

        self.sum_client.wait_for_server()

        self.send_goal_future = self.sum_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected...")
            return

        self.get_logger().info("Goal accepted!")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sum}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Recieved feedback: {feedback.tmp_sum}")

def main(args=None):
    rclpy.init(args=args)

    node = Client()

    node.send_goal(10)

    rclpy.spin(node)
```

## setup.pyの編集

```py
from setuptools import find_packages, setup

package_name = 'hello_action'

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
            'server_node = hello_action.server_node:main',
            'client_node = hello_action.client_node:main',
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
ros2 run hello_action server_node
ros2 run hello_action client_node
```

```{figure} action-terminal-output.png
端末での実行画面
```

## 参照

- [https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [hello_actionパッケージのソースコード](https://github.com/Rione/home_ros2_workshop/tree/main/hello_action)
