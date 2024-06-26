# Launchファイル

ROSでは複数のノードを同時に立ち上げて、それぞれのノードからデータを送受信したりしています。
今までは`ros2 run`コマンドで1つ1つノードを立ち上げていましたが、実際には複数のノードをLaunchファイルというもので起動します。
このLaunchファイルからトピック名やノード名、パラメータの値を変更出来るので非常に便利です。

```{note}
ここでは
{doc}`/programming/topic`
で作った`hello_topic`パッケージの`iteration_node`ノードと
{doc}`/programming/parameter`
で作った`hello_param`パッケージの`multiply_node`ノードを使います。
まだ作ってない人は先に、そちらをご覧ください。
```

## 今回の目標

今回は以下のようなLaunchファイルをもつ`hello_launch`パッケージを作りましょう。

- `hello.launch.py`
    - 以下の2つのノードを立ち上げるLaunchファイル
        - `hello_topic`パッケージの`iteration_node.py`
        - `hello_topic`パッケージの`double_node.py`
- `two_double.launch.py`
    - 以下の3つのノードを立ち上げるLaunchファイル
        - `hello_topic`パッケージの`iteration_node.py`
        - `hello_topic`パッケージの`double_node.py`
        - `hello_topic`パッケージの`double_node.py`
            - `double_node`ノードを`quad_node`ノードに変更
            - `/number`トピックを`/double_number`トピックにリマップ
            - `/double_number`トピックを`/quad_number`トピックにリマップ
- `double_and_multiply.launch.py`
    - 以下の3つのノードを立ち上げるLaunchファイル
        - `hello_topic`パッケージの`iteration_node.py`
        - `hello_topic`パッケージの`double_node.py`
        - `hello_param`パッケージの`multiply_node.py`
            - `/number`トピックを`/double_number`トピックにリマップ
            - `m_number`パラメータを`4`に変更

```{tip}
今回はPythonでLaunchファイルを書きますが、YAMLやROS 1時代のXMLでもLaunchファイルを書けます。
詳しくは[公式のチュートリアル(英語)](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)を読んでください。
```

## パッケージの作成

```bash
ros2 pkg create --build-type ament_cmake hello_launch
```

Launchファイルを保存する`launch`ディレクトリを作成します。

```bash
cd hello_launch
mkdir launch
```

## CMakelists.txtの編集

以下の3行を`CMakelists.txt`に追加

```cmake
# 追加
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

最終的な`CMakelists.txt`は以下のようになります。

```cmake
cmake_minimum_required(VERSION 3.8)
project(hello_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 追加
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## hello.launch.pyのコード

`launch/hello.launch.py`を以下の内容で書き込む。

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_topic',
            executable='iteration_node',
            name='iteration_node',
        ),
        Node(
            package='hello_topic',
            executable='double_node',
            name='double_node',
        ),
    ])
```

## two_double.launch.pyのコード

`launch/two_double.launch.py`を以下の内容で書き込む。

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_topic',
            executable='iteration_node',
            name='iteration_node',
        ),
        Node(
            package='hello_topic',
            executable='double_node',
            name='double_node',
        ),
        Node(
            package='hello_topic',
            executable='double_node',
            name='quad_node',
            remappings=[
                ('/number', '/double_number'),
                ('/double_number', '/quad_number'),
            ],
        ),
    ])
```

## double_and_multiply.launch.pyのコード

`launch/double_and_multiply.launch.py`を以下の内容で書き込む。

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_topic',
            executable='iteration_node',
            name='iteration_node',
        ),
        Node(
            package='hello_topic',
            executable='double_node',
            name='double_node',
        ),
        Node(
            package='hello_param',
            executable='multiply_node',
            name='multiply_node',
            remappings=[
                ('/number', '/double_number'),
            ],
            parameters=[
                {'m_number': 4},
            ]
        ),
    ])
```

## ビルドと実行

```bash
cd ~/my_ws
colcon build
source install/setup.bash
```

`ros2 launch`コマンドでLaunchファイルを立ち上げましょう。
立ち上がったら`rqt_graph`でノード間の通信の様子や`rqt`を使ってトピックを見てみましょう。

```{tip}
`rqt`でトピックを見るには`Plugins`タブから`Topics`、`Topics`から`Topic Monitor`を選択してトピックの様子を見てみましょう。
```

### hello.launch.py

```bash
ros2 launch hello_launch hello.launch.py
```

ノード間の通信の図は既に
{doc}`/programming/hello-world`
で紹介したグラフと同じなので省略します。


### two_double.launch.py

```bash
ros2 launch hello_launch two_double.launch.py
rqt_graph
rqt
```

```{figure} launch-two-double-rqt-graph.png
rqt_graph
```

```{figure} launch-two-double-rqt.png
rqtでトピックを見た様子
```

`/number`トピックの値を2倍したものが`/double_number`へ、`/double_number`トピックの値をさらに2倍したものが`/quad_number`へ送られているのが分かります。

### double_and_multiply.launch.py

```bash
ros2 launch hello_launch double_and_multiply.launch.py
rqt_graph
rqt
```

```{figure} launch-double-and-multiply-rqt-graph.png
rqt_graph
```

```{figure} launch-double-and-multiply-rqt.png
rqtでトピックを見た様子
```

`/number`トピックの値を2倍したものが`/double_number`へ、`/double_number`トピックの値をさらにパラメータで指定した4倍にしたものが`/multiplied_number`へ送られているのが分かります。

## 参照

- [https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [hello_launchパッケージのソースコード](https://github.com/Rione/home_ros2_workshop/tree/main/hello_launch)
