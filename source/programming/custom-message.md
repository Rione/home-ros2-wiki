# 自作メッセージ

これまで使ってきたメッセージの型は誰かが既に定義したものを使っていました。
ですが、使っているロボットなどに応じて独自のメッセージを定義して、トピック通信やサービス通信、アクション通信に使うことも可能です。

今回作ったメッセージファイルは以降の
{doc}`/programming/topic`
、
{doc}`/programming/service`
や
{doc}`/programming/action`
の講義で使うので、この部分は飛ばさないようにお願いします。

## パッケージの作成

```bash
ros2 pkg create --build-type ament_cmake hello_msgs
```

## メッセージの定義

### .msgファイル

`.msg`ファイルの形式は以下のようになっています。

```none
型 foo
型 bar
```

トピック通信で使うメッセージファイルはパッケージの`msg`フォルダで`.msg`ファイルで定義します。

```bash
mkdir msg
```

`msg/Person.msg`を以下の内容で保存してください。

```none
string name
uint8 age
```

### .srvファイル

`.srv`ファイルの形式は以下のようになっています。

```none
型 request
---
型 response
```

`---`より上の部分はサービスサーバに送るもので、下の部分はサービスサーバから送られてくるものです。
今回は送受信するメッセージをそれぞれ1つしか定義していませんが、複数定義することも出来ます。

サービス通信で使うメッセージファイルはパッケージの`srv`フォルダで`.srv`ファイルで定義します。

```bash
mkdir srv
```

`srv/Order.srv`を以下の内容で保存してください。

```none
string menu
---
string message
```

### .actionファイル

`.action`ファイルの形式は以下のようになっています。

```none
型 goal
---
型 result
---
型 feedback
```

`goal`の部分はアクションサーバに送るもので、`result`の部分はアクションサーバかた送られてくるものです。
下の`feedback`の部分は`result`が送られるまでの間に途中経過としてアクションサーバから送られてくるものです。

アクション通信で使うメッセージファイルはパッケージの`action`フォルダで`.action`ファイルで定義します。

```bash
mkdir action
```

`action/Sum.action`を以下の内容で保存してください。

```none
uint64 goal
---
uint64 sum
---
uint64 tmp_sum
```

## CMakelists.txtの編集

`CMakelists.txt`に以下を追加してください。

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "srv/Order.srv"
  "action/Sum.action"
)
```

最終的に以下のような`CMakelists.txt`になります。

```cmake
cmake_minimum_required(VERSION 3.8)
project(hello_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "srv/Order.srv"
  "action/Sum.action"
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

## package.xmlの編集

以下を`package.xml`に追加して依存関係を定義してください。

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

最終的に以下のような`package.xml`になります。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hello_msgs</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="ri-one@todo.todo">ri-one</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <!-- この3行を追加 -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## ビルド

```bash
cd ~/my_ws
colcon build
source install/setup.bash
```

`ros2 interface show`コマンドで作ったメッセージの型を確認しましょう。

```bash
$ ros2 interface show hello_msgs/msg/Person 
string name
uint8 age
```

```bash
$ ros2 interface show hello_msgs/srv/Order
string menu
---
string message
```

```bash
$ ros2 interface show hello_msgs/action/Sum 
uint64 goal
---
uint64 sum
---
uint64 tmp_sum
```

## 参照

- [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [hello_msgsパッケージのソースコード](https://github.com/Rione/home_ros2_workshop/tree/main/hello_msgs)
