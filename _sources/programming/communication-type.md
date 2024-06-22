# 通信の種類

前回の
{doc}`/programming/hello-world`
ではトピック通信というものを使ってデータの送受信を行いました。
ROS 2の世界ではトピック通信の他に、サービス通信やアクション通信というものがあります。

| 通信の種類     | 特徴                                     | 用途                                     |
| ---            | ---                                      | ---                                      |
| トピック通信   | 一方向、非同期通信                       | センサーからのデータの送信、受信など     |
| サービス通信   | 双方向、同期通信                         | 起動、終了、状態の確認など               |
| アクション通信 | 双方向、同期通信、サービス通信より高機能 | 時間がかかる処理で途中経過も知りたい時   |

## トピック通信

- センサからから得られた情報を送受信する場合に使う
- メッセージは`.msg`ファイルで定義
- 使用例
    - センサ情報の送受信
    - ロボットへの速度司令

## サービス通信

- 短時間で処理が終わるものに使う
- メッセージは`.srv`ファイルで定義
- 使用例
    - ロボットアームへの司令
    - 地図情報の読み込み

## アクション通信

- サービス通信とアクション通信を組み合わせたようなもの
- メッセージは`.action`ファイルで定義
- 使用例
    - ナビゲーションの司令

## データの種類

`std_msgs`パッケージでは以下のようなメッセージが使えます。

- `Bool`
- `Byte`
- `Char`
- `Float32`
- `Float64`
- `Int8`
- `Int16`
- `Int32`
- `Int64`
- `String`
- `UInt8`
- `UInt16`
- `UInt32`
- `UInt64`

また、データを配列にしたMultiArray型も存在します。

- `ByteMultiArray`
- `Float32MultiArray`
- `Float64MultiArray`
- `Int8MultiArray`
- `Int16MultiArray`
- `Int32MultiArray`
- `Int64MultiArray`
- `MultiArrayDimension`
- `MultiArrayLayout`
- `UInt16MultiArray`
- `UInt32MultiArray`
- `UInt64MultiArray`
- `UInt8MultiArray`

## 参照

- [https://github.com/ros2/common_interfaces](https://github.com/ros2/common_interfaces)
- [https://index.ros.org/p/std_msgs/github-ros2-common_interfaces/#humble](https://index.ros.org/p/std_msgs/github-ros2-common_interfaces/#humble)
