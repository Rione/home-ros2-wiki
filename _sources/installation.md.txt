# ROS 2のインストール

ROS 2では主にUbuntu上で開発します。
なので、皆さんのPCにUbuntuとROS 2をインストールして開発してもいます。

## Ubuntu 22.04のインストール

この部分はなるべく経験者や先輩と一緒に行いましょう。
最悪の場合、**パソコンのデータをすべて消す**可能性もあるので…

### データのバックアップ

万が一の場合を考えて、パソコンの必要なデータをバックアップしてください。

### ISOファイルの入手

Ubuntuの[ダウンロードページ](https://ubuntu.com/download/desktop)からISOファイルを入手します。
余裕があれば、ファイルのチェックサムを確認して正しいファイルをダウンロードしたか確認します。
ISOファイルが入手出来たら[Rufus](https://rufus.ie/en/)や[Etcher](https://etcher.balena.io/)などのツールを使ってUSBメモリにイメージを書き込みます。

Windowsを使っている人は他にもBitLockerの設定やセキュアブートの無効化など、必要に応じて行ってください。

## Ubuntuのインストール

ISOファイルを書き込んだUSBメモリをパソコンに差し込んで、USBメモリから起動するように指定してください。
あとはGUIのインストーラーが何をすればいいのか、案内してくれるのでそれに従ってください。
ここでデュアルブートにするかも指定できるので、先輩やパソコンのディスクの容量と相談して設定してください。

Ubuntuのインストールが終わったらパッケージの更新を行いましょう。

```none
sudo apt update
sudo apt upgrade
```

ついでに[Terminator](https://gnome-terminator.org/)をインストールしましょう。
Terminatorは画面を複数に分割出来るターミナルです。
デバッグ中に複数のターミナルを開く必要があるので必須級のパッケージです。

```none
sudo apt install terminator
```

あとは自分の好きなテキストエディタなどをインストールしておきましょう。

## VSCodeのインストール

ユーザ数が多いのでVSCodeのインストール方法はここに載せておきます。
ほぼ[公式ページ](https://code.visualstudio.com/docs/setup/linux)からの引用です。

```none
sudo apt-get install wget gpg apt-transport-https
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg
```

## ROS 2のセットアップ

[公式のページ](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
ではコマンドをひとつひとつ打ち込みながら解説しています。
ただかなり説明が長いので、コマンドを一つにまとめた
[シェルスクリプト](https://github.com/Rione/home_ros2_setup)
を実行します。

```none
sudo apt install git
git clone https://github.com/Rione/home_ros2_setup
cd home_ros2_setup
./setup.bash
```

以上のように実行するとROS 2を使える環境が整います。
このスクリプトは1000個以上のパッケージをダウンロードするので時間がかかるかもしれません。

## 参照

- [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [https://code.visualstudio.com/docs/setup/linux](https://code.visualstudio.com/docs/setup/linux)
- [https://github.com/Rione/home_ros2_setup](https://github.com/Rione/home_ros2_setup)
