## sd_sample_pkg (A班)

TAの[sskitajima](https://github.com/sskitajima)さんが作成したサンプルパッケージをフォークして追加機能を実装しました。

## ダウンロード

```shell
cd ~/catkin_ws/src
git clone https://github.com/Decwest/sd_sample_pkg.git
cd ~/catkin_ws
catkin_make
```

すでにsd_sample_pkgが存在する場合はダウンロードできないので、お手数ですが一旦別の場所に移動させてください。



## コーラ缶の追跡

```shell
roslaunch sd_sample_pkg pursuit_cokecan.launch
```

パターンマッチングでコーラ缶を検出し、その方向へ進みます。

ラズパイマウスのレーザーセンサで距離を測定し、コーラ缶の約0.2m手前で停止します。

※初めてシミュレーションをする際、モデルのダウンロードに時間がかかりGazeboがフリーズすることがあるので注意



## YOLOを使用する

### Install darknet_ros

```shell
cd ~/catkin_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```



### Download weights

```shell
cd ~/catkin_ws/src/sd_sample_pkg
./download_weights.sh
```

デフォルトではyolov3-tiny.weightsをダウンロードするようにしている。引数を与えることで他の重みもダウンロード可能。



## 実行

```shell
roslaunch sd_sample_pkg object_detection.launch
```

gazebo上でリスポーンさせることができるモデルのうち、yolov3-tinyで認識できるものはpersonくらいなので、personをリスポーンさせてみると良い。



## うまく行かない時は

以下のコマンドを実行するとROSコマンドにパスが通り、これで解決することが多い。

```shell
cd ~/catkin_ws
source devel/setup.bash
```

---

※以下は元のpkgのREADME.md

## ダウンロード

```shell
cd ~/catkin_ws/src
git clone https://github.com/sskitajima/sd_sample_pkg.git
cd ~/catkin_ws
catkin_make
```

## サンプルプログラム

- 各処理の詳細や内容についてはソースコードを見てほしい

### トピック通信のサンプルプログラム

- ```src/listener.cpp```
- ```src/talker.cpp```

```shell
# terminal 1
roscore

# terminal 2
rosrun sd_sample_pkg talker_node

# termianl 3
rosrun sd_sample_pkg listener_node
```

### レーザースキャナとカメラのサンプルプログラム

- ```src/SD_sample_laser.cpp```
- ```src/SD_sample_camera.cpp```

```shell
# terminal 1
roslaunch sd_sample_pkg robot_simulation.launch

# terminal 2
rosrun sd_sample_pkg SD_sample_laser

# terminal 3
rosrun sd_sample_pkg SD_sample_camera
```

### 遠隔操作のサンプルプログラム

- ```scripts/teleop.py```

```shell
# terminal 1
roscore

# terminal 2
rosrun sd_sample_pkg teleop.py
```

- もしくは、以下のようにして実行してもよい。

```sh
rosrun teleop_twist_keybod teleop_twist_keyboard.py
```

### 画像処理のサンプルプログラム
- ```src/metch_templete.cpp```内の```${USERNAME}```は、各自の環境のユーザ名に変更して有効なパスになるようにする

```shell
# teminal 1
roscore

# teminal 2
rosrun sd_sample_pkg matching_node
```



## シミュレーション

- ロボットモデル: ```urdf/robot.urdf.xacro```
- サンプルのworld: ```worlds/sample.world```

