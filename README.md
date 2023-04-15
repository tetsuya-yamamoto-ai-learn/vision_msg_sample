# 点群物体検出結果表示サンプル

vision_msgsを用いて検出結果を表示するサンプル

vision_msgs | https://github.com/ros-perception/vision_msgs/tree/ros2

## 事前準備

3次元描画用のツールであるvision_msgsをインストールして、3次元物体検出結果を3D-BoundingBoxで表示できるようにする

    cd ros2_ws/src
    git clone https://github.com/ros-perception/vision_msgs/tree/ros2
    git checkout ros2
    cd ..
    colcon build
    source install/local_setup.bash

## MEMO

- Detection.idとhypothesis.class_idの違い

    - Detection.id: 複数フレーム間での同一オブジェクトを説明するためのID
    - hypothesis.class_id: 物体のクラスを説明するID(車、人など) 

- 検出結果に対応した色を変更する方法

    `vision_msgs/vision_msgs_rviz_plugins/conf/example.yaml`の記述を参考に追記して、rviz上のConfigPathでそのyamlファイルを指定すれば変更できるらしい、デフォルトでは[car: orange, person: blue, cyclist: yellow, motorcycle: purple, other: grey](ラベルをRviz上で表示できないかなぁ。。。)