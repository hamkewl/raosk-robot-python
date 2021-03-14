# #raosk-edge-python
Python code for the program that runs on the edge that I'm trying to implement in my research  

## What is RAOSK?
RAOSK (Robotics Application Offloader Syetem on Kubernetes) は，hamstick (@hamstick, Koki Nagahama) がロボットアプリケーション研究の一環で開発し，動くように稼働したROSノードです．  
なお，本レポジトリに上がっているソースコードはシステム全体の機能においてはまだ未完了です．詳細は各自引継ぎ資料をみてください．

![RAOSK](readme_imgs/systemflow.png)

## Installation
ROS2の導入を済ませて，アプリケーションワークスペースを作ってから下記を実行してください  
ただし， `git clone` はワークスペースとは異なるディレクトリで実行してください  
```
$ git clone https://github.com/hamstick/raosk-robot-python.git
$ cp -r raosk-robot-python/rclpy_raosk/ [workspace-path]/ 
$ colcon build --symlink-install --cmake-clean-cache
$ source [workspace_path]/install/setup.bash
```
`ln -s` コマンドでrclpy_raoskディレクトリに対するシンボリックリンクを発行しても問題ありません  
その場合は次のようになります (ワークスペースとは異なるディレクトリで実行してください)  
```
$ git clone https://github.com/hamstick/raosk-robot-python.git
$ cd [workspace-path]
$ ln -s [path-of-raosk-robot-python-git]/rclpy_raosk/
$ colcon build --symlink-install --cmake-clean-cache
$ source [workspace_path]/install/setup.bash
```

## How to Use
```
$ ros2 run [package_name] [exec_node]
```
ROS1とは異なり，setup.pyで設定される起動コマンドになります  

**ex.** raosk_meminfoパッケージのノードを起動したい場合
```
$ ros2 run raosk_meminfo start
```
止めるときは `Ctrl+C` またはそれに相当するキーで  
プログラム中で `KeyboardInterrupt` をスルーするように書いています

## Problem
引継ぎ資料をご参照ください．  
3/24ぐらいまでは作業する予定なのでソースコードに関してはそれまでノータッチでお願いします．
