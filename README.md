# 手指認識を用いたじゃんけん・数字判定 ROS 2 パッケージ
![test](https://github.com/ShogoYamashita1/robosys2025_assignment_2/actions/workflows/test.yml/badge.svg)


## 概要
本パッケージは、MediaPipe を用いて指の開閉状態を判定するノードと、  
その結果をもとに解釈処理をする2種類の加工ノードから構成される ROS 2 用パッケージです。  
指の開閉状態の解釈の幅を持たせるため、判定ノードと加工ノードに分けています。

また、それぞれのノードを組み合わせて起動する launch ファイルを提供します。


## 説明
### ノード一覧
#### hand_node
- `MediaPipe` を用いて各指それぞれの開閉状態を判定する
- 各指の開閉状態配列として publish する

#### janken_node
- 指の開閉配列を受信し、じゃんけん（グー・チョキ・パー）を判定する
- 文字列として publish する

#### number_node
- 指の開閉配列を受信し、手で表された数字を判定する
- 数値として publish する

### トピック一覧
#### finger_close_state (std_msgs/Int16MultiArray)
- 各指の開閉状態を表す配列
- 要素の順番: [親指, 人差し指, 中指, 薬指, 小指]
- 値: 0 = 開いている, 1 = 閉じている

#### janken_result (std_msgs/String)
- じゃんけんの判定結果
- "Rock": グー
- "Scissors": チョキ
- "Paper": パー

#### number_result (std_msgs/Int16)
- 指で表された 0 ~ 5 の数字
- 判定できない場合は -1


## 準備
### OpenCV のインストール
hand_node の画面出力に使用
```bash
python3 -m pip install opencv-contrib-python
```

### MediaPipe のインストール
hand_node のカメラ入力に使用
```bash
python3 -m pip install mediapipe==0.10.21
```

### インストールができない場合
`No module named pip` と出てきた場合、pyton3 に pip がインストールされていない可能性があります。
```bash
sudo apt install python3-pip
```


## 実行方法
### hand_node + janken_node
```bash
ros2 launch robosys2025_assignment_2 janken_out.launch.py
```

### hand_node + number_node
```bash
ros2 launch robosys2025_assignment_2 number_out.launch.py
```


## 必要なソフトウェア
- Python: 3.10.12
- MediaPipe: 0.10.21
- OpenCV


## 動作環境
- OS: Ubuntu 22.04
- ROS 2: Humble


## ライセンス
このリポジトリはApache-2.0ライセンスで提供しています。  
詳細はLICENSEファイルを参照してください。  
© 2025-2026 Shogo Yamashita
