#!/bin/bash
# SPDX-FileCopyrightText: 2025-2026 Shogo Yamashita
# SPDX-License-Identifier: Apache-2.0


### 異常終了関数 ###
# テスト終了時にも表示できるようng_lineで失敗した行を記録
ng () {
    if [ "${1}" != 0 ]; then
        echo ${1}行目で失敗しました
        res=1
    fi
}


res=0
ng_line=0
dir=~


[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build --packages-select robosys2025_assignment_2
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
source $dir/.bashrc


### hand_inputの出力テスト ###
timeout 10 ros2 run robosys2025_assignment_2 hand_input &
sleep 3
ros2 topic echo /finger_close_state --once > data.tmp
cat data.tmp | tr -d "\n -" | grep "00000"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
wait


### janken_outputの入力テスト ###
timeout 30 ros2 run robosys2025_assignment_2 janken_output &
sleep 3

## 正常入力 ##
ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,1,1,1,1]}" -1
ros2 topic echo /janken_result --once > out.tmp
cat out.tmp | grep "Rock"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,0,0,1,1]}" -1
ros2 topic echo /janken_result --once > out.tmp
cat out.tmp | grep "Scissors"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [0,0,0,0,0]}" -1
ros2 topic echo /janken_result --once > out.tmp
cat out.tmp | grep "Paper"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## 異常入力 ##
ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,0,0,0,0]}" -1
ros2 topic echo /janken_result --once > out.tmp
cat out.tmp | grep "None"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [0,1,1,1,1]}" -1
ros2 topic echo /janken_result --once > out.tmp
cat out.tmp | grep "None"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,1,0,1,1]}" -1
ros2 topic echo /janken_result --once > out.tmp
cat out.tmp | grep "None"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
rm out.tmp
wait


### janken_outputの入力テスト ###
timeout 60 ros2 run robosys2025_assignment_2 number_output &
sleep 3

## 正常入力 ##
ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,1,1,1,1]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep "0"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,0,1,1,1]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep "1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,0,0,1,1]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep "2"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,0,0,0,1]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep "3"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,0,0,0,0]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep "4"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [0,0,0,0,0]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep "5"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }


## 異常入力 ##
ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,1,0,0,0]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep -- "-1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [0,1,1,1,1]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep -- "-1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [1,1,0,1,1]}" -1
ros2 topic echo /number_result --once > out.tmp
cat out.tmp | grep -- "-1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
rm out.tmp
wait


### janken_out.launchのテスト ###
timeout 20 ros2 launch robosys2025_assignment_2 janken_out.launch.py &
sleep 3

## トピックチェック ##
ros2 topic list | grep "finger_close_state"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
ros2 topic list | grep "janken_result"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## メッセージ型チェック ##
ros2 topic info /finger_close_state | grep "std_msgs/msg/Int16MultiArray"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
ros2 topic info /janken_result | grep "std_msgs/msg/String"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## データチェック ##
ros2 topic echo /finger_close_state --once>data.tmp
cat data.tmp | tr -d "\n -" | grep "00000"
ros2 topic echo /janken_result --once>data.tmp
cat data.tmp | grep -E "Rock|Scissors|Paper|None"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
rm data.tmp
wait


### number_out.launchのテスト ###
timeout 20 ros2 launch robosys2025_assignment_2 number_out.launch.py &
sleep 3

## トピックチェック ##
ros2 topic list | grep "finger_close_state"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
ros2 topic list | grep "number_result"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## メッセージ型チェック ##
ros2 topic info /finger_close_state | grep "std_msgs/msg/Int16MultiArray"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
ros2 topic info /number_result | grep "std_msgs/msg/Int16"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## データチェック ##
ros2 topic echo /finger_close_state --once>data.tmp
cat data.tmp | tr -d "\n -" | grep "00000"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
ros2 topic echo /number_result --once>data.tmp
cat data.tmp | grep -E -- "-1|0|1|2|3|4|5"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
rm data.tmp
wait


### テスト結果 ###
ng "$ng_line"
if [ "${res}" = 0 ]; then
    echo -e "\e[32mAll test cases passed successfully.\e[0m"
else
    echo -e "\e[31mTest failed\e[0m"
fi

exit $res
