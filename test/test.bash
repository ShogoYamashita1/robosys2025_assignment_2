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

### 入出力チェック関数 ###
in_out_check () {
    ros2 topic pub /finger_close_state std_msgs/msg/Int16MultiArray "{data: [${2}]}" -1
    ros2 topic echo /${1}_result --once > out.tmp
    cat out.tmp | grep -- "${3}"
}

res=0
ng_line=0
dir=~

[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build --packages-select robosys2025_assignment_2
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
source $dir/.bashrc


### janken_outputの入力テスト ###
ros2 run robosys2025_assignment_2 janken_output &
sleep 3

## トピックチェック ##
ros2 topic list | grep "janken_result"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## メッセージ型チェック ##
ros2 topic info /janken_result | grep "std_msgs/msg/String"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## 正常入力 ##
in_out_check "janken" "1,1,1,1,1" "Rock"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "janken" "1,0,0,1,1" "Scissors"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "janken" "0,0,0,0,0" "Paper"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## 異常入力 ##
in_out_check "janken" "1,0,0,0,0" "None"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "janken" "0,1,1,1,1" "None"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "janken" "1,1,0,1,1" "None"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
rm out.tmp
pkill -x janken_output
wait


### number_outputの入力テスト ###
ros2 run robosys2025_assignment_2 number_output &
sleep 3

## トピックチェック ##
ros2 topic list | grep "number_result"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## メッセージ型チェック ##
ros2 topic info /number_result | grep "std_msgs/msg/Int16"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

## 正常入力 ##
in_out_check "number" "1,1,1,1,1" "0"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "number" "1,0,1,1,1" "1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "number" "1,0,0,1,1" "2"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "number" "1,0,0,0,1" "3"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "number" "1,0,0,0,0" "4"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "number" "0,0,0,0,0" "5"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }


## 異常入力 ##
in_out_check "number" "1,1,0,0,0" "-1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "number" "0,1,1,1,1" "-1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }

in_out_check "number" "1,1,0,1,1" "-1"
[ "$?" = 0 ] || { ng_line="$LINENO" ; ng "$ng_line"; }
rm out.tmp
pkill -x number_output
wait


### テスト結果 ###
ng "$ng_line"
if [ "${res}" = 0 ]; then
    echo -e "\e[32mAll test cases passed successfully.\e[0m"
else
    echo -e "\e[31mTest failed\e[0m"
fi

exit $res
