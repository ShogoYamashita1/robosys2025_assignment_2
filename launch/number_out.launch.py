import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

     talker = launch_ros.actions.Node(
         package='robosys2025_assignment_2',      #パッケージの名前を指定
         executable='hand_input',  #実行するファイルの指定
         )
     listener = launch_ros.actions.Node(
         package='robosys2025_assignment_2',
         executable='number_output',
         output='screen'        #ログを端末に出すための設定
         )

     return launch.LaunchDescription([talker, listener])
