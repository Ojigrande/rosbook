from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

  kobuki_cmd = Node(package='my_fsm_bumpgo_py',
                    executable='bump_go_main',
                    output='screen',
                    parameters=[{
                      'use_sim_time': True
                    }],
                    remappings=[
                      ('input_scan', '/scan_raw'),
                      ('output_vel', '/cmd_vel')
                    ])

  ld = LaunchDescription()
  ld.add_action(kobuki_cmd)

  return ld
