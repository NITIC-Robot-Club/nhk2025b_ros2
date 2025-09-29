from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    description_file = get_package_share_directory('nhk2025b_description') + '/urdf/nhk2025b.urdf'

    robot_description_content = Command(['xacro ', description_file])

    cmd_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace="/discripution/cmd",
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'frame_prefix': 'cmd/',
        }]
    )

    cmd_joint_state_publisher = Node(
        package='nhk2025b_joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace="/discripution/cmd",
        output='screen',
        parameters=[{
            'topicname': 'cmd',
        }]
    )

    result_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace="/discripution/result",
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'frame_prefix': '',
        }]
    )

    result_joint_state_publisher = Node(
        package='nhk2025b_joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace="/discripution/result",
        output='screen',
        parameters=[{
            'topicname': 'result',
        }]
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_cmd',
        arguments=["-d", get_package_share_directory("nhk2025b_launch") + "/rviz/urdf_cmd.rviz"],
        output='screen'
    )

    rviz_result = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_result',
        arguments=["-d", get_package_share_directory("nhk2025b_launch") + "/rviz/urdf_result.rviz"],
        output='screen'
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        namespace="/discripution/cmd",
    )

    return LaunchDescription([
        cmd_robot_state_publisher,
        cmd_joint_state_publisher,
        result_robot_state_publisher,
        result_joint_state_publisher,
        rviz_cmd,
        rviz_result,
    ])
