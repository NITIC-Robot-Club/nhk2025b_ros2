
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id')
    output_topic0 = LaunchConfiguration('output_topic0')
    output_topic1 = LaunchConfiguration('output_topic1')
    inverted = LaunchConfiguration('inverted')
    hostip = LaunchConfiguration('hostip')
    port0 = LaunchConfiguration('port0')
    port1 = LaunchConfiguration('port1')
    angle_offset = LaunchConfiguration('angle_offset')
    scanfreq = LaunchConfiguration('scanfreq')
    filter = LaunchConfiguration('filter')
    laser_enable = LaunchConfiguration('laser_enable')
    scan_range_start = LaunchConfiguration('scan_range_start')
    scan_range_stop = LaunchConfiguration('scan_range_stop')
    sensorip = LaunchConfiguration('sensorip')


    # frame_id = LaunchConfiguration('frame_id', default='laser')
    # output_topic0 = LaunchConfiguration('output_topic0', default='pcd0')
    # output_topic1 = LaunchConfiguration('output_topic1', default='pcd1')
    # inverted = LaunchConfiguration('inverted', default='false')
    # hostip = LaunchConfiguration('hostip',default='0.0.0.0')
    # port0 = LaunchConfiguration('port0',default="2368")
    # port1 = LaunchConfiguration('port1',default='2369')
    # angle_offset = LaunchConfiguration('angle_offset',default='0')
    

    declare_frame_id_cmd = DeclareLaunchArgument(
    'frame_id',
    default_value='lidar_front',
    )
    declare_output_topic0_cmd = DeclareLaunchArgument(
    'output_topic0',
    default_value='/sensor/lidar/front/scan',
    )
    declare_output_topic1_cmd = DeclareLaunchArgument(
    'output_topic1',
    default_value='scan1',
    )
    declare_inverted_cmd = DeclareLaunchArgument(
    'inverted',
    default_value='false',
    )
    declare_hostip_cmd = DeclareLaunchArgument(
    'hostip',
    default_value='0.0.0.0',
    )
    declare_port0_cmd = DeclareLaunchArgument(
    'port0',
    default_value=TextSubstitution(text='"2368"'),
    )
    declare_port1_cmd = DeclareLaunchArgument(
    'port1',
    default_value='"2369"',
    )
    declare_angle_offset_cmd = DeclareLaunchArgument(
    'angle_offset',
    default_value='0',
    )
    declare_filter_cmd = DeclareLaunchArgument(
    'filter',
    default_value='"3"',
    )
    declare_scanfreq_cmd = DeclareLaunchArgument(
    'scanfreq',
    default_value='"30"',
    )
    declare_laser_enable_cmd = DeclareLaunchArgument(
    'laser_enable',
    default_value='"true"',
    )
    declare_scan_range_start_cmd = DeclareLaunchArgument(
    'scan_range_start',
    default_value='"45"',
    )
    declare_scan_range_stop_cmd = DeclareLaunchArgument(
    'scan_range_stop',
    default_value='"315"',
    )
    declare_sensorip_cmd = DeclareLaunchArgument(
    'sensorip',
    default_value='192.168.198.2',
    )

    richbeam_lidar_node0 = Node(
        package='lakibeam1',
        name='richbeam_lidar_node0',
        executable='lakibeam1_scan_node',
        parameters=[{
            'frame_id':frame_id,
            'output_topic':output_topic0,
            'inverted':inverted,
            'hostip':hostip,
            'port':port0,
            'angle_offset':angle_offset,
            'sensorip':sensorip,
            'scanfreq':scanfreq,
            'filter':filter,
            'laser_enable':laser_enable,
            'scan_range_start':scan_range_start,
            'scan_range_stop':scan_range_stop
        }],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_frame_id_cmd)
    ld.add_action(declare_output_topic0_cmd)
    ld.add_action(declare_output_topic1_cmd)
    ld.add_action(declare_inverted_cmd)
    ld.add_action(declare_hostip_cmd)
    ld.add_action(declare_port0_cmd)
    ld.add_action(declare_port1_cmd)
    ld.add_action(declare_angle_offset_cmd)
    ld.add_action(declare_filter_cmd)
    ld.add_action(declare_scanfreq_cmd)
    ld.add_action(declare_laser_enable_cmd)
    ld.add_action(declare_scan_range_start_cmd)
    ld.add_action(declare_scan_range_stop_cmd)
    ld.add_action(declare_sensorip_cmd)
    ld.add_action(richbeam_lidar_node0)
    # ld.add_action(richbeam_lidar_node1)
    # ld.add_action(rviz_node)
    return ld