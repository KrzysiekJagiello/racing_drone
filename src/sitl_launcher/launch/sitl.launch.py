import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get config params
    set_sim_time = SetParameter(name='use_sim_time', value=True)

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz sim -v4 -r iris_runway.sdf'],
        shell=True,
        output='screen',
        
    )

    # Start ArduPilot SITL
    ardupilot_sitl = ExecuteProcess(
        cmd=['sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -N --out=udp:127.0.0.1:14551'],
        cwd='/home/ros/ardupilot',
        shell=True,
        output='screen'
    )

    # Start MAVROS
    mavros_launch_path = os.path.join(
            FindPackageShare('mavros').find('mavros'),
            'launch',
            'apm.launch'
        )
        
    mavros = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch_path),
            launch_arguments={
                'fcu_url': 'udp://127.0.0.1:14551@',
                'namespace': 'mavros'
            }.items()
        )

    return LaunchDescription([
        set_sim_time,
        gazebo,
        ardupilot_sitl,
        mavros
    ])