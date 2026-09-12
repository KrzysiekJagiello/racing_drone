import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set use_sim_time to True for all nodes
    set_sim_time = SetParameter(name='use_sim_time', value=True)

    # Set GZ_SIM_RESOURCE_PATH to include the models directory of sitl_launcher package and the ArduPilot Gazebo models
    pkg_share = get_package_share_directory('sitl_launcher')
    world_path = os.path.join(pkg_share, 'worlds', 'iris_runway_rgb.sdf')
    models_path = os.path.join(pkg_share, 'models')

    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f"{models_path}:/home/ros/ardupilot_gazebo/models"
    )

    # Set path to URDF/Xacro model for drone_state_publisher
    xacro_path = os.path.join(pkg_share, 'urdf', 'iris_rgb.urdf.xacro')

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=[f'gz sim -v4 -r {world_path}'],
        shell=True,
        output='screen',
    )

    # Start ArduPilot SITL
    ardupilot_sitl = ExecuteProcess(
        cmd=['sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -N --out=udp:127.0.0.1:14551 --mavproxy-args="--streamrate=50"'],
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
        set_model_path,
        gazebo,
        ardupilot_sitl,
        mavros
    ])