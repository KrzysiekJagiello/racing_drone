from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                # Sim time
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                # RGB camera
                '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                # IMU data
                '/world/iris_runway/model/iris_rgb/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            remappings=[
                ('/camera', '/iris_rgb/camera_image'),
                ('/camera_info', '/iris_rgb/camera_info'),
                ('/world/iris_runway/model/iris_rgb/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu', '/iris_rgb/imu')
            ],


            output='screen'
        )
    ])