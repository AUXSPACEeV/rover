import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    models_path = os.path.join(pkg_share, 'models')

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    def spawn(model_name, x, y, z):
        return Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', model_name,
                '-file', os.path.join(models_path, model_name, 'model.sdf'),
                '-x', str(x), '-y', str(y), '-z', str(z),
            ],
            output='screen',
        )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        spawn('rover_body', 0.0, 0.0, 0.5),
        spawn('rocker_assembly', 0.0, 0.0, 0.5),
        spawn('steering_wheel', 0.3, 0.0, 0.5),
        spawn('wheel_assembly', -0.3, 0.0, 0.5),
    ])
