from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    tf2_tutorial_robot_path = FindPackageShare('tf2_tutorial_robot')
    default_model_path = PathJoinSubstitution(['urdf', 'robot.urdf'])
    default_rviz_config_path = PathJoinSubstitution([tf2_tutorial_robot_path, 'rviz', 'urdf.rviz'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to tf2_tutorial_robot package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'tf2_tutorial_robot',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': 'shapes.sdf'
        #    'gui': LaunchConfiguration('gui'),
        #    'pause': 'true',
        }.items(),
    ))

    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        name='robot_spawner',
        arguments=['-topic', '/robot_description', '-name', 'robot', '-z', '0.25', '-unpause'],
        output='screen',
    ))

    return ld