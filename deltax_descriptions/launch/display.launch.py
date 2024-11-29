from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# ros2 launch deltax_descriptions display.launch.py model:='urdf/deltaxs_v5_d800.urdf'

def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'deltax_descriptions'

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'urdf.rviz']), description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    model_arg = DeclareLaunchArgument(name='model', default_value=PathJoinSubstitution(['urdf', 'deltaxs_v5_d800.urdf']), description='Path to robot urdf file relative to urdf_tutorial package')
    ld.add_action(model_arg)

    package_arg = DeclareLaunchArgument(name='package', default_value=package_name, description='Path to package with urdf_file')
    ld.add_action(package_arg)

    
    # add a nestested launch file from urdf_launch package
    # this launch files requires that you define certain launch arguments!

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('package') , #name of package where urdf model is in
            'urdf_package_path': LaunchConfiguration('model'), #path to urdf file in the package
            'rviz_config': LaunchConfiguration('rvizconfig'),
            # 'jsp_gui': LaunchConfiguration('gui')
            }.items()
    ))

    return ld
