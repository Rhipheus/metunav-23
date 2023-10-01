from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True")

    local_params_file = os.path.join(get_package_share_directory(
        "metu_localization"), "config", "ekf.yaml")
    
    param_substitutions = {
        "use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=local_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    local_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="log",
        parameters=[configured_params],
        remappings=[("odometry/filtered", "/odometry/local")
        #            ("accel/filtered", "/accel")
        ]
        )
    
    global_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="log",
        parameters=[configured_params],
        remappings=[("odometry/filtered", "/odometry/global")
        #            ("accel/filtered", "/accel")
        ])
    nav_sat=Node(
        package='robot_localization', 
        executable='navsat_transform_node', 
        name='navsat_transform',
        output='screen',
        parameters=[configured_params],
        remappings=[('imu/data', 'imu'),
                    ('gps/fix', 'gps/fix'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', '/odometry/global')]           
    ) 
    
    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)

    ld.add_action(local_ekf_cmd)
    ld.add_action(global_ekf_cmd)
    ld.add_action(nav_sat)
    return ld
