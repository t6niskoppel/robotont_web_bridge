"""Web stack for Blockly and the dashboard: rosbridge + web_video_server + the Robotont helper.

Run on the machine that can see the robot in DDS (the laptop, or the robot itself):
  ros2 launch robotont_web_bridge web.launch.py
rosbridge listens on port 9090, the video server on 8080 and the terminal (ttyd) on 7681, on all interfaces. Use a private network.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('video_port', default_value='8080'),
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'ROBOTONT_NAMESPACE', default_value=''),
                              description='Robot namespace; empty offers every robot found'),
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket', output='screen',
             parameters=[sim, {'port': ParameterValue(LaunchConfiguration('rosbridge_port'), value_type=int),
                               'address': ''}]),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen', parameters=[sim]),
        Node(package='web_video_server', executable='web_video_server', name='web_video_server', output='screen',
             parameters=[sim, {'port': ParameterValue(LaunchConfiguration('video_port'), value_type=int),
                               'address': '0.0.0.0'}]),
        # A shell in the browser (ttyd), started in this environment so ROS and the workspace are loaded.
        # It is unauthenticated unless TTYD_CREDENTIAL=user:password is set: private networks only.
        ExecuteProcess(cmd=['bash', '-c', 'exec ttyd -p ${TTYD_PORT:-7681} -W ${TTYD_CREDENTIAL:+-c "$TTYD_CREDENTIAL"} bash -i'],
                       name='ttyd', output='screen'),
        Node(package='robotont_web_bridge', executable='web_bridge', name='web_bridge', output='screen',
             parameters=[sim, {'namespace': ParameterValue(LaunchConfiguration('namespace'), value_type=str)}]),
    ])
