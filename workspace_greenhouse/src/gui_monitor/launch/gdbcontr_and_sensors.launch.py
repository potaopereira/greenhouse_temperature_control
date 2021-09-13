import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            # default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            # default_value=[launch.substitutions.EnvironmentVariable('LOGNAME'), '_'],
            default_value=[''],
            description='Prefix for node names'
        )
        ,
        launch_ros.actions.Node(
            package='temperature_sensor',
            executable='temperature_sensor',
            output='screen',
            # namespace='temperature_sensor',
            remappings = [
                ("temperature", "temperature_sensor1")
                ,
                ("read_temperature", "temperature_sensor1_sim")
            ],
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'temperature_sensor1']
        )
        ,
        launch_ros.actions.Node(
            package='temperature_sensor',
            executable='temperature_sensor',
            output='screen',
            # namespace='temperature_sensor',
            remappings = [
                ("temperature", "temperature_sensor2")
                ,
                ("read_temperature", "temperature_sensor2_sim")
            ],
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'temperature_sensor2']
        )
        ,
        launch_ros.actions.Node(
            package='window_controller',
            executable='window_controller',
            output='screen',
            parameters = [{"pgain": 10.0, "igain": 0.05, "ierrormax": 10., "target": 25.0}],
            # namespace='window_controller',
            # namespace='greenhouse_simulator',
            # remappings = [
            #     ("from", "to")
            # ],
            prefix=['gdbserver localhost:3000'],
            emulate_tty=True,
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'window_controller']
        )
        ,
        launch_ros.actions.Node(
            package='gui_monitor',
            executable='gui_monitor',
            output='screen',
            # parameters = [],
            # namespace='window_controller',
            # namespace='greenhouse_simulator',
            # remappings = [
            #     ("from", "to")
            # ],
            # prefix=['gdbserver localhost:3000'],
            # emulate_tty=True,
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'gui_monitor']
        )
    ])