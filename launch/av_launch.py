import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def launch_setup(context, *args, **kwargs):
    world_path = LaunchConfiguration('world').perform(context)

    # Start physics automatically
    os.environ["WEBOTS_ROS2_AUTOSTART"] = "1"

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    urdf_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'av_navigation',
        'resource',
        'tesla_ros2.urdf'
    )

    # Attach controller to the car (the name must be "vehicle" in the world)
    av_driver = WebotsController(
        robot_name='vehicle',
        parameters=[
            {'robot_description': urdf_path, 'use_sim_time': True},
            os.path.join(
                os.getenv('HOME'),
                'ros2_ws',
                'src',
                'av_navigation',
                'config',
                'tesla_sensors.yaml'
            )
        ],
        respawn=True
    )

    delayed_driver = TimerAction(period=2.5, actions=[av_driver])

    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return [webots, delayed_driver, shutdown_event]

def generate_launch_description():
    default_world_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'av_navigation',
        'worlds',
        'av_world.wbt'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world_path),
        OpaqueFunction(function=launch_setup)
    ])