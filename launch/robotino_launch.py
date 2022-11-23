import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('robotino_driver')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'robotino_3.urdf')).read_text()

    camera_package_dir = get_package_share_directory('camera_localization')
    camera_description = pathlib.Path(os.path.join(camera_package_dir, 'resource', 'camera_handler.urdf')).read_text()
    
    webots = WebotsLauncher(
        world = os.path.join(package_dir, 'worlds', 'test_site.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    robotino3 = Node(
        package = 'webots_ros2_driver',
        executable = 'driver',
        output = 'screen',
        additional_env = {'WEBOTS_CONTROLLER_URL': 'robotino_3'},
        parameters = [
            {'robot_description': robot_description},
        ]
    )

    camera = Node(
        package = 'webots_ros2_driver',
        executable = 'driver',
        output = 'screen',
        additional_env = {'WEBOTS_CONTROLLER_URL': 'camera_handler'},
        parameters = [
            {'robot_description': camera_description},
        ]
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    return LaunchDescription([
        webots,
        robotino3,
        camera,
        ros2_supervisor,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action = webots,
                on_exit = [launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])