from ament_index_python.packages import get_package_share_path
import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(name='detection2landmark', package='detection2landmark', executable='detection2landmark',
                                parameters=[str(get_package_share_path('detection2landmark') / 'config/detection2landmark.yaml')]
                                )
                                    
    ])