from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_gazebo_plugins',
            executable='drone_sensor_node'
        ),
        
        Node(
            package = 'drone_estimator',
            executable = 'estimator_node',
            name = 'drone_estimator',
            parameters = [
                '/home/jigar/Robotics/Drone/config/drone_params.yaml'
            ] 
        ),

        # Node(
        #     package = 'drone_command',
        #     executable = 'drone_setpoint',
        #     name = 'drone_command',
        #     parameters = [
        #         '/home/jigar/Robotics/Drone/config/drone_params.yaml'
        #     ]
        # ),

        Node(
            package = 'drone_control',
            executable = 'attitude_control',
            name = 'attitude_control',
            parameters = [
                '/home/jigar/Robotics/Drone/config/drone_params.yaml'
            ]
        ),

        # Node(
        #     package = 'drone_control',
        #     executable = 'position_control',
        #     name = 'position_control',
        #     parameters = [
        #         '/home/jigar/Robotics/Drone/config/drone_params.yaml'
        #     ]
        # ),

        # Node(
        #     package = 'drone_control',
        #     executable = 'altitude_velocity_control',
        #     name = 'altitude_velocity_control',
        #     parameters = [
        #         '/home/jigar/Robotics/Drone/config/drone_params.yaml'
        #     ]
        # ),

        # Node(
        #     package = 'drone_control',
        #     executable = 'xy_velocity_control',
        #     name = 'xy_velocity_control',
        #     parameters = [
        #         '/home/jigar/Robotics/Drone/config/drone_params.yaml'
        #     ]
        # ),
        
        Node(
            package = 'drone_mixer',
            executable = 'motor_mixer'
        ),

        Node(
            package = 'drone_actuator',
            executable = 'drone_pwm',
            name = 'drone_pwm',
            parameters = [
                '/home/jigar/Robotics/Drone/config/drone_params.yaml'
            ]
        )
    ])
