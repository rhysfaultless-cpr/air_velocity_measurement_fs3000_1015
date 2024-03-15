from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  air_velocity_measurement_node = Node(
    package='air_velocity_measurement',
    executable='air_velocity_measurement',
    parameters=[
      {
        'port': '/dev/ttyUSB0',
        'baud': 115200
      }
    ]
  )

  ld = LaunchDescription()
  ld.add_action(air_velocity_measurement_node)

  return ld