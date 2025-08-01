"""
Launch both the obstacle detector and navigation nodes for the Moebius challenge.

This launch file brings up the two high‑level nodes in this package.
It assumes that the RealSense camera and the LDROBOT lidar drivers are
already running and publishing on their default topics (see ``README.md``
for details).  Remappings can be used to connect the lidar driver to
``/scan_raw`` if necessary.

Usage:

.. code-block:: bash

    ros2 launch autonomy_project challenge.launch.py

You can also pass additional command‑line arguments recognised by
``launch`` to override parameters or remappings.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Node: obstacle detector
    obstacle_node = Node(
        package='autonomy_project',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        output='screen',
    )
    # Node: navigation control
    navigation_node = Node(
        package='autonomy_project',
        executable='navigation_node',
        name='navigation',
        output='screen',
        # You can specify remappings here.  For example, if your lidar
        # driver publishes on /ldlidar_node/scan, uncomment the next
        # line to remap it to /scan_raw expected by the navigation node:
        # remappings=[('/scan_raw', '/ldlidar_node/scan')],
    )
    return LaunchDescription([
        obstacle_node,
        navigation_node,
    ])