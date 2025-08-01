Autonomous Control for the Moebius Mini‑Car
===========================================

This package contains a reference implementation of the high‑level
autonomy required for the Moebius mini‑car challenge.  It follows the
technical specification provided by the competition organisers: the
car must complete three laps between a 3×3 m outer square and a
central 1×1 m inner square, determine its travel direction
automatically, hug the inner wall, and avoid coloured obstacles (red
blocks mean turn right, green blocks mean turn left).

The code is designed for a Raspberry Pi 5 running ROS 2 Humble or
Iron.  It integrates with an Intel RealSense D455 depth camera and an
LDROBOT LD500 2D lidar.  A separate STM32‑based controller handles
wheel and steering actuation; commands are published on
``/controller/cmd_vel``.

Overview
--------

The package consists of two Python nodes:

* **`obstacle_detector_node`** (``autonomy_project/obstacle_detector_node.py``)
  subscribes to synchronised colour and aligned depth images from the
  RealSense camera.  It thresholds the colour image in HSV space to
  locate red and green obstacles.  The centre of the largest contour
  is used to compute range and bearing; results are published on the
  custom message topic ``/obstacles``.  The RealSense ROS 2 wrapper
  publishes colour and depth images on standard topics such as
  ``/camera/color/image_raw`` and ``/camera/depth/image_aligned_to_color/image_raw``【196241295484956†L975-L993】.

* **`navigation_node`** (``autonomy_project/navigation_node.py``)
  implements a state machine to determine travel direction, perform
  wall following with a PID controller, count laps, and execute
  avoidance manoeuvres when signalled by the obstacle detector.  It
  subscribes to ``/scan_raw`` for lidar data (remap this topic if
  necessary) and uses odometry from ``/odom`` to track heading and
  measure lap completion.  The lidar driver for the LD500 exposes a
  LaserScan topic named ``/ldlidar_node/scan`` once activated
  【994921289684312†L392-L401】—use a remapping in your launch file to connect it to
  ``/scan_raw``.

Installation and Build
----------------------

1. **Prerequisites:** A working ROS 2 Humble or Iron installation.
   Install additional dependencies with:

   ```bash
   sudo apt install ros-<distro>-cv-bridge
   pip install opencv-python
   ```

2. **Set up your workspace:**

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # copy or clone this autonomy_project directory here
   cp -r /path/to/autonomy_project_improved/autonomy_project .
   cd ..
   colcon build --packages-select autonomy_project
   source install/setup.bash
   ```

3. **Sensor drivers:**
   * **RealSense D455:** install the official ROS 2 driver from
     [`realsense-ros`](https://github.com/IntelRealSense/realsense-ros).
     When running ``rs_launch.py`` ensure that colour and depth streams
     are enabled.  The driver will publish topics such as
     ``/camera/color/image_raw`` and ``/camera/depth/image_aligned_to_color/image_raw``【196241295484956†L975-L993】 which are used by the obstacle detector.
   * **LD500 lidar:** use the ROS 2 lifecycle driver from
     [`ldrobot-lidar-ros2`](https://github.com/Myzhar/ldrobot-lidar-ros2).  Once
     activated, it exposes ``/ldlidar_node/scan``【994921289684312†L392-L401】.  You can
     remap this to ``/scan_raw`` when launching ``navigation_node``.

Running the System
------------------

1. **Start sensors and low‑level bringup:** Use your existing
   bringup scripts or the drivers mentioned above to start the
   RealSense camera, lidar and chassis controller.  For example:

   ```bash
   # RealSense
   ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true align_depth.enable:=true

   # Lidar (activate lifecycle and publish scans)
   ros2 launch ldlidar_node ldlidar_with_mgr.launch.py

   # Your robot chassis bringup
   ros2 launch bringup bringup.launch.py
   ```

2. **Launch the autonomy nodes:** In a separate terminal, source
   your workspace and run the challenge launch file:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch autonomy_project challenge.launch.py
   ```

3. **Tuning parameters:** The following constants are meant to be
   adjusted before competition:

   * **Colour thresholds** in ``obstacle_detector_node.py``.
   * **PID gains**, **desired wall distance** and **obstacle distance
     threshold** in ``navigation_node.py``.
   * **Time durations** for avoidance manoeuvres in
     ``navigation_node.py``.

   Use RViz2 to visualise lidar scans and camera images while tuning.

Notes and Troubleshooting
-------------------------

* If the robot turns the wrong way at startup, verify that the
  direction determination logic is correct for your lidar mounting.
  The code looks at the minimum distance in the left vs right sectors
  of the scan to infer which side the inner wall is on.
* Remap topics as needed.  For example, if your lidar publishes
  ``/scan`` instead of ``/scan_raw``, update the ``remappings``
  argument in ``challenge.launch.py`` accordingly.
* Ensure that the odometry frame is consistent and that the yaw
  extracted from ``/odom`` corresponds to heading in the arena plane.
* Always verify obstacle colours with the actual blocks and lighting.

Licence and Support
-------------------

This code is provided as a reference implementation without
warranty.  Feel free to modify it to suit your platform.  If you
encounter issues or have suggestions for improvement, please open an
issue or submit a pull request.