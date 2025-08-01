"""
Obstacle detection node for the Moebius challenge.

This module contains a single ROS 2 node that subscribes to synchronized
color and depth images from an Intel RealSense camera, identifies
colored obstacles, estimates their range and angular position relative
to the camera, and publishes the result on a custom ``Obstacle``
message.  The colour thresholds are deliberately exposed as module
constants so they can be tuned for the lighting conditions in the
competition arena.

The node listens on the RealSense colour and depth topics (provided by
the upstream ``realsense2_camera`` package【196241295484956†L975-L993】 and uses
``message_filters.ApproximateTimeSynchronizer`` to ensure that the
colour and depth frames correspond to one another.  Contours are
extracted from HSV thresholded images to detect red and green blocks.
The centre of the largest contour is used to compute the angle of the
obstacle based on the horizontal field‑of‑view of the D455 sensor.
Distances are read directly from the aligned depth image.

Tuning:  The HSV thresholds below are placeholders.  You must
calibrate them in the competition environment using a colour picker or
similar tool; lighting variations have a large impact on what values
work best.  See ``README.md`` for more details.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from autonomy_project.msg import Obstacle

# ----------------------- Colour thresholds -------------------------------
# These ranges define the HSV thresholds for red and green obstacles.
# They will need to be tuned for the specific lighting conditions at
# competition time.  Red is split into two ranges to cover the wrap
# around of the hue axis (0–10 degrees and 170–180 degrees).
RED_LOWER1 = np.array([0, 150, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 150, 100])
RED_UPPER2 = np.array([180, 255, 255])
GREEN_LOWER = np.array([40, 100, 80])
GREEN_UPPER = np.array([80, 255, 255])


class ObstacleDetectorNode(Node):
    """Detect red and green obstacles from RealSense images.

    The node publishes ``autonomy_project/Obstacle`` messages on the
    ``/obstacles`` topic whenever a sufficiently large red or green
    contour is detected in the image.  Each message contains the
    colour (``"red"`` or ``"green"``), range (metres) and bearing
    (radians).  Bearing is calculated from the horizontal pixel
    coordinate assuming a D455 field of view of 69.4 degrees.
    """

    def __init__(self) -> None:
        super().__init__('obstacle_detector_node')
        self.bridge = CvBridge()
        # Publisher for obstacle messages
        self.obstacle_pub = self.create_publisher(Obstacle, '/obstacles', 10)
        # Subscribers for synchronized colour and depth images
        # The topic names correspond to the defaults provided by the
        # ``realsense2_camera`` driver【196241295484956†L975-L993】.
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_aligned_to_color/image_raw')
        # Use an approximate time synchroniser with 0.1 s slop
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)
        self.get_logger().info('ObstacleDetectorNode started')

    def image_callback(self, img_msg: Image, depth_msg: Image) -> None:
        """Process synchronised colour and depth images to find obstacles."""
        # Convert ROS messages to OpenCV images
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        # Convert colour image to HSV for colour thresholding
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape
        # Generate masks for red and green
        mask_r1 = cv2.inRange(hsv_image, RED_LOWER1, RED_UPPER1)
        mask_r2 = cv2.inRange(hsv_image, RED_LOWER2, RED_UPPER2)
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        green_mask = cv2.inRange(hsv_image, GREEN_LOWER, GREEN_UPPER)
        # Detect and publish obstacles for each colour
        self.find_and_publish_obstacle(red_mask, "red", depth_image, w)
        self.find_and_publish_obstacle(green_mask, "green", depth_image, w)

    def find_and_publish_obstacle(self, mask: np.ndarray, colour: str, depth_image: np.ndarray, width: int) -> None:
        """Locate the largest contour in the mask and publish its position.

        Args:
            mask: binary image where the desired colour pixels are white.
            colour: name of the colour being processed (``"red"`` or
                ``"green"``).
            depth_image: 32‑bit float depth image aligned to the colour
                image (metres).  Depth values greater than zero and not
                NaN/Inf are considered valid.
            width: width of the colour image in pixels.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        # Choose the largest contour by area
        contour = max(contours, key=cv2.contourArea)
        # Filter out small contours (noise) by area threshold
        if cv2.contourArea(contour) < 1000:
            return
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        # Convert depth from millimetres to metres
        distance = depth_image[cy, cx] / 1000.0
        if np.isnan(distance) or np.isinf(distance) or distance <= 0.2:
            return
        # Compute angle relative to camera centre.  The D455 has a
        # horizontal field of view of approximately 69.4 degrees【196241295484956†L975-L993】.
        fov_h_rad = np.deg2rad(69.4)
        angle = (float(cx) / width - 0.5) * fov_h_rad
        # Populate and publish obstacle message
        msg = Obstacle()
        msg.color = colour
        msg.distance = float(distance)
        msg.angle = angle
        self.obstacle_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for the obstacle detector node."""
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()