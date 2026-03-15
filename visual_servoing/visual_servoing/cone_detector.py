#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from visual_servoing.computer_vision.color_segmentation import cd_color_segmentation

# Calibration points — must stay in sync with homography_transformer.py
PTS_IMAGE_PLANE = [[403, 286],
                   [203, 280],
                   [378, 249],
                   [569, 279],
                   [366, 195]]

PTS_GROUND_PLANE = [[10,  0],
                    [10,  10],
                    [15,  0],
                    [10, -10],
                    [35,  0]]

METERS_PER_INCH = 0.0254

# Line follower parameters
LOOKAHEAD_DISTANCE  = 0.9   # meters ahead to place the "virtual cone"; must be > parking_distance
LOOKAHEAD_BAND_PX   = 20    # height of the pixel band scanned around the lookahead row

# HSV thresholds for orange (same as color_segmentation.py)
LOWER_ORANGE_1 = np.array([0,   150,  80])
UPPER_ORANGE_1 = np.array([23,  255, 255])
LOWER_ORANGE_2 = np.array([165,  50,  50])
UPPER_ORANGE_2 = np.array([185, 255, 255])


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """

    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = True

        # Subscribe to ZED camera RGB frames
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge()  # Converts between ROS images and OpenCV Images

        # Build the homography (image → ground) then invert it to find which
        # pixel row corresponds to LOOKAHEAD_DISTANCE straight ahead.
        np_pts_ground = np.array(PTS_GROUND_PLANE, dtype=np.float32) * METERS_PER_INCH
        np_pts_image  = np.array(PTS_IMAGE_PLANE,  dtype=np.float32)
        h, _ = cv2.findHomography(np_pts_image[:, np.newaxis, :],
                                   np_pts_ground[:, np.newaxis, :])
        h_inv = np.linalg.inv(h)

        # Project (LOOKAHEAD_DISTANCE, 0) in ground plane back to image
        ground_pt = np.array([[LOOKAHEAD_DISTANCE], [0.0], [1.0]])
        img_pt = h_inv @ ground_pt
        img_pt /= img_pt[2]
        self.lookahead_row = int(img_pt[1, 0])

        self.get_logger().info(
            f"Cone Detector Initialized | mode={'line_follower' if self.LineFollower else 'cone_parker'} "
            f"| lookahead_row={self.lookahead_row} ({LOOKAHEAD_DISTANCE} m)"
        )

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        if self.LineFollower:
            cone_msg = self._line_follower_pixel(image)
        else:
            cone_msg = self._cone_parker_pixel(image)

        if cone_msg is not None:
            self.cone_pub.publish(cone_msg)

            # Draw debug visualization
            u, v = int(cone_msg.u), int(cone_msg.v)
            cv2.circle(image, (u, v), 5, (0, 0, 255), -1)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

    def _cone_parker_pixel(self, image):
        """Original mode: bottom-center of the cone bounding box."""
        bbox = cd_color_segmentation(image)
        ((x1, y1), (x2, y2)) = bbox
        if bbox == ((0, 0), (0, 0)):
            return None
        msg = ConeLocationPixel()
        msg.u = float((x1 + x2) / 2)
        msg.v = float(y2)  # bottom-center pixel (on the ground plane)
        return msg

    def _line_follower_pixel(self, image):
        """
        Line follower mode: find the orange line's column centroid at a fixed
        pixel row that corresponds to LOOKAHEAD_DISTANCE ahead of the car.
        The parking controller will steer toward this point, chasing it like a
        carrot since parking_distance < LOOKAHEAD_DISTANCE.
        """
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(img_hsv, LOWER_ORANGE_1, UPPER_ORANGE_1) |
                cv2.inRange(img_hsv, LOWER_ORANGE_2, UPPER_ORANGE_2))

        # Crop to a horizontal band around the lookahead row
        img_h = image.shape[0]
        r0 = max(0, self.lookahead_row - LOOKAHEAD_BAND_PX // 2)
        r1 = min(img_h, self.lookahead_row + LOOKAHEAD_BAND_PX // 2)
        band = mask[r0:r1, :]

        orange_cols = np.where(band > 0)[1]
        if len(orange_cols) == 0:
            return None  # line not visible at lookahead distance; don't publish

        msg = ConeLocationPixel()
        msg.u = float(np.mean(orange_cols))   # lateral centroid of the line
        msg.v = float(self.lookahead_row)      # fixed row = fixed forward distance
        return msg


def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
