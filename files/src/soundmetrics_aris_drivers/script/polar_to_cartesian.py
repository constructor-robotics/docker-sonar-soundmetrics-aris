#!/usr/bin/python3
"""
MIT License

Copyright (c) 2025 Robotics Group - Constructor University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rospy
import numpy as np
import math
import message_filters
from sensor_msgs.msg import Image
from soundmetrics_aris_drivers.msg import SonarInfo
from cv_bridge import CvBridge, CvBridgeError


class PolarToCartesianConverter(object):
    """Converts polar sonar images to fan-shaped cartesian representation."""

    def __init__(self, name):
        self.name = name

        # CvBridge for image conversion
        self.bridge = CvBridge()

        # Topic prefix from parameter
        self.topic_prefix = rospy.get_param('~topic_prefix', '/soundmetrics_aris3000/')

        # Mapping cache
        self.cached_params = None
        self.range_bin_map = None
        self.beam_index_map = None
        self.valid_mask = None
        self.cart_height = 0
        self.cart_width = 0

        # Subscribers using message_filters for time synchronization
        polar_sub = message_filters.Subscriber(
            self.topic_prefix + 'image/polar_fan/raw',
            Image
        )
        info_sub = message_filters.Subscriber(
            self.topic_prefix + 'sonar_info',
            SonarInfo
        )

        ts = message_filters.ApproximateTimeSynchronizer(
            [polar_sub, info_sub],
            queue_size=10,
            slop=0.05
        )
        ts.registerCallback(self.sync_callback)

        # Publisher
        self.cartesian_pub = rospy.Publisher(
            self.topic_prefix + 'image/cartesian_fan/raw',
            Image,
            queue_size=2
        )

        rospy.loginfo('%s: Polar-to-Cartesian converter initialized', self.name)
        rospy.loginfo('%s: Subscribing to %s and %s', self.name,
                      self.topic_prefix + 'image/polar_fan/raw',
                      self.topic_prefix + 'sonar_info')
        rospy.loginfo('%s: Publishing to %s', self.name,
                      self.topic_prefix + 'image/cartesian_fan/raw')

    def needs_remapping(self, sonar_info):
        """Check if the mapping needs to be recomputed."""
        cache_key = (
            sonar_info.window_start,
            sonar_info.window_length,
            sonar_info.samples_per_beam,
            sonar_info.beams,
            sonar_info.half_field_of_view
        )
        if cache_key != self.cached_params:
            self.cached_params = cache_key
            return True
        return False

    def compute_mapping(self, rmin, rmax, samples_per_beam, beams, half_fov_deg):
        """Compute the cartesian-to-polar index mapping.

        For each pixel (x, y) in the output cartesian image, compute which
        (range_bin, beam_index) in the polar image it corresponds to.
        """
        half_fov_rad = math.radians(half_fov_deg)

        # Resolution: match the polar image's range resolution
        pixels_per_meter = samples_per_beam / (rmax - rmin)

        # Output image dimensions
        cart_width = int(2 * rmax * math.sin(half_fov_rad) * pixels_per_meter)
        cart_height = int(rmax * pixels_per_meter)

        # Force odd for symmetry
        if cart_width % 2 == 0:
            cart_width += 1
        if cart_height % 2 == 0:
            cart_height += 1

        # Build coordinate grids in meters
        # x: centered, spanning the full angular width at rmax
        x_pixels = np.arange(cart_width)
        x_meters = (x_pixels - cart_width / 2.0 + 0.5) / pixels_per_meter

        # y: top of image = rmax (far range), bottom = 0 (sonar position)
        y_pixels = np.arange(cart_height)
        y_meters = rmax - y_pixels / pixels_per_meter

        X, Y = np.meshgrid(x_meters, y_meters)

        # Convert to polar coordinates
        R = np.sqrt(X ** 2 + Y ** 2)
        Theta_deg = np.degrees(np.arctan2(X, Y))  # angle from forward (Y) axis

        # Map to polar image indices (nearest-neighbor)
        range_bin = np.round((R - rmin) / (rmax - rmin) * (samples_per_beam - 1)).astype(np.int32)
        beam_index = np.round((Theta_deg + half_fov_deg) / (2 * half_fov_deg) * (beams - 1)).astype(np.int32)

        # Validity mask: within sonar field of view and range window
        valid = (R >= rmin) & (R <= rmax) & \
                (Theta_deg >= -half_fov_deg) & (Theta_deg <= half_fov_deg)

        # Clip indices to valid range (for safety, even though mask handles it)
        range_bin = np.clip(range_bin, 0, samples_per_beam - 1)
        beam_index = np.clip(beam_index, 0, beams - 1)

        # Store cached mapping
        self.range_bin_map = range_bin
        self.beam_index_map = beam_index
        self.valid_mask = valid
        self.cart_height = cart_height
        self.cart_width = cart_width

        rospy.loginfo('%s: Mapping computed - cartesian image size: %dx%d',
                      self.name, cart_width, cart_height)

    def apply_mapping(self, polar_image):
        """Apply the cached mapping to convert a polar image to cartesian."""
        cartesian_image = np.zeros((self.cart_height, self.cart_width), dtype=np.uint8)
        cartesian_image[self.valid_mask] = polar_image[
            self.range_bin_map[self.valid_mask],
            self.beam_index_map[self.valid_mask]
        ]
        return cartesian_image

    def sync_callback(self, polar_msg, sonar_info):
        """Handle synchronized polar image and sonar info messages."""
        try:
            polar_cv = self.bridge.imgmsg_to_cv2(polar_msg, desired_encoding='mono8')

            # Recompute mapping if sonar parameters changed
            if self.needs_remapping(sonar_info):
                rospy.loginfo('%s: Sonar parameters changed, recomputing mapping', self.name)
                rmin = sonar_info.window_start
                rmax = sonar_info.window_start + sonar_info.window_length
                self.compute_mapping(
                    rmin, rmax,
                    sonar_info.samples_per_beam,
                    sonar_info.beams,
                    sonar_info.half_field_of_view
                )

            # Apply mapping
            cartesian_cv = self.apply_mapping(polar_cv)

            # Publish with same header (timestamp + frame_id)
            cart_msg = self.bridge.cv2_to_imgmsg(cartesian_cv, encoding='mono8')
            cart_msg.header = polar_msg.header
            self.cartesian_pub.publish(cart_msg)

        except CvBridgeError as e:
            rospy.logwarn('%s: CvBridge error: %s', self.name, e)
        except Exception as e:
            rospy.logerr('%s: Error in conversion: %s', self.name, e)


if __name__ == '__main__':
    try:
        rospy.init_node('polar_to_cartesian')
        converter = PolarToCartesianConverter(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
