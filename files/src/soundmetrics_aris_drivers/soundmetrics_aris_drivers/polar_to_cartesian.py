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

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import message_filters
from sensor_msgs.msg import Image
from soundmetrics_aris_interfaces.msg import SonarInfo
from cv_bridge import CvBridge, CvBridgeError


class PolarToCartesianConverter(Node):
    """Converts polar sonar images to fan-shaped cartesian representation."""

    def __init__(self):
        super().__init__('polar_to_cartesian')
        self.name = self.get_name()

        # CvBridge for image conversion
        self.bridge = CvBridge()

        # Mapping cache
        self.cached_params = None
        self.range_bin_map = None
        self.beam_index_map = None
        self.valid_mask = None
        self.cart_height = 0
        self.cart_width = 0
        self.pixels_per_meter = 0
        self.rmin = 0
        self.rmax = 0
        self.half_fov_deg = 0
        self.cached_grid_overlay = None
        self.cached_grid_mask = None

        # Subscribers using message_filters for time synchronization
        polar_sub = message_filters.Subscriber(
            self,
            Image,
            'image/polar/raw'
        )
        info_sub = message_filters.Subscriber(
            self,
            SonarInfo,
            'sonar_info'
        )

        ts = message_filters.ApproximateTimeSynchronizer(
            [polar_sub, info_sub],
            queue_size=10,
            slop=0.4
        )
        ts.registerCallback(self.sync_callback)

        # Keep references to prevent garbage collection
        self._polar_sub = polar_sub
        self._info_sub = info_sub
        self._ts = ts

        # Publisher
        self.cartesian_pub = self.create_publisher(
            Image,
            'image/cartesian_fan/raw',
            1
        )

        self.get_logger().info('%s: Polar-to-Cartesian converter initialized' % self.name)

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
        
        # Note: cart_height spans from range=0 (sonar origin) to rmax, but sonar
        # data only exists from rmin to rmax. The bottom portion (0 to rmin) is
        # black. This preserves correct fan-arc geometry with the origin at the
        # bottom center of the image.

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

        # Store cached mapping and geometry
        self.range_bin_map = range_bin
        self.beam_index_map = beam_index
        self.valid_mask = valid
        self.cart_height = cart_height
        self.cart_width = cart_width
        self.pixels_per_meter = pixels_per_meter
        self.rmin = rmin
        self.rmax = rmax
        self.half_fov_deg = half_fov_deg

        self._prerender_grid()

        self.get_logger().info('%s: Mapping computed - cartesian image size: %dx%d' %
                      (self.name, cart_width, cart_height))

    def apply_mapping(self, polar_image):
        """Apply the cached mapping to convert a polar image to cartesian."""
        cartesian_image = np.zeros((self.cart_height, self.cart_width), dtype=np.uint8)
        cartesian_image[self.valid_mask] = polar_image[
            self.range_bin_map[self.valid_mask],
            self.beam_index_map[self.valid_mask]
        ]
        return cartesian_image

    def _prerender_grid(self, padding=40):
        """Pre-render the grid overlay onto a blank image. Called once per parameter change."""
        h = self.cart_height + 2 * padding
        w = self.cart_width + 2 * padding
        overlay = np.zeros((h, w, 3), dtype=np.uint8)

        origin_x = padding + self.cart_width // 2
        origin_y = padding + self.cart_height

        grid_color = (0, 200, 0)
        text_color = (0, 255, 0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = max(0.3, self.cart_height / 1500.0)

        # --- Range rings ---
        range_extent = self.rmax - self.rmin
        if range_extent <= 2.0:
            ring_step = 0.25
        elif range_extent <= 5.0:
            ring_step = 0.5
        else:
            ring_step = 1.0

        r = math.ceil(self.rmin / ring_step) * ring_step
        while r <= self.rmax:
            radius_px = int(r * self.pixels_per_meter)
            if radius_px > 0:
                start_angle = 270 - self.half_fov_deg
                end_angle = 270 + self.half_fov_deg
                cv2.ellipse(overlay, (origin_x, origin_y), (radius_px, radius_px),
                            0, start_angle, end_angle, grid_color, 1)
                label_angle_rad = math.radians(self.half_fov_deg)
                label_x = int(origin_x + r * math.sin(label_angle_rad) * self.pixels_per_meter)
                label_y = int(origin_y - r * math.cos(label_angle_rad) * self.pixels_per_meter)
                cv2.putText(overlay, '%.1fm' % r, (label_x + 3, label_y),
                            font, font_scale, text_color, 1, cv2.LINE_AA)
            r += ring_step

        # --- Angle lines ---
        angle_step = 5.0 if self.half_fov_deg <= 15 else 10.0
        angle = -self.half_fov_deg
        while angle <= self.half_fov_deg:
            angle_rad = math.radians(angle)
            x1 = int(origin_x + self.rmin * math.sin(angle_rad) * self.pixels_per_meter)
            y1 = int(origin_y - self.rmin * math.cos(angle_rad) * self.pixels_per_meter)
            x2 = int(origin_x + self.rmax * math.sin(angle_rad) * self.pixels_per_meter)
            y2 = int(origin_y - self.rmax * math.cos(angle_rad) * self.pixels_per_meter)
            cv2.line(overlay, (x1, y1), (x2, y2), grid_color, 1)
            if angle != 0:
                cv2.putText(overlay, '%ddeg' % int(angle), (x2 + 3, y2 - 3),
                            font, font_scale, text_color, 1, cv2.LINE_AA)
            angle += angle_step

        self.cached_grid_overlay = overlay
        self.cached_grid_mask = overlay.any(axis=2)

    def draw_grid_overlay(self, image, padding=40):
        """Stamp the pre-rendered grid onto the image. No drawing happens here."""
        bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        bgr = cv2.copyMakeBorder(bgr, padding, padding, padding, padding,
                                 cv2.BORDER_CONSTANT, value=(0, 0, 0))
        bgr[self.cached_grid_mask] = self.cached_grid_overlay[self.cached_grid_mask]
        return bgr

    def sync_callback(self, polar_msg, sonar_info):
        """Handle synchronized polar image and sonar info messages."""
        try:
            polar_cv = self.bridge.imgmsg_to_cv2(polar_msg, desired_encoding='mono8')

            # Recompute mapping if sonar parameters changed
            if self.needs_remapping(sonar_info):
                self.get_logger().info('%s: Sonar parameters changed, recomputing mapping' % self.name)
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

            # Draw grid overlay (converts to BGR)
            cartesian_bgr = self.draw_grid_overlay(cartesian_cv)

            # Publish with same header (timestamp + frame_id)
            cart_msg = self.bridge.cv2_to_imgmsg(cartesian_bgr, encoding='bgr8')
            cart_msg.header = polar_msg.header
            self.cartesian_pub.publish(cart_msg)

        except CvBridgeError as e:
            self.get_logger().warn('%s: CvBridge error: %s' % (self.name, e))
        except Exception as e:
            self.get_logger().error('%s: Error in conversion: %s' % (self.name, e))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PolarToCartesianConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
