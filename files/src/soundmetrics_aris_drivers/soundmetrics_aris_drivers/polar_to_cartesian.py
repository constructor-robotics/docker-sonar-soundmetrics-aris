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
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
import math
import time
import message_filters
from sensor_msgs.msg import Image, CompressedImage
from soundmetrics_aris_interfaces.msg import SonarInfo
from cv_bridge import CvBridge, CvBridgeError


class PolarToCartesianConverter(Node):
    """Converts polar sonar images to fan-shaped cartesian representation."""

    def __init__(self):
        super().__init__('polar_to_cartesian')
        self.name = self.get_name()

        # CvBridge for image conversion
        self.bridge = CvBridge()

        # Scaling parameters
        self.declare_parameter('enable_scaling', False)
        self.declare_parameter('scale_factor', 0.5)
        self.enable_scaling = self.get_parameter('enable_scaling').value
        self.scale_factor = self.get_parameter('scale_factor').value

        self.declare_parameter('compressed_format', 'png')
        self.declare_parameter('compressed_quality', 80)
        self.compressed_format = self.get_parameter('compressed_format').value
        self.compressed_quality = self.get_parameter('compressed_quality').value

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

        # Diagnostic counters
        self._polar_recv = 0
        self._info_recv = 0
        self._sync_recv = 0

        # Subscribers using message_filters for time synchronization.
        # qos_profile MUST match the publishers in soundmetrics_aris3000.py
        # (sensor_data: BEST_EFFORT). Without this, ROS rejects the connection
        # as QoS-incompatible and no frames flow through.
        polar_sub = message_filters.Subscriber(
            self,
            Image,
            'image/polar/raw',
            qos_profile=qos_profile_sensor_data
        )
        info_sub = message_filters.Subscriber(
            self,
            SonarInfo,
            'sonar_info',
            qos_profile=qos_profile_sensor_data
        )
        polar_sub.registerCallback(lambda msg: self._count('polar'))
        info_sub.registerCallback(lambda msg: self._count('info'))

        ts = message_filters.ApproximateTimeSynchronizer(
            [polar_sub, info_sub],
            queue_size=10,
            slop=0.4
        )
        ts.registerCallback(self.sync_callback)

        self.create_timer(5.0, self._log_counts)

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
        self.cartesian_compressed_pub = self.create_publisher(
            CompressedImage,
            'image/cartesian_fan/raw/compressed',
            1
        )

        self.get_logger().info('%s: Polar-to-Cartesian converter initialized' % self.name)

    def needs_remapping(self, cache_key):
        """Return True (and update the cache) if this geometry/size key is new."""
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
        """Pre-render the grid overlay onto a blank mono8 image. Called once per parameter change."""
        h = self.cart_height + 2 * padding
        w = self.cart_width + 2 * padding
        overlay = np.zeros((h, w), dtype=np.uint8)

        origin_x = padding + self.cart_width // 2
        origin_y = padding + self.cart_height

        color = 255
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
                            0, start_angle, end_angle, color, 1)
                label_angle_rad = math.radians(self.half_fov_deg)
                label_x = int(origin_x + r * math.sin(label_angle_rad) * self.pixels_per_meter)
                label_y = int(origin_y - r * math.cos(label_angle_rad) * self.pixels_per_meter)
                cv2.putText(overlay, '%.1fm' % r, (label_x + 3, label_y),
                            font, font_scale, color, 1, cv2.LINE_AA)
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
            cv2.line(overlay, (x1, y1), (x2, y2), color, 1)
            if angle != 0:
                cv2.putText(overlay, '%ddeg' % int(angle), (x2 + 3, y2 - 3),
                            font, font_scale, color, 1, cv2.LINE_AA)
            angle += angle_step

        self.cached_grid_overlay = overlay
        self.cached_grid_mask = overlay > 0

    def draw_grid_overlay(self, image, padding=40):
        """Stamp the pre-rendered grid onto the image. No drawing happens here."""
        mono = cv2.copyMakeBorder(image, padding, padding, padding, padding,
                                  cv2.BORDER_CONSTANT, value=0)
        mono[self.cached_grid_mask] = self.cached_grid_overlay[self.cached_grid_mask]
        return mono

    def _count(self, topic):
        if topic == 'polar':
            self._polar_recv += 1
        elif topic == 'info':
            self._info_recv += 1

    def _log_counts(self):
        self.get_logger().debug(
            '%s: [last 5s] polar_recv=%d  info_recv=%d  sync_fired=%d' % (
            self.name, self._polar_recv, self._info_recv, self._sync_recv))
        self._polar_recv = 0
        self._info_recv = 0
        self._sync_recv = 0

    def sync_callback(self, polar_msg, sonar_info):
        """Handle synchronized polar image and sonar info messages."""
        self._sync_recv += 1
        try:
            t0 = time.perf_counter()
            polar_cv = self.bridge.imgmsg_to_cv2(polar_msg, desired_encoding='mono8')
            t1 = time.perf_counter()

            # The polar IMAGE is the source of truth for sample/beam counts. Sizing the
            # map from sonar_info instead (firmware header) let the map be built for a
            # different shape than the image we actually received — e.g. during a sonar
            # re-sync or a transient/garbage frame header. That produced either frozen
            # frames (out-of-range index -> exception -> nothing published, with the bad
            # key cached so it stayed frozen) or wrong-geometry noise. So: take the SIZE
            # from the image, and only the fan GEOMETRY (range window + FOV) from
            # sonar_info — after validating it.
            if polar_cv.ndim != 2 or polar_cv.shape[0] < 2 or polar_cv.shape[1] < 2:
                self.get_logger().warn('%s: skipping frame with bad polar shape %s'
                                       % (self.name, str(polar_cv.shape)))
                return
            samples_per_beam, beams = int(polar_cv.shape[0]), int(polar_cv.shape[1])

            rmin = float(sonar_info.window_start)
            rmax = float(sonar_info.window_start) + float(sonar_info.window_length)
            half_fov = float(sonar_info.half_field_of_view)
            if not (rmax > rmin) or half_fov <= 0.0:
                self.get_logger().warn(
                    '%s: skipping frame with degenerate geometry '
                    '(window_start=%.3f window_length=%.3f half_fov=%.3f)'
                    % (self.name, sonar_info.window_start, sonar_info.window_length, half_fov))
                return

            # Recompute mapping only when geometry or image size actually changes.
            cache_key = (rmin, rmax, samples_per_beam, beams, half_fov)
            if self.needs_remapping(cache_key):
                self.get_logger().info('%s: Sonar geometry/size changed, recomputing mapping' % self.name)
                self.compute_mapping(rmin, rmax, samples_per_beam, beams, half_fov)

            # Apply mapping (map is guaranteed to match polar_cv's shape now)
            cartesian_cv = self.apply_mapping(polar_cv)
            t2 = time.perf_counter()

            # Draw grid overlay
            cartesian_mono = self.draw_grid_overlay(cartesian_cv)
            t3 = time.perf_counter()

            # Apply scaling if enabled
            if self.enable_scaling:
                new_w = max(1, int(cartesian_mono.shape[1] * self.scale_factor))
                new_h = max(1, int(cartesian_mono.shape[0] * self.scale_factor))
                cartesian_mono = cv2.resize(cartesian_mono, (new_w, new_h),
                                            interpolation=cv2.INTER_LINEAR)

            # Publish with same header (timestamp + frame_id)
            cart_msg = self.bridge.cv2_to_imgmsg(cartesian_mono, encoding='mono8')
            cart_msg.header = polar_msg.header
            self.cartesian_pub.publish(cart_msg)
            encode_ext = '.jpg' if self.compressed_format == 'jpeg' else '.png'
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.compressed_quality] if self.compressed_format == 'jpeg' else []
            _, buf = cv2.imencode(encode_ext, cartesian_mono, encode_params)
            comp_msg = CompressedImage()
            comp_msg.header = polar_msg.header
            comp_msg.format = self.compressed_format
            comp_msg.data = buf.tobytes()
            self.cartesian_compressed_pub.publish(comp_msg)
            t4 = time.perf_counter()

            dt = abs((polar_msg.header.stamp.sec - sonar_info.header.stamp.sec) +
                     (polar_msg.header.stamp.nanosec - sonar_info.header.stamp.nanosec) * 1e-9)
            self.get_logger().debug(
                '%s: imgmsg_to_cv2=%.1fms  apply_mapping=%.1fms  draw_grid=%.1fms  publish=%.1fms  total=%.1fms  stamp_delta=%.1fms' % (
                self.name, (t1-t0)*1000, (t2-t1)*1000, (t3-t2)*1000, (t4-t3)*1000, (t4-t0)*1000, dt*1000))

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
