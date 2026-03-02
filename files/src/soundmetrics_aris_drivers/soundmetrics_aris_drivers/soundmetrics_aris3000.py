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
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

from soundmetrics_aris_interfaces.msg import SonarInfo
from soundmetrics_aris_interfaces.srv import SetSonarParams

import cv2
import socket
import struct
import threading
import time
import numpy as np
import subprocess
import sys

# COMMANDS DEFINITION
PING = 1
P2_SONAR_AVAILABILITY_BROADCAST = 97
P2_SONAR_NOT_AVAILABILITY_BROADCAST = 98
P2_SET_FOCUS = 12
P2_SET_DATE_TIME = 14
P2_SET_FREQUENCY = 23
P2_SET_TRANSMIT_ENABLE = 24
P2_V150_ENABLE = 25
P2_SET_PULSE_WIDTH = 26
P2_SET_SONAR_PARAMS = 34
P2_SET_RECEIVER_GAIN = 11
P2_HOME_FOCUS_MOTOR = 33
P2_RESET_ARIS = 31
P2_SET_TARGET_FRAME_PERIOD_USEC = 43
P2_SET_SALINITY = 42
P2_SET_LARGE_LENS = 44
P2_RESTART_ARIS_APP = 45
P2_REBOOT_SONAR = 48
P2_STOP_ALL_SOFTWARE = 49
P2_SET_X2_MOUNT = 41
P2_SET_X2_AXIS_POSITION = 35
P2_SET_X2_AXIS_VELOCITY = 36
P2_SET_X2_STOP = 37
P2_SET_X2_SET_ZERO = 39
P2_X2_RESET = 38

# ADDITIONAL DEFINITIONS
ARIS_NUMBER_A2D_CHANNELS = 16
HALF_FIELD_OF_VIEW = 14.4 #14.4

# PORT DEFINITIONS
TCP_COMMAND_PORT = 56888
UDP_DATA_PORT = 56444
UDP_AVAILABILITY_PORT = 56123
COMMAND_DEST_PORT = 56555

# PROTOCOL CONSTANTS
HEADER_SIZE_BYTES = 68
PAYLOAD_SIZE_BYTES = 1332
FRAME_HEADER_SIZE_BYTES = 1024
COMMAND_HEADER_MAGIC_1 = 2175520024
COMMAND_HEADER_MAGIC_2 = 2868936984
PROTOCOL_VERSION = 256

# TIMING CONSTANTS
PING_INTERVAL_SEC = 3.0
CYCLE_PERIOD_OVERHEAD_USEC = 360

# PARAMETER LIMITS
FRAME_PERIOD_SEC_MIN = 0.075                    # This means max frame rate is 13.33 Hz, in manual is 15 Hz
FRAME_PERIOD_SEC_MAX = 1.0
GAIN_MIN = 0
GAIN_MAX = 24                                   # Suggested initial value is 12
FOCUS_MIN = 0
FOCUS_MAX = 1000                                # Focus range in meters
SAMPLE_START_DELAY_MIN = 930                    #~0.7 meters
SAMPLE_START_DELAY_MAX = 60000                  #~45 meters
SAMPLE_PERIOD_MIN = 4
SAMPLE_PERIOD_MAX = 500                         #Documentation says 100, need to check
CYCLE_PERIOD_MIN = 1802
CYCLE_PERIOD_MAX = 60000                        #Documentation says 150000, need to check


class SonarSoundMetricsAris3000(Node):
    """ This class configures and receive images from Sound Metrics
        ARIS3000 forward looking sonar. """

    def __init__(self):
        """ Soundmetrics ARIS 3000 driver """
        super().__init__('soundmetrics_aris3000')
        self.name = self.get_name()
        self.local_network_interface_name = ""

        #debug images
        self.use_64_bit_os = True

        self.nt = 0
        self.need_sync = True
        self.lock = threading.RLock()

        # predefined parameters
        self.PREDEFINED_ALTITUDE = 1.2
        self.frame_period_sec = 1.0
        self.gain_binary = 1103101952
        self.gain = 24
        self.frequency = 1
        self.focus = 364
        self.pulse_width = 8
        self.ping_mode = 9
        self.samples_per_beam = 512
        self.window_start = 0.7
        self.window_length = 3.5
        self.ixsize = 350
        self.sound_velocity = 1500.0
        self.offset_fls_to_dvl = 0.25
        self.host_ip = "169.254.7.10"
        self.sonar_ip = "169.254.7.147"

        # Param to be init later
        self.beams = 0
        self.pings = 0
        self.bundle_size = 0
        self.iysize = 0
        self.pitch_vehicle = 0.0
        self.compass_pitch = 0.0

        self.altitude = 1.0

        # Load ROS PARAM SERVER parameters
        self.get_config()


        ### Check whether network interface is available
        if self.getNetworkInterface(self.local_network_interface_name) == -1:
            self.get_logger().fatal('Required local network interface %s not found!' % self.local_network_interface_name)
            sys.exit(1)

        # Use IPs from config
        self.local_IP = self.host_ip
        ip = self.local_IP.split('.')
        self.local_IP_dec = (int(ip[0]) << 24) + (int(ip[1]) << 16) + (int(ip[2]) << 8) + int(ip[3])
        self.get_logger().info('%s: Local ip: %s' % (self.name, self.local_IP))

        self.sender_IP_text = self.sonar_ip
        parts = self.sender_IP_text.split('.')
        self.sender_IP = (int(parts[0]) << 24) + (int(parts[1]) << 16) + (int(parts[2]) << 8) + int(parts[3])
        self.get_logger().info('%s: Sender ip: %s' % (self.name, self.sender_IP_text))

        # Create TCP socket at localhost:56888
        try:
            self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp.connect((self.sender_IP_text, TCP_COMMAND_PORT))
            self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except socket.error as e:
            self.get_logger().fatal('%s: Failed to create TCP connection to %s:%d - %s' % (self.name, self.sender_IP_text, TCP_COMMAND_PORT, e))
            raise

        # Send inital configuration
        self.send_config()

        # Initialize PING command -- Keep alive mechanism to let the sensor know the clients is still connected
        tp = threading.Thread(target = self.send_ping, args=[])
        tp.daemon = True
        tp.start()

        # Open data receive port
        self.udp_data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_data.bind(('', UDP_DATA_PORT))
        self.get_logger().info('%s: UDP DATA @ %d connected!' % (self.name, UDP_DATA_PORT))

        ### Create ROS Publishers and Services
        # Create publishers
        # The image below is the polar fan image, where x-axis is the range bins (samples per beam) and y-axis is beam angle (index)
        self.polar_pub = self.create_publisher(Image, 'image/polar/raw', 2)
        self.polar_compressed_pub = self.create_publisher(CompressedImage, 'image/polar/compressed', 2)
        self.sonar_info_pub = self.create_publisher(SonarInfo, 'sonar_info', 2)

        ## Create Service -- to be tested
        #self.load_configuration_srv = self.create_service(SetSonarParams, 'configuration', self.set_configuration)

        self.bridge = CvBridge()
        self.get_logger().info('%s: Finish creating ROS Publishers and Services' % self.name)


    @staticmethod
    def getNetworkInterface(local_network_interface_name):
        """ Get the network interface index for the given interface name """
        splits = subprocess.getoutput("/sbin/ifconfig").split("\n")
        found_line = -1
        for i_line in range(0, len(splits)):
            if local_network_interface_name in splits[i_line]:
                found_line = i_line
        return found_line

    @staticmethod
    def compute_checksum(header_fmt):
        """ Compute the checksum for an ARIS3000 command header """
        ret = struct.pack('<17I', *header_fmt)
        chk_header = struct.unpack('<68B', ret)
        checksum = 0
        for i in range(4, len(chk_header)):
            checksum = checksum + chk_header[i]

        header_fmt[0] = checksum

        cmd = struct.pack('<17I', *header_fmt)
        return cmd

    @staticmethod
    def get_beams_and_pings(mode):
        """ given a mode returns the mode, the number of beams and
            the number of pings. Default mode 9. """
        if mode == 1:
            return [1, 48, 3, True]
        elif mode == 3:
            return [3, 96, 6, True]
        elif mode == 6:
            return [6, 64, 4, True]
        elif mode == 9:
            return [9, 128, 8, True]
        else:
            return [9, 128, 8, False]

    def get_config(self):
        """ Read configurations from ROS PARAM SERVER """

        self.declare_parameter('use_64_bit_os', True)
        self.declare_parameter('local_network_interface_name', "")
        self.declare_parameter('frame_id', "aris3000")
        self.declare_parameter('frame_period_sec', 1.0)
        self.declare_parameter('gain', 24)
        self.declare_parameter('frequency', 1)
        self.declare_parameter('focus', 364)
        self.declare_parameter('pulse_width', 8)
        self.declare_parameter('ping_mode', 9)
        self.declare_parameter('samples_per_beam', 512)
        self.declare_parameter('window_start', 0.7)
        self.declare_parameter('window_length', 3.5)
        self.declare_parameter('cartesian_width', 350)
        self.declare_parameter('sound_velocity', 1500.0)
        self.declare_parameter('host_ip', "169.254.7.10")
        self.declare_parameter('sonar_ip', "169.254.7.147")

        self.use_64_bit_os = self.get_parameter('use_64_bit_os').value
        self.local_network_interface_name = self.get_parameter('local_network_interface_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.frame_period_sec = self.get_parameter('frame_period_sec').value
        self.gain = self.get_parameter('gain').value
        self.frequency = self.get_parameter('frequency').value
        self.focus = self.get_parameter('focus').value
        self.pulse_width = self.get_parameter('pulse_width').value
        self.ping_mode = self.get_parameter('ping_mode').value
        self.samples_per_beam = self.get_parameter('samples_per_beam').value
        self.window_start = self.get_parameter('window_start').value
        self.window_length = self.get_parameter('window_length').value
        self.ixsize = self.get_parameter('cartesian_width').value
        self.sound_velocity = self.get_parameter('sound_velocity').value
        self.host_ip = self.get_parameter('host_ip').value
        self.sonar_ip = self.get_parameter('sonar_ip').value

        self.declare_parameter('compressed_format', 'png')
        self.declare_parameter('compressed_quality', 80)
        self.compressed_format = self.get_parameter('compressed_format').value
        self.compressed_quality = self.get_parameter('compressed_quality').value

        [self.mode, self.beams, self.pings, success] = self.get_beams_and_pings(self.ping_mode)
        # Repack the float values as an unsigned 32-bit integer representing a binary value
        self.gain_binary = struct.unpack('I', struct.pack('f', self.gain))[0]

    def send_ping(self):
        """ Send a ping command to keep sensor connection alive """
        while rclpy.ok():
            # Send ping
            cmd = self.create_command(PING, [0, 0, 0, 0, 0, 0])
            self.send_command(cmd)
            time.sleep(PING_INTERVAL_SEC)


    def set_configuration(self, request, response):
        """ Service to change Sonar configuration """
        self.lock.acquire()
        self.frame_period_sec = request.frame_period_sec

        # Set receiver gain
        self.gain = max(GAIN_MIN, min(request.gain, GAIN_MAX))
        self.gain_binary = struct.unpack('I', struct.pack('f', request.gain))[0]

        if request.frequency_hi:
            self.frequency = 1
        else:
            self.frequency = 0
        self.focus = request.focus
        self.pulse_width = request.pulse_width
        self.window_start = request.window_start
        self.window_length = request.window_length
        self.samples_per_beam = request.samples_per_beam

        # Send new configuration
        self.send_config()
        self.lock.release()
        response.attempted = True
        return response


    def send_config(self):
        """ Send configuration to ARIS3000 """
        self.lock.acquire()
        self.need_sync = True
        self.get_logger().info('%s: Config sonar' % self.name)

        # Set sonar frame rate
        self.frame_period_sec = max(FRAME_PERIOD_SEC_MIN, min(self.frame_period_sec, FRAME_PERIOD_SEC_MAX))
        cmd = self.create_command(P2_SET_TARGET_FRAME_PERIOD_USEC,
                                 [int(self.frame_period_sec*1e6), 0, 0, 0, 0, 0])
        self.send_command(cmd)

        # Set sonar binary gain
        cmd = self.create_command(P2_SET_RECEIVER_GAIN,
                                 [self.gain_binary, 0, 0, 0, 0, 0])
        self.send_command(cmd)

        # Set sonar frequency
        cmd = self.create_command(P2_SET_FREQUENCY,
                                 [self.frequency, 0, 0, 0, 0, 0])
        self.send_command(cmd)

        # Set focus
        self.focus = max(FOCUS_MIN, min(self.focus, FOCUS_MAX))
        cmd = self.create_command(P2_SET_FOCUS,
                                 [self.focus, 0, 0, 0, 0, 0])
        self.send_command(cmd)

        # Set pulse width
        cmd = self.create_command(P2_SET_PULSE_WIDTH,
                                 [self.pulse_width, 0, 0, 0, 0, 0])
        self.send_command(cmd)

        # Set sonar frame rate
        # NOTE: Although this value was already set, it needs to be sent again to ensure the sonar is configured correctly (Reverse engineered)
        cmd = self.create_command(P2_SET_TARGET_FRAME_PERIOD_USEC,
                                 [self.frame_period_sec*1e6, 0, 0, 0, 0, 0])
        self.send_command(cmd)

        # Compute and set sonar parameters
        sample_start_delay = float(self.window_start * 2 / self.sound_velocity)*10**6
        sample_start_delay = max(SAMPLE_START_DELAY_MIN, min(sample_start_delay, SAMPLE_START_DELAY_MAX))

        sample_period = (float(self.window_length * 2) /
                        float(self.samples_per_beam * self.sound_velocity)*10**6)
        sample_period = max(SAMPLE_PERIOD_MIN, min(sample_period, SAMPLE_PERIOD_MAX))

        cycle_period = (sample_start_delay +
                        self.samples_per_beam * sample_period + CYCLE_PERIOD_OVERHEAD_USEC)
        cycle_period = max(CYCLE_PERIOD_MIN, min(cycle_period, CYCLE_PERIOD_MAX))

        self.get_logger().info('window_length: %f' % self.window_length)
        self.get_logger().info('samples_per_beam: %f' % self.samples_per_beam)
        self.get_logger().info('sound_velocity: %f' % self.sound_velocity)
        self.get_logger().info('sample period: %f' % sample_period)
        self.get_logger().info('cycle period: %f' % cycle_period)

        cmd = self.create_command(P2_SET_SONAR_PARAMS,
                                 [self.ping_mode, sample_start_delay,
                                  sample_period, cycle_period,
                                  self.samples_per_beam, 0])
        self.send_command(cmd)

        # Set extra parameters
        cmd = self.create_command(P2_SET_TRANSMIT_ENABLE, [1, 0, 0, 0, 0, 0])
        self.send_command(cmd)
        cmd = self.create_command(P2_V150_ENABLE, [1, 0, 0, 0, 0, 0])
        self.send_command(cmd)


        self.lock.release()


    def create_command(self, cmd, params=[0, 0, 0, 0, 0, 0]):
        """ Create an ARIS3000 command header """

        header_fmt = []
        [header_fmt.append(0) for i in range(17)]

        header_fmt[0] = 0                   # Checksum
        header_fmt[1] = COMMAND_HEADER_MAGIC_1
        header_fmt[2] = COMMAND_HEADER_MAGIC_2
        header_fmt[3] = PROTOCOL_VERSION
        header_fmt[4] = cmd                 # Command
        header_fmt[5] = 0                   # Body size
        header_fmt[6] = self.local_IP_dec   # Local IP
        header_fmt[7] = 0                   # port
        header_fmt[8] = self.sender_IP      # ARIS IP
        header_fmt[9] = COMMAND_DEST_PORT
        header_fmt[10] = self.nt            # transaction number
        header_fmt[11:] = [int(x) for x in params]

        for i, val in enumerate(header_fmt):
            if not isinstance(val, int):
                raise TypeError(f"header_fmt[{i}] = {val} is not an int")

        header = self.compute_checksum(header_fmt)
        self.nt = self.nt + 1

        return header


    def send_command(self, cmd):
        """ Send commad through TCP """
        for i in range(2):
            self.tcp.send(cmd)


    def read_sonar_image(self):
        """ Read ARIS3000 acoustic images """
        self.lock.acquire()

        if self.need_sync:
            self.get_logger().info('%s: Wait image sync' % self.name)
            self.udp_data.settimeout(5.0)
            # Sync with first bundle
            while self.need_sync:
                try:
                    header = self.udp_data.recv(68)
                except socket.timeout:
                    self.get_logger().warn('%s: No UDP data received on port %d within 5s, retrying...' % (self.name, UDP_DATA_PORT))
                    continue
                header_fmt = list(struct.unpack('< 17I', header))
                self.get_logger().debug('%s: Sync packet - body_size=%d, packet_num=%d, total_packets=%d' %
                               (self.name, header_fmt[5], header_fmt[11], header_fmt[12]))

                # check if there is data in the body and the number of
                # transaction is the last one of the frame
                if header_fmt[5] > 0 and header_fmt[12]-1 == header_fmt[11]:
                    self.bundle_size = header_fmt[12]
                    self.get_logger().info('%s: Synced - bundle_size=%d' % (self.name, self.bundle_size))
                    self.need_sync = False
                    self.udp_data.settimeout(None)
                    self.get_logger().info('%s: Reading data' % self.name)

        # Read sonar image
        # ARIS sample data is stored as "one unsigned byte per sample" with valid values 0-255
        img = []
        for i in range(self.bundle_size):
            data = self.udp_data.recv(HEADER_SIZE_BYTES + PAYLOAD_SIZE_BYTES)
            packet = struct.unpack('17I ' + str(len(data) - HEADER_SIZE_BYTES) + 'B', data)
            if i == 0:
                frame_header = list(packet)[17:1041]
                sonar_info = self.read_frame_header(struct.pack('<%dB' % FRAME_HEADER_SIZE_BYTES, *frame_header))
                img = img + list(packet)[1041:]
            else:
                img = img + list(packet)[17:]

        # Reorder image and the data comes not in order due to multiplexing of the sensor
        ordered_image = self.reorder_samples(img)

        try:
            cv_ordered_image_bgr = cv2.cvtColor(ordered_image,  cv2.COLOR_GRAY2BGR)
            polar_img_msg = self.bridge.cv2_to_imgmsg(ordered_image, "mono8")
            polar_img_msg.header.stamp = self.get_clock().now().to_msg()
            polar_img_msg.header.frame_id = self.frame_id
            self.polar_pub.publish(polar_img_msg)
            encode_ext = '.jpg' if self.compressed_format == 'jpeg' else '.png'
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.compressed_quality] if self.compressed_format == 'jpeg' else []
            _, buf = cv2.imencode(encode_ext, ordered_image, encode_params)
            comp_msg = CompressedImage()
            comp_msg.header = polar_img_msg.header
            comp_msg.format = self.compressed_format
            comp_msg.data = buf.tobytes()
            self.polar_compressed_pub.publish(comp_msg)
        except CvBridgeError as e:
            self.get_logger().warn('CvBridgeError: %s' % e)

        sonar_info.header.stamp = polar_img_msg.header.stamp
        self.sonar_info_pub.publish(sonar_info)

        self.lock.release()


    def reorder_samples(self, frame_data):
        """ Put every sample in is correct order """
        outbuf = np.zeros(self.samples_per_beam * self.beams, dtype=np.uint8)
        beams_per_ping = int(self.beams / self.pings)

        bits16_channel_reverse_map = [10, 2, 14, 6, 8, 0, 12, 4, 11, 3,
                                      15, 7, 9, 1, 13, 5 ]
        channel_reverse_multipled_map = []

        for i in range(beams_per_ping):
            channel_reverse_multipled_map.insert(
                i, bits16_channel_reverse_map[i] * self.pings)

        frame_data_index = 0

        for ping_index in range(self.pings):
            for sample_index in range(self.samples_per_beam):
                composed = sample_index * self.beams + ping_index
                for channel in range(0, beams_per_ping, 4):
                    out_index_1 = channel_reverse_multipled_map[channel] + composed
                    out_index_2 = channel_reverse_multipled_map[channel + 1] + composed
                    out_index_3 = channel_reverse_multipled_map[channel + 2] + composed
                    out_index_4 = channel_reverse_multipled_map[channel + 3] + composed

                    outbuf[out_index_1] = frame_data[frame_data_index]
                    outbuf[out_index_2] = frame_data[frame_data_index + 1]
                    outbuf[out_index_3] = frame_data[frame_data_index + 2]
                    outbuf[out_index_4] = frame_data[frame_data_index + 3]
                    frame_data_index = frame_data_index + 4

        reordered_mat = outbuf.reshape(self.samples_per_beam,
                                       self.beams,
                                       order='C').copy()


        return np.asarray(np.fliplr(reordered_mat), order='C')


    def read_frame_header(self, frameheader):
        """ Parses ARIS 3000 header into sonar_info msg """

        frameheader_bytes = frameheader ### Python 3
        # TODO: We have added 4 extra chars to work on 64bits machine!

        offset_64_bit_os = ''
        if self.use_64_bit_os:
            offset_64_bit_os = b'\x00\x00\x00\x00'

        frame_header_fields = struct.unpack('IQIIQIIIIIIffIiIIIIIIffffffffffffffffffffddfIfffIIIIfIfffffffffdfIIIfffIIIIIIIffffff16fffffIIIIIIffIIIIIIQIIIIIIf124I',
                                            offset_64_bit_os + frameheader_bytes)


        sonar_info = SonarInfo()
        sonar_info.header.stamp = self.get_clock().now().to_msg()
        sonar_info.header.frame_id = self.frame_id
        sonar_info.index = frame_header_fields[0]
        sonar_info.time = frame_header_fields[1]
        # sonar_info.version = frame_header_fields[2]
        sonar_info.window_start = frame_header_fields[11]
        sonar_info.window_length = frame_header_fields[12]
        sonar_info.receiver_gain = frame_header_fields[15]
        sonar_info.focus = frame_header_fields[19]
        sonar_info.compass_heading = frame_header_fields[38]
        sonar_info.compass_pitch = frame_header_fields[39]
        self.compass_pitch = sonar_info.compass_pitch
        sonar_info.compass_roll = frame_header_fields[40]
        sonar_info.sample_rate = frame_header_fields[100]
        sonar_info.accel_x = frame_header_fields[101]
        sonar_info.accel_y = frame_header_fields[102]
        sonar_info.accel_z = frame_header_fields[103]
        sonar_info.ping_mode = frame_header_fields[104]
        sonar_info.frequency_hi = (frame_header_fields[105] == 1)
        sonar_info.pulse_width = frame_header_fields[106]
        sonar_info.cycle_period = frame_header_fields[107]
        sonar_info.sample_period = frame_header_fields[108]
        sonar_info.transmit_enable = (frame_header_fields[109] == 1)
        sonar_info.frame_rate = frame_header_fields[110]
        sonar_info.sound_speed = frame_header_fields[111] # NOTE: Need to check if this varies in open-water, so far gives zero
        sonar_info.samples_per_beam = frame_header_fields[112]
        sonar_info.salinity = frame_header_fields[124]

        [res, sonar_info.beams, sonar_info.pings_per_frame, success] = self.get_beams_and_pings(sonar_info.ping_mode)
        sonar_info.half_field_of_view = HALF_FIELD_OF_VIEW
        if not success:
            self.need_sync = True
            self.get_logger().warn("Invalid sonar ping mode, %d" % sonar_info.ping_mode)
            self.get_logger().warn("Synchronization needed ...")


        return sonar_info


def main(args=None):
    rclpy.init(args=args)
    node = SonarSoundMetricsAris3000()

    # Spin in a background thread so ROS2 callbacks (services, parameters) are processed
    # while the main thread blocks on UDP socket reads in read_sonar_image()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.read_sonar_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
