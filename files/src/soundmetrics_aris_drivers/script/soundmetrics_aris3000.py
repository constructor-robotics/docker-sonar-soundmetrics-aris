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
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image

from soundmetrics_aris_drivers.msg import SonarInfo
from soundmetrics_aris_drivers.srv import SetSonarParams, SetSonarParamsResponse

import cv2
import socket
import struct
import threading
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


class SonarSoundMetricsAris3000(object) :
    """ This class configures and receive images from Sound Metrics
        ARIS3000 forward looking sonar. """

    def __init__(self, name):
        """ Soundmetrics ARIS 3000 driver """
        self.name = name
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
        self.local_ip = "169.254.7.10"
        self.sender_ip = "169.254.7.147"

        # Param to be init later
        self.beams = 0
        self.pings = 0
        self.bundle_size = 0
        self.iysize = 0
        self.pitch_vehicle = 0.0
        #self.odometry = Odometry()
        self.compass_pitch = 0.0

        self.altitude = 1.0

        # Load ROS PARAM SERVER parameters
        self.get_config()


        ### Check whether network interface is available
        if self.getNetworkInterface(self.local_network_interface_name) == -1:
            rospy.logfatal('Required local network interface %s not found!', self.local_network_interface_name)
            sys.exit(1)

        ### NOTE: This line in case IP is extracted automatically - to be tested
        # self.local_IP = subprocess.getoutput("/sbin/ifconfig").split("\n")[self.getNetworkInterface(self.local_network_interface_name)+1].split()[1][5:]

        ### NOTE: The following block extracts the sensor IP by parsing a UDP package
        ### Needs to be tested for reliabilty, it sometimes gives 0 at boot

        # # Connect to UDP socket at port 56123
        # udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # udp.bind(('', 56123))

        # header = udp.recv(68)
        # rospy.loginfo('%s: Read header from UDP:56123', self.name)
        # udp.close()

        # header_fmt = list(struct.unpack('< 17I', header))
        # rospy.loginfo('%s: Received header at UDP:56123\n %s',
        #             self.name, header_fmt)

        # # Save sender IP
        # self.sender_IP = header_fmt[6]
        # byte_1 = self.sender_IP >> 24
        # byte_2 = (self.sender_IP - byte_1*(2**24)) >> 16
        # byte_3 = (self.sender_IP - byte_1*(2**24) - byte_2*(2**16)) >> 8
        # byte_4 = (self.sender_IP - byte_1*(2**24) - byte_2*(2**16) - byte_3*(2**8))
        # self.sender_IP_text = (str(byte_1) + '.' + str(byte_2) + '.' +
        #                     str(byte_3) + '.' + str(byte_4))

        ### ------------------------------------------------------------------------------  

        # Use IPs from config
        self.local_IP = self.local_ip
        ip = self.local_IP.split('.')
        self.local_IP_dec = (int(ip[0]) << 24) + (int(ip[1]) << 16) + (int(ip[2]) << 8) + int(ip[3])
        rospy.loginfo('%s: Local ip: %s', self.name, self.local_IP)

        self.sender_IP_text = self.sender_ip
        parts = self.sender_IP_text.split('.')
        self.sender_IP = (int(parts[0]) << 24) + (int(parts[1]) << 16) + (int(parts[2]) << 8) + int(parts[3])
        rospy.loginfo('%s: Sender ip: %s', self.name, self.sender_IP_text)

        # Create TCP socket at localhost:56888
        try:
            self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp.connect((self.sender_IP_text, TCP_COMMAND_PORT))
            self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except socket.error as e:
            rospy.logfatal('%s: Failed to create TCP connection to %s:%d - %s', self.name, self.sender_IP_text, TCP_COMMAND_PORT, e)
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
        rospy.loginfo('%s: UDP DATA @ %d connected!', self.name, UDP_DATA_PORT)

        ### Create ROS Publishers and Services
        # Create publishers
        # The image below is the polar fan image, where x-axis is the range bins (samples per beam) and y-axis is beam angle (index)
        self.polar_pub = rospy.Publisher('image/polar/raw', Image, queue_size = 2)
        self.sonar_info_pub = rospy.Publisher('sonar_info', SonarInfo, queue_size = 2)

        ## Create Service -- to be tested
        #self.load_configuration_srv = rospy.Service('configuration', SetSonarParams, self.set_configuration)
        
        self.bridge = CvBridge()
        rospy.loginfo('%s: Finish creating ROS Publishers and Services', self.name)


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
            rospy.logfatal('Invalid mode!')
            return [9, 128, 8, False]
    
    def get_config(self):
        """ Read configurations from ROS PARAM SERVER """
        
        self.use_64_bit_os = rospy.get_param('~use_64_bit_os', True)
        self.local_network_interface_name = rospy.get_param('~local_network_interface_name', "")
        self.frame_id = rospy.get_param('~frame_id', "aris3000")
        self.frame_period_sec = rospy.get_param('~frame_period_sec', 1.0)
        self.gain = rospy.get_param('~gain', 24)
        self.frequency = rospy.get_param('~frequency', 1)
        self.focus = rospy.get_param('~focus', 364)
        self.pulse_width = rospy.get_param('~pulse_width', 8)
        self.ping_mode = rospy.get_param('~ping_mode', 9)
        self.samples_per_beam = rospy.get_param('~samples_per_beam', 512)
        self.window_start = rospy.get_param('~window_start', 0.7)
        self.window_length = rospy.get_param('~window_length', 3.5)
        self.ixsize = rospy.get_param('~cartesian_width', 350)
        self.sound_velocity = rospy.get_param('~sound_velocity', 1500.0)
        self.local_ip = rospy.get_param('~local_ip', "169.254.7.10")
        self.sender_ip = rospy.get_param('~sender_ip', "169.254.7.147")

        [self.mode, self.beams, self.pings, success] = self.get_beams_and_pings(self.ping_mode)
        # Repack the float values as an unsigned 32-bit integer representing a binary value
        self.gain_binary = struct.unpack('I', struct.pack('f', self.gain))[0]

    def send_ping(self):
        """ Send a ping command to keep sensor connection alive """
        while True:
            # Send ping
            cmd = self.create_command(PING, [0, 0, 0, 0, 0, 0])
            self.send_command(cmd)
            rospy.sleep(PING_INTERVAL_SEC)


    def set_configuration(self, req):
        """ Service to change Sonar configuration """
        self.lock.acquire()
        #self.debug = req.debug
        #self.use_64_bit_os = req.use_64_bit_os 
        #self.local_network_interface_name = req.local_network_interface_name
        #self.publisher_topic = req.publisher_topic
        self.frame_period_sec = req.frame_period_sec

        # Set receiver gain
        self.gain = max(GAIN_MIN, min(req.gain, GAIN_MAX))
        self.gain_binary = struct.unpack('I', struct.pack('f', req.gain))[0]

        if req.frequency_hi:
            self.frequency = 1
        else:
            self.frequency = 0
        self.focus = req.focus
        self.pulse_width = req.pulse_width
        self.window_start = req.window_start
        self.window_length = req.window_length
        self.samples_per_beam = req.samples_per_beam
	
        # Send new configuration
        self.send_config()
        self.lock.release()
        ret = SetSonarParamsResponse()
        ret.attempted = True
        return ret


    def send_config(self):
        """ Send configuration to ARIS3000 """
        self.lock.acquire()
        self.need_sync = True
        rospy.loginfo('%s: Config sonar', self.name)

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

        rospy.loginfo('window_length: %f', self.window_length)
        rospy.loginfo('samples_per_beam: %f', self.samples_per_beam)
        rospy.loginfo('sound_velocity: %f', self.sound_velocity)
        rospy.loginfo('sample period: %f', sample_period)
        rospy.loginfo('cycle period: %f', cycle_period)

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

        # print("===================================")
        # print(header_fmt)

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

        # time.sleep(0.01)


    def read_sonar_image(self):
        """ Read ARIS3000 acoustic images """
        self.lock.acquire()

        if self.need_sync:
            rospy.loginfo('%s: Wait image sync', self.name)
            self.udp_data.settimeout(5.0)
            # Sync with first bundle
            while self.need_sync:
                try:
                    header = self.udp_data.recv(68)
                except socket.timeout:
                    rospy.logwarn('%s: No UDP data received on port %d within 5s, retrying...', self.name, UDP_DATA_PORT)
                    continue
                header_fmt = list(struct.unpack('< 17I', header))
                rospy.logdebug('%s: Sync packet - body_size=%d, packet_num=%d, total_packets=%d',
                               self.name, header_fmt[5], header_fmt[11], header_fmt[12])

                # check if there is data in the body and the number of
                # transaction is the last one of the frame
                if header_fmt[5] > 0 and header_fmt[12]-1 == header_fmt[11]:
                    self.bundle_size = header_fmt[12]
                    rospy.loginfo('%s: Synced - bundle_size=%d', self.name, self.bundle_size)
                    self.need_sync = False
                    self.udp_data.settimeout(None)
                    rospy.loginfo('%s: Reading data', self.name)

        # Read sonar image
        # ARIS sample data is stored as “one unsigned byte per sample” with valid values 0-255
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
            #polar_img_msg = self.bridge.cv2_to_imgmsg(cv_ordered_image_bgr, encoding="bgr8")	
            polar_img_msg = self.bridge.cv2_to_imgmsg(ordered_image, "mono8")
            polar_img_msg.header.stamp = rospy.Time().now()
            polar_img_msg.header.frame_id = self.frame_id
            self.polar_pub.publish(polar_img_msg)
        except CvBridgeError as e:
            rospy.logwarn('CvBridgeError: %s', e)

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

        # frameheader_bytes = str(frameheader) ### Python 2
        frameheader_bytes = frameheader ### Python 3
        # TODO: We have added 4 extra chars to work on 64bits machine!

        # # Ensure you're working with bytes
        # if self.use_64_bit_os:
        #     format_prefix = '<'  # Use little-endian, adjust if needed
        #     format_string = format_prefix + 'Q' + 'IQIIQIIIIIIffIiIIIIIIffffffffffffffffffffddfIfffIIIIfIfffffffffdfIIIfffIIIIIIIffffff16fffffIIIIIIffIIIIIIQIIIIIIf124I'
        # else:
        #     format_prefix = '<'
        #     format_string = format_prefix + 'IQIIQIIIIIIffIiIIIIIIffffffffffffffffffffddfIfffIIIIfIfffffffffdfIIIfffIIIIIIIffffff16fffffIIIIIIffIIIIIIQIIIIIIf124I'


        offset_64_bit_os = ''
        if self.use_64_bit_os:
            offset_64_bit_os = b'\x00\x00\x00\x00'
            #offset_64_bit_os = '0000'
		
        # frame_header_fields = struct.unpack(format_string, frameheader_bytes)
        frame_header_fields = struct.unpack('IQIIQIIIIIIffIiIIIIIIffffffffffffffffffffddfIfffIIIIfIfffffffffdfIIIfffIIIIIIIffffff16fffffIIIIIIffIIIIIIQIIIIIIf124I',
                                            offset_64_bit_os + frameheader_bytes)
        

        sonar_info = SonarInfo()
        sonar_info.header.stamp = rospy.Time().now()
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
            rospy.logwarn("Invalid sonar ping mode, %d", sonar_info.ping_mode)
            rospy.logwarn("Synchronization needed ...")


        return sonar_info



if __name__ == '__main__':
    try:
        rospy.init_node('soundmetrics_aris3000')
        soundmetrics_aris3000 = SonarSoundMetricsAris3000(rospy.get_name())
        while not rospy.is_shutdown():
            pass
            soundmetrics_aris3000.read_sonar_image()
    except rospy.ROSInterruptException:
        pass




# def map_scan(self, rmax, rmin):
#         """ Computes cartesian image height as well as a map vector
#             to transform a polar image into a cartesian one. """


#         # precalcualtion of constants used in do loop below
#         # (bottom of image frame to r,theta origin in meters)
#         d3 = rmin * math.cos(math.radians(HALF_FIELD_OF_VIEW))

#         # samples/m
#         c1 = (self.samples_per_beam)/(rmax-rmin)

#         # beams/deg
#         c2 = (self.beams)/(2 * HALF_FIELD_OF_VIEW)

#         # Ratio pixel/meters will depend on number of samples
#         gamma = self.samples_per_beam/(rmax-rmin)
#         #gamma= self.ixsize/(2 * rmax * math.sin(math.radians(HALF_FIELD_OF_VIEW)))

#         # number of pixels in image in vertical direction (forced to odd)
#         iysize = math.floor( gamma * (rmax - d3) )
#         if not iysize%2:
#             iysize = iysize-1

#         # number of pixels in image in vertical direction (forced to odd)
#         ixsize =  math.floor(gamma * (2 * rmax * math.sin(math.radians(HALF_FIELD_OF_VIEW))))
#         if not ixsize%2:
#             ixsize = ixsize-1

#         # make vector and fill in later
#         svector = np.zeros(ixsize * iysize)
#         # pixels in x dimension
#         ix = np.arange(ixsize)
#         # convert from pixels to meters
#         x = ((ix) - ixsize/2 + 0.5)/gamma #0.5 so that the vector is symmetric

#         for iy in np.arange(iysize):
#             # convert from pixels to meters
#             y = rmax - (iy)/gamma
#             # convert to polar cooridinates
#             r = np.sqrt(y*y + x*x)
#             # theta is in degrees
#             theta = np.degrees(np.arctan2(x, y))
#             # the rangebin number
#             binnum = np.floor((r - rmin) * c1 )
#             # the linear function to get beam number
#             beamnum = np.floor((theta + HALF_FIELD_OF_VIEW) * c2 )
#             #find position in sample array expressed as a vector
#             #make pos = 0 if outside sector, else give it the offset in the sample array
#             pos = (beamnum >= 0)*(beamnum < self.beams)*(binnum >= 0)*(binnum < self.samples_per_beam)*((beamnum-1)*self.samples_per_beam + binnum)
#             indvec = (ix)*iysize + iy
#             # The offset in this array is the pixel offset in the image array
#             # The value at this offset is the offset in the sample array
#             svector[np.ix_(indvec.astype(int))] = pos.copy()

#         mapvector = svector
#         return mapvector, int(iysize), int(ixsize)