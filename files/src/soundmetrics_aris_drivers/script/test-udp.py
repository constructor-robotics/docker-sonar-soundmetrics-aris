import socket
import struct


# Connect to UDP socket at port 56123
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.bind(('', 56123))

header = udp.recv(68)
print(f'Read header from UDP:56123')
udp.close()

header_fmt = list(struct.unpack('< 17I', header))
print(f'Received header at UDP:56123\n {header_fmt}')