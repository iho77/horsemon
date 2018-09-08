import socket
import struct
import datetime

UDP_IP = "192.168.1.2"
UDP_PORT = 9999
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

f = open("logfile.txt", "a")

while True:
    data, addr = sock.recvfrom(12) # buffer size is 1024 bytes
    ax,ay,az = struct.unpack('iii',data)
    line = str(datetime.datetime.now())+":"+str(ax)+":"+str(ay)+":"+str(az)+"\n"
    f.write(line)
    print line
