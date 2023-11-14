import socket

UDP_IP = "10.0.0.111"
UDP_PORT = 8888
MESSAGE = b"{ distance: 90, materialLength: 90, newSpeed: 15 }\n"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

# data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
# print("received message: %s" % data)