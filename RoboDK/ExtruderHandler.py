# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project

# You can also use the new version of the API:
# from robodk import robolink    # RoboDK API
# from robodk import robomath    # Robot toolbox
# RDK = robolink.Robolink()

# Forward and backwards compatible use of the RoboDK API:
# Remove these 2 lines to follow python programming guidelines
# from robodk import *      # RoboDK API
# from robolink import *    # Robot toolbox
# Link to RoboDK
# RDK = Robolink()

import socket

message = b'{ distance: %f, materialLength: %f, newSpeed: %f }\n' % (1, 1, 0)

UDP_IP = "10.0.0.111"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
sock.sendto(message, (UDP_IP, UDP_PORT))

# Provoking an exception will display the console output if you run this script from RoboDK
# raise Exception('Display output. If program was run accidentally, move the error message above the pause button on RoboDK and click fast. (Shortcut is Backspace)')
