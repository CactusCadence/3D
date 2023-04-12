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

import sys

from robodk import robolink    # RoboDK API
# from robodk import robomath    # Robot toolbox
RDK = robolink.Robolink()

if len(sys.argv) > 1:
    materialLength = float(sys.argv[1])
    print(materialLength)

# Grab the 3D print program
prog = RDK.Item('Print3D', robolink.ITEM_TYPE_PROGRAM)
robot = RDK.Item('Arm', robolink.ITEM_TYPE_ROBOT)

# Get the current instruction id (returns current id if program is running)
# See https://robodk.com/doc/en/PythonAPI/robodk.html#robodk.robolink.Item.InstructionSelect
currID = prog.InstructionSelect()
print(currID)

# Check the next instruction. If it is a speed instruction, record it in a variable & the param. Otherwise,
# use the recorded speed value in the param
#
# If the next instruction was not a speed instruction, it should be a linear move. For the linear move,
# record the target, then grab the robot's current position and compare the two to obtain the distance.

speed = -1.0
currentPosition = robot.Joints().list()

# need to convert from string to float
isSpeed = RDK.getParam("lastSpeed")

#if speed
if(isSpeed is None):
    RDK.setParam("lastSpeed", 0)
else:
    RDK.setParam("lastSpeed", speed + 1.0)
# elif(speed is float):
#     speed += 1.0
#     RDK.setParam("lastSpeed", )

#if move

print(isSpeed)
print(currentPosition)

# Provoking an exception will display the console output if you run this script from RoboDK
raise Exception('Display output. If program was run accidentally, move the error message above the pause button on RoboDK and click fast. (Shortcut is Backspace)')
