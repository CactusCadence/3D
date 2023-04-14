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
from robodk import robomath    # Robot toolbox
RDK = robolink.Robolink()

LASTSPEED = "LASTSPEED"

if len(sys.argv) > 1:
    materialLength = float(sys.argv[1])

# Grab the 3D print program
prog = RDK.Item('Print3D', robolink.ITEM_TYPE_PROGRAM)
robot = RDK.Item('Arm', robolink.ITEM_TYPE_ROBOT)

# Since Targets are taken using the Tool Frame wrt Reference Frame,
# we have to calculate the same thing to calculate the distance.
extruder = RDK.Item("MDPH2", robolink.ITEM_TYPE_TOOL)
printing_frame = RDK.Item("Printing Frame", robolink.ITEM_TYPE_FRAME)

currentPosition = extruder.PoseWrt(printing_frame)

# Get the current instruction id (returns current id if program is running)
# See https://robodk.com/doc/en/PythonAPI/robodk.html#robodk.robolink.Item.InstructionSelect
currID = prog.InstructionSelect()

# Check the next instruction. If it is a speed instruction, record it in a variable & the param. Otherwise,
# use the recorded speed value in the param
#
# If the next instruction was not a speed instruction, it should be a linear move. For the linear move,
# record the target, then grab the robot's current position and compare the two to obtain the distance.

# Note: Target uses tool frame with respect to Reference Frame
name, instructionType, moveType, isJointTarget, target, joints = prog.Instruction(currID + 1)

distance = None
speed = None

if(instructionType == robolink.INS_TYPE_MOVE):
    distance = robomath.distance(robomath.Pose_2_Motoman(currentPosition), robomath.Pose_2_Motoman(target))

elif(instructionType == robolink.INS_TYPE_CHANGESPEED):

    # extract speed from command string
    # sample string "Set speed (130.0 mm/s)"

    startValIndex = name.find('(') + 1
    endValIndex = name.find(' ', startValIndex)

    speed = float(name[startValIndex:endValIndex])
    RDK.setParam(LASTSPEED, speed)

    #after speed, there will be a movement instruction to parse.
    name, instructionType, moveType, isJointTarget, target, joints = prog.Instruction(currID + 2)
    distance = robomath.distance(robomath.Pose_2_Motoman(currentPosition), robomath.Pose_2_Motoman(target))

else:
    raise Exception("Unhandled instruction type after Extruder Call")


# Provoking an exception will display the console output if you run this script from RoboDK
# raise Exception('Display output. If program was run accidentally, move the error message above the pause button on RoboDK and click fast. (Shortcut is Backspace)')
