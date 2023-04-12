import sys

def handleFanOn():
    pass

def handleFanOff():
    pass

def handleTemperatureAsync():
    pass

def handleTemperatureSync():
    pass

# Create a mapping for the instruction codes
callbacks = {
    106: handleFanOn,
    107: handleFanOff,
    104: handleTemperatureAsync,
    109: handleTemperatureSync,
}

if len(sys.argv) > 1:
    instructionCode = int(sys.argv[1])
    print(instructionCode)

    if(len(sys.argv) > 2):
        arg = int(sys.argv[2])
        print(arg)
    
    #check for a second argument depending on the first

# Provoking an exception will display the console output if you run this script from RoboDK
raise Exception('Display output. If program was run accidentally, move the error message above the pause button on RoboDK and click fast. (Shortcut is Backspace)')
