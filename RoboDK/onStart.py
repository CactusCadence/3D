import serial

ser = serial.Serial('COM6', 9600, timeout=1.0)

ser.write(b"NEW_PRINT\n")
print(ser.readline())