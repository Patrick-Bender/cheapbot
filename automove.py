import serial
import time
ser = serial.Serial('/dev/ttyACM0', baudrate = 9600, timeout = 1, write_timeout = 10)
time.sleep(2)
f = open('instructions.txt', 'r')
def getValues():
    ser.write(b'g')
    arduinoData = ser.readline().decode('ascii')
    return arduinoData
filelength = 0
for l in f:
    print('next line')
    print(l)
    ser.flushInput()
    ser.flushOutput()
    response = False
    if l == "wait\n":
        print('wait loop')
        while 1:
            userInput = input("Input x,y,z,g coords to adjust, input y to continue\n")
            if userInput == 'y':
                break
            ser.write(bytes(userInput + '\r\n', 'ascii'))
            print('past write')
    else:
        print('else loop')
        ser.write(bytes(l + '\r\n', 'ascii'))
    while response != b'30053\r\n' and response!= b"continue\r\n":
        response = ser.readline()
        print("Response: ")
        print(response)
        if response == b'failed\r\n':
            ser.close()
            exit()

