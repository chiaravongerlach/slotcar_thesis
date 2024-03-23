import serial
from struct import *
import sys
import time
import random
import ast

try:
    ser=serial.Serial(baudrate='9600', timeout=.5, port='/dev/tty.usbmodem1101')
except:
    print('Port open error')

time.sleep(1)

data = 0
increasing = True

while True:
    try:
        # the first and last number are used to check if the returned data is correct, 127
        # the 8 other values in between are dynamic data
        ser.write(pack('3B', 255, data, 255))
        time.sleep(.01)
        dat=ser.readline()
        
        if dat!=b''and dat!=b'\r\n':
            try:
                dats=str(dat)
                dat1=dats.replace("b","")
                dat2=dat1.replace("'",'')
                dat3=dat2[:-4]
                data_list=ast.literal_eval(dat3)
                # check to see if data returned is correct
                if data_list[0] == 255 and data_list[-1] == 255:
                    print("Correct:", data_list)
                    #data = ANGLE
                    if increasing:
                        data = data + 1
                        if data > 180:
                            data = 180
                            increasing = False
                    else:
                        data = data - 1
                        if data < 0:
                            data = 0
                            increasing = True
                else:
                    print("Wrong data:", data_list)
                # print(dat3)
            except:
                print('Error in corvert, readed: ', dats)
        time.sleep(0.1)
    except KeyboardInterrupt:
        break
    except:
        print(str(sys.exc_info())) #print error
        break