__author__ = 'jianxin'


import serial
import time
import re
import sys

#regular expression pattern for serial input
#expected to receive two integers with space in between from each input line
r = re.compile('(\d+) (\d+)')

#attempt to connect to Arduino within 10s, change '/dev/ttyACM0' to your serial port name
print ("Waiting for Arduino Connection...")
counter = 0
while counter < 10*100:
    try:
        ser = serial.Serial('COM6', 57600)
        print ("Connected to Arduino.")
        break
    except:
        counter += 1
        time.sleep(0.01)
if counter >= 10*100:
    print ( "Connection timeout.")
    sys.exit(0)

#start collecting data from serial
#sampling frequency should be consistent with the one used in Arduino
freq = 1000/2.
#collect data for a period of T seconds
T = 10
#lists for storing all the signals and timestamps
input_signal = []
output_signal = []
input_time = []
output_time = []

#flush outdated serial data
ser.flush()
#remember initial time
start = time.time()
current_time = 0
while len(input_signal) < freq*T and current_time < T:
    #use regular expression to match serial input with pattern "integer integer"
    line = ser.readline()
    x = r.match(line)
    print x
    current_time = time.time()-start
    if x is not None:
        #the first integer is voltage analog signal through a 10bit ADC
        #multiply by 5.0/1024 to recover the analog voltage signal
        input_signal.append(int(x.group(1))/1024.*5)
        input_time.append(current_time)
        #the second integer is the encoder reading
        output_signal.append(int(x.group(2)))
        output_time.append(current_time)
#if serial port is not correctly configured (wrong baud rate, wrong output format, etc
#this will raise a warning
if not input_signal and current_time > T:
    print ( "No data received, check Arduino code for proper output format.")
    print ( "Arduino output format should be:")
    print ( "1000 500")
    print ( "1000 is voltage/5*1024, 500 is encoder reading")
    ser.close()
    sys.exit(0)
ser.close()

#store the data collected into two txt files
f = open('input.txt','w')
for item in zip(input_time,input_signal):
    f.write('time {0} voltage {1}\n'.format(item[0],item[1]))
f.close()

f = open('output.txt','w')
for item in zip(output_time,output_signal):
    f.write('time {0} pos {1}\n'.format(item[0],item[1]))
f.close()


