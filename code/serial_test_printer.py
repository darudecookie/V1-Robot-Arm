#THIS PROGRMA WORKS - IT PRINTS 'instructions' TO SERIAL AND THE ARDUINO READS AND PARESES IT
#SECRETE WAS GIVING ARDUINO TIME TO INITALIAZE AFTER OPENING PORT
#PROBABLY CAN BE LESS THAN 5 SECS, BUT SAFE VALUE

import serial
import time

ser = serial.Serial("COM5", 9600)
print(ser.readline())

for i in range(5):
    print(5-i)
    time.sleep(1)

instructions = "A12,56,0,12 \n"
ser.write(bytes(instructions,'utf-8'))
print(ser.readline())

#ser.close()

