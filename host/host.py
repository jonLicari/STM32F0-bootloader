#!/usr/bin/env python

import serial
import time
import os

# Constants
fileName = 'fv2.txt'
portNum = 'COM3'
SIZE = 8

# Open data file for r -read b -binary (for images)
dataFile = open(fileName, 'rb') 

# Open serial COM port
port = serial.Serial(
  port = portNum, 
  baudrate = 115200, 
  bytesize = serial.EIGHTBITS, 
  parity = serial.PARITY_NONE, 
  stopbits = serial.STOPBITS_ONE
)

# Ensure Port is open
if (port.isOpen() == False):
    port.open()

print("Connected to port")

# End of file flag
endFlag = 0

def main():
    ctr = 0
    while True:
        # Read UART for ready flag
        flag = port.read(1)  
        if (flag == b'2'): 
            print(flag)
            time.sleep(1)
            # Send the number of packets to be expected
            NumberOfPackets()
            break

    time.sleep(1)

    while True:
        # Read UART for ready flag
        flag = port.read(1)  
        if (flag == b'1'): 
            print(flag)
            # Reset ready flag
            flag = 0 
            time.sleep(1)
            # Transmit data packet 
            transmit()
            # Increment the packet counter
            ctr+=1
            print(ctr)
            # End of packet 
            if (endFlag == 1):
                print("Close port")
                port.close()
                # Exit main() if end of file is reached
                break 
        else:
            print("Waiting..")

def transmit():
    # Send bytes to com port to be received
    print("Begin Transmission")
    index = 0

    while (index < SIZE):
        byte = dataFile.read(1) # read 1 byte

        if (byte == b''):
            global endFlag
            endFlag = 1 # Set end of file flag
            port.write(b'0xFF')
            #print(b'0xFF')
            #break # out of while loop, to transmit()
        else:
            port.write(byte) # Write byte to UART
            #print(byte)
        
        index += 1
        
    if (endFlag == 1):
        print("End of file ")
        # Close file 
        dataFile.close()
        
    print("End packet ")

def NumberOfPackets():
    totalBytes = os.stat(fileName).st_size 
    packetSize = totalBytes//SIZE

    # File size perfectly divisible by packet size
    if (totalBytes%SIZE == 0): 
        port.write(str(packetSize).encode())
        print(str(packetSize).encode())
    else:
        port.write(str(packetSize+1).encode())
        print(str(packetSize+1).encode())

if __name__ == "__main__":
    main()