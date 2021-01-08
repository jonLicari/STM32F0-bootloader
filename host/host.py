import serial
import time
import os

# Constants
fileName = 'cli.bin'
portNum = 'COM3'
SIZE = 256

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
        flag = port.read(1) # Read UART 
        if (flag == b'2'): # Only transmit once STM is ready
            print(flag)
            time.sleep(1)
            # Send the number of packets to be expected
            NumberOfPackets()
            break

    while True:
        flag = port.read(1) # Read UART 
        if (flag == b'1'): # Only transmit once STM is ready
            print(flag)
            flag = 0 # reset ready bit
            transmit()
            ctr+=1
            print(ctr)
            if (endFlag == 1):
                print("Close port")
                port.close()
                break # Exit main() if end of file is reached
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
            port.write(b'\xFF')
        else:
            port.write(byte) # Write byte to UART

        index += 1
        
    if (endFlag == 1):
        print("End of file ")
        # Close file 
        dataFile.close()

    print("End packet ")

def NumberOfPackets():
    totalBytes = os.stat(fileName).st_size 
    
    if (totalBytes%SIZE == 0): # File size perfectly divisible by packet size
        packetSize = totalBytes//SIZE
    else:
        packetSize = (totalBytes//SIZE)+1

    # convert integer to list
    a = list(map(int, str(packetSize))) 
    
    # calculate number of digits
    packLength = len(a)

    # create a list
    array = [0,0,0,0]
    
    # fill in the array with the number of expected packets
    for i in range(0, packLength): 
        array.remove(array[0])
        array.append(a[i])

    port.write(array)


if __name__ == "__main__":
    main()
