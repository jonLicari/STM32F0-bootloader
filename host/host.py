import binascii
import os
import serial
import sys
import time

# Constants
fileName = 'tv1.bin'
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
    global endFlag
    packetBuf = 0
    ctr = 0

    while True:
        flag = port.read(1) # Read UART 
        if (flag == b'2'): # Only transmit once STM is ready
            print(flag)
            flag = 0
            time.sleep(1)
            
            # Send the number of packets to be expected
            NumberOfPackets()
            break

    while True:
        flag = port.read(1) # Read UART 

        if (flag == b'1'): # Transmit next packet
            print("Ready for next packet")
            flag = 0 # reset ready bit
            print(ctr) # Print number ID of packet to be sent

            packetBuf = ReadNextPacket()

            # EOF handling
            if (endFlag == 1):
                endFlag = (endFlag and 0) # Clear flag
                Transmit(packetBuf)
                print("Close Port")
                port.close()
                break
            else:
                Transmit(packetBuf)
                ctr += 1 # Increment packet ID counter if transmission is successful

        elif(flag == b'2'): # Checksum did not match, send packet again
            print("Checksum Error: Resending packet " + str(ctr-1) + "...")
            flag = 0 # reset ready bit
            # Do not update packet buffer
            Transmit(packetBuf) # Resend packet
            # Do not increment packet ID
            
        else:
            print("Waiting..")


def ReadNextPacket():
    global endFlag

    packet = dataFile.read(SIZE) # Read bytes from file 

    if (packet == b''):
        endFlag = 1              # Set end of file flag
        print("End of file ")
        dataFile.close()         # Close file 
    
    return packet


def Transmit(dataPacket):
    dLength = len(dataPacket)
    
    if (dLength != SIZE):                   # If end of file reached
        for x in range(dLength, SIZE):
                #port.write(b'\xFF')
                dataPacket += b'\xFF'       # Append null characters to fill packet 
                x += 1

    checkVal = Checksum(dataPacket)         # String of hexadecimal format
    crcDecimal = int(checkVal, 16)          # convert to decimal 
    crc = list(map(int, str(crcDecimal)))   # convert integer to list
    if (len(crc) != 10):                    # STM32 expects 10 digits
        crc.insert(0, 0)

    print(crc)

    # Send bytes to com port to be received
    print("Begin Transmission")
    port.write(dataPacket)  # Send data packet 
    port.write(crc)         # Send checksum of packet
    print("End packet ")


def NumberOfPackets():
    totalBytes = os.stat(fileName).st_size 
    
    if (totalBytes%SIZE == 0): # File size perfectly divisible by packet size
        packetSize = totalBytes//SIZE
    else:
        packetSize = (totalBytes//SIZE)+1
    
    print("Number of packets: " + str(packetSize))
    
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


def Checksum(array):
    buf = (binascii.crc32(array) & 0xFFFFFFFF) # Calculate checksum of the bytearray
    return "%08X" % buf


if __name__ == "__main__":
    main()
