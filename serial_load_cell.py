from dataclasses import dataclass
import serial
import struct
import numpy as np
import time

# port_name = "/dev/tty.usbserial-00002014"
port_name = "/dev/ttyUSB0"
serialPort = serial.Serial(port = port_name, baudrate=230400,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

data = []

# time.sleep(2)
# serialPort.write("a".encode())
# time.sleep(0.5)
# serialPort.write("y".encode())

t0 = time.time()
while(time.time()-t0 < 15):
    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):

        serialMsg = serialPort.read(26)
        # print((serialMsg))

        if len(serialMsg) >= 4:
            startingByte = struct.unpack('i', serialMsg[0:4])[0]
            if startingByte == -328961:

                force = struct.unpack('>hhh', serialMsg[4:10])
                torque = struct.unpack('>hhh', serialMsg[10:16])

                wrench = np.concatenate((force, torque))

                data.append(np.concatenate(([time.time()-t0], wrench)))

                timeMsg = struct.unpack('I', serialMsg[18:22])

                print(struct.unpack('>h', serialMsg[8:10]))


                # print("Force: {} | Torque: {} ".format(force, torque))
        
np.savetxt("data.txt", data)
