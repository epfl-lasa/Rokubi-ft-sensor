# Rokubi F/T Drivers
Drivers to read Rokubi 2.00/2.10 F/T sensor using serial over USB

Need to update FTDI Linux USB latency to use faster reading frequencies.
On Linux, modify the file /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
(We recommend 2ms instead of 16ms)
