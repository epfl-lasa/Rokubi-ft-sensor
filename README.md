# Rokubi F/T Drivers
Drivers to read Rokubi 2.00/2.10 F/T sensor using serial over USB

## Setup

### Install serial library
See https://github.com/wjwwood/serial for the installation procedure.

### Update FTDI latency
To use faster reading frequencies, modify this file (on Linux):

`/sys/bus/usb-serial/devices/ttyUSB0/latency_timer`

(We recommend 2ms instead of 16ms)

### Install this driver
Clone this repo in your catkin workspace and run `catkin_make`

## Utilisation
Launch ROS: `roslaunch rokubi_ft_driver sensor_reading.launch`. 

Data are accessible on the topic `/wrench_data`.

Once the experiment is done, you can visualize the data by running the script `plot_ros_data.py`
