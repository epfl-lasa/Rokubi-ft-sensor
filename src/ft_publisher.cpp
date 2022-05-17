#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <serial/serial.h>

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ft_publisher");

    ros::NodeHandle nh;

    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench_data", 1000);

    ros::Rate loop_rate(250);
    
    serial::Serial serialLine;

    try {
        serialLine.setPort("/dev/ttyUSB0");
        serialLine.setBaudrate(230400);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
        serialLine.setTimeout(timeout);
        serialLine.setBytesize(serial::eightbits);
        serialLine.setParity(serial::parity_none);
        serialLine.setStopbits(serial::stopbits_one);
        serialLine.open();

        if (!serialLine.isOpen()) {
            std::cout << "The serial port is not open" << std::endl;
            }
    }
    catch (serial::IOException& e) {
        std::cout << "Error with the serial port" << std::endl;
    }
    std::cout << "Serial port successfully opened" << std::endl;

    size_t bufferLength = 26;
    while (ros::ok()){

        // Fast infinite loop to poll messages
        // We wait to have received at least two messages to discard the fist one
        // and guarantee that the second one is correctly read
        while ( (!serialLine.isOpen()) || (serialLine.available() < 2*bufferLength) )  {}
        
        
        // Read message and flush buffer
        std::string message = serialLine.read(serialLine.available());
        
        size_t headerPosition = message.rfind("\377\372\372\377");
        headerPosition = message.substr(0, headerPosition).rfind("\377\372\372\377");

        std::cout << headerPosition << " " << message.length() << std::endl;
        
        serialLine.flush(); 

        if (headerPosition < message.length() - bufferLength){
            std::string sensorMsg = message.substr(headerPosition, 26);

            geometry_msgs::WrenchStamped sensorWrench;

            sensorWrench.wrench.force.x = (int)(((int8_t)sensorMsg.at(4))*255 + (uint8_t)sensorMsg.at(5));
            sensorWrench.wrench.force.y = (int)(((int8_t)sensorMsg.at(6))*255 + (uint8_t)sensorMsg.at(7));
            sensorWrench.wrench.force.z = (int)(((int8_t)sensorMsg.at(8))*255 + (uint8_t)sensorMsg.at(9));

            sensorWrench.header.stamp = ros::Time::now();

            wrench_pub.publish(sensorWrench);
        }
        else{
            std::cout << "Lost message !!" << std::endl;
        }
        
        ros::spinOnce();
        loop_rate.sleep();

    }

    
}
