#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <serial/serial.h>

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ft_publisher");

    ros::NodeHandle nh;

    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::Wrench>("wrench_data", 1000);

    ros::Rate loop_rate(100);
    
    serial::Serial serialLine;

    try {
        serialLine.setPort("/dev/ttyUSB0");
        serialLine.setBaudrate(230400);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
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

    double time0 = ros::Time::now().toSec();
    while (ros::ok()){

        if (!serialLine.isOpen()) {
            std::cout << "The serial port is not open anymore" << std::endl;
            return 0;
            }
        
        if(serialLine.available()){
            // double receiveTime = ros::Time::now().toSec() -time0;

            std::string message = serialLine.read(serialLine.available());
            
            serialLine.flush();

            size_t headerPosition = message.rfind("\377\372\372\377");
            headerPosition = message.substr(0, headerPosition).rfind("\377\372\372\377");

            std::cout << headerPosition << " " << message.length() << std::endl;
            std::string sensorMsg;

            if (headerPosition < message.length() - 26){
                sensorMsg = message.substr(headerPosition, 26);
                // int id = (uint8_t)message.at(0) << 8;

                geometry_msgs::Wrench sensorWrench;

                sensorWrench.force.x = (int)(((int8_t)sensorMsg.at(4))*255 + (uint8_t)sensorMsg.at(5));
                sensorWrench.force.y = (int)(((int8_t)sensorMsg.at(6))*255 + (uint8_t)sensorMsg.at(7));
                sensorWrench.force.z = (int)(((int8_t)sensorMsg.at(8))*255 + (uint8_t)sensorMsg.at(9));

                // sensorWrench.force.x = fx;
                // sensorWrench.force.y = fy;
                // sensorWrench.force.z = fz;

                wrench_pub.publish(sensorWrench);
  
                // std::cout << "Time: " << receiveTime << " Fx: " << fx << " Fy: " << fy << " Fz: " << fz << std::endl;

            }
            else{
                std::cout << "Lost message !!" << std::endl;
            }

        }

    ros::spinOnce();
    loop_rate.sleep();

    }
}
