#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <serial/serial.h>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ft_publisher");

    ros::NodeHandle nh;
    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench_data", 1000);
    ros::Rate loop_rate(250);

    // Getting calibration data
    std::vector<double> forceBias_data = {0, 0, 0};
    nh.getParam("/forceBias", forceBias_data);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> forceBias(forceBias_data.data(), 3, 3);
    std::cout << forceBias << std::endl;

    std::vector<double> forceTransform_data;
    nh.getParam("/forceTransform", forceTransform_data);
    Eigen::Map<Eigen::Matrix<double, 3, 3>> forceTransform(forceTransform_data.data(), 3, 3);
    std::cout << forceTransform << std::endl;

    std::vector<double> torqueBias_data = {0, 0, 0};
    nh.getParam("/torqueBias", torqueBias_data);
    Eigen::Map<Eigen::Matrix<double, 3, 1>> torqueBias(torqueBias_data.data(), 3, 3);
    std::cout << torqueBias << std::endl;

    std::vector<double> torqueTransform_data;
    nh.getParam("/torqueTransform", torqueTransform_data);
    Eigen::Map<Eigen::Matrix<double, 3, 3>> torqueTransform(torqueTransform_data.data(), 3, 3);
    std::cout << torqueTransform << std::endl;

    double calibWeight;
    nh.getParam("/calibrationWeight", calibWeight);

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

        // std::cout << headerPosition << " " << message.length() << std::endl;
        
        serialLine.flush(); 

        if (headerPosition < message.length() - bufferLength){
            std::string sensorMsg = message.substr(headerPosition, 26);

            // Converting data into SI units
            Eigen::Matrix<double, 3, 1> newForce{   (double)(((int8_t)sensorMsg.at(4))*255 + (uint8_t)sensorMsg.at(5)),
                                                    (double)(((int8_t)sensorMsg.at(6))*255 + (uint8_t)sensorMsg.at(7)),
                                                    (double)(((int8_t)sensorMsg.at(8))*255 + (uint8_t)sensorMsg.at(9)) };
            newForce = forceTransform*newForce + forceBias;

            Eigen::Matrix<double, 3, 1> newTorque{  (int)(((int8_t)sensorMsg.at(10))*255 + (uint8_t)sensorMsg.at(11)),
                                                    (int)(((int8_t)sensorMsg.at(12))*255 + (uint8_t)sensorMsg.at(13)),
                                                    (int)(((int8_t)sensorMsg.at(14))*255 + (uint8_t)sensorMsg.at(15)) };
            newTorque = torqueTransform*newTorque + torqueBias;

            // Filling ROS message
            geometry_msgs::WrenchStamped sensorWrench;

            sensorWrench.wrench.force.x = newForce(0);
            sensorWrench.wrench.force.y = newForce(1);
            sensorWrench.wrench.force.z = newForce(2);

            sensorWrench.wrench.torque.x = newTorque(0);
            sensorWrench.wrench.torque.y = newTorque(1);
            sensorWrench.wrench.torque.z = newTorque(2);

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
