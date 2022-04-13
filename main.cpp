#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <serial/serial.h>

int main(int, char**) {
    
    serial::Serial serialLine;

    try {
        serialLine.setPort("/dev/ttyUSB0");
        serialLine.setBaudrate(230400);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
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

    std::ofstream myfile;
    myfile.open("data.txt");

    int i = 0;
    auto startTime = std::chrono::steady_clock::now();
    while (i < 800*10){

        if (!serialLine.isOpen()) {
            std::cout << "The serial port is not open anymore" << std::endl;
            return 0;
            }
        
        if(serialLine.available()){
            std::string message = serialLine.read(26);
            //std::string message = serialLine.readline();
            size_t headerPosition = message.find("\377\372\372\377");
            std::string headerMsg;

            if (headerPosition == 0){
                headerMsg = message.substr(headerPosition, 1);
                auto number = (uint8_t)message.at(21);


                int fx = ((int8_t)message.at(4))*255 + (uint8_t)message.at(5);
                int fy = ((int8_t)message.at(6))*255 + (uint8_t)message.at(7);
                int fz = ((int8_t)message.at(8))*255 + (uint8_t)message.at(9);

                auto timeNow = 1e-6*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startTime).count();

                int id = (uint8_t)message.at(0) << 8;
  
                std::cout << "Time: " << timeNow << " Fx: " << fx << " Fy: " << fy << " Fz: " << fz << std::endl;
                myfile  << timeNow << " " << fx << " " << fy << " " << fz << std::endl;
                i++;
                
            }

        }

    }
    myfile.close();
}
