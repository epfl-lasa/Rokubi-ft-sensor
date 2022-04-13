#include <iostream>
#include <sstream>
#include <serial/serial.h>

int main(int, char**) {
    
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


    while (1){

        if (!serialLine.isOpen()) {
            std::cout << "The serial port is not open anymore" << std::endl;
            return 0;
            }
        
        if(serialLine.available()){
            // std::string message = serialLine.read(serialLine.available());
            std::string message = serialLine.readline(serialLine.available());
            size_t start = message.find("\377\372\372\377");

            // uint8_t msg[600];
            // serialLine.read(msg, serialLine.available());
            // \377\372\372\377
 
            // std::cout << message << std::endl;

            // float x;   
            // std::stringstream ss;
            // ss << std::hex << message.substr(2, 4);
            // ss >> x;
            // // output it as a signed type
            // std::cout << x << std::endl;
        }

    }
}
