#include"include/aoa_reader.hpp"
#include<serial/serial.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sstream>

AOAReader ::AOAReader(const std::string &port, unsigned long baud_)
    : valid_(false), port_(port), baud_rate(baud_){
        connect(); 
    }


AOAReader::~AOAReader(){
     if (aoa_serial.isOpen()){
        aoa_serial.close();
     }
}


void AOAReader::connect() {
    try {
        aoa_serial.setPort(port);
        aoa_serial.setBaudrate(baud_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // 1 second timeout
        aoa_serial.setTimeout(to);
        aoa_serial.setParity(serial::parity_none);
        aoa_serial.setStopbits(serial::stopbits_one);
        aoa_serial.setBytesize(serial::eightbits);
        aoa_serial_.setFlowcontrol(serial::flowcontrol_rtscts);

        aoa_serial.open();
        if (aoa_serial.isOpen()){
            std::count<<"AOA device is connected on " << port_ << std::endl;
        }else{
            throw std::runtime_error("Failed to open serial port");
        } catch (const serial::IOException &e) {
            std::cerr << "Serial exception: " << e.what() << std::endl;
            throw;
        }

    }
}

float AOAReader::parseData() {
    try {
        std::string line;
        while (true) {
            line = aoa_serial_.readline(256, "\n");

            if (line.length() > 60) {
                std::vector<std::string> tokens;
                std::stringstream ss(line);
                std::string item;

                while (std::getline(ss, item, ',')) {
                    tokens.push_back(item);
                }

                if (tokens.size() >= 3) {
                    try {
                        float angle = std::stof(tokens[2]);
                        valid = true;
                        return angle;
                    } catch (const std::exception &e) {
                        std::cerr << "Invalid float conversion: " << e.what() << std::endl;
                        valid = false;
                        return 0.0f;
                    }
                }
            }
        }

    } catch (const std::exception &e) {
        std::cerr << "Serial read error: " << e.what() << std::endl;
    }

    valid = false;
    return 0.0f;
}

float AOAReader::getAngle() {
    return parseData();
}
bool AOAReader::isValid() {
    return valid ;
}