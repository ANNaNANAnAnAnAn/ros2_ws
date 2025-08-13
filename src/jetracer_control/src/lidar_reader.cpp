#include"include/lidar_reader.hpp"
#include<serial/serial.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sstream>


LidarReader::LidarReader(const std::string &port, unsigned baud_){
    :valid_(false), port_(port_), baud_rate(baud_){
        connect();
    }
}

LidarReader::~LidarReader(){
    if (lidar_serial.isOpen()){
        lidar_serial.close();
    }
}

void AOAReader::connect() {
    try {
        aoa_serial.setPort(port_);
        aoa_serial.setBaudrate(baud_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // 1 second timeout
        aoa_serial.setTimeout(to);
        aoa_serial.setPartity(serial::parity_none);
        aoa_serial.setStopbits(serial::stopbits_one);
        aoa_serial.setBitesize(serial::eightbits);
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


float LidarReader::ParseData(){
    
}