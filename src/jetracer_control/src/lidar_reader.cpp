#include"lidar_reader.hpp"
#include<serial/serial.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sstream>


LidarReader::LidarReader(const std::string &port, unsigned long baud_){
    :valid_(false), port_(port), baud_rate(baud_){
        connect();
    }
}

LidarReader::~LidarReader(){
    if (lidar_serial.isOpen()){
        lidar_serial.close();
    }
}

void LidarReader::connect() {
    try {
        lidar_serial.setPort(port_);
        lidar_serial.setBaudrate(baud_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // 1 second timeout
        lidar_serial.setTimeout(to);
        lidar_serial.setParity(serial::parity_none);
        lidar_serial.setStopbits(serial::stopbits_one);
        lidar_serial.setBytesize(serial::eightbits);
        lidar_serial.setFlowcontrol(serial::flowcontrol_rtscts);

        lidar_serial.open();
        if (lidar_serial.isOpen()){
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
    while (true){
        try {
            uint8_t first_byte;
            if (lidar_serial.read(&first_byte,  1) != 1) continue;
            if (first_byte != 0x59) continue;

            uint8_t second_byte;
            if (lidar_serial.read(&second_byte, 1) != 1)continue;
            if (second_byte != 0x59) continue;

             uint8_t rest[7];
                size_t n = lidar_serial_.read(rest, 7);
                if (n != 7) continue;

                int distance = rest[1] * 256 + rest[0]; // dist_h * 256 + dist_l
                return distance;

        } catch (const std::exception &e) {
                std::cerr << "LiDAR read error: " << e.what() << std::endl;
                return -1;
        }
    }
}
