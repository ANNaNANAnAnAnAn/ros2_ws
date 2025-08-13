#pragma once 
#include <iostream>
#include <serial/serial.h>

class LidarReader{
public:
    LidarReader(const std::string &port, unsigned long baud_);
    ~LidarReader();

    float getDistance();
    bool isValid();

private: 
    serial::Serial lidar_serial;
    std::string port_;
    unsigned long baud_;
    bool valid_;

    void connect();
    float ParseData();

};
