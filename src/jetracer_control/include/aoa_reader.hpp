#pragma once 
#include <iostream>
#include <serial/serial.h>
class AOAReader {
public:
    AOAReader(const std::string &port, unsigned long baud);
    ~AOAReader();

    float getAngle();
    bool isValid();

private:
    serial::Serial aoa_serial; 
    std::string port;
    bool valid;
    unsigned long baud_;

    void connect();
    float parseData();
}; 