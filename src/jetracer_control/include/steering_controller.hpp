#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <stdexcept>
#include <cstdint>
#include <algorithm>

namespace jetracer_control {

class SteeringController {
public:
    SteeringController();
    ~SteeringController();
    void setSteering(float value);

private:
    int fd_;
    const char* device_ = "/dev/i2c-1";
    const uint8_t address_ = 0x40;
    const int SERVO_CHANNEL = 0;
    const int MIN_PULSE = 205;
    const int MAX_PULSE = 410;

    void init();
    void writeRegister(uint8_t reg, uint8_t val);
    void setPWM(int channel, int on, int off);
};

}  // namespace jetracer_control
