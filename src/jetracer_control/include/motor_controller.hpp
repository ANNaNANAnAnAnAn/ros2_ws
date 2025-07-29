//motor_controller.cpp
#pragma once
#include <cstdint>

class MotorController {
public:
    MotorController();
   ~MotorController();
    void setThrottle(float value);
    void stop();

private: 
    int fd_; 
    void initPCA9685();
    void writeRegister(uint8_t reg, uint8_t value);
    void setPWM(int channel, int on, int off);
    void setPWM_DutyCycle(int channel, float duty_cycle);
};