
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <cstdint>
#include <algorithm>
#include "motor_controller.hpp"
#define I2C_ADDR 0x60 // Motor board address
#define I2C_DEV "/dev/i2c-1"

// PCA9685 register map
#define MODE1      0x00
#define PRESCALE   0xFE
#define LED0_ON_L  0x06


MotorController::MotorController() {
    fd_ = open(I2C_DEV, O_RDWR);
    if (fd_ < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
        throw std::runtime_error("I2C device open failed");
    }
    if (ioctl(fd_, I2C_SLAVE, I2C_ADDR) < 0) {
        std::cerr << "Failed to set I2C address" << std::endl;
        close(fd_);
        throw std::runtime_error("I2C address set failed");
    }
    initPCA9685();
    stop();
}

MotorController::~MotorController() {
    stop();
    if (fd_ >= 0) {
        close(fd_);
    }
}

void MotorController::initPCA9685(){
    writeRegister(MODE1, 0x10);
    usleep(5000);
    writeRegister(PRESCALE, 3); 
    writeRegister(MODE1, 0x80);
    usleep(5000);
}

void MotorController::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    if (write(fd_, buffer, 2) != 2) {
        std::cerr << "Failed to write to I2C register" << std::endl;
    }
} 

void MotorController::setPWM(int channel, int on, int off){
    int reg = LED0_ON_L + 4*channel;
    writeRegister(reg, on & 0xFF);
    writeRegister(reg + 1, on >>8);
    writeRegister(reg + 2, off & 0xFF);
    writeRegister(reg + 3, off >> 8);               
}

void MotorController::setPWM_DutyCycle(int channel, float duty_cycle){
    int off = static_cast<int>(std::clamp(duty_cycle, 0.0f, 1.0f) * 4095); //clamp in c++17 or later versions 
    setPWM(channel, 0, off);
}

void MotorController::setThrottle(float value) {
    value = std::clamp(value, -1.0f, 1.0f);
    float speed = std::abs(value);

    if (value > 0) {
        // Right motor forward
        setPWM_DutyCycle(0, speed);   // CH0: Right PWM
        setPWM_DutyCycle(1, 0.0f);    // CH1: IN1 LOW
        setPWM_DutyCycle(2, 1.0f);    // CH2: IN2 HIGH

        // Left motor forward
        setPWM_DutyCycle(7, speed);   // CH7: Left PWM
        setPWM_DutyCycle(5, 1.0f);    // CH5: IN1 LOW
        setPWM_DutyCycle(6, 0.0f);    // CH6: IN2 HIGH

    } else if (value < 0) {
        // Right motor backward
        setPWM_DutyCycle(0, speed);
        setPWM_DutyCycle(1, 1.0f);
        setPWM_DutyCycle(2, 0.0f);

        // Left motor backward
        setPWM_DutyCycle(7, speed);
        setPWM_DutyCycle(5, 0.0f);
        setPWM_DutyCycle(6, 1.0f);

    } else {
        stop();  // This should stop both motors
    }
}

void MotorController::stop() {
    for (int i = 0; i < 3; ++i) {
        setPWM_DutyCycle(i, 0.0f);
    }
    for (int i = 5; i < 8; i++) {
	setPWM_DutyCycle(i, 0.0f); 
   } 
}

