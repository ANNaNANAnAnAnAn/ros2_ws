#include "steering_controller.hpp"

#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

namespace jetracer_control {

SteeringController::SteeringController() {
    fd_ = open(device_, O_RDWR);
    if (fd_ < 0) throw std::runtime_error("Failed to open I2C device");

    if (ioctl(fd_, I2C_SLAVE, address_) < 0)
        throw std::runtime_error("Failed to connect to I2C device at 0x40");

    init();
}

SteeringController::~SteeringController() {
    if (fd_ >= 0) close(fd_);
}

void SteeringController::init() {
    writeRegister(MODE1, 0x10);  // sleep
    usleep(5000);
    writeRegister(PRESCALE, 121);  // 50 Hz
    writeRegister(MODE1, 0x00);  // wake
    usleep(5000);
    writeRegister(MODE1, 0xA1);  // auto-increment
}

void SteeringController::writeRegister(uint8_t reg, uint8_t val) {
    uint8_t buffer[2] = {reg, val};
    if (write(fd_, buffer, 2) != 2)
        std::cerr << "Failed to write to register 0x" << std::hex << int(reg) << std::endl;
}

void SteeringController::setPWM(int channel, int on, int off) {
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };
    if (write(fd_, data, 5) != 5)
        std::cerr << "Failed to set PWM for channel " << channel << std::endl;
}

void SteeringController::setSteering(float value) {
    value = std::clamp(value, -1.0f, 1.0f);
    int pulse = MIN_PULSE + static_cast<int>((value + 1.0f) / 2.0f * (MAX_PULSE - MIN_PULSE));
    setPWM(SERVO_CHANNEL, 0, pulse);
}

}  // namespace jetracer_control
