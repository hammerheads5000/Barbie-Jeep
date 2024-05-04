#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

// GPIO library
// Install (for raspbian) with sudo apt-get install pigpio python-pigpio python3-pigpio
#include <pigpiod_if2.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonSRX leftMotor(0);
TalonSRX rightMotor(0);

const uint8_t STEERING_WHEEL_ENCODER_PIN = 0;
const uint8_t THROTTLE_PIN = 0;
const uint16_t ENCODER_PWM_FREQ_HZ = 976;
const uint16_t MAX_ENCODER_PWM_RANGE = 1024;
const uint16_t STEERING_NEUTRAL = 512;
const uint16_t STEERING_NEUTRAL_TO_MAX = 256;
const uint16_t THROTTLE_NEUTRAL = 128;
const uint16_t THROTTLE_NEUTRAL_TO_MAX = 256;

const float MAX_SPEED = 1.0;
float currentSpeed = 0.0;

struct DriveState
{
    float leftDutycycle;
    float rightDutycycle;
};

void init() {
    // GPIO
    gpioInitialize();
    gpioSetMode(STEERING_WHEEL_ENCODER_PIN, PI_INPUT);
    gpioSetMode(THROTTLE_PIN, PI_INPUT);
    gpioSetPWMfrequency(STEERING_WHEEL_ENCODER_PIN, ENCODER_PWM_FREQ_HZ);
    gpioSetPWMfrequency(THROTTLE_PIN, ENCODER_PWM_FREQ_HZ);
    gpioSetPWMrange(STEERING_WHEEL_ENCODER_PIN, MAX_ENCODER_PWM_RANGE);
    gpioSetPWMrange(THROTTLE_PIN, MAX_ENCODER_PWM_RANGE);
}

void exit() {
    gpioTerminate();
}

float getSteeringAmount() {
    uint16_t pwm_value = gpioGetPWMdutycycle(STEERING_WHEEL_ENCODER_PIN) - STEERING_NEUTRAL;
    float dutycycle = pwm_value / STEERING_NEUTRAL_TO_MAX;
    dutycycle = std::clamp(dutycycle, -1.0, 1.0);
    return dutycycle;
}

float getThrottle() {
    uint16_t pwm_value = gpioGetPWMdutycycle(THROTTLE_PIN) - THROTTLE_NEUTRAL;
    float dutycycle = pwm_value / THROTTLE_NEUTRAL_TO_MAX;
    dutycycle = std::clamp(dutycycle, 0, 1.0);
    return dutycycle;
}

DriveState steerAmountToDriveState(float steerAmount) {
    float innerMotorSpeed = (1 - steerAmount) * currentSpeed;
    if (steerAmount > 0) {
        return {currentSpeed, innerMotorSpeed};
    }
    return {innerMotorSpeed, currentSpeed};
}

void drive(DriveState driveState) {
    leftMotor.Set(TalonSRXControlMode::PercentOutput, driveState.leftDutycycle);
    rightMotor.Set(TalonSRXControlMode::PercentOutput, driveState.rightDutycycle);
}

int main(int argc, char const *argv[])
{
    init();
    
    while (true) {
        currentSpeed = getThrottle();
        float steering = getSteeringAmount();
        DriveState driveState = steerAmountToDriveState(steering);
        drive(driveState);
    }

    exit();
    return 0;
}
