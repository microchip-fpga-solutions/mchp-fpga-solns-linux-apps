// example_main.cpp
#include "motor.h"
#include <iostream>
#include <unistd.h>

int main() {
    using namespace motorcontrol;

    Motor bldc(MotorType::BLDC);
    Motor stepper(MotorType::STEPPER);

    if (bldc.is_valid()) {
        std::cout << "Initializing BLDC...\n";
        bldc.init();
        bldc.start();
        sleep(3);
        bldc.set_direction(0);
        sleep(3);
        bldc.stop();
        bldc.clear_fault();
    } else {
        std::cerr << "BLDC devices missing; skipping\n";
    }

    if (stepper.is_valid()) {
        std::cout << "Initializing Stepper...\n";
        stepper.init();
        stepper.start();
        sleep(2);
        stepper.stop();
        stepper.clear_fault();
    } else {
        std::cerr << "Stepper devices missing; skipping\n";
    }

    return 0;
}
