// SPDX-License-Identifier: MIT
/*
 * @file motor.h
 * @brief PolarFire SoC Motor Control Library - BLDC and Stepper motor interface
 * Copyright (C) 2026 Microchip Technology Inc. and its subsidiaries
 */
#pragma once

#include "motorclass_def.h"
#include <string>
#include <cstdint>
#include <optional>

namespace motorcontrol {

// Speed limits
constexpr int BLDC_SPEED_MIN = 10;
constexpr int BLDC_SPEED_MAX = 4000;
constexpr int BLDC_SPEED_SCALE = 16;

// Direction constants
constexpr int DIRECTION_CLOCKWISE = 0;
constexpr int DIRECTION_COUNTER_CLOCKWISE = 1;
inline const std::string DIRECTION_CW_STR = "CLOCKWISE";
inline const std::string DIRECTION_CCW_STR = "COUNTER CLOCKWISE";
// Stepper SLEW_CNT register bounds: 0x20 (32) to 0xFF (255)
// Lower value = faster speed (inverse relationship)
constexpr int STEPPER_SPEED_MIN = 0x20;  // 32 - fastest
constexpr int STEPPER_SPEED_MAX = 0xFF;  // 255 - slowest

// Bit manipulation for stepper direction
constexpr uint32_t CMD_STEP_VALUE_MASK = 0x007FFFFF;
constexpr uint32_t CMD_STEP_SIGN_BIT = (1 << 23);
constexpr uint32_t CMD_STEP_FULL_MASK = 0xFFFFFF;

// Constants from SoftConsole config.h
constexpr uint32_t SYS_CLK = 100;           // MHz
constexpr uint32_t ADC_RES = 12;            // ADC bit resolution
constexpr uint32_t ADC_REF_VOLT = 5000;     // mV
constexpr uint32_t SHUNT_RES = 20;          // mOhm
constexpr uint32_t CURR_GAIN = 20;          // Current shunt amplifier gain
constexpr uint32_t ANGLE_MAX = 262144;      // 360 degrees
constexpr uint32_t TWO_POWER_10 = 1024;
constexpr uint32_t RAMP_TI = 10;            // ms
constexpr uint32_t OVER_CURRENT_THRESHOLD = 98304;

enum class MotorType { BLDC, STEPPER };

// Startup Mode for sensorless
enum class StartupMode { CURRENT_BY_FREQ = 0, VOLTAGE = 1 };

// BLDC Motor Specification Parameters
struct BLDCSpecs {
    uint32_t dc_voltage_mV = 24000;
    uint32_t motor_current_mA = 1790;
    uint32_t motor_speed_rpm = 4000;
    uint32_t pole_pairs = 4;
    uint32_t resistance_mOhm = 1800;
    uint32_t inductance_uH = 2600;
    uint32_t switching_freq_kHz = 20;
};

// BLDC Motor Control Parameters
struct BLDCParams {
    uint32_t current_pi_kp = 124;
    uint32_t current_pi_ki = 275;
    uint32_t speed_pi_kp = 3000;
    uint32_t speed_pi_ki = 10;

    StartupMode startup_mode = StartupMode::CURRENT_BY_FREQ;

    bool auto_restart = true;
    bool soft_stop = false;

    uint32_t closed_loop_speed_rpm = 400;
    uint32_t open_loop_current_pct = 50;
    uint32_t open_loop_voltage_pct = 5;
    uint32_t angle_pi_kp = 10000;
    uint32_t angle_pi_ki = 5000;
};

// Stepper Motor Specification Parameters
struct StepperSpecs {
    uint32_t dc_voltage_mV = 24000;
    uint32_t motor_current_mA = 1000;
    uint32_t step_number = 200;
    uint32_t microstep_resolution = 64;
    uint32_t resistance_mOhm = 5300;
    uint32_t inductance_uH = 6500;
    uint32_t switching_freq_kHz = 20;
};

// Stepper Motor Control Parameters
struct StepperParams {
    uint32_t current_pi_kp = 124;
    uint32_t current_pi_ki = 275;

    uint32_t current_reference_mA = 500;
    int32_t cmd_steps = 100;
    uint32_t speed_rpm = 100;
};

class Motor {
public:
    explicit Motor(MotorType type);
    ~Motor();

    // non-copyable
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    // movable
    Motor(Motor&&) = default;
    Motor& operator=(Motor&&) = default;

    // device validation
    bool is_valid() const;

    // Initialize motor with default values (writes to hardware)
    void init();

    // Control actions
    void start();
    void stop();
    void set_direction(int dir);
    void set_direction(const std::string& dir);  // Accepts "CLOCKWISE" or "COUNTER CLOCKWISE"
    int set_speed(int speed);
    void clear_fault();

    // Status methods
    int get_state();           // Returns sequencer FSM state
    int get_speed();           // Returns current speed (from omega filter)

    // BLDC-specific status methods
    std::optional<int> get_bldc_speed_raw();     // Returns raw speed value from hardware (for debugging)
    std::optional<int> get_bldc_speed();         // Returns current BLDC speed in RPM (scaled by pole_pairs and motor_speed_rpm)
    std::optional<std::string> get_bldc_direction();  // Returns "CLOCKWISE" or "COUNTER CLOCKWISE"
    std::optional<int> get_bldc_iq();            // Returns current IQ value

    // ===== BLDC Parameter Access =====
    void set_bldc_specs(const BLDCSpecs& specs);
    BLDCSpecs get_bldc_specs() const { return bldc_specs_; }

    void set_bldc_params(const BLDCParams& params);
    BLDCParams get_bldc_params() const { return bldc_params_; }

    // ===== Stepper Parameter Access =====
    void set_stepper_specs(const StepperSpecs& specs);
    StepperSpecs get_stepper_specs() const { return stepper_specs_; }

    void set_stepper_params(const StepperParams& params);
    StepperParams get_stepper_params() const { return stepper_params_; }

    // Apply current parameters to hardware (call after setting specs/params)
    void apply_parameters();

    // Get motor type
    MotorType get_type() const { return type_; }

private:
    MotorType type_;
    BLDCSpecs bldc_specs_;
    BLDCParams bldc_params_;
    StepperSpecs stepper_specs_;
    StepperParams stepper_params_;

    // Device objects (from motorclass_def.h)
    ADC          adc_;
    SQMNGDriver  sqmn_;
    PIController picon_;
    PWMDriver    pwmdrive_;
    RateLimit    rlimit_;
    STPTheta     theta_;  // used only by STEPPER

    // Calculated intermediate values for BLDC
    uint32_t bldc_adc_scale_ = 0;
    uint32_t bldc_pwm_period_ = 0;
    uint32_t bldc_pwm_gain_ = 0;
    uint32_t bldc_theta_factor_ = 0;
    uint32_t bldc_speed_scale_ = 0;
    uint32_t bldc_speed_descale_ = 0;
    uint32_t bldc_slew_count_ = 0;
    uint32_t bldc_rate_count_ = 0;
    uint32_t bldc_resistance_pu_ = 0;
    uint32_t bldc_ls_pu_by_ts_ = 0;

    // Calculated intermediate values for Stepper
    uint32_t stepper_adc_scale_ = 0;
    uint32_t stepper_pwm_period_ = 0;
    uint32_t stepper_rate_count_ = 0;
    uint32_t stepper_slew_count_ = 0;

    // Private helpers
    void calculate_bldc_constants();
    void calculate_stepper_constants();
    void init_bldc();
    void init_stepper();
    void write_bldc_registers();
    void write_stepper_registers();
    uint8_t build_bldc_seq_config();
    uint8_t build_stepper_seq_config();
};

} // namespace motorcontrol
