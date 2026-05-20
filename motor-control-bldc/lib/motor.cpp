// SPDX-License-Identifier: MIT
/*
 * @file motor.cpp
 * @brief PolarFire SoC Motor Control Library - BLDC and Stepper motor implementation
 * Copyright (C) 2026 Microchip Technology Inc. and its subsidiaries
 */
#include "motor.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdint>

namespace motorcontrol {

Motor::Motor(MotorType type)
    : type_(type),
      bldc_specs_(),
      bldc_params_(),
      stepper_specs_(),
      stepper_params_(),
      adc_(),
      sqmn_(),
      picon_(),
      pwmdrive_(),
      rlimit_(),
      theta_()
{}

Motor::~Motor() = default;

bool Motor::is_valid() const {
    if (!adc_.isValid())      return false;
    if (!sqmn_.isValid())     return false;
    if (!picon_.isValid())    return false;
    if (!pwmdrive_.isValid()) return false;
    if (!rlimit_.isValid())   return false;
    if (type_ == MotorType::STEPPER && !theta_.isValid()) return false;
    return true;
}

// Build BLDC sequence controller config byte
// Bit layout: [3:soft_stop_en][2:c_by_f][1:sensor_calib_en][0:auto_restart_en]
uint8_t Motor::build_bldc_seq_config() {
    uint8_t config = 0;
    config |= (bldc_params_.soft_stop ? 1 : 0) << 3;
    config |= (bldc_params_.startup_mode == StartupMode::CURRENT_BY_FREQ ? 1 : 0) << 2;
    config |= 0 << 1;  // sensor_calib_en = 0 for sensorless
    config |= (bldc_params_.auto_restart ? 1 : 0);
    return config;
}

// Build Stepper sequence controller config byte
uint8_t Motor::build_stepper_seq_config() {
    // Stepper typically doesn't use the same flags, return basic config
    return 0x0;
}

// Calculate BLDC constants from motor specs (from MC_BLDCConstCal in SoftConsole)
void Motor::calculate_bldc_constants() {
    // ADC Scale calculation
    // g_adc_scale = (ADC_REF_VOLT * 256000) / (SHUNT_RES * CURR_GAIN * motor_current) << (16 - ADC_RES)
    bldc_adc_scale_ = ((ADC_REF_VOLT * 256000UL) /
                  (SHUNT_RES * CURR_GAIN * bldc_specs_.motor_current_mA)) << (16 - ADC_RES);

    // PWM Period calculation
    // g_pwm_period = (SYS_CLK * 500) / switching_freq_kHz
    bldc_pwm_period_ = (SYS_CLK * 500) / bldc_specs_.switching_freq_kHz;

    // PWM Gain calculation
    // g_pwm_gain = (pwm_period * 1182) / 1024
    bldc_pwm_gain_ = (bldc_pwm_period_ * 1182) / 1024;

    // Calculate Zbase for impedance calculations
    // Zbase = (dc_voltage_mV * 577) / motor_current_mA
    uint32_t zbase = (bldc_specs_.dc_voltage_mV * 577UL) / bldc_specs_.motor_current_mA;

    // Motor frequency
    // Motor_freq = (speed_RPM * Npp) / 60
    uint32_t motor_freq = (bldc_specs_.motor_speed_rpm * bldc_specs_.pole_pairs) / 60;

    // Theta factor calculation (depends on PWM period)
    if (bldc_pwm_period_ > 267) {
        bldc_theta_factor_ = (motor_freq * 134UL) / (bldc_specs_.switching_freq_kHz * 4);
    } else if (bldc_pwm_period_ > 133) {
        bldc_theta_factor_ = (motor_freq * 268UL) / (bldc_specs_.switching_freq_kHz * 4);
    } else {
        bldc_theta_factor_ = (motor_freq * 402UL) / (bldc_specs_.switching_freq_kHz * 4);
    }

    // Speed scale and descale
    // g_speed_scale = (65536 * 1024) / speed_RPM
    bldc_speed_scale_ = (65536UL * 1024UL) / bldc_specs_.motor_speed_rpm;
    // g_speed_descale = speed_RPM / 16
    bldc_speed_descale_ = bldc_specs_.motor_speed_rpm / 16;

    // Slew count
    // g_slew_count = RAMP_TI * switching_freq_kHz
    bldc_slew_count_ = RAMP_TI * bldc_specs_.switching_freq_kHz;

    // Rate count
    // g_rate_count = (67110 * RAMP_TI) / speed_RPM
    bldc_rate_count_ = (67110UL * RAMP_TI) / bldc_specs_.motor_speed_rpm;

    // Sensorless specific calculations
    // g_resistance_s_pu = (Rs_mohm * 512) / Zbase
    bldc_resistance_pu_ = (bldc_specs_.resistance_mOhm * 512UL) / zbase;

    // g_l_s_pu_by_ts = (Ls_uhenry * switching_freq_kHz * 512) / Zbase
    bldc_ls_pu_by_ts_ = (bldc_specs_.inductance_uH * bldc_specs_.switching_freq_kHz * 512UL) / zbase;
}

// Calculate Stepper constants from motor specs
void Motor::calculate_stepper_constants() {
    // ADC Scale for stepper (using stepper current rating)
    stepper_adc_scale_ = ((ADC_REF_VOLT * 256000UL) /
                          (SHUNT_RES * CURR_GAIN * stepper_specs_.motor_current_mA)) << (16 - ADC_RES);

    // PWM Period for stepper
    stepper_pwm_period_ = (SYS_CLK * 500) / stepper_specs_.switching_freq_kHz;

    // Rate count for stepper: ANGLE_MAX / (microstep_res * 4)
    stepper_rate_count_ = ANGLE_MAX / (stepper_specs_.microstep_resolution * 4);

    // Slew count calculation
    // g_slew_count_st = (240 * 1000000) / (speed_rpm * microsteps * 4 * step_num)
    if (stepper_params_.speed_rpm > 0) {
        stepper_slew_count_ = (240UL * 1000000UL) /
                              (stepper_params_.speed_rpm * stepper_specs_.microstep_resolution *
                               4 * stepper_specs_.step_number);
    } else {
        stepper_slew_count_ = 0xF0;  // Default
    }
}

void Motor::init() {
    if (!is_valid()) {
        std::cerr << "Motor::init(): device(s) missing; aborting init\n";
        return;
    }

    if (type_ == MotorType::BLDC) {
        calculate_bldc_constants();
        init_bldc();
    } else {
        calculate_stepper_constants();
        init_stepper();
    }
}

void Motor::start() {
    if (!is_valid()) {
        std::cerr << "Motor::start(): device(s) missing; aborting\n";
        return;
    }
    if (type_ == MotorType::BLDC) {
        sqmn_.write(SQMNGDriver::START_MOTOR_BLDC, "1");
    } else {
        sqmn_.write(SQMNGDriver::START_MOTOR_STEPPER, "1");
    }
}

void Motor::stop() {
    if (!is_valid()) {
        std::cerr << "Motor::stop(): device(s) missing; aborting\n";
        return;
    }
    if (type_ == MotorType::BLDC) {
        sqmn_.write(SQMNGDriver::STOP_MOTOR_BLDC, "1");
    } else {
        sqmn_.write(SQMNGDriver::STOP_MOTOR_STEPPER, "1");
    }
}

void Motor::clear_fault() {
    if (!is_valid()) {
        std::cerr << "Motor::clear_fault(): device(s) missing; aborting\n";
        return;
    }
    if (type_ == MotorType::BLDC) {
        sqmn_.write(SQMNGDriver::START_MOTOR_BLDC, "0");
        sqmn_.write(SQMNGDriver::STOP_MOTOR_BLDC, "0");
        sqmn_.write(SQMNGDriver::CLR_FAULT_BLDC, "1");
        sqmn_.write(SQMNGDriver::CLR_FAULT_BLDC, "0");
    } else {
        sqmn_.write(SQMNGDriver::START_MOTOR_STEPPER, "0");
        sqmn_.write(SQMNGDriver::STOP_MOTOR_STEPPER, "0");
        sqmn_.write(SQMNGDriver::CLR_FAULT_STEPPER, "1");
        sqmn_.write(SQMNGDriver::CLR_FAULT_STEPPER, "0");
    }
}

void Motor::set_direction(int dir) {
    if (!is_valid()) {
        std::cerr << "Motor::set_direction(): device(s) missing; aborting\n";
        return;
    }

    if (type_ == MotorType::BLDC) {
        if (dir != 0 && dir != 1) {
            std::cerr << "Invalid BLDC direction value " << dir << ", expected 0 or 1\n";
            return;
        }
        std::string dir_val = (dir == 0) ? "0" : "1";
        rlimit_.write(RateLimit::DIRECTION_CONFIG_BLDC, dir_val);
    } else {
        // Stepper: modify sign bit in CMD_STEP register
        std::optional<std::string> cmd_val_opt = theta_.read(STPTheta::CMD_STEP);
        if (!cmd_val_opt.has_value()) {
            std::cerr << "Failed to read CMD_STEP value.\n";
            return;
        }

        int cmd_int;
        try {
            cmd_int = std::stoi(*cmd_val_opt, nullptr, 0);
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse CMD_STEP value '" << *cmd_val_opt << "': " << e.what() << "\n";
            return;
        }

        if (dir == 0) {
            cmd_int &= CMD_STEP_VALUE_MASK;
        } else if (dir == 1) {
            cmd_int |= CMD_STEP_SIGN_BIT;
        } else {
            std::cerr << "Invalid direction value " << dir << ", expected 0 or 1\n";
            return;
        }

        std::stringstream ss;
        ss << "0x" << std::hex << std::uppercase
           << std::setw(6) << std::setfill('0')
           << (cmd_int & CMD_STEP_FULL_MASK);
        theta_.write(STPTheta::CMD_STEP, ss.str());
    }
}

void Motor::set_direction(const std::string& dir) {
    // Convert string to integer direction and call the int overload
    // Accepts: "CLOCKWISE", "CW", "0" for clockwise
    //          "COUNTER CLOCKWISE", "CCW", "COUNTER-CLOCKWISE", "1" for counter-clockwise
    int dir_val;
    if (dir == DIRECTION_CW_STR || dir == "CW" || dir == "0" || dir == "CLOCKWISE") {
        dir_val = DIRECTION_CLOCKWISE;
    } else if (dir == DIRECTION_CCW_STR || dir == "CCW" || dir == "1" ||
               dir == "COUNTER-CLOCKWISE" || dir == "COUNTER CLOCKWISE") {
        dir_val = DIRECTION_COUNTER_CLOCKWISE;
    } else {
        std::cerr << "Motor::set_direction(): invalid direction string '" << dir
                  << "', use 'CLOCKWISE'/'CW'/0 or 'COUNTER CLOCKWISE'/'CCW'/1\n";
        return;
    }
    set_direction(dir_val);
}

int Motor::set_speed(int speed) {
    if (!is_valid()) {
        std::cerr << "Motor::set_speed(): device(s) missing; aborting\n";
        return -1;
    }

    if (type_ == MotorType::BLDC) {
        if (speed < BLDC_SPEED_MIN || speed > BLDC_SPEED_MAX) {
            std::cerr << "BLDC speed " << speed << " out of range ["
                      << BLDC_SPEED_MIN << ", " << BLDC_SPEED_MAX << "]\n";
            return -1;
        }

        // Scale speed and write to rate limit reference
        // Based on: temp_scl = speed_ref * g_speed_scale / TWO_POWER_10
        uint32_t scaled_speed = (speed * bldc_speed_scale_) / TWO_POWER_10;
        std::stringstream ss;
        ss << "0x" << std::hex << std::uppercase
           << std::setw(6) << std::setfill('0')
           << (scaled_speed & CMD_STEP_FULL_MASK);
        rlimit_.write(RateLimit::RATE_LIMIT_REF_BLDC, ss.str());

        // Update params structure so GET reflects current speed
        // Note: BLDC doesn't have a speed_rpm in params, slider controls target speed directly
        return 0;
    } else {
        if (speed < STEPPER_SPEED_MIN || speed > STEPPER_SPEED_MAX) {
            std::cerr << "Stepper speed 0x" << std::hex << speed
                      << " out of range [0x" << STEPPER_SPEED_MIN
                      << ", 0x" << STEPPER_SPEED_MAX << "]\n";
            return -1;
        }

        std::stringstream ss;
        ss << "0x" << std::hex << std::uppercase
           << std::setw(6) << std::setfill('0')
           << (speed & CMD_STEP_FULL_MASK);
        theta_.write(STPTheta::SLEW_CNT, ss.str());

        // Reverse-calculate RPM from SLEW_CNT and update params structure
        // Formula: RPM = (240 * 1000000) / (slew_cnt * microsteps * 4 * steps_per_rev)
        if (speed > 0 && stepper_specs_.microstep_resolution > 0 && stepper_specs_.step_number > 0) {
            uint32_t divisor = speed * stepper_specs_.microstep_resolution * 4 * stepper_specs_.step_number;
            stepper_params_.speed_rpm = (240UL * 1000000UL) / divisor;
        }
        return 0;
    }
}

int Motor::get_state() {
    if (!is_valid()) {
        std::cerr << "Motor::get_state(): device(s) missing; returning -1\n";
        return -1;
    }

    std::optional<std::string> state_str;
    if (type_ == MotorType::BLDC) {
        state_str = sqmn_.read(SQMNGDriver::SEQ_STATE_BLDC);
    } else {
        state_str = sqmn_.read(SQMNGDriver::SEQ_STATE_STEPPER);
    }

    if (!state_str.has_value()) {
        std::cerr << "Failed to read sequencer state\n";
        return -1;
    }

    try {
        return std::stoi(*state_str, nullptr, 0);
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse sequencer state '" << *state_str << "': " << e.what() << "\n";
        return -1;
    }
}

int Motor::get_speed() {
    // Returns -1 as placeholder (speed is read via IIO buffer)
    return -1;
}

std::optional<int> Motor::get_bldc_speed_raw() {
    if (!is_valid() || type_ != MotorType::BLDC) {
        return std::nullopt;
    }

    auto speed_str = adc_.read(ADC::SPEED_BLDC);
    if (!speed_str.has_value()) {
        return std::nullopt;
    }

    try {
        return std::stoi(*speed_str, nullptr, 0);
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<int> Motor::get_bldc_speed() {
    if (!is_valid() || type_ != MotorType::BLDC) {
        return std::nullopt;
    }

    auto speed_str = adc_.read(ADC::SPEED_BLDC);
    if (!speed_str.has_value()) {
        return std::nullopt;
    }

    try {
        // Read raw speed as unsigned 32-bit value first
        uint32_t raw_unsigned = static_cast<uint32_t>(std::stoul(*speed_str, nullptr, 0));

        // The speed from hardware is an 18-bit signed value (2's complement)
        // Bit 17 is sign bit, bits 16:0 are magnitude
        constexpr uint32_t SPEED_18BIT_SIGN = (1U << 17);      // 0x20000
        constexpr uint32_t SPEED_18BIT_MASK = 0x3FFFF;         // 18 bits

        // Mask to 18 bits
        uint32_t speed_18bit = raw_unsigned & SPEED_18BIT_MASK;

        int32_t signed_speed;
        if (speed_18bit & SPEED_18BIT_SIGN) {
            // Negative in 18-bit 2's complement - sign extend to 32-bit
            signed_speed = static_cast<int32_t>(speed_18bit | 0xFFFC0000);
        } else {
            signed_speed = static_cast<int32_t>(speed_18bit);
        }

        // Get absolute magnitude
        int32_t abs_speed = (signed_speed < 0) ? -signed_speed : signed_speed;

        // The raw speed from hardware is in estimator units, needs /16 to get base RPM
        // Then apply scale factor based on motor specs (same formula as UI):
        // Combined scale = (current_poles / base_poles) * (current_motor_rpm / base_motor_rpm)
        // Base calibration: 4 poles, 4000 RPM
        constexpr int BASE_POLES = 4;
        constexpr int BASE_MOTOR_RPM = 4000;

        double pole_scale = static_cast<double>(bldc_specs_.pole_pairs) / BASE_POLES;
        double rpm_scale = static_cast<double>(bldc_specs_.motor_speed_rpm) / BASE_MOTOR_RPM;
        double combined_scale = pole_scale * rpm_scale;

        // Divide by BLDC_SPEED_SCALE (16) to convert from estimator units to RPM
        // Then apply the pole/rpm scaling
        int scaled_rpm = static_cast<int>((abs_speed / static_cast<double>(BLDC_SPEED_SCALE)) * combined_scale);
        return scaled_rpm;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<std::string> Motor::get_bldc_direction() {
    if (!is_valid() || type_ != MotorType::BLDC) {
        return std::nullopt;
    }

    auto dir_str = rlimit_.read(RateLimit::DIRECTION_CONFIG_BLDC);
    if (!dir_str.has_value()) {
        return std::nullopt;
    }

    try {
        int dir_val = std::stoi(*dir_str, nullptr, 0) & 0x1;
        // Direction mapping from UI: 0 = CLOCKWISE, 1 = COUNTER CLOCKWISE
        return (dir_val == 0) ? "CLOCKWISE" : "COUNTER CLOCKWISE";
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<int> Motor::get_bldc_iq() {
    if (!is_valid() || type_ != MotorType::BLDC) {
        return std::nullopt;
    }

    auto iq_str = adc_.read(ADC::IQ_BLDC);
    if (!iq_str.has_value()) {
        return std::nullopt;
    }

    try {
        return std::stoi(*iq_str, nullptr, 0);
    } catch (...) {
        return std::nullopt;
    }
}

// ===== BLDC Specs/Params Setters =====

void Motor::set_bldc_specs(const BLDCSpecs& specs) {
    bldc_specs_ = specs;
}

void Motor::set_bldc_params(const BLDCParams& params) {
    bldc_params_ = params;
}

// ===== Stepper Specs/Params Setters =====

void Motor::set_stepper_specs(const StepperSpecs& specs) {
    stepper_specs_ = specs;
}

void Motor::set_stepper_params(const StepperParams& params) {
    stepper_params_ = params;
}

// Apply parameters to hardware
void Motor::apply_parameters() {
    if (!is_valid()) {
        std::cerr << "Motor::apply_parameters(): device(s) missing; aborting\n";
        return;
    }

    if (type_ == MotorType::BLDC) {
        calculate_bldc_constants();
        write_bldc_registers();
    } else {
        calculate_stepper_constants();
        write_stepper_registers();
    }
}

// Helper to format hex value
static std::string to_hex(uint32_t val) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::uppercase << val;
    return ss.str();
}

void Motor::write_bldc_registers() {
    // Use calculated values from specs where applicable
    // PI gains and sensorless params are from GUI

    // ADC parameters - calculated from motor current specs
    adc_.write(ADC::ADC_SCALE_BLDC, to_hex(bldc_adc_scale_));
    adc_.write(ADC::OC_THRE_BLDC, "0x18000");  // Overcurrent threshold - keep fixed

    // PWM parameters - calculated from switching frequency
    pwmdrive_.write(PWMDriver::PWM_PERIOD_VAL_BLDC, to_hex(bldc_pwm_period_));
    pwmdrive_.write(PWMDriver::DEAD_TIME_BLDC, "0xA");  // Dead time - keep fixed for safety
    pwmdrive_.write(PWMDriver::PWM_GAIN_BLDC, to_hex(bldc_pwm_gain_));

    // PI Controller parameters - from GUI (user can tune these)
    picon_.write(PIController::SPEED_PI_KP_BLDC, to_hex(bldc_params_.speed_pi_kp));
    picon_.write(PIController::SPEED_PI_KI_BLDC, to_hex(bldc_params_.speed_pi_ki));
    picon_.write(PIController::IDQ_KP_BLDC, to_hex(bldc_params_.current_pi_kp));
    picon_.write(PIController::IDQ_KI_BLDC, to_hex(bldc_params_.current_pi_ki));

    // Rate limit parameters - some hardcoded, some calculated from specs
    // NOTE: Don't reset RATE_LIMIT_REF_BLDC here - preserve current speed setting
    // The speed is set separately via set_bldc_speed() command
    rlimit_.write(RateLimit::RATE_LIMIT_SLEW_CNT_BLDC, to_hex(bldc_slew_count_));
    rlimit_.write(RateLimit::RATE_LIMIT_RATE_CNT_BLDC, to_hex(bldc_rate_count_));
    rlimit_.write(RateLimit::DIRECTION_CONFIG_BLDC, "0x1");
    // Theta factor depends on pole_pairs and motor_speed_rpm - use calculated value
    rlimit_.write(RateLimit::SQMNG_THETA_FACTOR_BLDC, to_hex(bldc_theta_factor_));

    // Sequence controller config - from GUI flags
    uint8_t seq_config = build_bldc_seq_config();
    sqmn_.write(SQMNGDriver::SEQ_CTL_BLDC, to_hex(seq_config));

    // Sensorless parameters - from GUI (user can tune these)
    // Open loop voltage: (open_loop_voltage_pct * 65535) / 100
    uint32_t ol_dv = (bldc_params_.open_loop_voltage_pct * 65535UL) / 100;
    rlimit_.write(RateLimit::SQMNG_DV_BLDC, to_hex(ol_dv));

    // Open loop current: (open_loop_current_pct * 65535) / 100
    uint32_t ol_iq_ref = (bldc_params_.open_loop_current_pct * 65535UL) / 100;
    rlimit_.write(RateLimit::SQMNG_IQ_REF_BLDC, to_hex(ol_iq_ref));

    // Closed loop omega: (closed_loop_speed_rpm * 65535) / motor_speed_rpm
    uint32_t cl_omega = (bldc_params_.closed_loop_speed_rpm * 65535UL) / bldc_specs_.motor_speed_rpm;
    sqmn_.write(SQMNGDriver::SQMNG_CL_OMEGA_BLDC, to_hex(cl_omega));

    sqmn_.write(SQMNGDriver::NUM_AUTO_RESTARTS_BLDC, "0x4");

    // Clear faults
    sqmn_.write(SQMNGDriver::CLR_FAULT_BLDC, "0");
}

void Motor::write_stepper_registers() {
    // Use hardcoded base values for low-level registers (proven working values)
    // Only user-changeable parameters (PI gains, speed, steps) are from GUI

    // ADC parameters - hardcoded
    adc_.write(ADC::ADC_SCALE_STEPPER, "0x6600");
    adc_.write(ADC::OC_THRE_STEPPER, "0x18000");

    // PWM parameters - hardcoded
    pwmdrive_.write(PWMDriver::PWM_PERIOD_VAL_STEPPER, "0x9C4");
    pwmdrive_.write(PWMDriver::DEAD_TIME_STEPPER, "0xA");
    pwmdrive_.write(PWMDriver::PWM_GAIN_STEPPER, "0xB46");

    // PI Controller parameters - from GUI (user can tune these)
    picon_.write(PIController::IDQ_KP_STEPPER, to_hex(stepper_params_.current_pi_kp));
    picon_.write(PIController::IDQ_KI_STEPPER, to_hex(stepper_params_.current_pi_ki));

    // Rate limit parameters - hardcoded base values
    rlimit_.write(RateLimit::RATE_LIMIT_REF_STEPPER, "0x7FFF");
    rlimit_.write(RateLimit::RATE_LIMIT_SLEW_CNT_STEPPER, "0xC8");
    rlimit_.write(RateLimit::RATE_LIMIT_RATE_CNT_STEPPER, "0xA3");
    rlimit_.write(RateLimit::DIRECTION_CONFIG_STEPPER, "0x1");
    rlimit_.write(RateLimit::SQMNG_THETA_FACTOR_STEPPER, "0x1BF");

    // Sequence controller config
    sqmn_.write(SQMNGDriver::SEQ_CTL_STEPPER, "0x5");
    rlimit_.write(RateLimit::SQMNG_DV_STEPPER, "0xCCD");
    sqmn_.write(SQMNGDriver::SQMNG_CL_OMEGA_STEPPER, "0x199A");
    sqmn_.write(SQMNGDriver::NUM_AUTO_RESTARTS_STEPPER, "0x4");

    // Stepper theta generator - speed from GUI params
    // SLEW_CNT formula: higher value = slower speed (inverse relationship)
    // SLEW_CNT = (240 * 1000000) / (speed_rpm * microsteps * 4 * steps_per_rev)
    // With defaults (64 microsteps, 200 steps): SLEW_CNT ≈ 4687 / speed_rpm
    uint32_t slew_cnt;
    if (stepper_params_.speed_rpm > 0) {
        slew_cnt = (240UL * 1000000UL) /
                   (stepper_params_.speed_rpm * stepper_specs_.microstep_resolution *
                    4 * stepper_specs_.step_number);
        // Clamp to valid range
        if (slew_cnt < 0x20) slew_cnt = 0x20;  // Min (fastest)
        if (slew_cnt > 0xFF) slew_cnt = 0xFF;  // Max (slowest)
    } else {
        slew_cnt = 0x4F;  // Default
    }
    theta_.write(STPTheta::SLEW_CNT, to_hex(slew_cnt));
    theta_.write(STPTheta::RATE_LIMIT, "0x400");

    // CMD_STEP with direction (sign bit) and step count
    int32_t cmd_step_val = stepper_params_.cmd_steps;
    uint32_t cmd_step_reg;
    if (cmd_step_val < 0) {
        cmd_step_reg = (CMD_STEP_SIGN_BIT | ((-cmd_step_val) & CMD_STEP_VALUE_MASK));
    } else {
        cmd_step_reg = cmd_step_val & CMD_STEP_VALUE_MASK;
    }

    std::stringstream ss;
    ss << "0x" << std::hex << std::uppercase
       << std::setw(6) << std::setfill('0')
       << (cmd_step_reg & CMD_STEP_FULL_MASK);
    theta_.write(STPTheta::CMD_STEP, ss.str());

    // Current reference for stepper: (current_ref_mA * 65535) / motor_current_mA
    uint32_t iq_ref_val = (stepper_params_.current_reference_mA * 65535UL) / stepper_specs_.motor_current_mA;
    rlimit_.write(RateLimit::SQMNG_IQ_REF_STEPPER, to_hex(iq_ref_val));

    // Clear faults
    sqmn_.write(SQMNGDriver::CLR_FAULT_STEPPER, "0");
}

void Motor::init_bldc() {
    // Use exact hardcoded values from working original version
    // These are the proven values that work - do NOT compute them
    adc_.write(ADC::ADC_SCALE_BLDC, "0x6600");
    adc_.write(ADC::OC_THRE_BLDC, "0x18000");

    pwmdrive_.write(PWMDriver::PWM_PERIOD_VAL_BLDC, "0x9C4");
    pwmdrive_.write(PWMDriver::DEAD_TIME_BLDC, "0xA");
    pwmdrive_.write(PWMDriver::PWM_GAIN_BLDC, "0xB46");

    picon_.write(PIController::SPEED_PI_KP_BLDC, "0xBB8");
    picon_.write(PIController::SPEED_PI_KI_BLDC, "0x0A");
    picon_.write(PIController::IDQ_KP_BLDC, "0x7C");
    picon_.write(PIController::IDQ_KI_BLDC, "0x113");

    rlimit_.write(RateLimit::RATE_LIMIT_REF_BLDC, "0x7FFF");
    rlimit_.write(RateLimit::RATE_LIMIT_SLEW_CNT_BLDC, "0xC8");
    rlimit_.write(RateLimit::RATE_LIMIT_RATE_CNT_BLDC, "0xA3");
    rlimit_.write(RateLimit::DIRECTION_CONFIG_BLDC, "0x1");
    rlimit_.write(RateLimit::SQMNG_THETA_FACTOR_BLDC, "0x1BF");

    sqmn_.write(SQMNGDriver::SEQ_CTL_BLDC, "0x5");
    rlimit_.write(RateLimit::SQMNG_DV_BLDC, "0xCCD");
    sqmn_.write(SQMNGDriver::SQMNG_CL_OMEGA_BLDC, "0x199A");
    rlimit_.write(RateLimit::SQMNG_IQ_REF_BLDC, "0x8000");
    sqmn_.write(SQMNGDriver::NUM_AUTO_RESTARTS_BLDC, "0x4");

    sqmn_.write(SQMNGDriver::CLR_FAULT_BLDC, "0");
}

void Motor::init_stepper() {
    // Use exact hardcoded values from working original version
    adc_.write(ADC::ADC_SCALE_STEPPER, "0x6600");
    adc_.write(ADC::OC_THRE_STEPPER, "0x18000");

    pwmdrive_.write(PWMDriver::PWM_PERIOD_VAL_STEPPER, "0x9C4");
    pwmdrive_.write(PWMDriver::DEAD_TIME_STEPPER, "0xA");
    pwmdrive_.write(PWMDriver::PWM_GAIN_STEPPER, "0xB46");

    picon_.write(PIController::SPEED_PI_KP_STEPPER, "0xBB8");
    picon_.write(PIController::SPEED_PI_KI_STEPPER, "0x0A");
    picon_.write(PIController::IDQ_KP_STEPPER, "0x7C");
    picon_.write(PIController::IDQ_KI_STEPPER, "0x113");

    rlimit_.write(RateLimit::RATE_LIMIT_REF_STEPPER, "0x7FFF");
    rlimit_.write(RateLimit::RATE_LIMIT_SLEW_CNT_STEPPER, "0xC8");
    rlimit_.write(RateLimit::RATE_LIMIT_RATE_CNT_STEPPER, "0xA3");
    rlimit_.write(RateLimit::DIRECTION_CONFIG_STEPPER, "0x1");
    rlimit_.write(RateLimit::SQMNG_THETA_FACTOR_STEPPER, "0x1BF");

    sqmn_.write(SQMNGDriver::SEQ_CTL_STEPPER, "0x5");
    rlimit_.write(RateLimit::SQMNG_DV_STEPPER, "0xCCD");
    sqmn_.write(SQMNGDriver::SQMNG_CL_OMEGA_STEPPER, "0x199A");
    rlimit_.write(RateLimit::SQMNG_IQ_REF_STEPPER, "0x8000");
    sqmn_.write(SQMNGDriver::NUM_AUTO_RESTARTS_STEPPER, "0x4");

    sqmn_.write(SQMNGDriver::CLR_FAULT_STEPPER, "0");

    // STPTheta-specific writes (stepper-only)
    theta_.write(STPTheta::SLEW_CNT, "0x4F");
    theta_.write(STPTheta::RATE_LIMIT, "0x400");
    theta_.write(STPTheta::CMD_STEP, "0xFFE700");
}

} // namespace motorcontrol
