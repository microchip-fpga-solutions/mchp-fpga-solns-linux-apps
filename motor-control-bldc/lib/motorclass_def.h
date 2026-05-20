// SPDX-License-Identifier: MIT
/*
 * @file motorclass_def.h
 * @brief PolarFire SoC Motor Control IIO device sysfs interface definitions
 * Copyright (C) 2026 Microchip Technology Inc. and its subsidiaries
 */
#ifndef MOTOR_HARDWARE_H
#define MOTOR_HARDWARE_H

#include <string>
#include <string_view>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

namespace motorcontrol {

// Device names for sysfs path resolution
constexpr std::string_view MOTOR_ADC_NAME      = "mpfs_mc_adc";
constexpr std::string_view MOTOR_PICON_NAME    = "mpfs_mc_picon";
constexpr std::string_view MOTOR_PWM_NAME      = "mpfs_mc_pwm";
constexpr std::string_view MOTOR_RTLIMIT_NAME  = "mpfs_mc_ratelim";
constexpr std::string_view MOTOR_SQMNG_NAME    = "mpfs_mc_sqmng";
constexpr std::string_view MOTOR_STPTHETA_NAME = "mpfs_mc_stptheta";

// Helper function: Find sysfs device path by device name
inline std::optional<std::string> findDevicePathByName(std::string_view target_name) {
    const std::string base_path = "/sys/bus/iio/devices/";

    try {
        if (!std::filesystem::exists(base_path)) {
            std::cerr << "IIO devices path does not exist: " << base_path << "\n";
            return std::nullopt;
        }

        for (const auto& entry : std::filesystem::directory_iterator(base_path)) {
            if (!entry.is_directory()) continue;

            auto path = entry.path();
            auto name_file = path / "name";

            std::ifstream file(name_file);
            if (!file) continue;

            std::string device_name;
            std::getline(file, device_name);
            if (device_name == target_name) {
                return path.string();
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error while searching for device: " << e.what() << "\n";
        return std::nullopt;
    }

    return std::nullopt;
}

// Base class for common read/write helpers
class SysfsReg {
protected:
    std::string device_path;

    explicit SysfsReg(std::string dev_path) : device_path(std::move(dev_path)) {}

    bool writeReg(const std::string& reg, const std::string& value) {
        std::string filepath = device_path + "/" + reg;
        std::ofstream file(filepath);
        if (!file) {
            std::cerr << "Failed to open " << filepath << " for writing\n";
            return false;
        }
        file << value << std::flush;
        return file.good();
    }

    std::optional<std::string> readReg(const std::string& reg) {
        std::string filepath = device_path + "/" + reg;
        std::ifstream file(filepath);
        if (!file) {
            std::cerr << "Failed to open " << filepath << " for reading\n";
            return std::nullopt;
        }
        std::string value;
        std::getline(file, value);
        if (!file.good() && !file.eof()) {
            std::cerr << "Error reading " << filepath << "\n";
            return std::nullopt;
        }
        return value;
    }
};

// ADC driver registers and class
class ADC : public SysfsReg {
public:
    // Sysfs register names - Stepper
    static constexpr std::string_view ADC_ADDR_STEPPER  = "adc_addr_stepper";
    static constexpr std::string_view CH0_VAL_STEPPER   = "ch0_val_stepper";
    static constexpr std::string_view CH1_VAL_STEPPER   = "ch1_val_stepper";
    static constexpr std::string_view CH2_VAL_STEPPER   = "ch2_val_stepper";
    static constexpr std::string_view CH3_VAL_STEPPER   = "ch3_val_stepper";
    static constexpr std::string_view ADC_SCALE_STEPPER = "adc_scale_stepper";
    static constexpr std::string_view OC_THRE_STEPPER   = "oc_thre_stepper";
    static constexpr std::string_view ADC_IB_STEPPER    = "adc_ib_stepper";
    static constexpr std::string_view ADC_IA_STEPPER    = "adc_ia_stepper";

    // Sysfs register names - BLDC
    static constexpr std::string_view ADC_ADDR_BLDC  = "adc_addr_bldc";
    static constexpr std::string_view CH0_VAL_BLDC   = "ch0_val_bldc";
    static constexpr std::string_view CH1_VAL_BLDC   = "ch1_val_bldc";
    static constexpr std::string_view CH2_VAL_BLDC   = "ch2_val_bldc";
    static constexpr std::string_view CH3_VAL_BLDC   = "ch3_val_bldc";
    static constexpr std::string_view ADC_SCALE_BLDC = "adc_scale_bldc";
    static constexpr std::string_view OC_THRE_BLDC   = "oc_thre_bldc";
    static constexpr std::string_view ADC_IB_BLDC    = "adc_ib_bldc";
    static constexpr std::string_view ADC_IA_BLDC    = "adc_ia_bldc";
    static constexpr std::string_view SPEED_BLDC     = "speed_bldc";
    static constexpr std::string_view IQ_BLDC        = "iq_bldc";

    explicit ADC(std::string_view device_name = MOTOR_ADC_NAME)
        : SysfsReg(findDevicePathByName(device_name).value_or("")) {}

    bool isValid() const { return !device_path.empty(); }

    bool write(std::string_view reg, const std::string& value) {
        return writeReg(std::string(reg), value);
    }

    std::optional<std::string> read(std::string_view reg) {
        return readReg(std::string(reg));
    }
};

// PI Controller driver registers and class
class PIController : public SysfsReg {
public:
    // Stepper registers
    static constexpr std::string_view SPEED_PI_KP_STEPPER  = "speed_pi_kp_stepper";
    static constexpr std::string_view SPEED_PI_KI_STEPPER  = "speed_pi_ki_stepper";
    static constexpr std::string_view IDQ_KP_STEPPER       = "idq_kp_stepper";
    static constexpr std::string_view IDQ_KI_STEPPER       = "idq_ki_stepper";
    static constexpr std::string_view SPEED_PI_STEPPER     = "speed_pi_stepper";
    static constexpr std::string_view SPEED_ID_PI_STEPPER  = "speed_id_pi_stepper";
    static constexpr std::string_view SPEED_IQ_PI_STEPPER  = "speed_iq_pi_stepper";

    // BLDC registers
    static constexpr std::string_view SPEED_PI_KP_BLDC  = "speed_pi_kp_bldc";
    static constexpr std::string_view SPEED_PI_KI_BLDC  = "speed_pi_ki_bldc";
    static constexpr std::string_view IDQ_KP_BLDC       = "idq_kp_bldc";
    static constexpr std::string_view IDQ_KI_BLDC       = "idq_ki_bldc";
    static constexpr std::string_view SPEED_PI_BLDC     = "speed_pi_bldc";
    static constexpr std::string_view SPEED_ID_PI_BLDC  = "speed_id_pi_bldc";
    static constexpr std::string_view SPEED_IQ_PI_BLDC  = "speed_iq_pi_bldc";

    explicit PIController(std::string_view device_name = MOTOR_PICON_NAME)
        : SysfsReg(findDevicePathByName(device_name).value_or("")) {}

    bool isValid() const { return !device_path.empty(); }

    bool write(std::string_view reg, const std::string& value) {
        return writeReg(std::string(reg), value);
    }

    std::optional<std::string> read(std::string_view reg) {
        return readReg(std::string(reg));
    }
};

// PWM driver registers and class
class PWMDriver : public SysfsReg {
public:
    // Stepper registers
    static constexpr std::string_view PWM_PERIOD_VAL_STEPPER = "pwm_period_val_stepper";
    static constexpr std::string_view DEAD_TIME_STEPPER      = "dead_time_stepper";
    static constexpr std::string_view DELAY_TIME_STEPPER     = "delay_time_stepper";
    static constexpr std::string_view PWM_GAIN_STEPPER       = "pwm_gain_stepper";

    // BLDC registers
    static constexpr std::string_view PWM_PERIOD_VAL_BLDC = "pwm_period_val_bldc";
    static constexpr std::string_view DEAD_TIME_BLDC      = "dead_time_bldc";
    static constexpr std::string_view DELAY_TIME_BLDC     = "delay_time_bldc";
    static constexpr std::string_view PWM_GAIN_BLDC       = "pwm_gain_bldc";

    explicit PWMDriver(std::string_view device_name = MOTOR_PWM_NAME)
        : SysfsReg(findDevicePathByName(device_name).value_or("")) {}

    bool isValid() const { return !device_path.empty(); }

    bool write(std::string_view reg, const std::string& value) {
        return writeReg(std::string(reg), value);
    }

    std::optional<std::string> read(std::string_view reg) {
        return readReg(std::string(reg));
    }
};

// Rate Limit driver registers and class
class RateLimit : public SysfsReg {
public:
    // Stepper registers
    static constexpr std::string_view RATE_LIMIT_REF_STEPPER      = "rate_limit_ref_stepper";
    static constexpr std::string_view RATE_LIMIT_SLEW_CNT_STEPPER = "rate_limit_slew_cnt_stepper";
    static constexpr std::string_view RATE_LIMIT_RATE_CNT_STEPPER = "rate_limit_rate_cnt_stepper";
    static constexpr std::string_view DIRECTION_CONFIG_STEPPER    = "direction_config_stepper";
    static constexpr std::string_view SQMNG_DV_STEPPER            = "sqmng_dv_stepper";
    static constexpr std::string_view SQMNG_IQ_REF_STEPPER        = "sqmng_iq_ref_stepper";
    static constexpr std::string_view SQMNG_THETA_FACTOR_STEPPER  = "sqmng_theta_factor_stepper";
    static constexpr std::string_view OLMNG_THETA_STEPPER         = "olmng_theta_stepper";
    static constexpr std::string_view RLIMIT_RATE_STEPPER         = "rlimit_rate_stepper";

    // BLDC registers
    static constexpr std::string_view RATE_LIMIT_REF_BLDC      = "rate_limit_ref_bldc";
    static constexpr std::string_view RATE_LIMIT_SLEW_CNT_BLDC = "rate_limit_slew_cnt_bldc";
    static constexpr std::string_view RATE_LIMIT_RATE_CNT_BLDC = "rate_limit_rate_cnt_bldc";
    static constexpr std::string_view DIRECTION_CONFIG_BLDC    = "direction_config_bldc";
    static constexpr std::string_view SQMNG_DV_BLDC            = "sqmng_dv_bldc";
    static constexpr std::string_view SQMNG_IQ_REF_BLDC        = "sqmng_iq_ref_bldc";
    static constexpr std::string_view SQMNG_THETA_FACTOR_BLDC  = "sqmng_theta_factor_bldc";
    static constexpr std::string_view OLMNG_THETA_BLDC         = "olmng_theta_bldc";
    static constexpr std::string_view RLIMIT_RATE_BLDC         = "rlimit_rate_bldc";

    explicit RateLimit(std::string_view device_name = MOTOR_RTLIMIT_NAME)
        : SysfsReg(findDevicePathByName(device_name).value_or("")) {}

    bool isValid() const { return !device_path.empty(); }

    bool write(std::string_view reg, const std::string& value) {
        return writeReg(std::string(reg), value);
    }

    std::optional<std::string> read(std::string_view reg) {
        return readReg(std::string(reg));
    }
};

// SQMNG driver registers and class
class SQMNGDriver : public SysfsReg {
public:
    // Stepper registers
    static constexpr std::string_view START_MOTOR_STEPPER       = "start_motor_stepper";
    static constexpr std::string_view SEQ_CTL_STEPPER           = "seq_ctl_stepper";
    static constexpr std::string_view SQMNG_CL_OMEGA_STEPPER    = "sqmng_cl_omega_stepper";
    static constexpr std::string_view STOP_MOTOR_STEPPER        = "stop_motor_stepper";
    static constexpr std::string_view NUM_AUTO_RESTARTS_STEPPER = "num_auto_restarts_stepper";
    static constexpr std::string_view CLR_FAULT_STEPPER         = "clr_fault_stepper";
    static constexpr std::string_view SELECTOR_CHA_STEPPER      = "selector_cha_stepper";
    static constexpr std::string_view FSM_DEBUG_STEPPER         = "fsm_debug_stepper";
    static constexpr std::string_view SEQ_STATE_STEPPER         = "seq_state_stepper";

    // BLDC registers
    static constexpr std::string_view START_MOTOR_BLDC       = "start_motor_bldc";
    static constexpr std::string_view SEQ_CTL_BLDC           = "seq_ctl_bldc";
    static constexpr std::string_view SQMNG_CL_OMEGA_BLDC    = "sqmng_cl_omega_bldc";
    static constexpr std::string_view STOP_MOTOR_BLDC        = "stop_motor_bldc";
    static constexpr std::string_view NUM_AUTO_RESTARTS_BLDC = "num_auto_restarts_bldc";
    static constexpr std::string_view CLR_FAULT_BLDC         = "clr_fault_bldc";
    static constexpr std::string_view SELECTOR_CHA_BLDC      = "selector_cha_bldc";
    static constexpr std::string_view FSM_DEBUG_BLDC         = "fsm_debug_bldc";
    static constexpr std::string_view SEQ_STATE_BLDC         = "seq_state_bldc";

    explicit SQMNGDriver(std::string_view device_name = MOTOR_SQMNG_NAME)
        : SysfsReg(findDevicePathByName(device_name).value_or("")) {}

    bool isValid() const { return !device_path.empty(); }

    bool write(std::string_view reg, const std::string& value) {
        return writeReg(std::string(reg), value);
    }

    std::optional<std::string> read(std::string_view reg) {
        return readReg(std::string(reg));
    }
};

// STPTheta driver registers and class
class STPTheta : public SysfsReg {
public:
    static constexpr std::string_view SLEW_CNT   = "slew_cnt";
    static constexpr std::string_view RATE_LIMIT = "rate_limit";
    static constexpr std::string_view CMD_STEP   = "cmd_step";

    explicit STPTheta(std::string_view device_name = MOTOR_STPTHETA_NAME)
        : SysfsReg(findDevicePathByName(device_name).value_or("")) {}

    bool isValid() const { return !device_path.empty(); }

    bool write(std::string_view reg, const std::string& value) {
        return writeReg(std::string(reg), value);
    }

    std::optional<std::string> read(std::string_view reg) {
        return readReg(std::string(reg));
    }
};

} // namespace motorcontrol

#endif // MOTOR_HARDWARE_H
