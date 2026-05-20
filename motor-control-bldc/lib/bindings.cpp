// SPDX-License-Identifier: MIT
/*
 * @file bindings.cpp
 * @brief PolarFire SoC Motor Control Library - Python bindings using pybind11
 * Copyright (C) 2026 Microchip Technology Inc. and its subsidiaries
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "motor.h"

namespace py = pybind11;
using namespace motorcontrol;

PYBIND11_MODULE(motor_lib_py, m) {
    m.doc() = "PolarFire SoC Motor Control Library";

    // Direction constants
    m.attr("DIRECTION_CLOCKWISE") = DIRECTION_CLOCKWISE;
    m.attr("DIRECTION_COUNTER_CLOCKWISE") = DIRECTION_COUNTER_CLOCKWISE;
    m.attr("DIRECTION_CW") = DIRECTION_CLOCKWISE;
    m.attr("DIRECTION_CCW") = DIRECTION_COUNTER_CLOCKWISE;
    m.attr("CLOCKWISE") = DIRECTION_CW_STR;
    m.attr("COUNTER_CLOCKWISE") = DIRECTION_CCW_STR;

    py::enum_<MotorType>(m, "MotorType", "Motor type enumeration")
        .value("BLDC", MotorType::BLDC, "Brushless DC motor")
        .value("STEPPER", MotorType::STEPPER, "Stepper motor")
        .export_values();

    py::enum_<StartupMode>(m, "StartupMode", "Startup mode for sensorless operation")
        .value("CURRENT_BY_FREQ", StartupMode::CURRENT_BY_FREQ, "Current/Frequency mode (C/F)")
        .value("VOLTAGE", StartupMode::VOLTAGE, "Voltage mode")
        .export_values();

    // ===== BLDC Structures =====

    py::class_<BLDCSpecs>(m, "BLDCSpecs", "BLDC motor electrical specifications")
        .def(py::init<>())
        .def_readwrite("dc_voltage_mV", &BLDCSpecs::dc_voltage_mV,
                       "DC bus voltage in millivolts")
        .def_readwrite("motor_current_mA", &BLDCSpecs::motor_current_mA,
                       "Motor rated current in milliamps")
        .def_readwrite("motor_speed_rpm", &BLDCSpecs::motor_speed_rpm,
                       "Motor rated speed in RPM")
        .def_readwrite("pole_pairs", &BLDCSpecs::pole_pairs,
                       "Number of pole pairs")
        .def_readwrite("resistance_mOhm", &BLDCSpecs::resistance_mOhm,
                       "Motor phase resistance in milli-ohms")
        .def_readwrite("inductance_uH", &BLDCSpecs::inductance_uH,
                       "Motor phase inductance in micro-henries")
        .def_readwrite("switching_freq_kHz", &BLDCSpecs::switching_freq_kHz,
                       "PWM switching frequency in kHz");

    py::class_<BLDCParams>(m, "BLDCParams", "BLDC motor control parameters")
        .def(py::init<>())
        .def_readwrite("current_pi_kp", &BLDCParams::current_pi_kp,
                       "Current PI controller Kp gain (IDQ Kp)")
        .def_readwrite("current_pi_ki", &BLDCParams::current_pi_ki,
                       "Current PI controller Ki gain (IDQ Ki)")
        .def_readwrite("speed_pi_kp", &BLDCParams::speed_pi_kp,
                       "Speed PI controller Kp gain")
        .def_readwrite("speed_pi_ki", &BLDCParams::speed_pi_ki,
                       "Speed PI controller Ki gain")
        .def_readwrite("startup_mode", &BLDCParams::startup_mode,
                       "Startup mode (CURRENT_BY_FREQ or VOLTAGE)")
        .def_readwrite("auto_restart", &BLDCParams::auto_restart,
                       "Enable auto restart on fault")
        .def_readwrite("soft_stop", &BLDCParams::soft_stop,
                       "Enable soft stop")
        .def_readwrite("closed_loop_speed_rpm", &BLDCParams::closed_loop_speed_rpm,
                       "Speed threshold for closed loop transition (RPM)")
        .def_readwrite("open_loop_current_pct", &BLDCParams::open_loop_current_pct,
                       "Open loop current percentage (0-100)")
        .def_readwrite("open_loop_voltage_pct", &BLDCParams::open_loop_voltage_pct,
                       "Open loop voltage percentage (0-100)")
        .def_readwrite("angle_pi_kp", &BLDCParams::angle_pi_kp,
                       "Angle correction (PLL) PI Kp gain")
        .def_readwrite("angle_pi_ki", &BLDCParams::angle_pi_ki,
                       "Angle correction (PLL) PI Ki gain");

    // ===== Stepper Structures =====

    py::class_<StepperSpecs>(m, "StepperSpecs", "Stepper motor electrical specifications")
        .def(py::init<>())
        .def_readwrite("dc_voltage_mV", &StepperSpecs::dc_voltage_mV,
                       "DC bus voltage in millivolts")
        .def_readwrite("motor_current_mA", &StepperSpecs::motor_current_mA,
                       "Motor rated current in milliamps")
        .def_readwrite("step_number", &StepperSpecs::step_number,
                       "Number of steps per revolution")
        .def_readwrite("microstep_resolution", &StepperSpecs::microstep_resolution,
                       "Microstep resolution (e.g., 64 for 64x microstepping)")
        .def_readwrite("resistance_mOhm", &StepperSpecs::resistance_mOhm,
                       "Motor phase resistance in milli-ohms")
        .def_readwrite("inductance_uH", &StepperSpecs::inductance_uH,
                       "Motor phase inductance in micro-henries")
        .def_readwrite("switching_freq_kHz", &StepperSpecs::switching_freq_kHz,
                       "PWM switching frequency in kHz");

    py::class_<StepperParams>(m, "StepperParams", "Stepper motor control parameters")
        .def(py::init<>())
        .def_readwrite("current_pi_kp", &StepperParams::current_pi_kp,
                       "Current PI controller Kp gain (IDQ Kp)")
        .def_readwrite("current_pi_ki", &StepperParams::current_pi_ki,
                       "Current PI controller Ki gain (IDQ Ki)")
        .def_readwrite("current_reference_mA", &StepperParams::current_reference_mA,
                       "Current reference in milliamps")
        .def_readwrite("cmd_steps", &StepperParams::cmd_steps,
                       "Command steps (positive=forward, negative=reverse)")
        .def_readwrite("speed_rpm", &StepperParams::speed_rpm,
                       "Step speed in RPM");

    // ===== Motor class =====

    py::class_<Motor>(m, "Motor", "Motor control class with separate BLDC and Stepper parameters")
        .def(py::init<MotorType>(), py::arg("type"),
             "Create a Motor instance of the specified type")
        .def("is_valid", &Motor::is_valid,
             "Check if all required IIO devices are available")
        .def("init", &Motor::init,
             "Initialize motor with current specs and parameters")
        .def("start", &Motor::start,
             "Start the motor")
        .def("stop", &Motor::stop,
             "Stop the motor")
        .def("set_direction", py::overload_cast<int>(&Motor::set_direction), py::arg("dir"),
             "Set motor direction (0/CLOCKWISE=forward, 1/COUNTER_CLOCKWISE=reverse)")
        .def("set_direction", py::overload_cast<const std::string&>(&Motor::set_direction), py::arg("dir"),
             "Set motor direction ('CLOCKWISE'/'CW' or 'COUNTER CLOCKWISE'/'CCW')")
        .def("set_speed", &Motor::set_speed, py::arg("speed"),
             "Set motor speed. BLDC: 10-4000 RPM, Stepper: 0x20-0xFF (32-255). Returns 0 on success, -1 on error")
        .def("clear_fault", &Motor::clear_fault,
             "Clear motor fault registers")
        .def("get_state", &Motor::get_state,
             "Get sequencer FSM state. Returns -1 on error")
        .def("get_speed", &Motor::get_speed,
             "Get current motor speed (from omega filter)")
        .def("get_type", &Motor::get_type,
             "Get motor type (BLDC or STEPPER)")

        // ===== BLDC Status Methods =====
        .def("get_bldc_speed_raw", &Motor::get_bldc_speed_raw,
             "Get raw BLDC speed value from hardware (for debugging). Returns None on error")
        .def("get_bldc_speed", &Motor::get_bldc_speed,
             "Get current BLDC speed in RPM (scaled by pole_pairs and motor_speed_rpm). Returns None on error")
        .def("get_bldc_direction", &Motor::get_bldc_direction,
             "Get current BLDC direction as string ('CLOCKWISE' or 'COUNTER CLOCKWISE'). Returns None on error")
        .def("get_bldc_iq", &Motor::get_bldc_iq,
             "Get current BLDC IQ value. Returns None on error")

        // ===== BLDC Specs/Params =====
        .def("set_bldc_specs", &Motor::set_bldc_specs, py::arg("specs"),
             "Set BLDC motor specifications")
        .def("get_bldc_specs", &Motor::get_bldc_specs,
             "Get current BLDC motor specifications")
        .def("set_bldc_params", &Motor::set_bldc_params, py::arg("params"),
             "Set BLDC motor parameters")
        .def("get_bldc_params", &Motor::get_bldc_params,
             "Get current BLDC motor parameters")

        // ===== Stepper Specs/Params =====
        .def("set_stepper_specs", &Motor::set_stepper_specs, py::arg("specs"),
             "Set Stepper motor specifications")
        .def("get_stepper_specs", &Motor::get_stepper_specs,
             "Get current Stepper motor specifications")
        .def("set_stepper_params", &Motor::set_stepper_params, py::arg("params"),
             "Set Stepper motor parameters")
        .def("get_stepper_params", &Motor::get_stepper_params,
             "Get current Stepper motor parameters")

        // Apply parameters to hardware
        .def("apply_parameters", &Motor::apply_parameters,
             "Apply current specs and params to hardware (call after modifying settings)");
}
