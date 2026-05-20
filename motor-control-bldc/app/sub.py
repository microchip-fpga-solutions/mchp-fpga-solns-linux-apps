#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# @file sub.py
# @brief PolarFire SoC Motor Control Service - ROS2 node for motor control
# Copyright (C) 2026 Microchip Technology Inc. and its subsidiaries
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import motor_lib_py as ml
import json

# Speed bounds for validation (all values in decimal)
BLDC_SPEED_MIN = 10
BLDC_SPEED_MAX = 4000
# Stepper SLEW_CNT register bounds: 0x20-0xFF (32-255)
# Lower SLEW_CNT = faster speed (inverse relationship)
STEPPER_SPEED_MIN = 32              # 0x20 - fastest (highest RPM)
STEPPER_SPEED_MAX = 255             # 0xFF - slowest (lowest RPM)


class MotorControlService(Node):
    """
    ROS2 Node providing:
    - Pub/Sub on 'motor_command' for control commands
    - Request/Reply on 'motor_param_request'/'motor_param_response' for parameter operations
    """

    def __init__(self):
        super().__init__('motor_control_service')

        # Initialize motors
        self.bldc_motor = ml.Motor(ml.MotorType.BLDC)
        self.stepper_motor = ml.Motor(ml.MotorType.STEPPER)

        if not (self.bldc_motor.is_valid() and self.stepper_motor.is_valid()):
            self.get_logger().error("Motor modules not loaded")
            raise RuntimeError("Failed to initialize motors - modules not loaded")

        try:
            self.bldc_motor.init()
            self.stepper_motor.init()
            self.get_logger().info("Motors initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Motor initialization failed: {e}")
            raise

        # === PUB/SUB for control commands ===
        self.command_sub = self.create_subscription(
            String, 'motor_command', self.handle_command, 10
        )

        # === REQUEST/REPLY for parameter operations ===
        self.param_request_sub = self.create_subscription(
            String, 'motor_param_request', self.handle_param_request, 10
        )
        self.param_response_pub = self.create_publisher(String, 'motor_param_response', 10)

        self.get_logger().info("Motor Control Service ready")

    def destroy_node(self):
        """Clean up motors before destroying node."""
        self.get_logger().info("Shutting down motors...")
        try:
            self.bldc_motor.stop()
            self.stepper_motor.stop()
        except Exception as e:
            self.get_logger().warn(f"Error stopping motors during shutdown: {e}")
        super().destroy_node()

    # =========================================================================
    # PUB/SUB HANDLER - Control Commands
    # =========================================================================
    def handle_command(self, msg):
        """Handle control commands via pub/sub."""
        data = msg.data.strip()
        parts = data.split()
        if len(parts) != 2:
            self.get_logger().warn(f"Invalid command format: {data}")
            return

        command, value_str = parts[0], parts[1]

        try:
            value = int(value_str)
        except ValueError:
            self.get_logger().warn(f"Invalid integer value in command: {data}")
            return

        # Dispatch command
        if command == "bldc_start":
            self._handle_bldc_start(value)
        elif command == "stepper_start":
            self._handle_stepper_start(value)
        elif command == "bldc_dir":
            self._handle_bldc_dir(value)
        elif command == "stepper_dir":
            self._handle_stepper_dir(value)
        elif command == "bldc_rpm":
            self._handle_bldc_rpm(value)
        elif command == "stepper_rpm":
            self._handle_stepper_rpm(value)
        elif command == "clear_fault":
            self.bldc_motor.clear_fault()
            self.stepper_motor.clear_fault()
            self.get_logger().info("Faults cleared")
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def _handle_bldc_start(self, val):
        self.get_logger().info(f"BLDC Start set to {val}")
        if val == 1:
            # Clear any existing fault before starting
            self.bldc_motor.clear_fault()
            self.bldc_motor.start()
        else:
            self.bldc_motor.stop()
            self.bldc_motor.clear_fault()

    def _handle_stepper_start(self, val):
        self.get_logger().info(f"Stepper Start set to {val}")
        if val == 1:
            # Clear any existing fault before starting
            self.stepper_motor.clear_fault()
            self.stepper_motor.start()
        else:
            self.stepper_motor.stop()
            self.stepper_motor.clear_fault()

    def _handle_bldc_dir(self, val):
        if val not in (0, 1):
            self.get_logger().warn(f"Invalid BLDC direction value: {val}, expected 0 or 1")
            return
        self.get_logger().info(f"BLDC Direction set to {val}")
        self.bldc_motor.set_direction(val)

    def _handle_stepper_dir(self, val):
        if val not in (0, 1):
            self.get_logger().warn(f"Invalid Stepper direction value: {val}, expected 0 or 1")
            return
        self.get_logger().info(f"Stepper Direction set to {val}")
        self.stepper_motor.set_direction(val)

    def _handle_bldc_rpm(self, val):
        if not (BLDC_SPEED_MIN <= val <= BLDC_SPEED_MAX):
            self.get_logger().warn(
                f"BLDC RPM {val} out of range [{BLDC_SPEED_MIN}, {BLDC_SPEED_MAX}]"
            )
            return
        self.get_logger().info(f"BLDC RPM set to {val}")
        self.bldc_motor.set_speed(val)

    def _handle_stepper_rpm(self, val):
        if not (STEPPER_SPEED_MIN <= val <= STEPPER_SPEED_MAX):
            self.get_logger().warn(
                f"Stepper speed {val} out of range [{STEPPER_SPEED_MIN:#x}, {STEPPER_SPEED_MAX:#x}]"
            )
            return
        self.get_logger().info(f"Stepper Step speed set to {val}")
        self.stepper_motor.set_speed(val)

    # =========================================================================
    # REQUEST/REPLY HANDLER - Parameter Operations (v2 separate structures)
    # =========================================================================
    def handle_param_request(self, msg):
        """
        Handle parameter GET/SET requests via request/reply.
        Request format (JSON):
        {
            "action": "get_specs" | "set_specs" | "get_params" | "set_params" | "apply" | "get_all",
            "motor": "bldc" | "stepper",
            "specs": { ... },  # for set_specs
            "params": { ... }, # for set_params
            "id": "<request_id>"
        }
        """
        try:
            request = json.loads(msg.data)
            action = request.get('action', '')
            motor_type = request.get('motor', '')
            request_id = request.get('id', '')

            motor = self.bldc_motor if motor_type == 'bldc' else self.stepper_motor

            if action == 'get_specs':
                result = self._get_specs(motor_type, motor)
            elif action == 'set_specs':
                specs_data = request.get('specs', {})
                result = self._set_specs(motor_type, motor, specs_data)
            elif action == 'get_params':
                result = self._get_params(motor_type, motor)
            elif action == 'set_params':
                params_data = request.get('params', {})
                result = self._set_params(motor_type, motor, params_data)
            elif action == 'apply':
                result = self._apply_params(motor)
            elif action == 'get_all':
                result = self._get_all(motor_type, motor)
            else:
                result = {'success': False, 'error': f'Unknown action: {action}'}

            result['id'] = request_id
            result['motor'] = motor_type
            result['action'] = action

            response_msg = String()
            response_msg.data = json.dumps(result)
            self.param_response_pub.publish(response_msg)
            self.get_logger().info(f"Sent response for {action} {motor_type}: success={result.get('success', False)}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in param request: {e}")
            self._send_param_error(str(e), '')
        except Exception as e:
            self.get_logger().error(f"Error handling param request: {e}")
            self._send_param_error(str(e), '')

    def _send_param_error(self, error_msg, request_id):
        """Send error response for parameter request."""
        response = {'success': False, 'error': error_msg, 'id': request_id}
        response_msg = String()
        response_msg.data = json.dumps(response)
        self.param_response_pub.publish(response_msg)

    def _get_specs(self, motor_type, motor):
        """Get motor specifications."""
        try:
            if motor_type == 'bldc':
                specs = motor.get_bldc_specs()
                return {
                    'success': True,
                    'specs': {
                        'dc_voltage_mV': specs.dc_voltage_mV,
                        'motor_current_mA': specs.motor_current_mA,
                        'motor_speed_rpm': specs.motor_speed_rpm,
                        'pole_pairs': specs.pole_pairs,
                        'resistance_mOhm': specs.resistance_mOhm,
                        'inductance_uH': specs.inductance_uH,
                        'switching_freq_kHz': specs.switching_freq_kHz
                    }
                }
            else:  # stepper
                specs = motor.get_stepper_specs()
                return {
                    'success': True,
                    'specs': {
                        'dc_voltage_mV': specs.dc_voltage_mV,
                        'motor_current_mA': specs.motor_current_mA,
                        'step_number': specs.step_number,
                        'microstep_resolution': specs.microstep_resolution,
                        'resistance_mOhm': specs.resistance_mOhm,
                        'inductance_uH': specs.inductance_uH,
                        'switching_freq_kHz': specs.switching_freq_kHz
                    }
                }
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _set_specs(self, motor_type, motor, specs_data):
        """Set motor specifications."""
        try:
            if motor_type == 'bldc':
                specs = ml.BLDCSpecs()
                if 'dc_voltage_mV' in specs_data:
                    specs.dc_voltage_mV = int(specs_data['dc_voltage_mV'])
                if 'motor_current_mA' in specs_data:
                    specs.motor_current_mA = int(specs_data['motor_current_mA'])
                if 'motor_speed_rpm' in specs_data:
                    specs.motor_speed_rpm = int(specs_data['motor_speed_rpm'])
                if 'pole_pairs' in specs_data:
                    specs.pole_pairs = int(specs_data['pole_pairs'])
                if 'resistance_mOhm' in specs_data:
                    specs.resistance_mOhm = int(specs_data['resistance_mOhm'])
                if 'inductance_uH' in specs_data:
                    specs.inductance_uH = int(specs_data['inductance_uH'])
                if 'switching_freq_kHz' in specs_data:
                    specs.switching_freq_kHz = int(specs_data['switching_freq_kHz'])
                motor.set_bldc_specs(specs)
            else:  # stepper
                specs = ml.StepperSpecs()
                if 'dc_voltage_mV' in specs_data:
                    specs.dc_voltage_mV = int(specs_data['dc_voltage_mV'])
                if 'motor_current_mA' in specs_data:
                    specs.motor_current_mA = int(specs_data['motor_current_mA'])
                if 'step_number' in specs_data:
                    specs.step_number = int(specs_data['step_number'])
                if 'microstep_resolution' in specs_data:
                    specs.microstep_resolution = int(specs_data['microstep_resolution'])
                if 'resistance_mOhm' in specs_data:
                    specs.resistance_mOhm = int(specs_data['resistance_mOhm'])
                if 'inductance_uH' in specs_data:
                    specs.inductance_uH = int(specs_data['inductance_uH'])
                if 'switching_freq_kHz' in specs_data:
                    specs.switching_freq_kHz = int(specs_data['switching_freq_kHz'])
                motor.set_stepper_specs(specs)

            self.get_logger().info(f"{motor_type.upper()} specs updated")
            return {'success': True}
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _get_params(self, motor_type, motor):
        """Get motor parameters."""
        try:
            if motor_type == 'bldc':
                params = motor.get_bldc_params()
                return {
                    'success': True,
                    'params': {
                        'current_pi_kp': params.current_pi_kp,
                        'current_pi_ki': params.current_pi_ki,
                        'speed_pi_kp': params.speed_pi_kp,
                        'speed_pi_ki': params.speed_pi_ki,
                        'startup_mode': 'C/F' if params.startup_mode == ml.StartupMode.CURRENT_BY_FREQ else 'V',
                        'auto_restart': params.auto_restart,
                        'soft_stop': params.soft_stop,
                        'closed_loop_speed_rpm': params.closed_loop_speed_rpm,
                        'open_loop_current_pct': params.open_loop_current_pct,
                        'open_loop_voltage_pct': params.open_loop_voltage_pct,
                        'angle_pi_kp': params.angle_pi_kp,
                        'angle_pi_ki': params.angle_pi_ki
                    }
                }
            else:  # stepper
                params = motor.get_stepper_params()
                return {
                    'success': True,
                    'params': {
                        'current_pi_kp': params.current_pi_kp,
                        'current_pi_ki': params.current_pi_ki,
                        'current_reference_mA': params.current_reference_mA,
                        'cmd_steps': params.cmd_steps,
                        'speed_rpm': params.speed_rpm
                    }
                }
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _set_params(self, motor_type, motor, params_data):
        """Set motor parameters."""
        try:
            if motor_type == 'bldc':
                # Get current params and update only changed fields
                params = motor.get_bldc_params()
                if 'current_pi_kp' in params_data:
                    params.current_pi_kp = int(params_data['current_pi_kp'])
                if 'current_pi_ki' in params_data:
                    params.current_pi_ki = int(params_data['current_pi_ki'])
                if 'speed_pi_kp' in params_data:
                    params.speed_pi_kp = int(params_data['speed_pi_kp'])
                if 'speed_pi_ki' in params_data:
                    params.speed_pi_ki = int(params_data['speed_pi_ki'])
                if 'startup_mode' in params_data:
                    mode = params_data['startup_mode']
                    params.startup_mode = ml.StartupMode.CURRENT_BY_FREQ if mode in ['C/F', 'CURRENT_BY_FREQ'] else ml.StartupMode.VOLTAGE
                if 'auto_restart' in params_data:
                    params.auto_restart = bool(params_data['auto_restart'])
                if 'soft_stop' in params_data:
                    params.soft_stop = bool(params_data['soft_stop'])
                if 'closed_loop_speed_rpm' in params_data:
                    params.closed_loop_speed_rpm = int(params_data['closed_loop_speed_rpm'])
                if 'open_loop_current_pct' in params_data:
                    params.open_loop_current_pct = int(params_data['open_loop_current_pct'])
                if 'open_loop_voltage_pct' in params_data:
                    params.open_loop_voltage_pct = int(params_data['open_loop_voltage_pct'])
                if 'angle_pi_kp' in params_data:
                    params.angle_pi_kp = int(params_data['angle_pi_kp'])
                if 'angle_pi_ki' in params_data:
                    params.angle_pi_ki = int(params_data['angle_pi_ki'])
                motor.set_bldc_params(params)
                self.get_logger().info(f"BLDC params: kp={params.current_pi_kp}, ki={params.current_pi_ki}, speed_kp={params.speed_pi_kp}")
            else:  # stepper
                # Get current params and update only changed fields
                params = motor.get_stepper_params()
                if 'current_pi_kp' in params_data:
                    params.current_pi_kp = int(params_data['current_pi_kp'])
                if 'current_pi_ki' in params_data:
                    params.current_pi_ki = int(params_data['current_pi_ki'])
                if 'current_reference_mA' in params_data:
                    params.current_reference_mA = int(params_data['current_reference_mA'])
                if 'cmd_steps' in params_data:
                    params.cmd_steps = int(params_data['cmd_steps'])
                if 'speed_rpm' in params_data:
                    params.speed_rpm = int(params_data['speed_rpm'])
                motor.set_stepper_params(params)
                self.get_logger().info(f"Stepper params: kp={params.current_pi_kp}, ki={params.current_pi_ki}, speed={params.speed_rpm}")

            self.get_logger().info(f"{motor_type.upper()} params updated")
            return {'success': True}
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _apply_params(self, motor):
        """Apply current specs and params to hardware."""
        try:
            motor.apply_parameters()
            self.get_logger().info("Parameters applied to hardware")
            return {'success': True}
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _get_all(self, motor_type, motor):
        """Get all specs and params."""
        specs_result = self._get_specs(motor_type, motor)
        params_result = self._get_params(motor_type, motor)

        if specs_result['success'] and params_result['success']:
            return {
                'success': True,
                'specs': specs_result['specs'],
                'params': params_result['params'],
                'state': motor.get_state()
            }
        else:
            return {'success': False, 'error': 'Failed to get all parameters'}


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorControlService()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
