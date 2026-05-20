#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# @file gui.py
# @brief PolarFire SoC Motor Control Dashboard - Web-based GUI for BLDC and Stepper motors
# Copyright (C) 2026 Microchip Technology Inc. and its subsidiaries
#

import threading
import atexit
import struct
import json
import uuid
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bokeh.plotting import curdoc, figure
from bokeh.layouts import layout, row, column
from bokeh.models import Div, Paragraph, ColumnDataSource, FixedTicker, Slider, Spacer, TextInput, Select, CheckboxGroup
from bokeh.models.widgets import Button


def find_iio_device_by_name(target_name):
    """Find IIO device path by scanning /sys/bus/iio/devices/*/name."""
    base_path = "/sys/bus/iio/devices/"
    try:
        if not os.path.exists(base_path):
            return None
        for entry in os.listdir(base_path):
            device_dir = os.path.join(base_path, entry)
            if not os.path.isdir(device_dir):
                continue
            name_file = os.path.join(device_dir, "name")
            if os.path.exists(name_file):
                with open(name_file, 'r') as f:
                    device_name = f.read().strip()
                    if device_name == target_name:
                        return f"/dev/{entry}"
    except (OSError, IOError):
        pass
    return None


# Color constants
FOREST = "#388e3c"
FOREST_D = "#2e7d32"
MOSS = "#a5d6a7"
SKY = "#81d4fa"
SUNRISE = "#ffb74d"
EARTH = "#6d4c41"

START_C = "#1DE466"
STOP_C = "#E4311D"
DIR_C = "#404040"
DIR_STATUS_C = "#B3B3CE"

HEAD2_C = "#76CBC0"
PARAM_C = "#E0E0E0"
LABEL_C = "#B0B0B0"

SPEED_PLOT_C = "#97F7F7"
PHASE_PLOT_C = "#AFE996"

# IIO Configuration
# Buffer channel layout (from mpfs_mc_adc driver):
# ch0 = Speed (from REG_SPEED_BLDC - bldc_speed_filtered_o from BLDC estimator)
# ch1 = IQ (from REG_IQ_BLDC)
# ch2 = Current IB (from REG_ADC_IB_BLDC - converted to mV by driver)
IIO_DEVICE_PATH = find_iio_device_by_name("mpfs_mc_adc") or "/dev/iio:device0"
NUM_SAMPLES = 500
CHANNELS = 3
BYTES_PER_SAMPLE = 2
PLOT_WINDOW = 512
TOTAL_BYTES = NUM_SAMPLES * CHANNELS * BYTES_PER_SAMPLE

# Plot configuration
NUM_POINTS = 500

# Debounce configuration
DEBOUNCE_TIMEOUT_MS = 500
PARAM_DEBOUNCE_MS = 800

# Speed bounds (all values in decimal)
BLDC_SPEED_MIN = 10
BLDC_SPEED_MAX = 4000
# Stepper SLEW_CNT register bounds: 0x20-0xFF (32-255)
# Lower SLEW_CNT = faster speed (inverse relationship)
STEPPER_SLEW_CNT_MIN = 32           # 0x20 - fastest (highest RPM)
STEPPER_SLEW_CNT_MAX = 255          # 0xFF - slowest (lowest RPM)

# Default BLDC Specifications (reverse-calculated from init_bldc register values)
DEFAULT_BLDC_SPECS = {
    'dc_voltage_mV': 24000,
    'motor_current_mA': 1790,
    'motor_speed_rpm': 4000,
    'pole_pairs': 4,
    'resistance_mOhm': 1800,
    'inductance_uH': 2600,
    'switching_freq_kHz': 20
}

# Default BLDC Parameters (reverse-calculated from init_bldc register values)
# Register values: speed_pi_kp=3000, speed_pi_ki=10, idq_kp=124, idq_ki=275
# sqmng_dv=3277 -> 5%, sqmng_iq_ref=32768 -> 50%, sqmng_cl_omega=6554 -> 400 RPM
# seq_ctl=5 -> auto_restart=1, c_by_f=1, soft_stop=0
DEFAULT_BLDC_PARAMS = {
    'current_pi_kp': 124,
    'current_pi_ki': 275,
    'speed_pi_kp': 3000,
    'speed_pi_ki': 10,
    'startup_mode': 'C/F',
    'auto_restart': True,
    'soft_stop': False,
    'closed_loop_speed_rpm': 400,
    'open_loop_current_pct': 50,
    'open_loop_voltage_pct': 5,
    'angle_pi_kp': 10000,
    'angle_pi_ki': 5000
}

# Default Stepper Specifications (from init_stepper and config.h)
DEFAULT_STEPPER_SPECS = {
    'dc_voltage_mV': 24000,
    'motor_current_mA': 1000,
    'step_number': 200,
    'microstep_resolution': 64,
    'resistance_mOhm': 5300,
    'inductance_uH': 6500,
    'switching_freq_kHz': 20
}

# Default Stepper Parameters (reverse-calculated from init_stepper register values)
# Register values: current_pi_kp=124, current_pi_ki=275
DEFAULT_STEPPER_PARAMS = {
    'current_pi_kp': 124,
    'current_pi_ki': 275,
    'current_reference_mA': 500,
    'cmd_steps': 100,
    'speed_rpm': 100
}

css_style = Div(text="""
<style>
    html, body {
        height: 100%;
        margin: 0;
        padding: 0;
    }

   body {
        background-color: #0E3689;
        color: #ffd6a5 !important;
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }

    .param-input input {
        background: #1a3a5c !important;
        color: #E0E0E0 !important;
        border: 1px solid #2a5a8c !important;
        font-family: monospace !important;
        font-size: 11px !important;
    }

    .param-label {
        color: #B0B0B0 !important;
        font-size: 11px !important;
    }

    .bk-tabs-header .bk-tab {
        background: #1a3a5c !important;
        color: #90CAF9 !important;
        border: 1px solid #2a5a8c !important;
        padding: 6px 12px !important;
    }

    .bk-tabs-header .bk-tab.bk-active {
        background: #2a5a8c !important;
        color: #FFFFFF !important;
        border-bottom: 2px solid #64B5F6 !important;
    }

</style>
""", width=0, height=0)


dir_css = Div(text="""
<style>
  .circle-button button {
     background: #404040 !important;
     color: white !important;
     border: 2px solid #404040 !important;
     border-radius: 8px !important;
     width: 40px !important;
     height: 40px !important;
     padding: 0 !important;
     font-weight: bold !important;
     font-size: 14px !important;
     cursor: pointer;
  }

  .apply-button button {
     background: #2196F3 !important;
     color: white !important;
     border: 2px solid #2196F3 !important;
     border-radius: 4px !important;
     font-weight: bold !important;
  }

  .refresh-button button {
     background: #FF9800 !important;
     color: white !important;
     border: 2px solid #FF9800 !important;
     border-radius: 4px !important;
     font-weight: bold !important;
  }

  .defaults-button button {
     background: #9C27B0 !important;
     color: white !important;
     border: 2px solid #9C27B0 !important;
     border-radius: 4px !important;
     font-weight: bold !important;
  }

  .config-motor-spec button {
     background: #00897B !important;
     color: white !important;
     border: 2px solid #00897B !important;
     border-radius: 4px !important;
     font-weight: bold !important;
  }

  .param-tab-active button {
     background: #2a5a8c !important;
     color: #FFFFFF !important;
     border: 2px solid #64B5F6 !important;
     border-radius: 6px !important;
     font-weight: bold !important;
     font-size: 12px !important;
  }

  .param-tab-inactive button {
     background: #1a3a5c !important;
     color: #90CAF9 !important;
     border: 1px solid #2a5a8c !important;
     border-radius: 6px !important;
     font-weight: normal !important;
     font-size: 12px !important;
  }
</style>
""", width=0, height=0)


radio_css = Div(text=f"""
<style>
  .my-start-button button {{
     background: {START_C} !important;
     color: white !important;
     border: 2px solid {START_C} !important;
     border-radius: 8px !important;
     width: 40px !important;
     height: 40px !important;
     padding: 0 !important;
     font-weight: bold !important;
     font-size: 14px !important;
     cursor: pointer;
  }}

  .my-stop-button button {{
     background: {STOP_C} !important;
     color: white !important;
     border: 2px solid {STOP_C} !important;
     border-radius: 8px !important;
     width: 40px !important;
     height: 40px !important;
     padding: 0 !important;
     font-weight: bold !important;
     font-size: 14px !important;
     cursor: pointer;
  }}
</style>
""", width=0, height=0)


title1 = Div(text="""
    <div style="
        text-align: center;
        font-size: 28px;
        font-weight: bold;
        color: white;
        padding: 15px 0;
        margin-bottom: 25px;
        margin-left: auto;
        margin-right: auto;
    ">
       PolarFire® SoC Motor Control Dashboard
    </div>
""", width=1200, height=80, sizing_mode="stretch_width")


# ==============================================================================
# ROS2 Hybrid Node - Pub/Sub for control, Request/Reply for parameters
# ==============================================================================
bokeh_doc = curdoc()


class MotorControlClient(Node):
    def __init__(self):
        super().__init__('motor_control_gui')

        # === PUB/SUB for control commands ===
        self.command_pub = self.create_publisher(String, 'motor_command', 10)

        # === REQUEST/REPLY for parameter operations ===
        self.param_request_pub = self.create_publisher(String, 'motor_param_request', 10)
        self.param_response_sub = self.create_subscription(
            String, 'motor_param_response', self.param_response_callback, 10
        )
        self.response_handlers = {}
        self.get_logger().info("Motor control GUI node initialized")

    def send_command(self, cmd):
        """Send control command via pub/sub (fire-and-forget)."""
        msg = String()
        msg.data = cmd
        self.command_pub.publish(msg)
        self.get_logger().info(f"Sent command: {cmd}")

    def send_param_request(self, action, motor, data=None, callback=None):
        """Send parameter request via request/reply."""
        request_id = str(uuid.uuid4())[:8]
        request = {
            'id': request_id,
            'action': action,
            'motor': motor,
        }
        if data:
            request.update(data)

        if callback:
            self.response_handlers[request_id] = callback
        msg = String()
        msg.data = json.dumps(request)
        self.param_request_pub.publish(msg)
        self.get_logger().info(f"Sent param request: {action} {motor}")
        return request_id

    def param_response_callback(self, msg):
        """Handle response from motor control service."""
        try:
            response = json.loads(msg.data)
            request_id = response.get('id', '')
            if request_id in self.response_handlers:
                callback = self.response_handlers.pop(request_id)
                self.get_logger().info(f"Processing response for {response.get('action', 'unknown')}")
                bokeh_doc.add_next_tick_callback(lambda r=response: callback(r))
        except Exception as e:
            self.get_logger().error(f"Response Error: {e}")


# --- Initialize ROS2 with error handling ---
motor_client = None
spin_thread = None
ros2_error_message = None

try:
    rclpy.init()
    motor_client = MotorControlClient()

    def ros_spin():
        rclpy.spin(motor_client)

    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()
except Exception as e:
    print(f"[ROS2 Init Error] Failed to initialize ROS2: {e}")
    motor_client = None
    ros2_error_message = str(e)


# --- Cleanup on exit ---
def cleanup():
    if motor_client is not None:
        motor_client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

atexit.register(cleanup)


# ==============================================================================
# Parameter Input Widgets Storage
# ==============================================================================
bldc_spec_inputs = {}
bldc_param_inputs = {}
stepper_spec_inputs = {}
stepper_param_inputs = {}


def create_param_input(label, default_value, width=80):
    """Create a labeled text input for a parameter."""
    input_widget = TextInput(value=str(default_value), width=width)
    input_widget.css_classes = ['param-input']
    label_widget = Paragraph(text=label, width=130, style={'color': LABEL_C, 'font-size': '14px'})
    return input_widget, label_widget


def create_param_row(label, default_value, param_dict, param_name, width=80):
    """Create a row with label and input, storing reference in dict."""
    input_widget, label_widget = create_param_input(label, default_value, width)
    param_dict[param_name] = input_widget
    return row(label_widget, input_widget, sizing_mode='fixed')


# --- Status display ---
bldc_status_ss = Div(text=f"<div style='color:{SUNRISE};font-size:15px'>Status: READY</div>")
bldc_status_ca = Div(text=f"<div style='color:{SUNRISE};font-size:15px'>Direction: COUNTER CLOCKWISE</div>")
stepper_status_ss = Div(text=f"<div style='color:{SUNRISE};font-size:15px'>Status: READY</div>")
stepper_status_ca = Div(text=f"<div style='color:{SUNRISE};font-size:15px'>Direction: COUNTER CLOCKWISE</div>")
bldc_param_status = Div(text="<div style='color:#888;font-size:12px'>Parameter: Ready</div>")
stepper_param_status = Div(text="<div style='color:#888;font-size:12px'>Parameter: Ready</div>")

# --- Control Widgets ---
bldc_rpm_slider = Slider(start=20, end=4000, value=2000, step=1, title="Motor Speed (RPM)", width=190)
bldc_start = Button(label="START", width=70, height=35)
bldc_start.css_classes.append("my-start-button")
bldc_stop = Button(label="STOP", width=70, height=35)
bldc_stop.css_classes.append("my-stop-button")
bldc_rev = Button(label="DIRECTION \u21BA", width=120, height=40)  # Counter-clockwise arrow (default)
bldc_rev.css_classes = ["circle-button"]

stepper_rpm_slider = Slider(start=20, end=100, value=100, step=10, title="Motor Speed (RPM)", width=190)
stepper_start = Button(label="START", width=70, height=35)
stepper_start.css_classes.append("my-start-button")
stepper_stop = Button(label="STOP", width=70, height=35)
stepper_stop.css_classes.append("my-stop-button")
stepper_rev = Button(label="DIRECTION \u21BA", width=120, height=40)  # Counter-clockwise arrow (default)
stepper_rev.css_classes = ["circle-button"]


# ==============================================================================
# BLDC Control Callbacks
# ==============================================================================
def bldc_start_on_change():
    global bldc_default_dir
    bldc_status_ss.text = f"<div style='color:{SUNRISE};font-size:15px'>Status: <span style='color:{START_C};font-weight:700'>RUNNING</span></div>"
    if motor_client:
        motor_client.send_command("bldc_start 1")
        # Re-sync direction to hardware after start (in case of mismatch)
        motor_client.send_command(f"bldc_dir {bldc_default_dir}")


def bldc_stop_on_change():
    bldc_status_ss.text = f"<div style='color:{SUNRISE};font-size:15px'>Status: <span style='color:{STOP_C};font-weight:700'>STOPPED</span></div>"
    if motor_client:
        motor_client.send_command("bldc_start 0")


bldc_default_dir = 1


def bldc_rev_on_change():
    global bldc_default_dir
    if bldc_default_dir == 1:
        if motor_client:
            motor_client.send_command("bldc_dir 0")
        bldc_default_dir = 0
        bldc_rev.label = "DIRECTION \u21BB"  # Clockwise arrow (now clockwise)
        bldc_status_ca.text = f"<div style='color:{SUNRISE};font-size:15px'>Direction: <span style='color:{DIR_STATUS_C};font-weight:700'>CLOCKWISE</span></div>"
    else:
        if motor_client:
            motor_client.send_command("bldc_dir 1")
        bldc_default_dir = 1
        bldc_rev.label = "DIRECTION \u21BA"  # Counter-clockwise arrow (now counter-clockwise)
        bldc_status_ca.text = f"<div style='color:{SUNRISE};font-size:15px'>Direction: <span style='color:{DIR_STATUS_C};font-weight:700'>COUNTER CLOCKWISE</span></div>"


pending_callback = None


def apply_bldc_slider_value():
    global pending_callback
    bldc_rpm = bldc_rpm_slider.value

    # Clamp speed to configured motor_speed_rpm
    try:
        configured_max_rpm = int(bldc_spec_inputs['motor_speed_rpm'].value)
        if bldc_rpm > configured_max_rpm:
            bldc_rpm = configured_max_rpm
            bldc_rpm_slider.value = configured_max_rpm
    except (ValueError, KeyError):
        pass

    if motor_client:
        motor_client.send_command(f"bldc_rpm {bldc_rpm}")
    pending_callback = None


def bldc_schedule_update(attr, old, new):
    global pending_callback
    if pending_callback is not None:
        try:
            curdoc().remove_timeout_callback(pending_callback)
        except ValueError:
            pass
        pending_callback = None
    pending_callback = curdoc().add_timeout_callback(apply_bldc_slider_value, DEBOUNCE_TIMEOUT_MS)


# ==============================================================================
# Stepper Control Callbacks
# ==============================================================================
def stepper_start_on_change():
    global stepper_default_dir
    stepper_status_ss.text = f"<div style='color:{SUNRISE};font-size:15px'>Status: <span style='color:{START_C};font-weight:700'>RUNNING</span></div>"
    if motor_client:
        motor_client.send_command("stepper_start 1")
        # Re-sync direction to hardware after start (in case of mismatch)
        motor_client.send_command(f"stepper_dir {stepper_default_dir}")


def stepper_stop_on_change():
    stepper_status_ss.text = f"<div style='color:{SUNRISE};font-size:15px'>Status: <span style='color:{STOP_C};font-weight:700'>STOPPED</span></div>"
    if motor_client:
        motor_client.send_command("stepper_start 0")


stepper_default_dir = 1


def stepper_rev_on_change():
    global stepper_default_dir
    if stepper_default_dir == 1:
        if motor_client:
            motor_client.send_command("stepper_dir 0")
        stepper_default_dir = 0
        stepper_rev.label = "DIRECTION \u21BB"  # Clockwise arrow (now clockwise)
        stepper_status_ca.text = f"<div style='color:{SUNRISE};font-size:15px'>Direction: <span style='color:{DIR_STATUS_C};font-weight:700'>CLOCKWISE</span></div>"
    else:
        if motor_client:
            motor_client.send_command("stepper_dir 1")
        stepper_default_dir = 1
        stepper_rev.label = "DIRECTION \u21BA"  # Counter-clockwise arrow (now counter-clockwise)
        stepper_status_ca.text = f"<div style='color:{SUNRISE};font-size:15px'>Direction: <span style='color:{DIR_STATUS_C};font-weight:700'>COUNTER CLOCKWISE</span></div>"


pending_callback_stepper = None


def rpm_to_slew_cnt(rpm, microsteps=64, steps_per_rev=200):
    """Convert RPM to SLEW_CNT register value for stepper motor.

    Formula: SLEW_CNT = (240 * 1000000) / (rpm * microsteps * 4 * steps_per_rev)
    Higher SLEW_CNT = slower speed (inverse relationship)

    Register bounds: 0x20 (32) to 0xFF (255)
    - SLEW_CNT=32 → fastest (~234 RPM with 64 microsteps, 200 steps/rev)
    - SLEW_CNT=255 → slowest (~29 RPM with 64 microsteps, 200 steps/rev)
    """
    if rpm <= 0:
        return 79  # Default 0x4F
    slew_cnt = (240 * 1000000) // (rpm * microsteps * 4 * steps_per_rev)
    # Clamp to valid register range: 0x20 to 0xFF
    slew_cnt = max(STEPPER_SLEW_CNT_MIN, min(STEPPER_SLEW_CNT_MAX, slew_cnt))
    return int(slew_cnt)


def apply_stepper_slider_value():
    global pending_callback_stepper
    rpm = stepper_rpm_slider.value
    slew_cnt = rpm_to_slew_cnt(rpm)
    if motor_client:
        motor_client.send_command(f"stepper_rpm {slew_cnt}")
    # Sync config panel speed input with slider
    if 'speed_rpm' in stepper_param_inputs:
        stepper_param_inputs['speed_rpm'].value = str(rpm)
    pending_callback_stepper = None


def stepper_schedule_update(attr, old, new):
    global pending_callback_stepper
    if pending_callback_stepper is not None:
        try:
            curdoc().remove_timeout_callback(pending_callback_stepper)
        except ValueError:
            pass
        pending_callback_stepper = None
    pending_callback_stepper = curdoc().add_timeout_callback(apply_stepper_slider_value, DEBOUNCE_TIMEOUT_MS)


# --- Connect widget events ---
bldc_start.on_click(bldc_start_on_change)
bldc_stop.on_click(bldc_stop_on_change)
bldc_rev.on_click(bldc_rev_on_change)
bldc_rpm_slider.on_change("value", bldc_schedule_update)

stepper_start.on_click(stepper_start_on_change)
stepper_stop.on_click(stepper_stop_on_change)
stepper_rev.on_click(stepper_rev_on_change)
stepper_rpm_slider.on_change("value", stepper_schedule_update)


# ==============================================================================
# Parameter Apply/Refresh Functions (v5 - simplified parameters)
# ==============================================================================
def apply_bldc_params():
    """Apply all BLDC parameters from input fields."""
    bldc_param_status.text = "<div style='color:#2196F3;font-size:12px'>Applying parameter...</div>"

    # Collect params data
    params_data = {
        'current_pi_kp': bldc_param_inputs['current_pi_kp'].value,
        'current_pi_ki': bldc_param_inputs['current_pi_ki'].value,
        'speed_pi_kp': bldc_param_inputs['speed_pi_kp'].value,
        'speed_pi_ki': bldc_param_inputs['speed_pi_ki'].value,
        'startup_mode': bldc_startup_mode_select.value,
        'auto_restart': 0 in bldc_control_flags.active,
        'soft_stop': 1 in bldc_control_flags.active,
        'closed_loop_speed_rpm': bldc_param_inputs['closed_loop_speed_rpm'].value,
        'open_loop_current_pct': bldc_param_inputs['open_loop_current_pct'].value,
        'open_loop_voltage_pct': bldc_param_inputs['open_loop_voltage_pct'].value,
        'angle_pi_kp': bldc_param_inputs['angle_pi_kp'].value,
        'angle_pi_ki': bldc_param_inputs['angle_pi_ki'].value,
    }

    # Collect specs data
    specs_data = {
        'dc_voltage_mV': bldc_spec_inputs['dc_voltage_mV'].value,
        'motor_current_mA': bldc_spec_inputs['motor_current_mA'].value,
        'motor_speed_rpm': bldc_spec_inputs['motor_speed_rpm'].value,
        'pole_pairs': bldc_spec_inputs['pole_pairs'].value,
        'resistance_mOhm': bldc_spec_inputs['resistance_mOhm'].value,
        'inductance_uH': bldc_spec_inputs['inductance_uH'].value,
        'switching_freq_kHz': bldc_spec_inputs['switching_freq_kHz'].value,
    }

    if motor_client:
        # Set specs first
        motor_client.send_param_request('set_specs', 'bldc', {'specs': specs_data})
        # Set params
        motor_client.send_param_request('set_params', 'bldc', {'params': params_data})
        # Apply to hardware
        motor_client.send_param_request('apply', 'bldc')
        # Re-send current slider speed so new parameters take effect
        bldc_rpm = bldc_rpm_slider.value
        motor_client.send_command(f"bldc_rpm {bldc_rpm}")

    bldc_param_status.text = f"<div style='color:{START_C};font-size:12px'>Parameter applied</div>"


def apply_stepper_params():
    """Apply all Stepper parameters from input fields."""
    stepper_param_status.text = "<div style='color:#2196F3;font-size:12px'>Applying parameter...</div>"

    # Collect params data
    params_data = {
        'current_pi_kp': stepper_param_inputs['current_pi_kp'].value,
        'current_pi_ki': stepper_param_inputs['current_pi_ki'].value,
        'current_reference_mA': stepper_param_inputs['current_reference_mA'].value,
        'cmd_steps': stepper_param_inputs['cmd_steps'].value,
        'speed_rpm': stepper_param_inputs['speed_rpm'].value,
    }

    # Collect specs data
    specs_data = {
        'dc_voltage_mV': stepper_spec_inputs['dc_voltage_mV'].value,
        'motor_current_mA': stepper_spec_inputs['motor_current_mA'].value,
        'step_number': stepper_spec_inputs['step_number'].value,
        'microstep_resolution': stepper_spec_inputs['microstep_resolution'].value,
        'resistance_mOhm': stepper_spec_inputs['resistance_mOhm'].value,
        'inductance_uH': stepper_spec_inputs['inductance_uH'].value,
        'switching_freq_kHz': stepper_spec_inputs['switching_freq_kHz'].value,
    }

    if motor_client:
        # Set specs first
        motor_client.send_param_request('set_specs', 'stepper', {'specs': specs_data})
        # Set params
        motor_client.send_param_request('set_params', 'stepper', {'params': params_data})
        # Apply to hardware
        motor_client.send_param_request('apply', 'stepper')
        # Re-send current slider speed so new parameters take effect
        rpm = stepper_rpm_slider.value
        slew_cnt = rpm_to_slew_cnt(rpm)
        motor_client.send_command(f"stepper_rpm {slew_cnt}")

    # Sync slider with config panel speed value
    try:
        speed_rpm = int(stepper_param_inputs['speed_rpm'].value)
        # Clamp to slider range
        speed_rpm = max(stepper_rpm_slider.start, min(stepper_rpm_slider.end, speed_rpm))
        stepper_rpm_slider.value = speed_rpm
    except (ValueError, KeyError):
        pass

    stepper_param_status.text = f"<div style='color:{START_C};font-size:12px'>Parameter applied</div>"


bldc_refresh_timeout = None
stepper_refresh_timeout = None


def refresh_bldc_params():
    """Refresh BLDC parameters from hardware."""
    global bldc_refresh_timeout
    bldc_param_status.text = "<div style='color:#FF9800;font-size:12px'>Refreshing...</div>"

    if bldc_refresh_timeout is not None:
        try:
            curdoc().remove_timeout_callback(bldc_refresh_timeout)
        except ValueError:
            pass

    def timeout_handler():
        global bldc_refresh_timeout
        bldc_param_status.text = "<div style='color:#F44336;font-size:12px'>Timeout - no response</div>"
        bldc_refresh_timeout = None

    def handle_response(response):
        global bldc_refresh_timeout
        if bldc_refresh_timeout is not None:
            try:
                curdoc().remove_timeout_callback(bldc_refresh_timeout)
            except ValueError:
                pass
            bldc_refresh_timeout = None

        if response.get('success'):
            # Update specs
            if 'specs' in response:
                specs = response['specs']
                for key, value in specs.items():
                    if key in bldc_spec_inputs and value is not None:
                        bldc_spec_inputs[key].value = str(value)

            # Update params
            if 'params' in response:
                params = response['params']
                for key, value in params.items():
                    if key in bldc_param_inputs and value is not None:
                        bldc_param_inputs[key].value = str(value)

                # Update startup mode select
                if 'startup_mode' in params:
                    bldc_startup_mode_select.value = params['startup_mode']

                # Update control flags
                active = []
                if params.get('auto_restart'):
                    active.append(0)
                if params.get('soft_stop'):
                    active.append(1)
                bldc_control_flags.active = active

            bldc_param_status.text = "<div style='color:#4CAF50;font-size:12px'>Parameter refreshed</div>"
        else:
            bldc_param_status.text = "<div style='color:#F44336;font-size:12px'>Refresh failed</div>"

    if motor_client:
        motor_client.send_param_request('get_all', 'bldc', callback=handle_response)
        bldc_refresh_timeout = curdoc().add_timeout_callback(timeout_handler, 3000)
    else:
        bldc_param_status.text = "<div style='color:#F44336;font-size:12px'>ROS2 not connected</div>"


def set_bldc_defaults():
    """Reset BLDC parameters to default values."""
    # Set specs defaults
    for key, value in DEFAULT_BLDC_SPECS.items():
        if key in bldc_spec_inputs:
            bldc_spec_inputs[key].value = str(value)

    # Set params defaults
    for key, value in DEFAULT_BLDC_PARAMS.items():
        if key in bldc_param_inputs:
            bldc_param_inputs[key].value = str(value)

    # Set startup mode
    bldc_startup_mode_select.value = DEFAULT_BLDC_PARAMS['startup_mode']

    # Set control flags
    active = []
    if DEFAULT_BLDC_PARAMS['auto_restart']:
        active.append(0)
    if DEFAULT_BLDC_PARAMS['soft_stop']:
        active.append(1)
    bldc_control_flags.active = active

    bldc_param_status.text = "<div style='color:#9C27B0;font-size:12px'>Defaults set - click SET</div>"


def refresh_stepper_params():
    """Refresh Stepper parameters from hardware."""
    global stepper_refresh_timeout
    stepper_param_status.text = "<div style='color:#FF9800;font-size:12px'>Refreshing...</div>"

    if stepper_refresh_timeout is not None:
        try:
            curdoc().remove_timeout_callback(stepper_refresh_timeout)
        except ValueError:
            pass

    def timeout_handler():
        global stepper_refresh_timeout
        stepper_param_status.text = "<div style='color:#F44336;font-size:12px'>Timeout - no response</div>"
        stepper_refresh_timeout = None

    def handle_response(response):
        global stepper_refresh_timeout
        if stepper_refresh_timeout is not None:
            try:
                curdoc().remove_timeout_callback(stepper_refresh_timeout)
            except ValueError:
                pass
            stepper_refresh_timeout = None

        if response.get('success'):
            # Update specs
            if 'specs' in response:
                specs = response['specs']
                for key, value in specs.items():
                    if key in stepper_spec_inputs and value is not None:
                        stepper_spec_inputs[key].value = str(value)

            # Update params
            if 'params' in response:
                params = response['params']
                for key, value in params.items():
                    if key in stepper_param_inputs and value is not None:
                        stepper_param_inputs[key].value = str(value)

                # Sync slider with speed_rpm from params
                if 'speed_rpm' in params and params['speed_rpm'] is not None:
                    try:
                        speed_rpm = int(params['speed_rpm'])
                        speed_rpm = max(stepper_rpm_slider.start, min(stepper_rpm_slider.end, speed_rpm))
                        stepper_rpm_slider.value = speed_rpm
                    except (ValueError, TypeError):
                        pass

            stepper_param_status.text = "<div style='color:#4CAF50;font-size:12px'>Parameter refreshed</div>"
        else:
            stepper_param_status.text = "<div style='color:#F44336;font-size:12px'>Refresh failed</div>"

    if motor_client:
        motor_client.send_param_request('get_all', 'stepper', callback=handle_response)
        stepper_refresh_timeout = curdoc().add_timeout_callback(timeout_handler, 3000)
    else:
        stepper_param_status.text = "<div style='color:#F44336;font-size:12px'>ROS2 not connected</div>"


def set_stepper_defaults():
    """Reset Stepper parameters to default values."""
    # Set specs defaults
    for key, value in DEFAULT_STEPPER_SPECS.items():
        if key in stepper_spec_inputs:
            stepper_spec_inputs[key].value = str(value)

    # Set params defaults
    for key, value in DEFAULT_STEPPER_PARAMS.items():
        if key in stepper_param_inputs:
            stepper_param_inputs[key].value = str(value)

    # Sync slider with default speed_rpm
    stepper_rpm_slider.value = DEFAULT_STEPPER_PARAMS['speed_rpm']

    stepper_param_status.text = "<div style='color:#9C27B0;font-size:12px'>Defaults set - click SET</div>"


# ==============================================================================
# BLDC Parameter Panel - Sub-column 1: Motor Parameters
# ==============================================================================
bldc_motor_params_heading = Div(text="<b style='color:#76CBC0;font-size:16px'>Motor Parameters</b>", width=200)

# PI Controllers section
bldc_pi_section = Div(text="<b style='color:#5DADE2;font-size:14px'>PI Controllers</b>", width=180)

bldc_current_pi_row = row(
    column(
        create_param_row("Current PI Kp:", DEFAULT_BLDC_PARAMS['current_pi_kp'], bldc_param_inputs, 'current_pi_kp'),
        create_param_row("Current PI Ki:", DEFAULT_BLDC_PARAMS['current_pi_ki'], bldc_param_inputs, 'current_pi_ki'),
    )
)

bldc_speed_pi_row = row(
    column(
        create_param_row("Speed PI Kp:", DEFAULT_BLDC_PARAMS['speed_pi_kp'], bldc_param_inputs, 'speed_pi_kp'),
        create_param_row("Speed PI Ki:", DEFAULT_BLDC_PARAMS['speed_pi_ki'], bldc_param_inputs, 'speed_pi_ki'),
    )
)

# Startup Mode dropdown
startup_mode_label = Paragraph(text="Startup Mode:", width=130, style={'color': LABEL_C, 'font-size': '14px'})
bldc_startup_mode_select = Select(value="C/F", options=["C/F", "V"], width=80)

# Control Flags
bldc_control_flags = CheckboxGroup(labels=["Auto Restart", "Soft Stop"], active=[0])

# Sensorless Parameters section
bldc_sensorless_section = Div(text="<b style='color:#5DADE2;font-size:14px'>Sensorless</b>", width=180)

bldc_sensorless_params = column(
    create_param_row("Closed Loop Speed (RPM):", DEFAULT_BLDC_PARAMS['closed_loop_speed_rpm'], bldc_param_inputs, 'closed_loop_speed_rpm'),
    create_param_row("Open Loop Current %:", DEFAULT_BLDC_PARAMS['open_loop_current_pct'], bldc_param_inputs, 'open_loop_current_pct'),
    create_param_row("Open Loop Voltage %:", DEFAULT_BLDC_PARAMS['open_loop_voltage_pct'], bldc_param_inputs, 'open_loop_voltage_pct'),
    create_param_row("Angle PI Kp:", DEFAULT_BLDC_PARAMS['angle_pi_kp'], bldc_param_inputs, 'angle_pi_kp'),
    create_param_row("Angle PI Ki:", DEFAULT_BLDC_PARAMS['angle_pi_ki'], bldc_param_inputs, 'angle_pi_ki'),
)

bldc_motor_params_col = column(
    bldc_motor_params_heading,
    Spacer(height=5),
    bldc_pi_section,
    bldc_current_pi_row,
    bldc_speed_pi_row,
    Spacer(height=5),
    row(startup_mode_label, bldc_startup_mode_select),
    bldc_control_flags,
    Spacer(height=10),
    bldc_sensorless_section,
    bldc_sensorless_params,
    width=220
)

# ==============================================================================
# BLDC Parameter Panel - Sub-column 2: Motor Specifications
# ==============================================================================
bldc_motor_specs_heading = Div(text="<b style='color:#76CBC0;font-size:13px'>Motor Specifications</b>", width=200)

bldc_motor_specs_col = column(
    bldc_motor_specs_heading,
    Spacer(height=5),
    create_param_row("DC Voltage (mV):", DEFAULT_BLDC_SPECS['dc_voltage_mV'], bldc_spec_inputs, 'dc_voltage_mV'),
    create_param_row("Motor Current (mA):", DEFAULT_BLDC_SPECS['motor_current_mA'], bldc_spec_inputs, 'motor_current_mA'),
    create_param_row("Motor Speed (RPM):", DEFAULT_BLDC_SPECS['motor_speed_rpm'], bldc_spec_inputs, 'motor_speed_rpm'),
    create_param_row("No. of Pole Pairs:", DEFAULT_BLDC_SPECS['pole_pairs'], bldc_spec_inputs, 'pole_pairs'),
    create_param_row("Motor Resistance (mOhm):", DEFAULT_BLDC_SPECS['resistance_mOhm'], bldc_spec_inputs, 'resistance_mOhm'),
    create_param_row("Motor Inductance (uH):", DEFAULT_BLDC_SPECS['inductance_uH'], bldc_spec_inputs, 'inductance_uH'),
    create_param_row("Switching Freq (kHz):", DEFAULT_BLDC_SPECS['switching_freq_kHz'], bldc_spec_inputs, 'switching_freq_kHz'),
    width=220
)

# Action buttons
bldc_apply_btn = Button(label="SET", width=60, height=28)
bldc_apply_btn.css_classes = ['apply-button']
bldc_apply_btn.on_click(apply_bldc_params)

bldc_refresh_btn = Button(label="GET", width=60, height=28)
bldc_refresh_btn.css_classes = ['refresh-button']
bldc_refresh_btn.on_click(refresh_bldc_params)

bldc_defaults_btn = Button(label="Defaults", width=70, height=28)
bldc_defaults_btn.css_classes = ['defaults-button']
bldc_defaults_btn.on_click(set_bldc_defaults)

# Combined BLDC parameter panel
bldc_param_panel = column(
    row(bldc_motor_params_col, Spacer(width=20), bldc_motor_specs_col),
    Spacer(height=10),
    row(bldc_apply_btn, Spacer(width=10), bldc_refresh_btn, Spacer(width=10), bldc_defaults_btn, Spacer(width=15), bldc_param_status),
)

# ==============================================================================
# Stepper Parameter Panel - Sub-column 1: Motor Parameters
# ==============================================================================
stepper_motor_params_heading = Div(text="<b style='color:#76CBC0;font-size:16px'>Motor Parameters</b>", width=200)

# PI Controllers section
stepper_pi_section = Div(text="<b style='color:#5DADE2;font-size:14px'>PI Controllers</b>", width=180)

stepper_current_pi_row = row(
    column(
        create_param_row("Current PI Kp:", DEFAULT_STEPPER_PARAMS['current_pi_kp'], stepper_param_inputs, 'current_pi_kp'),
        create_param_row("Current PI Ki:", DEFAULT_STEPPER_PARAMS['current_pi_ki'], stepper_param_inputs, 'current_pi_ki'),
    )
)

# Stepper Control section
stepper_control_section = Div(text="<b style='color:#5DADE2;font-size:14px'>Stepper Control</b>", width=180)

stepper_control_params = column(
    create_param_row("Current Reference (mA):", DEFAULT_STEPPER_PARAMS['current_reference_mA'], stepper_param_inputs, 'current_reference_mA'),
    create_param_row("Command Steps:", DEFAULT_STEPPER_PARAMS['cmd_steps'], stepper_param_inputs, 'cmd_steps'),
    create_param_row("Speed (RPM):", DEFAULT_STEPPER_PARAMS['speed_rpm'], stepper_param_inputs, 'speed_rpm'),
)

stepper_motor_params_col = column(
    stepper_motor_params_heading,
    Spacer(height=5),
    stepper_pi_section,
    stepper_current_pi_row,
    Spacer(height=10),
    stepper_control_section,
    stepper_control_params,
    width=220
)

# ==============================================================================
# Stepper Parameter Panel - Sub-column 2: Motor Specifications
# ==============================================================================
stepper_motor_specs_heading = Div(text="<b style='color:#76CBC0;font-size:13px'>Motor Specifications</b>", width=200)

stepper_motor_specs_col = column(
    stepper_motor_specs_heading,
    Spacer(height=5),
    create_param_row("DC Voltage (mV):", DEFAULT_STEPPER_SPECS['dc_voltage_mV'], stepper_spec_inputs, 'dc_voltage_mV'),
    create_param_row("Motor Current (mA):", DEFAULT_STEPPER_SPECS['motor_current_mA'], stepper_spec_inputs, 'motor_current_mA'),
    create_param_row("Step Number:", DEFAULT_STEPPER_SPECS['step_number'], stepper_spec_inputs, 'step_number'),
    create_param_row("Microstep Resolution:", DEFAULT_STEPPER_SPECS['microstep_resolution'], stepper_spec_inputs, 'microstep_resolution'),
    create_param_row("Motor Resistance (mOhm):", DEFAULT_STEPPER_SPECS['resistance_mOhm'], stepper_spec_inputs, 'resistance_mOhm'),
    create_param_row("Motor Inductance (uH):", DEFAULT_STEPPER_SPECS['inductance_uH'], stepper_spec_inputs, 'inductance_uH'),
    create_param_row("Switching Freq (kHz):", DEFAULT_STEPPER_SPECS['switching_freq_kHz'], stepper_spec_inputs, 'switching_freq_kHz'),
    width=220
)

# Action buttons
stepper_apply_btn = Button(label="SET", width=60, height=28)
stepper_apply_btn.css_classes = ['apply-button']
stepper_apply_btn.on_click(apply_stepper_params)

stepper_refresh_btn = Button(label="GET", width=60, height=28)
stepper_refresh_btn.css_classes = ['refresh-button']
stepper_refresh_btn.on_click(refresh_stepper_params)

stepper_defaults_btn = Button(label="Defaults", width=70, height=28)
stepper_defaults_btn.css_classes = ['defaults-button']
stepper_defaults_btn.on_click(set_stepper_defaults)

# Combined Stepper parameter panel
stepper_param_panel = column(
    row(stepper_motor_params_col, Spacer(width=20), stepper_motor_specs_col),
    Spacer(height=10),
    row(stepper_apply_btn, Spacer(width=10), stepper_refresh_btn, Spacer(width=10), stepper_defaults_btn, Spacer(width=15), stepper_param_status),
)


# --- Plot setup ---
x_vals = list(range(NUM_POINTS))
y_vals = [0] * NUM_POINTS

# Speed plot
source1 = ColumnDataSource(data=dict(x=x_vals, y1=y_vals))
plot1 = figure(
    title="Speed",
    x_axis_label="Index",
    y_axis_label="Speed Value(RPM)",
    x_range=(0, NUM_POINTS - 1),
    y_range=(0, 4500),
    width=380,
    height=200,
    toolbar_location=None
)
plot1.xaxis.axis_label_text_font_size = "9pt"
plot1.yaxis.axis_label_text_font_size = "9pt"
plot1.xaxis.major_label_text_font_size = "8pt"
plot1.yaxis.major_label_text_font_size = "8pt"
plot1.yaxis.ticker = FixedTicker(ticks=[0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500])
plot1.background_fill_color = None
plot1.border_fill_color = None
plot1.outline_line_color = None
plot1.line(x='x', y='y1', source=source1, line_width=2, line_color=SPEED_PLOT_C)

# Torque plot (free range - no fixed y_range or ticker)
source2 = ColumnDataSource(data=dict(x=x_vals, y2=y_vals))
plot2 = figure(
    title="Torque",
    x_axis_label="Index",
    y_axis_label="Torque (pu)",
    x_range=(0, NUM_POINTS - 1),
    width=380,
    height=180,
    toolbar_location=None
)
plot2.xaxis.axis_label_text_font_size = "9pt"
plot2.yaxis.axis_label_text_font_size = "9pt"
plot2.xaxis.major_label_text_font_size = "8pt"
plot2.yaxis.major_label_text_font_size = "8pt"
plot2.background_fill_color = None
plot2.border_fill_color = None
plot2.outline_line_color = None
plot2.line(x='x', y='y2', source=source2, line_width=2, line_color="yellow")

# Phase current plot (free range - no fixed y_range or ticker)
source3 = ColumnDataSource(data=dict(x=x_vals, y3=y_vals))
plot3 = figure(
    title="Phase Current",
    x_axis_label="Index",
    y_axis_label="Phase Current(mA)",
    x_range=(0, NUM_POINTS - 1),
    width=380,
    height=180,
    toolbar_location=None
)
plot3.xaxis.axis_label_text_font_size = "9pt"
plot3.yaxis.axis_label_text_font_size = "9pt"
plot3.xaxis.major_label_text_font_size = "8pt"
plot3.yaxis.major_label_text_font_size = "8pt"
plot3.background_fill_color = None
plot3.border_fill_color = None
plot3.outline_line_color = None
plot3.line(x='x', y='y3', source=source3, line_width=2, line_color=PHASE_PLOT_C)


def fix_speed_value(s):
    ADC_SPEED_XOR_MASK = 16362
    # Work with unsigned 16-bit value for correct XOR behavior
    unsigned_val = s & 0xFFFF
    # If value is way above max expected RPM , reverse XOR
    if unsigned_val > 6000:
        corrected = unsigned_val ^ ADC_SPEED_XOR_MASK
        return corrected
    return abs(s)


def read_iio():
    try:
        with open(IIO_DEVICE_PATH, "rb") as f:
            raw = f.read(TOTAL_BYTES)
        if len(raw) != TOTAL_BYTES:
            return None, None, None

        # Read all channels as signed 16-bit
        fmt = "<" + "h" * NUM_SAMPLES * CHANNELS
        unpacked = struct.unpack(fmt, raw)

        # Speed channel needs check
        ch0_raw = unpacked[::CHANNELS]
        ch0 = tuple(fix_speed_value(s) for s in ch0_raw)

        ch1 = unpacked[1::CHANNELS]     # IQ (for Torque plot)
        ch2 = unpacked[2::CHANNELS]     # Current IB (for Phase Current plot)
        return ch0, ch1, ch2
    except Exception as e:
        return None, None, None


def get_speed_display_scale_factor():
    """Get combined scale factor to convert estimator speed to actual mechanical RPM.

    The speed feedback from hardware is normalized. We need to scale based on:
    1. Pole pairs: affects theta_factor and electrical frequency
    2. Motor Speed RPM: affects speed_scale normalization

    Base calibration: 4 poles, 4000 RPM

    Combined scale = (current_poles / base_poles) * (current_motor_rpm / base_motor_rpm)

    Examples:
    - At 4 poles, 4000 RPM: scale = (4/4) * (4000/4000) = 1.0
    - At 2 poles, 4000 RPM: scale = (2/4) * (4000/4000) = 0.5 (half speed)
    - At 4 poles, 2000 RPM: scale = (4/4) * (2000/4000) = 0.5 (half speed)
    - At 2 poles, 2000 RPM: scale = (2/4) * (2000/4000) = 0.25 (quarter speed)
    """
    BASE_POLES = 4
    BASE_MOTOR_RPM = 4000

    pole_scale = 1.0
    rpm_scale = 1.0

    try:
        current_poles = int(bldc_spec_inputs['pole_pairs'].value)
        if current_poles > 0:
            pole_scale = current_poles / float(BASE_POLES)
    except (ValueError, KeyError, NameError):
        pass

    try:
        current_motor_rpm = int(bldc_spec_inputs['motor_speed_rpm'].value)
        if current_motor_rpm > 0:
            rpm_scale = current_motor_rpm / float(BASE_MOTOR_RPM)
    except (ValueError, KeyError, NameError):
        pass

    return pole_scale * rpm_scale


def update_data():
    # Read all channels from IIO buffer device
    # ch0 = Speed (from REG_SPEED_BLDC - bldc_speed_filtered_o from estimator)
    # ch1 = IQ (from REG_IQ_BLDC) - for Torque plot
    # ch2 = Current IB (from REG_ADC_IB_BLDC - converted to mV by driver) - for Phase Current plot
    ch0, ch1, ch2 = read_iio()
    if ch0 and ch1 and ch2:
        x_vals1 = list(range(len(ch0)))
        x_vals2 = list(range(len(ch1)))
        x_vals3 = list(range(len(ch2)))

        # ch0 is electrical speed from estimator - scale to mechanical RPM
        # The estimator is calibrated for base settings (4 poles, 4000 RPM)
        # Scale based on current pole_pairs and motor_speed_rpm settings
        scale = get_speed_display_scale_factor()
        speed_rpm = [int(s * scale) for s in ch0]
        source1.stream(dict(x=x_vals1, y1=speed_rpm), rollover=NUM_SAMPLES)

        # ch1 is IQ data (Torque plot) - multiply by 4 for display scaling
        source2.stream(dict(x=x_vals2, y2=[abs(i) * 4 for i in ch1]), rollover=NUM_SAMPLES)

        # ch2 is current data in mV (Phase Current plot)
        # Zero out phase current when motor is stopped (registers retain stale values)
        if "RUNNING" in bldc_status_ss.text:
            source3.stream(dict(x=x_vals3, y3=ch2), rollover=NUM_SAMPLES)
        else:
            source3.stream(dict(x=x_vals3, y3=[0] * len(ch2)), rollover=NUM_SAMPLES)


curdoc().add_periodic_callback(update_data, 1000)


# --- Layout ---
bldc_heading = Div(text="<h3>BLDC Motor</h3>", width=200)
stepper_heading = Div(text="<h3>STEPPER Motor</h3>", width=200)
plot_heading = Div(text="<h3>LIVE BLDC PLOTS</h3>", width=200)

# Configuration slider toggle button
config_toggle_state = [False]  # False = OFF (default), True = ON
params_heading = Div(text="<h3 style='display:inline;margin-right:10px;'>CONFIGURATION</h3>", width=150)

# Slider toggle button (no label, just a visual slider)
config_slider_btn = Button(label="", width=60, height=28)

# CSS for slider toggle button
checkbox_css = Div(text="""
<style>
    .slider-toggle-off .bk-btn {
        background: linear-gradient(to right, #555 50%, #333 50%) !important;
        border: 2px solid #666 !important;
        border-radius: 14px !important;
        position: relative !important;
        padding: 0 !important;
        box-shadow: inset 2px 2px 4px rgba(0,0,0,0.3) !important;
    }
    .slider-toggle-off .bk-btn::after {
        content: '' !important;
        position: absolute !important;
        left: 4px !important;
        top: 50% !important;
        transform: translateY(-50%) !important;
        width: 20px !important;
        height: 20px !important;
        background: #aaa !important;
        border-radius: 50% !important;
        box-shadow: 1px 1px 3px rgba(0,0,0,0.3) !important;
        transition: left 0.2s ease !important;
    }
    .slider-toggle-off .bk-btn:hover {
        border-color: #888 !important;
    }
    .slider-toggle-on .bk-btn {
        background: linear-gradient(to right, #1DE466 50%, #17a34a 50%) !important;
        border: 2px solid #1DE466 !important;
        border-radius: 14px !important;
        position: relative !important;
        padding: 0 !important;
        box-shadow: inset -2px 2px 4px rgba(0,0,0,0.2) !important;
    }
    .slider-toggle-on .bk-btn::after {
        content: '' !important;
        position: absolute !important;
        right: 4px !important;
        top: 50% !important;
        transform: translateY(-50%) !important;
        width: 20px !important;
        height: 20px !important;
        background: #fff !important;
        border-radius: 50% !important;
        box-shadow: -1px 1px 3px rgba(0,0,0,0.3) !important;
    }
    .slider-toggle-on .bk-btn:hover {
        border-color: #17c653 !important;
    }
</style>
""", width=0, height=0)

# Set initial state (OFF)
config_slider_btn.css_classes = ['slider-toggle-off']

# Container to hold the parameter content (initially empty but with fixed width)
config_content_container = column(width=450)

# BLDC Control column
bldc_control = column(
    bldc_heading,
    bldc_rpm_slider,
    row(bldc_start, bldc_stop),
    bldc_status_ss,
    Spacer(height=8),
    bldc_rev,
    bldc_status_ca,
)

# Stepper Control column
stepper_control = column(
    stepper_heading,
    stepper_rpm_slider,
    row(stepper_start, stepper_stop),
    stepper_status_ss,
    Spacer(height=8),
    stepper_rev,
    stepper_status_ca,
)

# --- Parameter Tab Buttons ---
bldc_param_tab_btn = Button(label="BLDC", width=80, height=32)
bldc_param_tab_btn.css_classes = ['param-tab-active']
stepper_param_tab_btn = Button(label="STEPPER", width=80, height=32)
stepper_param_tab_btn.css_classes = ['param-tab-inactive']

# Container for parameter panels
param_panel_container = column(bldc_param_panel)


def show_bldc_params():
    """Switch to BLDC params panel."""
    param_panel_container.children = [bldc_param_panel]
    bldc_param_tab_btn.css_classes = ['param-tab-active']
    stepper_param_tab_btn.css_classes = ['param-tab-inactive']


def show_stepper_params():
    """Switch to Stepper params panel."""
    param_panel_container.children = [stepper_param_panel]
    bldc_param_tab_btn.css_classes = ['param-tab-inactive']
    stepper_param_tab_btn.css_classes = ['param-tab-active']


bldc_param_tab_btn.on_click(show_bldc_params)
stepper_param_tab_btn.on_click(show_stepper_params)


def config_slider_click():
    """Toggle configuration panel visibility."""
    config_toggle_state[0] = not config_toggle_state[0]
    if config_toggle_state[0]:
        # Show configuration panel
        config_slider_btn.css_classes = ['slider-toggle-on']
        config_content_container.children = [
            row(bldc_param_tab_btn, Spacer(width=10), stepper_param_tab_btn),
            Spacer(height=10),
            param_panel_container
        ]
    else:
        # Hide configuration panel
        config_slider_btn.css_classes = ['slider-toggle-off']
        config_content_container.children = []


config_slider_btn.on_click(config_slider_click)

# Plot column - with gaps between plots
plot_column = column(
    plot_heading,
    plot1,
    Spacer(height=15),
    plot2,
    Spacer(height=15),
    plot3,
    margin=(0, 30, 0, 30)
)

# Control section (1st column) - added left margin and gap between BLDC and Stepper
control_section = column(
    bldc_control,
    Spacer(height=30),
    stepper_control,
    margin=(0, 30, 0, 40)
)

# Params section (3rd column) - with slider toggle and fixed width
param_section = column(
    row(params_heading, config_slider_btn),
    config_content_container,
    margin=(0, 10, 0, 30),
    width=480
)

# Main layout: Control (1st) -> Live Plots (2nd) -> Params (3rd)
main_content = row(
    control_section,
    plot_column,
    param_section
)

# ROS2 error banner (shown when ROS2 initialization fails)
ros2_error_banner = Div(text="", visible=False)
if ros2_error_message:
    ros2_error_banner.text = f"""<div style='background-color:#8B0000;color:white;padding:12px 20px;
        font-size:14px;font-weight:bold;text-align:center;border-radius:4px;margin-bottom:10px;'>
        ⚠ ROS2 UNAVAILABLE: Motor control commands will not work. Error: {ros2_error_message}
        </div>"""
    ros2_error_banner.visible = True

layout1 = layout(
    css_style,
    radio_css,
    dir_css,
    checkbox_css,
    column(
        ros2_error_banner,
        row(Spacer(width=1), title1, Spacer(width=1), sizing_mode="stretch_width"),
        main_content,
        sizing_mode="stretch_width"
    )
)

curdoc().title = "PolarFire® SoC Motor Control Dashboard"
curdoc().theme = 'night_sky'
curdoc().add_root(layout1)
