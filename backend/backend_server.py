import os
import subprocess
import serial
import serial.tools.list_ports
from flask import Flask, jsonify, request
from flask_cors import CORS
import time
import threading
import json
import math
import re

# --- FEETECH SDK-like components for direct serial communication ---
# This is necessary because we need to control torque directly and read positions
# for drag-teaching, which is not part of the existing ROS 2 control setup.
# This logic is adapted from your provided feetech_torque_gui.py and rotatum_arm_comm.py.

# Control table address
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_STS_PRESENT_POSITION = 56

# Default setting
BAUDRATE = 1000000  # SCServo default baudrate
SCS_LOBYTE = lambda w: w & 0xFF
SCS_HIBYTE = lambda w: (w >> 8) & 0xFF
SCS_MAKELONG = lambda a, b: (a << 8) | b

class FeetechComm:
    """
    A helper class to encapsulate direct, short-lived communication with Feetech servos.
    Each method opens the port, performs an action, and closes it, to avoid
    conflicts with the main ROS hardware interface node.
    """
    def __init__(self, port, baudrate=BAUDRATE):
        self.port_name = port
        self.baudrate = baudrate
        self.ser = None

    def _open_port(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            return True
        except serial.SerialException as e:
            print(f"Error opening port {self.port_name}: {e}")
            self.ser = None
            return False

    def _close_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _calculate_checksum(self, packet_data):
        return (~sum(packet_data)) & 0xFF

    def set_torque_for_all(self, enable, servo_ids):
        """Enable or disable torque for a list of servos."""
        if not self._open_port():
            return False, f"Port {self.port_name} could not be opened. Is it in use by another process (like the main ROS driver)?"

        all_success = True
        errors = []
        try:
            for servo_id in servo_ids:
                # Instruction packet: FF FF ID Len Inst Param... CS
                instruction = 0x03  # WRITE_DATA
                length = 4  # Len (1) + Inst (1) + Addr (1) + Value (1)
                params = [ADDR_SCS_TORQUE_ENABLE, 1 if enable else 0]
                
                packet_data = [servo_id, length, instruction] + params
                checksum = self._calculate_checksum(packet_data)
                
                packet = bytearray([0xFF, 0xFF] + packet_data + [checksum])
                
                self.ser.write(packet)
                time.sleep(0.01) # Small delay between commands
        except serial.SerialException as e:
            all_success = False
            errors.append(str(e))
        finally:
            self._close_port()

        if all_success:
            return True, "Commands sent successfully"
        else:
            return False, f"Some commands failed: {'; '.join(errors)}"

    def read_all_positions(self, servo_ids, joint_offsets):
        """Read the position of all specified servos."""
        if not self._open_port():
            return None, f"Port {self.port_name} could not be opened. Is it in use by another process?"

        positions = {}
        try:
            for i, servo_id in enumerate(servo_ids):
                instruction = 0x02  # READ_DATA
                length = 4
                params = [ADDR_STS_PRESENT_POSITION, 2] # Read 2 bytes from this address
                
                packet_data = [servo_id, length, instruction] + params
                checksum = self._calculate_checksum(packet_data)
                
                packet = bytearray([0xFF, 0xFF] + packet_data + [checksum])

                self.ser.flushInput()
                self.ser.write(packet)
                # Expected response: FF FF ID Len Err PosL PosH CS
                response = self.ser.read(8)

                if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
                    if response[4] != 0:
                        raise IOError(f"Servo {servo_id} returned error code: {response[4]}")
                    
                    position_count = SCS_MAKELONG(response[6], response[5])
                    # Convert from Feetech counts to radians
                    rad = (2 * math.pi * (position_count - joint_offsets[i])) / 4096.0
                    positions[f'j{i+1}'] = rad
                else:
                    raise IOError(f"Invalid or no response from servo {servo_id}")
                time.sleep(0.01)
        except (serial.SerialException, IOError) as e:
            self._close_port()
            return None, str(e)
        
        self._close_port()
        return positions, "Success"

# --- Flask App Setup ---
app = Flask(__name__)
CORS(app)

# Global dictionary to keep track of active ROS processes
active_ros_processes = {}

# --- Configuration ---
ROS2_PACKAGE_NAME = 'rotatum_arm_comm'
ROS2_EXECUTABLE_NAME = 'rotatum_arm_comm'
ROS2_BRINGUP_PACKAGE_NAME = 'rotatum_arm_bringup'
ROS2_BRINGUP_LAUNCH_FILE = 'bringup.launch.py'
RECORDINGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'teach_recordings')

# Paths to ROS environments to ensure 'ros2' sees overlay packages
ROS_DISTRO_SETUP = '/opt/ros/humble/setup.bash'
ROS_WS_SETUP = '/home/suhas/ros_ws/install/setup.bash'

def _bash_ros_command(cmd: str) -> list[str]:
    """Wrap a command to source ROS envs in a clean shell before execution.
    Uses a mostly-clean environment to avoid Conda/user PYTHONPATH conflicts with rclpy.
    """
    setup_chain = (
        f"unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV; "
        f"source {ROS_DISTRO_SETUP} && source {ROS_WS_SETUP}"
    )
    # Use env -i to start from a near-empty environment, preserving HOME and a safe PATH
    return ['env', '-i', f'HOME={os.environ.get("HOME", "/home/suhas")}', 'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/bin', 'bash', '-lc', f"{setup_chain} && {cmd}"]

def _ensure_controllers_running() -> None:
    """Check controller states and spawn only missing/inactive ones to avoid duplicates."""
    try:
        list_cmd = _bash_ros_command("ros2 control list_controllers")
        result = subprocess.run(list_cmd, capture_output=True, text=True, timeout=10)
        output = result.stdout
        jsb_active = False
        jtc_active = False
        for line in output.splitlines():
            if 'joint_state_broadcaster' in line and 'active' in line:
                jsb_active = True
            if 'joint_trajectory_position_controller' in line and 'active' in line:
                jtc_active = True
        if not jsb_active:
            subprocess.Popen(_bash_ros_command("ros2 run controller_manager spawner joint_state_broadcaster -c /controller_manager"),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1, universal_newlines=True)
            time.sleep(1.0)
        if not jtc_active:
            subprocess.Popen(_bash_ros_command("ros2 run controller_manager spawner joint_trajectory_position_controller -c /controller_manager"),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1, universal_newlines=True)
    except Exception:
        return

# --- IMPORTANT: USER CONFIGURATION ---
# The path to your 'send_joint_command.py' script is critical for playback and manual joint control.
# This path has been updated based on the information you provided.
SEND_JOINT_COMMAND_SCRIPT_PATH = '/home/suhas/ros_ws/src/rotatum_arm_jt_nano-ros2_humble/rotatum_arm_bringup/scripts/send_joint_command.py'

# Ensure the recordings directory exists
if not os.path.exists(RECORDINGS_DIR):
    os.makedirs(RECORDINGS_DIR)

# Helper function to stream output from a subprocess in a separate thread
def _stream_process_output(process_name, process):
    """Reads stdout and stderr of a subprocess line by line."""
    print(f"\n--- Streaming Output for {process_name} (PID: {process.pid}) Start ---")
    for line in iter(process.stdout.readline, ''):
        print(f"[{process_name} STDOUT]: {line.strip()}")
    for line in iter(process.stderr.readline, ''):
        print(f"[{process_name} STDERR]: {line.strip()}")
    process.stdout.close()
    process.stderr.close()
    process.wait()
    print(f"--- {process_name} (PID: {process.pid}) Exited with code: {process.returncode} ---")
    if process_name in active_ros_processes:
        del active_ros_processes[process_name]

# --- API Endpoints ---

@app.route('/api/ports', methods=['GET'])
def list_serial_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    print(f"Detected serial ports: {', '.join(ports)}")
    return jsonify(ports)

@app.route('/api/run_ros_node', methods=['POST'])
def run_ros_node():
    data = request.get_json()
    selected_port = data.get('port')
    process_id = "rotatum_arm_comm_node"

    if not selected_port:
        return jsonify({'success': False, 'error': 'No port selected.'}), 400

    if process_id in active_ros_processes and active_ros_processes[process_id].poll() is None:
        return jsonify({'success': False, 'error': f"Node '{process_id}' is already running."}), 409

    ros_cmd = f"ros2 run {ROS2_PACKAGE_NAME} {ROS2_EXECUTABLE_NAME}"
    command = _bash_ros_command(ros_cmd)
    print(f"Attempting to run command: {ros_cmd}")

    try:
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            bufsize=1, universal_newlines=True
        )
        active_ros_processes[process_id] = process
        threading.Thread(target=_stream_process_output, args=(process_id, process)).start()
        time.sleep(2) # Give node time to initialize

        if process.poll() is None:
            return jsonify({'success': True, 'output': "ROS 2 node launched successfully."}), 200
        else:
            error_message = f"Command failed with exit code {process.returncode}."
            return jsonify({'success': False, 'error': error_message}), 500
    except Exception as e:
        return jsonify({'success': False, 'error': f'An unexpected error occurred: {str(e)}'}), 500

@app.route('/api/bringup_robot', methods=['POST'])
def bringup_robot():
    process_id = "rotatum_arm_bringup_launch"
    if process_id in active_ros_processes and active_ros_processes[process_id].poll() is None:
        return jsonify({'success': False, 'error': f"Launch file '{process_id}' is already running."}), 409

    ros_launch = f"ros2 launch {ROS2_BRINGUP_PACKAGE_NAME} {ROS2_BRINGUP_LAUNCH_FILE} use_rviz:=false"
    launch_command = _bash_ros_command(ros_launch)
    print(f"Attempting to run launch command: {ros_launch}")
    try:
        process = subprocess.Popen(
            launch_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            bufsize=1, universal_newlines=True
        )
        active_ros_processes[process_id] = process
        threading.Thread(target=_stream_process_output, args=(process_id, process)).start()
        time.sleep(3) # Give launch file time to start up controllers

        if process.poll() is None:
            _ensure_controllers_running()
            return jsonify({'success': True, 'output': "ROS 2 bringup launch started successfully."}), 200
        else:
            return jsonify({'success': False, 'error': f"Launch command failed with exit code {process.returncode}."}), 500
    except Exception as e:
        return jsonify({'success': False, 'error': f'An unexpected error occurred: {str(e)}'}), 500

@app.route('/api/set_joint_positions', methods=['POST'])
def set_joint_positions():
    data = request.get_json()
    joint_values = [
        data.get('j1'), data.get('j2'), data.get('j3'),
        data.get('j4'), data.get('j5'), data.get('j6')
    ]
    time_from_start_sec = data.get('time_from_start')

    if any(v is None for v in joint_values) or time_from_start_sec is None:
        return jsonify({'success': False, 'error': 'Invalid or missing joint/time values.'}), 400

    joint_values_str = [str(v) for v in joint_values]
    time_from_start_str = str(time_from_start_sec)
    
    if not os.path.exists(SEND_JOINT_COMMAND_SCRIPT_PATH):
        error_msg = f"Script not found at {SEND_JOINT_COMMAND_SCRIPT_PATH}. Please check the path in backend_server.py"
        print(f"ERROR: {error_msg}")
        return jsonify({'success': False, 'error': error_msg}), 500

    py_cmd = f"/usr/bin/python3 {SEND_JOINT_COMMAND_SCRIPT_PATH} {' '.join(joint_values_str + [time_from_start_str])}"
    command = _bash_ros_command(py_cmd)
    print(f"Running joint command: {py_cmd}")
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=40
        )
        if result.returncode == 0:
            return jsonify({'success': True, 'message': 'Joint command sent.', 'output': result.stdout}), 200
        else:
            error_msg = f"Script failed: {result.stderr or result.stdout}"
            return jsonify({'success': False, 'error': error_msg}), 500
    except Exception as e:
        return jsonify({'success': False, 'error': f'An unexpected error occurred: {str(e)}'}), 500

@app.route('/api/disconnect_and_shutdown', methods=['POST'])
def disconnect_and_shutdown():
    print("Shutdown request received. Terminating ROS processes.")
    for proc_id, proc in list(active_ros_processes.items()):
        print(f"Terminating {proc_id} (PID: {proc.pid})")
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            print(f"Process {proc_id} did not terminate, killing.")
            proc.kill()
    active_ros_processes.clear()
    # A more aggressive cleanup
    subprocess.call("pkill -f ros2", shell=True)
    subprocess.call("pkill -f rviz2", shell=True)
    return jsonify({'success': True, 'message': 'Shutdown complete'}), 200

# --- NEW Teaching Functionality Endpoints ---

@app.route('/api/set_torque', methods=['POST'])
def set_torque():
    data = request.get_json()
    port_name = data.get('port')
    enable = data.get('enable')
    servo_ids = [1, 2, 3, 4, 5, 6] # Assuming 6 servos

    if port_name is None or enable is None:
        return jsonify({'success': False, 'error': 'Missing port or enable state.'}), 400

    feetech_comm = FeetechComm(port_name)
    success, message = feetech_comm.set_torque_for_all(enable, servo_ids)

    if success:
        return jsonify({'success': True, 'message': f'Torque for all servos set to {enable}.'})
    else:
        return jsonify({'success': False, 'error': message}), 500

@app.route('/api/get_current_joints', methods=['POST'])
def get_current_joints():
    data = request.get_json()
    port_name = data.get('port')
    if not port_name:
        return jsonify({'success': False, 'error': 'Missing port name.'}), 400

    servo_ids = [1, 2, 3, 4, 5, 6]
    joint_offsets = [2048, 2048, 2048, 2048, 2048, 2048]
    
    feetech_comm = FeetechComm(port_name)
    joint_positions_rad, message = feetech_comm.read_all_positions(servo_ids, joint_offsets)
    
    if joint_positions_rad is not None:
        return jsonify({'success': True, 'joints': joint_positions_rad})
    else:
        return jsonify({'success': False, 'error': message}), 500

@app.route('/api/get_recordings', methods=['POST'])
def get_recordings():
    try:
        files = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith('.json')]
        return jsonify({'success': True, 'recordings': sorted(files)})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/save_recording', methods=['POST'])
def save_recording():
    data = request.get_json()
    name = data.get('name')
    recording_data = data.get('recording_data')

    if not name or not recording_data:
        return jsonify({'success': False, 'error': 'Missing name or recording data.'}), 400

    filename = f"{name}.json"
    filepath = os.path.join(RECORDINGS_DIR, filename)

    try:
        with open(filepath, 'w') as f:
            json.dump(recording_data, f, indent=4)
        return jsonify({'success': True, 'message': f'Recording saved as {filename}.'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/load_recording', methods=['POST'])
def load_recording():
    data = request.get_json()
    name = data.get('name')
    if not name:
        return jsonify({'success': False, 'error': 'Missing recording name.'}), 400

    filepath = os.path.join(RECORDINGS_DIR, name)
    if not os.path.exists(filepath):
        return jsonify({'success': False, 'error': 'Recording not found.'}), 404

    try:
        with open(filepath, 'r') as f:
            recording_data = json.load(f)
        return jsonify({'success': True, 'recording_data': recording_data})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/delete_recording', methods=['POST'])
def delete_recording():
    data = request.get_json()
    name = data.get('name')
    if not name:
        return jsonify({'success': False, 'error': 'Missing recording name.'}), 400

    filepath = os.path.join(RECORDINGS_DIR, name)
    if not os.path.exists(filepath):
        return jsonify({'success': False, 'error': 'Recording not found.'}), 404

    try:
        os.remove(filepath)
        return jsonify({'success': True, 'message': f'Deleted {name}.'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/run_script', methods=['POST'])
def run_script():
    # This is a placeholder for running a full Python script.
    # For security reasons, executing arbitrary code from a web UI is dangerous.
    # This should be implemented with extreme care, e.g., by providing a sandboxed
    # environment or a limited API within the script.
    data = request.get_json()
    script_code = data.get('script_code')
    # In a real application, you would not use exec() directly.
    # You would parse the script and use your robot's API.
    output = "Script execution is a demo feature and is not fully implemented for security reasons."
    return jsonify({'success': True, 'output': output})


if __name__ == '__main__':
    # --- Startup Validation ---
    # Check if the user has updated the placeholder path for the command script.
    if 'path/to/your/script' in SEND_JOINT_COMMAND_SCRIPT_PATH:
        print("="*80)
        print("!!! CONFIGURATION REQUIRED !!!")
        print("The path to 'send_joint_command.py' has not been configured yet.")
        print("Please edit the 'SEND_JOINT_COMMAND_SCRIPT_PATH' variable in backend_server.py.")
        print("Manual joint control and playback will not work until this is fixed.")
        print("="*80)
    # Check if the configured path actually exists.
    elif not os.path.exists(SEND_JOINT_COMMAND_SCRIPT_PATH):
        print("="*80)
        print("!!! PATH CONFIGURATION ERROR !!!")
        print(f"The script 'send_joint_command.py' was NOT found at the configured path:")
        print(f"  -> {SEND_JOINT_COMMAND_SCRIPT_PATH}")
        print("Please verify the path in the 'SEND_JOINT_COMMAND_SCRIPT_PATH' variable is correct.")
        print("Manual joint control and playback will not work until this is fixed.")
        print("="*80)
    else:
        # If the path is valid, make the script executable
        print(f"Found command script at: {SEND_JOINT_COMMAND_SCRIPT_PATH}")
        print("Setting script as executable...")
        os.chmod(SEND_JOINT_COMMAND_SCRIPT_PATH, 0o755)
        print("Configuration check passed.")
        
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)