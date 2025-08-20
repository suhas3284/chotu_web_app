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
# This logic is adapted from the working set_torque.py script.

# Control table address
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_STS_PRESENT_POSITION = 56

# Default setting
BAUDRATE = 1000000  # SCServo default baudrate
SCS_LOBYTE = lambda w: w & 0xFF
SCS_HIBYTE = lambda w: (w >> 8) & 0xFF
SCS_MAKELONG = lambda a, b: (a << 8) | b

# Communication Results
COMM_SUCCESS = 0
COMM_TXFAIL = -1001
COMM_RXFAIL = -1002
COMM_TXERROR = -2001
COMM_RXWAITING = -2002
COMM_RXTIMEOUT = -2003
COMM_RXCORRUPT = -2004
COMM_PORTNOTOPEN = -2005

class FeetechComm:
    """
    A helper class to encapsulate direct, short-lived communication with Feetech servos.
    Each method opens the port, performs an action, and closes it, to avoid
    conflicts with the main ROS hardware interface node.
    Based on the working set_torque.py implementation.
    """
    def __init__(self, port, baudrate=BAUDRATE):
        self.port_name = port
        self.baudrate = baudrate
        self.ser = None

    def _open_port(self):
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=0.5,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            return True
        except serial.SerialException as e:
            print(f"Error opening port {self.port_name}: {e}")
            self.ser = None
            return False

    def _close_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _calculate_checksum(self, data_bytes):
        """Calculates the checksum for a Feetech servo packet."""
        checksum_sum = sum(data_bytes)
        checksum = (~checksum_sum) & 0xFF
        return checksum

    def _write1ByteTxRx(self, servo_id, address, value):
        """
        Writes 1 byte of data to a servo register (based on working set_torque.py).
        """
        if not self.ser or not self.ser.is_open:
            return COMM_PORTNOTOPEN, 0

        # WRITE_DATA = 0x03
        parameters = [address, value]
        
        # Calculate length field for packet: 1 (instruction) + len(parameters) + 1 (checksum)
        packet_data_for_len_cs = [0x03] + parameters  # 0x03 = WRITE_DATA
        packet_length_field = len(packet_data_for_len_cs) + 1 

        # Calculate checksum: ID + Length + Instruction + Parameters
        checksum_input_bytes = [servo_id, packet_length_field] + packet_data_for_len_cs
        checksum = self._calculate_checksum(checksum_input_bytes)

        # Build the packet
        packet = bytearray([
            0xFF,  # START_BYTE
            0xFF,  # START_BYTE
            servo_id,
            packet_length_field,
            0x03,  # WRITE_DATA
        ])
        packet.extend(parameters)
        packet.append(checksum)

        # Send the packet
        try:
            written_bytes = self.ser.write(packet)
            self.ser.flush()
            if written_bytes != len(packet):
                return COMM_TXFAIL, 0
            return COMM_SUCCESS, 0
        except Exception as e:
            print(f"Write error: {e}")
            return COMM_TXFAIL, 0

    def _read2ByteTxRx(self, servo_id, address):
        """
        Reads 2 bytes from a servo register (for position reading).
        More robust implementation that handles various error conditions.
        """
        if not self.ser or not self.ser.is_open:
            return COMM_PORTNOTOPEN, 0, 0

        # READ_DATA = 0x02
        parameters = [address, 2]  # Read 2 bytes
        
        # Calculate length field for packet: 1 (instruction) + len(parameters) + 1 (checksum)
        packet_data_for_len_cs = [0x02] + parameters  # 0x02 = READ_DATA
        packet_length_field = len(packet_data_for_len_cs) + 1 

        # Calculate checksum: ID + Length + Instruction + Parameters
        checksum_input_bytes = [servo_id, packet_length_field] + packet_data_for_len_cs
        checksum = self._calculate_checksum(checksum_input_bytes)

        # Build the packet
        packet = bytearray([
            0xFF,  # START_BYTE
            0xFF,  # START_BYTE
            servo_id,
            packet_length_field,
            0x02,  # READ_DATA
        ])
        packet.extend(parameters)
        packet.append(checksum)

        # Send the packet
        try:
            self.ser.flushInput()
            written_bytes = self.ser.write(packet)
            self.ser.flush()
            if written_bytes != len(packet):
                return COMM_TXFAIL, 0, 0

            # Read response: FF FF ID Len Err DataL DataH CS
            # Wait a bit longer for response
            time.sleep(0.01)
            response = self.ser.read(8)
            
            if len(response) < 8:
                # Try reading again with a longer timeout
                time.sleep(0.05)
                response = self.ser.read(8)
                if len(response) < 8:
                    return COMM_RXFAIL, 0, 0

            if response[0] != 0xFF or response[1] != 0xFF or response[2] != servo_id:
                return COMM_RXCORRUPT, 0, 0

            error_code = response[4]
            
            # For position reading, we can ignore voltage errors (error_code=1) as they don't prevent reading
            # Only fail on critical errors
            if error_code > 1:  # Allow voltage error (1) but fail on others
                return COMM_SUCCESS, error_code, 0

            # Extract 2-byte data (little endian)
            data = (response[6] << 8) | response[5]
            return COMM_SUCCESS, 0, data

        except Exception as e:
            print(f"Read error: {e}")
            return COMM_RXFAIL, 0, 0

    def set_torque_for_all(self, enable, servo_ids):
        """Enable or disable torque for a list of servos."""
        if not self._open_port():
            return False, f"Port {self.port_name} could not be opened. Is it in use by another process (like the main ROS driver)?"

        all_success = True
        errors = []
        try:
            for servo_id in servo_ids:
                comm_result, packet_error = self._write1ByteTxRx(servo_id, ADDR_SCS_TORQUE_ENABLE, 1 if enable else 0)
                if comm_result != COMM_SUCCESS:
                    all_success = False
                    errors.append(f"Servo {servo_id}: Communication failed")
                time.sleep(0.02)  # Small delay between commands
                    
        except Exception as e:
            all_success = False
            errors.append(str(e))
        finally:
            self._close_port()

        if all_success:
            return True, "Commands sent successfully"
        else:
            return False, f"Some commands failed: {'; '.join(errors)}"

    def ping_servo(self, servo_id):
        """
        Simple ping test to verify basic servo connectivity.
        Returns (success, message)
        """
        if not self._open_port():
            return False, f"Could not open port {self.port_name}"
        
        try:
            # Try to read servo ID (register 5) - this is a simple read that should work
            comm_result, packet_error, data = self._read2ByteTxRx(servo_id, 5)  # ADDR_STS_ID = 5
            
            if comm_result == COMM_SUCCESS:
                if packet_error <= 1:  # Allow voltage error
                    return True, f"Servo {servo_id} responded (ID: {data})"
                else:
                    return False, f"Servo {servo_id} error: {packet_error}"
            else:
                return False, f"Communication failed: {comm_result}"
                
        except Exception as e:
            return False, f"Exception: {str(e)}"
        finally:
            self._close_port()

    def read_all_positions(self, servo_ids, joint_offsets):
        """Read the position of all specified servos with improved error handling."""
        if not self._open_port():
            return None, f"Port {self.port_name} could not be opened. Is it in use by another process?"

        positions = {}
        try:
            for i, servo_id in enumerate(servo_ids):
                try:
                    comm_result, packet_error, data = self._read2ByteTxRx(servo_id, ADDR_STS_PRESENT_POSITION)
                    
                    if comm_result == COMM_SUCCESS and packet_error <= 1:  # Allow voltage error (1)
                        # Convert from Feetech counts to radians
                        rad = (2 * math.pi * (data - joint_offsets[i])) / 4096.0
                        positions[f'j{i+1}'] = rad
                    else:
                        print(f"Warning: Failed to read servo {servo_id}: comm_result={comm_result}, packet_error={packet_error}")
                        # Set a default position to avoid breaking the recording
                        positions[f'j{i+1}'] = 0.0
                    
                    time.sleep(0.02)  # Small delay between reads
                    
                except Exception as e:
                    print(f"Warning: Failed to read servo {servo_id}: {e}")
                    # Set a default position to avoid breaking the recording
                    positions[f'j{i+1}'] = 0.0
                    
        except Exception as e:
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

    print(f"Setting torque to {enable} for servos {servo_ids} on port {port_name}")
    
    feetech_comm = FeetechComm(port_name)
    success, message = feetech_comm.set_torque_for_all(enable, servo_ids)

    if success:
        print(f"Torque set successfully: {message}")
        return jsonify({'success': True, 'message': f'Torque for all servos set to {enable}.'})
    else:
        print(f"Torque setting failed: {message}")
        return jsonify({'success': False, 'error': message}), 500

@app.route('/api/get_current_joints', methods=['POST'])
def get_current_joints():
    data = request.get_json()
    port_name = data.get('port')
    if not port_name:
        return jsonify({'success': False, 'error': 'Missing port name.'}), 400

    servo_ids = [1, 2, 3, 4, 5, 6]
    joint_offsets = [2048, 2048, 2048, 2048, 2048, 2048]
    
    print(f"Reading joint positions from servos {servo_ids} on port {port_name}")
    
    feetech_comm = FeetechComm(port_name)
    joint_positions_rad, message = feetech_comm.read_all_positions(servo_ids, joint_offsets)
    
    if joint_positions_rad is not None:
        print(f"Successfully read joint positions: {joint_positions_rad}")
        return jsonify({'success': True, 'joints': joint_positions_rad})
    else:
        print(f"Failed to read joint positions: {message}")
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

@app.route('/api/ping_servo', methods=['POST'])
def ping_servo():
    """Simple ping test to verify basic servo connectivity."""
    data = request.get_json()
    port_name = data.get('port')
    servo_id = data.get('servo_id', 1)  # Default to servo 1
    
    if not port_name:
        return jsonify({'success': False, 'error': 'Missing port name.'}), 400

    print(f"Pinging servo {servo_id} on port {port_name}")
    
    feetech_comm = FeetechComm(port_name)
    success, message = feetech_comm.ping_servo(servo_id)
    
    if success:
        print(f"Ping successful: {message}")
        return jsonify({'success': True, 'message': message})
    else:
        print(f"Ping failed: {message}")
        return jsonify({'success': False, 'error': message}), 500

@app.route('/api/test_servo_communication', methods=['POST'])
def test_servo_communication():
    """Test endpoint to verify servo communication is working."""
    data = request.get_json()
    port_name = data.get('port')
    if not port_name:
        return jsonify({'success': False, 'error': 'Missing port name.'}), 400

    print(f"Testing servo communication on port {port_name}")
    
    try:
        # Test 1: Try to open the port
        feetech_comm = FeetechComm(port_name)
        if not feetech_comm._open_port():
            return jsonify({'success': False, 'error': f'Could not open port {port_name}'}), 500
        
        feetech_comm._close_port()
        
        # Test 2: Try to read servo IDs (ping test)
        feetech_comm = FeetechComm(port_name)
        if not feetech_comm._open_port():
            return jsonify({'success': False, 'error': f'Could not open port {port_name} for ping test'}), 500
        
        # Try to read from servo 1 to see if it responds
        try:
            comm_result, packet_error, data = feetech_comm._read2ByteTxRx(1, ADDR_STS_PRESENT_POSITION)
            feetech_comm._close_port()
            
            if comm_result == COMM_SUCCESS:
                if packet_error == 0:
                    return jsonify({
                        'success': True, 
                        'message': f'Port {port_name} is accessible and servo 1 responded with position {data}',
                        'response_length': 8
                    })
                elif packet_error == 1:
                    # Voltage error is common and doesn't prevent communication
                    return jsonify({
                        'success': True, 
                        'message': f'Port {port_name} is accessible and servo 1 responded (voltage warning, position {data})',
                        'response_length': 8,
                        'warning': 'Input voltage error detected but communication successful'
                    })
                else:
                    return jsonify({
                        'success': False, 
                        'error': f'Port accessible but servo error: {packet_error}'
                    }), 500
            else:
                return jsonify({
                    'success': False, 
                    'error': f'Port accessible but servo communication failed: comm_result={comm_result}, packet_error={packet_error}'
                }), 500
        except Exception as e:
            feetech_comm._close_port()
            return jsonify({
                'success': False, 
                'error': f'Port accessible but servo communication failed: {str(e)}'
            }), 500
            
    except Exception as e:
        return jsonify({'success': False, 'error': f'Test failed: {str(e)}'}), 500


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