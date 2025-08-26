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
import signal
import psutil
import logging

# --- Logging Configuration for Debugging ---
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

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

# --- ROS2 Launch Integration Configuration ---
logger.info("Initializing ROS2 Launch Integration Configuration")
ROS2_LAUNCH_TIMEOUT = 30  # seconds to wait for launch completion
ROS2_LAUNCH_CHECK_INTERVAL = 2  # seconds between launch status checks
RVIZ_WEB_PORT = 8081  # Port for RViz web interface
RVIZ_WEB_TIMEOUT = 10  # seconds to wait for RViz web to be ready

# --- WebSocket Bridge Configuration ---
logger.info("Initializing WebSocket Bridge Configuration")
WEBSOCKET_BRIDGE_PORT = 8765  # Port for WebSocket bridge
WEBSOCKET_BRIDGE_TIMEOUT = 10  # seconds to wait for bridge to be ready
WEBSOCKET_BRIDGE_PACKAGE = 'foxglove_bridge'
WEBSOCKET_BRIDGE_EXECUTABLE = 'foxglove_bridge'

# Launch file paths and configurations
LAUNCH_CONFIGS = {
    'bringup_with_rviz': {
        'package': ROS2_BRINGUP_PACKAGE_NAME,
        'launch_file': ROS2_BRINGUP_LAUNCH_FILE,
        'arguments': ['use_rviz:=true'],
        'description': 'Robot bringup with RViz visualization'
    },
    'bringup_without_rviz': {
        'package': ROS2_BRINGUP_PACKAGE_NAME,
        'launch_file': ROS2_BRINGUP_LAUNCH_FILE,
        'arguments': ['use_rviz:=false'],
        'description': 'Robot bringup without RViz'
    }
}

logger.info(f"Launch configurations loaded: {list(LAUNCH_CONFIGS.keys())}")
logger.info(f"RViz web port configured: {RVIZ_WEB_PORT}")

def _bash_ros_command(cmd: str) -> list[str]:
    """Wrap a command to source ROS envs in a clean shell before execution.
    Uses a mostly-clean environment to avoid Conda/user PYTHONPATH conflicts with rclpy.
    """
    setup_chain = (
        f"unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV; "
        f"source {ROS_DISTRO_SETUP} && source {ROS_WS_SETUP} && "
        f"export ROS_DISTRO=humble && export ROS_VERSION=2 && export ROS_DOMAIN_ID=0"
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

# --- ROS2 Launch Management Functions ---
logger.info("Adding ROS2 Launch Management Functions")

def _launch_ros2_launch_file(launch_config_name: str, timeout: int = None) -> dict:
    """
    Launch a ROS2 launch file with the specified configuration.
    
    Args:
        launch_config_name: Name of the launch configuration to use
        timeout: Timeout in seconds for launch completion check
    
    Returns:
        dict: Result with success status, process info, and any errors
    """
    logger.info(f"Attempting to launch ROS2 launch file: {launch_config_name}")
    
    if launch_config_name not in LAUNCH_CONFIGS:
        error_msg = f"Unknown launch configuration: {launch_config_name}"
        logger.error(error_msg)
        return {'success': False, 'error': error_msg}
    
    config = LAUNCH_CONFIGS[launch_config_name]
    process_id = f"{launch_config_name}_launch"
    
    # Check if already running
    if process_id in active_ros_processes and active_ros_processes[process_id].poll() is None:
        logger.warning(f"Launch process {process_id} is already running")
        return {'success': False, 'error': f"Launch process '{process_id}' is already running."}
    
    try:
        # Build launch command
        launch_cmd = f"ros2 launch {config['package']} {config['launch_file']}"
        if config['arguments']:
            launch_cmd += " " + " ".join(config['arguments'])
        
        logger.info(f"Launch command: {launch_cmd}")
        logger.info(f"Launch description: {config['description']}")
        
        # Execute launch command
        command = _bash_ros_command(launch_cmd)
        logger.info(f"Bash command array: {command}")
        
        # Test if ROS2 is available first
        logger.info("Testing ROS2 availability...")
        try:
            test_cmd = _bash_ros_command("ros2 --help")
            test_result = subprocess.run(test_cmd, capture_output=True, text=True, timeout=10)
            if test_result.returncode != 0:
                error_msg = f"ROS2 not available: {test_result.stderr}"
                logger.error(error_msg)
                return {'success': False, 'error': error_msg}
            logger.info("ROS2 is available and working")
        except Exception as e:
            error_msg = f"Failed to test ROS2 availability: {str(e)}"
            logger.error(error_msg)
            return {'success': False, 'error': error_msg}
        
        # Start launch process
        logger.info("Starting ROS2 launch process...")
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        logger.info(f"Launch process started with PID: {process.pid}")
        
        # Store process reference
        active_ros_processes[process_id] = process
        
        # Start output streaming in background thread
        threading.Thread(
            target=_stream_process_output,
            args=(process_id, process),
            daemon=True
        ).start()
        
        # Wait for launch to complete initialization
        logger.info("Waiting for launch initialization...")
        time.sleep(5)  # Give launch file time to start
        
        # Check if process is still running
        if process.poll() is not None:
            error_msg = f"Launch process failed to start, exit code: {process.returncode}"
            logger.error(error_msg)
            # Get stderr output for debugging
            stderr_output = process.stderr.read() if process.stderr else "No stderr available"
            logger.error(f"Launch stderr output: {stderr_output}")
            return {'success': False, 'error': error_msg}
        
        logger.info(f"Launch process {process_id} started successfully")
        
        # Verify the process is actually running
        logger.info("Verifying process is running...")
        try:
            # Check if the process is still alive
            if not psutil.pid_exists(process.pid):
                error_msg = f"Process {process.pid} is not running"
                logger.error(error_msg)
                return {'success': False, 'error': error_msg}
            
            # Check if it's our process
            proc = psutil.Process(process.pid)
            if proc.name() not in ['bash', 'ros2', 'python3']:
                error_msg = f"Process {process.pid} is not a ROS2 process: {proc.name()}"
                logger.error(error_msg)
                return {'success': False, 'error': error_msg}
                
            logger.info(f"Process verification successful: {proc.name()} (PID: {process.pid})")
            
        except Exception as e:
            error_msg = f"Failed to verify process: {str(e)}"
            logger.error(error_msg)
            return {'success': False, 'error': error_msg}
        
        # Return success with process info
        return {
            'success': True,
            'process_id': process_id,
            'pid': process.pid,
            'description': config['description'],
            'message': f"ROS2 launch '{launch_config_name}' started successfully"
        }
        
    except Exception as e:
        error_msg = f"Failed to launch ROS2 launch file: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return {'success': False, 'error': error_msg}

def _check_launch_status(process_id: str) -> dict:
    """
    Check the status of a running launch process.
    
    Args:
        process_id: ID of the launch process to check
    
    Returns:
        dict: Status information about the process
    """
    logger.debug(f"Checking status of launch process: {process_id}")
    
    if process_id not in active_ros_processes:
        return {'running': False, 'error': 'Process not found'}
    
    process = active_ros_processes[process_id]
    
    if process.poll() is None:
        # Process is still running
        return {
            'running': True,
            'pid': process.pid,
            'status': 'active'
        }
    else:
        # Process has finished
        return {
            'running': False,
            'pid': process.pid,
            'exit_code': process.returncode,
            'status': 'finished'
        }

def _stop_launch_process(process_id: str) -> dict:
    """
    Stop a running launch process.
    
    Args:
        process_id: ID of the launch process to stop
    
    Returns:
        dict: Result of the stop operation
    """
    logger.info(f"Attempting to stop launch process: {process_id}")
    
    if process_id not in active_ros_processes:
        return {'success': False, 'error': 'Process not found'}
    
    process = active_ros_processes[process_id]
    
    try:
        # Try graceful termination first
        logger.info(f"Sending SIGTERM to process {process_id} (PID: {process.pid})")
        process.terminate()
        
        # Wait for graceful shutdown
        try:
            process.wait(timeout=10)
            logger.info(f"Process {process_id} terminated gracefully")
        except subprocess.TimeoutExpired:
            # Force kill if graceful shutdown fails
            logger.warning(f"Process {process_id} did not terminate gracefully, forcing kill")
            process.kill()
            process.wait()
            logger.info(f"Process {process_id} force killed")
        
        # Remove from active processes
        del active_ros_processes[process_id]
        
        return {'success': True, 'message': f"Launch process {process_id} stopped successfully"}
        
    except Exception as e:
        error_msg = f"Failed to stop launch process {process_id}: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return {'success': False, 'error': error_msg}

def _get_all_launch_status() -> dict:
    """
    Get status of all running launch processes.
    
    Returns:
        dict: Status of all launch processes
    """
    logger.debug("Getting status of all launch processes")
    
    status = {}
    for process_id, process in active_ros_processes.items():
        if 'launch' in process_id:  # Only show launch processes
            status[process_id] = _check_launch_status(process_id)
    
    logger.debug(f"Launch processes status: {status}")
    return status

logger.info("ROS2 Launch Management Functions added successfully")

# --- ROS2 Context Validation Functions ---
logger.info("Adding ROS2 Context Validation Functions")

def _validate_ros2_context() -> dict:
    """
    Validate that ROS2 context is properly initialized and accessible.
    
    Returns:
        dict: Validation result with success status and details
    """
    try:
        logger.info("Validating ROS2 context...")
        
        # Test basic ROS2 functionality
        test_commands = [
            ("ros2 node list", "Node listing"),
            ("ros2 topic list", "Topic listing"),
            ("ros2 service list", "Service listing")
        ]
        
        results = {}
        for cmd, description in test_commands:
            try:
                init_cmd = _bash_ros_command(cmd)
                init_process = subprocess.run(init_cmd, capture_output=True, text=True, timeout=10)
                
                if init_process.returncode == 0:
                    results[description] = "SUCCESS"
                    logger.debug(f"{description}: SUCCESS")
                else:
                    results[description] = f"FAILED: {init_process.stderr.strip()}"
                    logger.warning(f"{description}: FAILED - {init_process.stderr.strip()}")
                    
            except subprocess.TimeoutExpired:
                results[description] = "TIMEOUT"
                logger.warning(f"{description}: TIMEOUT")
            except Exception as e:
                results[description] = f"ERROR: {str(e)}"
                logger.error(f"{description}: ERROR - {str(e)}")
        
        # Determine overall success
        success_count = sum(1 for result in results.values() if result == "SUCCESS")
        total_count = len(results)
        
        if success_count >= 2:  # At least 2 out of 3 commands should work
            logger.info(f"ROS2 context validation successful: {success_count}/{total_count} tests passed")
            return {
                'success': True,
                'message': f"ROS2 context validated: {success_count}/{total_count} tests passed",
                'details': results
            }
        else:
            logger.warning(f"ROS2 context validation failed: {success_count}/{total_count} tests passed")
            return {
                'success': False,
                'message': f"ROS2 context validation failed: {success_count}/{total_count} tests passed",
                'details': results
            }
            
    except Exception as e:
        error_msg = f"ROS2 context validation error: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return {
            'success': False,
            'message': error_msg,
            'details': {}
        }

logger.info("ROS2 Context Validation Functions added successfully")

# --- WebSocket Bridge Management Functions ---
logger.info("Adding WebSocket Bridge Management Functions")

def _start_websocket_bridge(port: int = None) -> dict:
    """
    Start the WebSocket bridge for real-time communication.
    
    Args:
        port: Port to use for WebSocket bridge (default: WEBSOCKET_BRIDGE_PORT)
    
    Returns:
        dict: Result with success status and process info
    """
    if port is None:
        port = WEBSOCKET_BRIDGE_PORT
    
    process_id = f"websocket_bridge_{port}"
    
    # Check if already running
    if process_id in active_ros_processes and active_ros_processes[process_id].poll() is None:
        logger.warning(f"WebSocket bridge on port {port} is already running")
        return {'success': False, 'error': f"WebSocket bridge on port {port} is already running"}
    
    try:
        # First, validate ROS2 context is properly initialized
        logger.info("Validating ROS2 context before starting bridge...")
        context_validation = _validate_ros2_context()
        
        if not context_validation['success']:
            logger.warning(f"ROS2 context validation failed: {context_validation['message']}")
            logger.warning("Proceeding anyway, but bridge may fail...")
        else:
            logger.info(f"ROS2 context validation successful: {context_validation['message']}")
        
        # Build bridge command
        bridge_cmd = f"ros2 run {WEBSOCKET_BRIDGE_PACKAGE} {WEBSOCKET_BRIDGE_EXECUTABLE}"
        bridge_cmd += f" --ros-args -p port:={port}"
        
        logger.info(f"Starting WebSocket bridge: {bridge_cmd}")
        
        # Execute bridge command
        command = _bash_ros_command(bridge_cmd)
        logger.info(f"Bash command array: {command}")
        
        # Start bridge process
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        logger.info(f"WebSocket bridge process started with PID: {process.pid}")
        
        # Store process reference
        active_ros_processes[process_id] = process
        
        # Start output streaming in background thread
        threading.Thread(
            target=_stream_process_output,
            args=(process_id, process),
            daemon=True
        ).start()
        
        # Wait for bridge to start with progressive delay and health checks
        logger.info("Waiting for WebSocket bridge to initialize...")
        
        # Progressive initialization with health checks
        max_wait_time = 15  # Maximum 15 seconds
        check_interval = 1   # Check every second
        total_wait_time = 0
        
        while total_wait_time < max_wait_time:
            time.sleep(check_interval)
            total_wait_time += check_interval
            
            # Check if process is still running
            if process.poll() is not None:
                error_msg = f"WebSocket bridge failed to start, exit code: {process.returncode}"
                logger.error(error_msg)
                return {'success': False, 'error': error_msg}
            
            # Try to connect to the bridge to verify it's working
            try:
                import socket
                test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                test_socket.settimeout(2)
                result = test_socket.connect_ex(('localhost', port))
                test_socket.close()
                
                if result == 0:
                    logger.info(f"WebSocket bridge {process_id} is accessible on port {port}")
                    break
                else:
                    logger.info(f"Waiting for bridge to become accessible... ({total_wait_time}s)")
                    
            except Exception as e:
                logger.debug(f"Socket test failed: {e}")
                continue
        
        if total_wait_time >= max_wait_time:
            logger.warning(f"WebSocket bridge took longer than expected to initialize ({total_wait_time}s)")
        
        # Final check if process is still running
        if process.poll() is not None:
            error_msg = f"WebSocket bridge failed to start, exit code: {process.returncode}"
            logger.error(error_msg)
            return {'success': False, 'error': error_msg}
        
        logger.info(f"WebSocket bridge {process_id} started successfully on port {port}")
        
        # Return success with process info
        return {
            'success': True,
            'process_id': process_id,
            'pid': process.pid,
            'port': port,
            'message': f"WebSocket bridge started successfully on port {port}"
        }
        
    except Exception as e:
        error_msg = f"Failed to start WebSocket bridge: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return {'success': False, 'error': error_msg}

def _stop_websocket_bridge(port: int = None) -> dict:
    """
    Stop the WebSocket bridge.
    
    Args:
        port: Port of the bridge to stop (default: WEBSOCKET_BRIDGE_PORT)
    
    Returns:
        dict: Result with success status
    """
    if port is None:
        port = WEBSOCKET_BRIDGE_PORT
    
    process_id = f"websocket_bridge_{port}"
    
    if process_id not in active_ros_processes:
        return {'success': False, 'error': f'WebSocket bridge on port {port} not found'}
    
    return _stop_launch_process(process_id)

def _get_websocket_bridge_status(port: int = None) -> dict:
    """
    Get status of WebSocket bridge.
    
    Args:
        port: Port of the bridge to check (default: WEBSOCKET_BRIDGE_PORT)
    
    Returns:
        dict: Status of the bridge
    """
    if port is None:
        port = WEBSOCKET_BRIDGE_PORT
    
    process_id = f"websocket_bridge_{port}"
    
    if process_id not in active_ros_processes:
        return {'running': False, 'error': 'Bridge not found'}
    
    return _check_launch_status(process_id)

logger.info("WebSocket Bridge Management Functions added successfully")

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

# --- ROS2 Launch Management API Endpoints ---
logger.info("Adding ROS2 Launch Management API Endpoints")

@app.route('/api/launch_robot_with_rviz', methods=['POST'])
def launch_robot_with_rviz():
    """
    Launch the robot with RViz visualization using the bringup launch file.
    This endpoint will:
    1. Launch the robot communication node
    2. Launch the bringup launch file with RViz enabled
    3. Return the status and RViz web interface information
    """
    logger.info("Received request to launch robot with RViz")
    
    try:
        # Step 1: Launch robot communication node (if not already running)
        comm_process_id = "rotatum_arm_comm_node"
        if comm_process_id not in active_ros_processes or active_ros_processes[comm_process_id].poll() is not None:
            logger.info("Launching robot communication node...")
            
            # Get port from request or use default
            data = request.get_json() or {}
            selected_port = data.get('port')
            
            if not selected_port:
                return jsonify({'success': False, 'error': 'Port is required for robot communication'}), 400
            
            # Launch communication node
            ros_cmd = f"ros2 run {ROS2_PACKAGE_NAME} {ROS2_EXECUTABLE_NAME}"
            command = _bash_ros_command(ros_cmd)
            logger.info(f"Launching communication node with command: {ros_cmd}")
            
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                bufsize=1, universal_newlines=True
            )
            active_ros_processes[comm_process_id] = process
            
            # Start output streaming
            threading.Thread(
                target=_stream_process_output, 
                args=(comm_process_id, process),
                daemon=True
            ).start()
            
            # Wait for node initialization
            logger.info("Waiting for communication node to initialize...")
            time.sleep(3)
            
            if process.poll() is not None:
                error_msg = f"Communication node failed to start, exit code: {process.returncode}"
                logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg}), 500
            
            logger.info("Communication node started successfully")
        else:
            logger.info("Communication node is already running")
        
        # Step 2: Launch bringup with RViz
        logger.info("Launching bringup launch file with RViz...")
        launch_result = _launch_ros2_launch_file('bringup_with_rviz')
        
        if not launch_result['success']:
            error_msg = f"Failed to launch bringup with RViz: {launch_result['error']}"
            logger.error(error_msg)
            return jsonify({'success': False, 'error': error_msg}), 500
        
        logger.info("Bringup with RViz launched successfully")
        
        # Step 3: Ensure controllers are running
        logger.info("Ensuring controllers are running...")
        _ensure_controllers_running()
        
        # Step 4: Return success response with RViz information
        response = {
            'success': True,
            'message': 'Robot launched successfully with RViz visualization',
            'rviz_web_port': RVIZ_WEB_PORT,
            'rviz_web_url': f'http://localhost:{RVIZ_WEB_PORT}/rviz',
            'processes': {
                'communication_node': {
                    'status': 'running',
                    'pid': active_ros_processes.get(comm_process_id, {}).pid if comm_process_id in active_ros_processes else None
                },
                'bringup_launch': launch_result
            }
        }
        
        logger.info(f"Robot launch completed successfully: {response}")
        return jsonify(response), 200
        
    except Exception as e:
        error_msg = f"Unexpected error during robot launch: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/launch_robot_without_rviz', methods=['POST'])
def launch_robot_without_rviz():
    """
    Launch the robot without RViz visualization using the bringup launch file.
    This endpoint will:
    1. Launch the robot communication node
    2. Launch the bringup launch file without RViz
    3. Return the status
    """
    logger.info("Received request to launch robot without RViz")
    
    try:
        # Step 1: Launch robot communication node (if not already running)
        comm_process_id = "rotatum_arm_comm_node"
        if comm_process_id not in active_ros_processes or active_ros_processes[comm_process_id].poll() is not None:
            logger.info("Launching robot communication node...")
            
            # Get port from request or use default
            data = request.get_json() or {}
            selected_port = data.get('port')
            
            if not selected_port:
                return jsonify({'success': False, 'error': 'Port is required for robot communication'}), 400
            
            # Launch communication node
            ros_cmd = f"ros2 run {ROS2_PACKAGE_NAME} {ROS2_EXECUTABLE_NAME}"
            command = _bash_ros_command(ros_cmd)
            logger.info(f"Launching communication node with command: {ros_cmd}")
            
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                bufsize=1, universal_newlines=True
            )
            active_ros_processes[comm_process_id] = process
            
            # Start output streaming
            threading.Thread(
                target=_stream_process_output, 
                args=(comm_process_id, process),
                daemon=True
            ).start()
            
            # Wait for node initialization
            logger.info("Waiting for communication node to initialize...")
            time.sleep(3)
            
            if process.poll() is not None:
                error_msg = f"Communication node failed to start, exit code: {process.returncode}"
                logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg}), 500
            
            logger.info("Communication node started successfully")
        else:
            logger.info("Communication node is already running")
        
        # Step 2: Launch bringup without RViz
        logger.info("Launching bringup launch file without RViz...")
        launch_result = _launch_ros2_launch_file('bringup_without_rviz')
        
        if not launch_result['success']:
            error_msg = f"Failed to launch bringup without RViz: {launch_result['error']}"
            logger.error(error_msg)
            return jsonify({'success': False, 'error': error_msg}), 500
        
        logger.info("Bringup without RViz launched successfully")
        
        # Step 3: Ensure controllers are running
        logger.info("Ensuring controllers are running...")
        _ensure_controllers_running()
        
        # Step 4: Return success response
        response = {
            'success': True,
            'message': 'Robot launched successfully without RViz visualization',
            'processes': {
                'communication_node': {
                    'status': 'running',
                    'pid': active_ros_processes.get(comm_process_id, {}).pid if comm_process_id in active_ros_processes else None
                },
                'bringup_launch': launch_result
            }
        }
        
        logger.info(f"Robot launch completed successfully: {response}")
        return jsonify(response), 200
        
    except Exception as e:
        error_msg = f"Unexpected error during robot launch: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/launch_status', methods=['GET'])
def get_launch_status():
    """
    Get the status of all running launch processes.
    """
    logger.info("Received request for launch status")
    
    try:
        status = _get_all_launch_status()
        logger.info(f"Launch status retrieved: {status}")
        return jsonify({
            'success': True,
            'launch_processes': status,
            'total_processes': len(status)
        }), 200
        
    except Exception as e:
        error_msg = f"Failed to get launch status: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/stop_launch', methods=['POST'])
def stop_launch():
    """
    Stop a specific launch process or all launch processes.
    """
    logger.info("Received request to stop launch process")
    
    try:
        data = request.get_json() or {}
        process_id = data.get('process_id')  # If None, stop all launch processes
        
        if process_id:
            # Stop specific process
            logger.info(f"Stopping specific launch process: {process_id}")
            result = _stop_launch_process(process_id)
            
            if result['success']:
                return jsonify(result), 200
            else:
                return jsonify(result), 500
        else:
            # Stop all launch processes
            logger.info("Stopping all launch processes")
            stopped_processes = []
            failed_processes = []
            
            for proc_id in list(active_ros_processes.keys()):
                if 'launch' in proc_id:
                    result = _stop_launch_process(proc_id)
                    if result['success']:
                        stopped_processes.append(proc_id)
                    else:
                        failed_processes.append(proc_id)
            
            response = {
                'success': True,
                'message': f"Stopped {len(stopped_processes)} launch processes",
                'stopped_processes': stopped_processes,
                'failed_processes': failed_processes
            }
            
            logger.info(f"Launch stop operation completed: {response}")
            return jsonify(response), 200
            
    except Exception as e:
        error_msg = f"Failed to stop launch process: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

logger.info("ROS2 Launch Management API Endpoints added successfully")

# --- WebSocket Bridge API Endpoints ---
logger.info("Adding WebSocket Bridge API Endpoints")

@app.route('/api/websocket_bridge/start', methods=['POST'])
def start_websocket_bridge():
    """
    Start the WebSocket bridge for real-time communication.
    """
    logger.info("Received request to start WebSocket bridge")
    
    try:
        data = request.get_json() or {}
        port = data.get('port', WEBSOCKET_BRIDGE_PORT)
        
        result = _start_websocket_bridge(port)
        
        if result['success']:
            logger.info(f"WebSocket bridge started successfully: {result}")
            return jsonify(result), 200
        else:
            logger.error(f"Failed to start WebSocket bridge: {result['error']}")
            return jsonify(result), 500
            
    except Exception as e:
        error_msg = f"Unexpected error starting WebSocket bridge: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/websocket_bridge/stop', methods=['POST'])
def stop_websocket_bridge():
    """
    Stop the WebSocket bridge.
    """
    logger.info("Received request to stop WebSocket bridge")
    
    try:
        data = request.get_json() or {}
        port = data.get('port', WEBSOCKET_BRIDGE_PORT)
        
        result = _stop_websocket_bridge(port)
        
        if result['success']:
            logger.info(f"WebSocket bridge stopped successfully: {result}")
            return jsonify(result), 200
        else:
            logger.error(f"Failed to stop WebSocket bridge: {result['error']}")
            return jsonify(result), 500
            
    except Exception as e:
        error_msg = f"Unexpected error stopping WebSocket bridge: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/websocket_bridge/status', methods=['GET'])
def get_websocket_bridge_status():
    """
    Get the status of the WebSocket bridge.
    """
    logger.info("Received request for WebSocket bridge status")
    
    try:
        data = request.args
        port = data.get('port', WEBSOCKET_BRIDGE_PORT)
        
        result = _get_websocket_bridge_status(port)
        
        logger.info(f"WebSocket bridge status: {result}")
        return jsonify({
            'success': True,
            'bridge_status': result,
            'port': port
        }), 200
        
    except Exception as e:
        error_msg = f"Failed to get WebSocket bridge status: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

logger.info("WebSocket Bridge API Endpoints added successfully")

# --- Phase 2: ROS2 Topic Management Endpoints ---
logger.info("Adding Phase 2: ROS2 Topic Management Endpoints")

@app.route('/api/ros2/topics', methods=['GET'])
def get_ros2_topics():
    """
    Get list of available ROS2 topics through the WebSocket bridge.
    """
    logger.info("Received request to get ROS2 topics")
    
    try:
        # Use our ROS2 environment to get topics directly
        cmd = _bash_ros_command("ros2 topic list")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            topics = [topic.strip() for topic in result.stdout.strip().split('\n') if topic.strip()]
            logger.info(f"Found {len(topics)} ROS2 topics")
            return jsonify({
                'success': True,
                'topics': topics,
                'count': len(topics),
                'status': 'active'
            }), 200
        else:
            # Check if it's a ROS2 context error
            if "!rclpy.ok()" in result.stderr or "RuntimeError" in result.stderr:
                logger.info("ROS2 context not available - no active nodes running")
                return jsonify({
                    'success': True,
                    'topics': [],
                    'count': 0,
                    'status': 'no_context',
                    'message': 'ROS2 context not available - no active nodes running. Start robot nodes to see topics.'
                }), 200
            else:
                error_msg = f"Failed to get ROS2 topics: {result.stderr}"
                logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg}), 500
            
    except Exception as e:
        error_msg = f"Failed to get ROS2 topics: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/ros2/topics/<topic_name>/info', methods=['GET'])
def get_topic_info(topic_name):
    """
    Get information about a specific ROS2 topic.
    """
    logger.info(f"Received request to get info for topic: {topic_name}")
    
    try:
        # Get topic info
        cmd = _bash_ros_command(f"ros2 topic info {topic_name}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            logger.info(f"Successfully got info for topic: {topic_name}")
            return jsonify({
                'success': True,
                'topic_name': topic_name,
                'info': result.stdout,
                'raw_output': result.stdout
            }), 200
        else:
            error_msg = f"Failed to get topic info: {result.stderr}"
            logger.error(error_msg)
            return jsonify({'success': False, 'error': error_msg}), 500
            
    except Exception as e:
        error_msg = f"Failed to get topic info: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/ros2/topics/<topic_name>/echo', methods=['GET'])
def echo_topic(topic_name):
    """
    Echo (read) messages from a ROS2 topic.
    """
    logger.info(f"Received request to echo topic: {topic_name}")
    
    try:
        # Get a single message from the topic
        cmd = _bash_ros_command(f"ros2 topic echo --once {topic_name}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        
        if result.returncode == 0:
            logger.info(f"Successfully echoed topic: {topic_name}")
            return jsonify({
                'success': True,
                'topic_name': topic_name,
                'message': result.stdout,
                'raw_output': result.stdout
            }), 200
        else:
            error_msg = f"Failed to echo topic: {result.stderr}"
            logger.error(error_msg)
            return jsonify({'success': False, 'error': error_msg}), 500
            
    except Exception as e:
        error_msg = f"Failed to echo topic: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/ros2/services', methods=['GET'])
def get_ros2_services():
    """
    Get list of available ROS2 services.
    """
    logger.info("Received request to get ROS2 services")
    
    try:
        cmd = _bash_ros_command("ros2 service list")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            services = [service.strip() for service in result.stdout.strip().split('\n') if service.strip()]
            logger.info(f"Found {len(services)} ROS2 services")
            return jsonify({
                'success': True,
                'services': services,
                'count': len(services),
                'status': 'active'
            }), 200
        else:
            # Check if it's a ROS2 context error
            if "!rclpy.ok()" in result.stderr or "RuntimeError" in result.stderr:
                logger.info("ROS2 context not available - no active nodes running")
                return jsonify({
                    'success': True,
                    'services': [],
                    'count': 0,
                    'status': 'no_context',
                    'message': 'ROS2 context not available - no active nodes running. Start robot nodes to see services.'
                }), 200
            else:
                error_msg = f"Failed to get ROS2 services: {result.stderr}"
                logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg}), 500
            
    except Exception as e:
        error_msg = f"Failed to get ROS2 services: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/ros2/nodes', methods=['GET'])
def get_ros2_nodes():
    """
    Get list of running ROS2 nodes.
    """
    logger.info("Received request to get ROS2 nodes")
    
    try:
        cmd = _bash_ros_command("ros2 node list")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            nodes = [node.strip() for node in result.stdout.strip().split('\n') if node.strip()]
            logger.info(f"Found {len(nodes)} ROS2 nodes")
            return jsonify({
                'success': True,
                'nodes': nodes,
                'count': len(nodes),
                'status': 'active'
            }), 200
        else:
            # Check if it's a ROS2 context error
            if "!rclpy.ok()" in result.stderr or "RuntimeError" in result.stderr:
                logger.info("ROS2 context not available - no active nodes running")
                return jsonify({
                    'success': True,
                    'nodes': [],
                    'count': 0,
                    'status': 'no_context',
                    'message': 'ROS2 context not available - no active nodes running. Start robot nodes to see nodes.'
                }), 200
            else:
                error_msg = f"Failed to get ROS2 nodes: {result.stderr}"
                logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg}), 500
            
    except Exception as e:
        error_msg = f"Failed to get ROS2 nodes: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

logger.info("Phase 2: ROS2 Topic Management Endpoints added successfully")

@app.route('/api/ros2/system_status', methods=['GET'])
def get_ros2_system_status():
    """
    Get overall ROS2 system status including context availability.
    """
    logger.info("Received request to get ROS2 system status")
    
    try:
        # Check if ROS2 context is available by testing a simple command
        cmd = _bash_ros_command("ros2 node list")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            # ROS2 context is active
            nodes = [node.strip() for node in result.stdout.strip().split('\n') if node.strip()]
            logger.info(f"ROS2 system is active with {len(nodes)} nodes")
            return jsonify({
                'success': True,
                'status': 'active',
                'context_available': True,
                'active_nodes': len(nodes),
                'message': f'ROS2 system is active with {len(nodes)} nodes'
            }), 200
        else:
            # Check if it's a ROS2 context error
            if "!rclpy.ok()" in result.stderr or "RuntimeError" in result.stderr:
                logger.info("ROS2 system status: no active context")
                return jsonify({
                    'success': True,
                    'status': 'no_context',
                    'context_available': False,
                    'active_nodes': 0,
                    'message': 'ROS2 context not available - no active nodes running. Start robot nodes to establish context.'
                }), 200
            else:
                # Some other error
                error_msg = f"Failed to check ROS2 system status: {result.stderr}"
                logger.error(error_msg)
                return jsonify({'success': False, 'error': error_msg}), 500
            
    except Exception as e:
        error_msg = f"Failed to check ROS2 system status: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/websocket_bridge/validate_context', methods=['GET'])
def validate_ros2_context():
    """
    Validate that ROS2 context is properly initialized and accessible.
    """
    logger.info("Received request to validate ROS2 context")
    
    try:
        result = _validate_ros2_context()
        
        if result['success']:
            logger.info("ROS2 context validation successful")
            return jsonify({
                'success': True,
                'message': result['message'],
                'details': result['details']
            }), 200
        else:
            logger.warning("ROS2 context validation failed")
            return jsonify({
                'success': False,
                'message': result['message'],
                'details': result['details']
            }), 500
            
    except Exception as e:
        error_msg = f"ROS2 context validation failed with exception: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

@app.route('/api/test_ros2', methods=['GET'])
def test_ros2():
    """
    Test endpoint to verify ROS2 is available and working.
    """
    logger.info("Received request to test ROS2 availability")
    
    try:
        # Test basic ROS2 command
        test_cmd = _bash_ros_command("ros2 --help")
        logger.info(f"Testing ROS2 with command: {test_cmd}")
        
        result = subprocess.run(test_cmd, capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            logger.info("ROS2 test successful")
            return jsonify({
                'success': True,
                'message': 'ROS2 is available and working',
                'stdout': result.stdout[:200] + '...' if len(result.stdout) > 200 else result.stdout,
                'stderr': result.stderr[:200] + '...' if len(result.stderr) > 200 else result.stderr
            }), 200
        else:
            error_msg = f"ROS2 test failed with return code {result.returncode}"
            logger.error(f"{error_msg}. Stderr: {result.stderr}")
            return jsonify({
                'success': False,
                'error': error_msg,
                'stderr': result.stderr
            }), 500
            
    except Exception as e:
        error_msg = f"ROS2 test failed with exception: {str(e)}"
        logger.error(error_msg, exc_info=True)
        return jsonify({'success': False, 'error': error_msg}), 500

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