#!/usr/bin/env python3
"""
TurtleBot3 Motor Control Interface
This script communicates with the OpenCR board via DYNAMIXEL protocol
to control the TurtleBot3 motors.
"""

import struct
import serial
import time
import logging
from enum import IntEnum
from typing import Optional, Dict, Union

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('TurtleBot3')


class CommunicationError(Exception):
    """Base exception for communication errors"""
    pass


class SerialConnectionError(CommunicationError):
    """Exception for serial connection issues"""
    pass


class PacketError(CommunicationError):
    """Exception for packet-related errors"""
    pass


class TimeoutError(CommunicationError):
    """Exception for timeout errors"""
    pass


class ControlTableAddr(IntEnum):
    """Control table addresses matching the OpenCR firmware"""
    MODEL_INFORM = 2
    MILLIS = 10
    DEBUG_MODE = 14
    CONNECT_ROS2 = 15
    CONNECT_MANIP = 16
    DEVICE_STATUS = 18
    HEARTBEAT = 19
    
    # Battery
    BATTERY_VOLTAGE = 42
    BATTERY_PERCENT = 46
    
    # IMU
    ANGULAR_VELOCITY_X = 60
    ANGULAR_VELOCITY_Y = 64
    ANGULAR_VELOCITY_Z = 68
    LINEAR_ACC_X = 72
    LINEAR_ACC_Y = 76
    LINEAR_ACC_Z = 80
    
    # Motor status
    PRESENT_CURRENT_L = 120
    PRESENT_CURRENT_R = 124
    PRESENT_VELOCITY_L = 128
    PRESENT_VELOCITY_R = 132
    PRESENT_POSITION_L = 136
    PRESENT_POSITION_R = 140
    
    # Motor control
    MOTOR_CONNECT = 148
    MOTOR_TORQUE = 149
    CMD_VEL_LINEAR_X = 150
    CMD_VEL_ANGULAR_Z = 170
    PROFILE_ACC_L = 174
    PROFILE_ACC_R = 178
        
class DynamixelProtocol:
    """DYNAMIXEL Protocol 2.0 implementation for communication"""
    
    # Instruction types
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_SYNC_WRITE = 0x83
    
    # Header
    HEADER = [0xFF, 0xFF, 0xFD, 0x00]
    
    # OpenCR slave ID (from firmware)
    OPENCR_ID = 200
    
    def __init__(self, port, baudrate=115200, timeout=0.5, max_retries=3):
        """
        Initialize serial connection with error handling
        
        Args:
            port: Serial port path (e.g., '/dev/ttyACM0')
            baudrate: Communication speed (default: 115200)
            timeout: Read timeout in seconds
            max_retries: Maximum retry attempts for failed operations
        
        Raises:
            SerialConnectionError: If connection cannot be established
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.max_retries = max_retries
        self.serial = None
        self.is_connected = False
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            time.sleep(0.1)  # Allow connection to stabilize
            self.is_connected = True
            logger.info(f"Serial connection established on {port} at {baudrate} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {port}: {e}")
            raise SerialConnectionError(f"Cannot open port {port}: {e}")
        except Exception as e:
            logger.error(f"Unexpected error opening serial port: {e}")
            raise SerialConnectionError(f"Unexpected error: {e}")
    
    def calculate_crc(self, data):
        """Calculate CRC16 for DYNAMIXEL Protocol 2.0"""
        crc = 0
        crc_table = [
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
        ]
        
        for byte in data:
            i = ((crc >> 8) ^ byte) & 0xFF
            crc = ((crc << 8) ^ crc_table[i]) & 0xFFFF
        
        return crc
    
    def make_packet(self, instruction, address=None, data=None):
        """
        Create a DYNAMIXEL protocol packet
        
        Args:
            instruction: Instruction type (READ/WRITE)
            address: Control table address
            data: Data to write or length to read
            
        Returns:
            bytes: Complete packet with CRC
            
        Raises:
            PacketError: If packet cannot be constructed
        """
        try:
            packet = bytearray(self.HEADER)
            packet.append(self.OPENCR_ID)
            
            if instruction == self.INST_READ and address is not None:
                length = 7  # Instruction + Address(2) + Length(2) + CRC(2)
                packet.extend(struct.pack('<H', length))
                packet.append(instruction)
                packet.extend(struct.pack('<H', address))
                packet.extend(struct.pack('<H', data if data else 4))
            elif instruction == self.INST_WRITE and address is not None and data is not None:
                if isinstance(data, (list, tuple)):
                    data_bytes = bytearray(data)
                elif isinstance(data, int):
                    data_bytes = struct.pack('<i', data)
                else:
                    data_bytes = data
                
                length = 5 + len(data_bytes)  # Instruction + Address(2) + Data + CRC(2)
                packet.extend(struct.pack('<H', length))
                packet.append(instruction)
                packet.extend(struct.pack('<H', address))
                packet.extend(data_bytes)
            else:
                raise PacketError(f"Invalid packet parameters: inst={instruction}, addr={address}")
            
            # Calculate and append CRC
            crc = self.calculate_crc(packet)
            packet.extend(struct.pack('<H', crc))
            
            return bytes(packet)
        except struct.error as e:
            raise PacketError(f"Error packing packet data: {e}")
        except Exception as e:
            raise PacketError(f"Unexpected error creating packet: {e}")
    
    def send_packet(self, packet):
        """
        Send packet to device with error handling
        
        Args:
            packet: bytes to send
            
        Raises:
            SerialConnectionError: If send fails
        """
        if not self.is_connected or self.serial is None:
            raise SerialConnectionError("Serial port not connected")
        
        try:
            self.serial.write(packet)
            self.serial.flush()  # Ensure data is sent
        except serial.SerialException as e:
            logger.error(f"Serial write error: {e}")
            self.is_connected = False
            raise SerialConnectionError(f"Failed to send packet: {e}")
        except Exception as e:
            logger.error(f"Unexpected error sending packet: {e}")
            raise SerialConnectionError(f"Unexpected send error: {e}")
    
    def read_response(self, retry_on_timeout=True):
        """
        Read response packet with error handling
        
        Args:
            retry_on_timeout: Whether to retry on timeout
            
        Returns:
            bytes: Response data (without CRC) or None
            
        Raises:
            SerialConnectionError: If read fails
            PacketError: If packet is malformed
            TimeoutError: If timeout occurs and retry is disabled
        """
        if not self.is_connected or self.serial is None:
            raise SerialConnectionError("Serial port not connected")
        
        try:
            # Clear any stale data
            if self.serial.in_waiting > 100:
                logger.warning(f"Clearing {self.serial.in_waiting} bytes of stale data")
                self.serial.reset_input_buffer()
            
            # Read header
            header = self.serial.read(4)
            if len(header) == 0:
                if retry_on_timeout:
                    return None
                raise TimeoutError("Timeout waiting for response header")
            
            if len(header) < 4:
                logger.warning(f"Incomplete header: got {len(header)} bytes")
                raise PacketError(f"Incomplete header received: {len(header)} bytes")
            
            if list(header) != self.HEADER:
                logger.warning(f"Invalid header: {list(header)}")
                raise PacketError(f"Invalid header: expected {self.HEADER}, got {list(header)}")
            
            # Read ID
            device_id = self.serial.read(1)
            if len(device_id) == 0:
                raise TimeoutError("Timeout reading device ID")
            
            # Read length
            length_bytes = self.serial.read(2)
            if len(length_bytes) < 2:
                raise PacketError(f"Incomplete length field: {len(length_bytes)} bytes")
            
            length = struct.unpack('<H', length_bytes)[0]
            
            if length > 1024:  # Sanity check
                raise PacketError(f"Invalid packet length: {length}")
            
            # Read remaining data
            remaining = self.serial.read(length)
            if len(remaining) < length:
                raise PacketError(f"Incomplete packet: expected {length}, got {len(remaining)} bytes")
            
            # Verify CRC
            packet = header + device_id + length_bytes + remaining
            received_crc = struct.unpack('<H', remaining[-2:])[0]
            calculated_crc = self.calculate_crc(packet[:-2])
            
            if received_crc != calculated_crc:
                logger.warning(f"CRC mismatch: received {received_crc:04X}, calculated {calculated_crc:04X}")
                raise PacketError(f"CRC check failed: {received_crc:04X} != {calculated_crc:04X}")
            
            # Check for error code in response
            if len(remaining) > 0:
                error_code = remaining[1]  # Second byte is error code
                if error_code != 0:
                    logger.warning(f"Device returned error code: {error_code}")
            
            return remaining[:-2]  # Return data without CRC
            
        except serial.SerialException as e:
            logger.error(f"Serial read error: {e}")
            self.is_connected = False
            raise SerialConnectionError(f"Failed to read response: {e}")
        except (TimeoutError, PacketError):
            raise  # Re-raise our custom errors
        except Exception as e:
            logger.error(f"Unexpected error reading response: {e}")
            raise PacketError(f"Unexpected read error: {e}")
    
    def write_data(self, address, data):
        """
        Write data to control table with retry logic
        
        Args:
            address: Control table address
            data: Data to write
            
        Returns:
            bytes: Response data or None
            
        Raises:
            CommunicationError: If write fails after retries
        """
        last_error = None
        
        for attempt in range(self.max_retries):
            try:
                packet = self.make_packet(self.INST_WRITE, address, data)
                if packet:
                    self.send_packet(packet)
                    response = self.read_response(retry_on_timeout=(attempt < self.max_retries - 1))
                    
                    if response is not None:
                        return response
                    
                    if attempt < self.max_retries - 1:
                        logger.debug(f"Write retry {attempt + 1}/{self.max_retries} for address {address}")
                        time.sleep(0.01)  # Brief delay before retry
                
            except (PacketError, SerialConnectionError, TimeoutError) as e:
                last_error = e
                if attempt < self.max_retries - 1:
                    logger.debug(f"Write attempt {attempt + 1} failed: {e}")
                    time.sleep(0.01)
                else:
                    logger.error(f"Write failed after {self.max_retries} attempts: {e}")
        
        if last_error:
            raise last_error
        return None
    
    def read_data(self, address, length=4):
        """
        Read data from control table with retry logic
        
        Args:
            address: Control table address
            length: Number of bytes to read
            
        Returns:
            bytes: Data read or None
            
        Raises:
            CommunicationError: If read fails after retries
        """
        last_error = None
        
        for attempt in range(self.max_retries):
            try:
                packet = self.make_packet(self.INST_READ, address, length)
                if packet:
                    self.send_packet(packet)
                    response = self.read_response(retry_on_timeout=(attempt < self.max_retries - 1))
                    
                    # if response and len(response) > 3:
                    #     return response[3:]  # Skip instruction, error, and parameter
                    if response and len(response) > 2:
                        return response[2:]  # Skip instruction, error, and parameter
                    
                    if attempt < self.max_retries - 1:
                        logger.debug(f"Read retry {attempt + 1}/{self.max_retries} for address {address}")
                        time.sleep(0.01)
                        
            except (PacketError, SerialConnectionError, TimeoutError) as e:
                last_error = e
                if attempt < self.max_retries - 1:
                    logger.debug(f"Read attempt {attempt + 1} failed: {e}")
                    time.sleep(0.01)
                else:
                    logger.error(f"Read failed after {self.max_retries} attempts: {e}")
        
        if last_error:
            raise last_error
        return None
    
    def reconnect(self):
        """
        Attempt to reconnect to the serial port
        
        Returns:
            bool: True if reconnection successful
        """
        logger.info("Attempting to reconnect...")
        
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except:
            pass
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            time.sleep(0.1)
            self.is_connected = True
            logger.info("Reconnection successful")
            return True
        except Exception as e:
            logger.error(f"Reconnection failed: {e}")
            self.is_connected = False
            return False
    
    def close(self):
        """Close serial connection safely"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.is_connected = False
                logger.info("Serial connection closed")
        except Exception as e:
            logger.error(f"Error closing serial connection: {e}")


class TurtleBot3Controller:
    """High-level controller for TurtleBot3 with error handling"""
    
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, auto_reconnect=True):
        """
        Initialize TurtleBot3 controller
        
        Args:
            port: Serial port path
            baudrate: Communication speed
            auto_reconnect: Automatically attempt to reconnect on errors
            
        Raises:
            SerialConnectionError: If initial connection fails
        """
        self.auto_reconnect = auto_reconnect
        self.dxl = DynamixelProtocol(port, baudrate)
        self.heartbeat_counter = 0
        self.last_heartbeat_time = time.time()
        self.last_command_time = time.time()
        self.connection_lost_notified = False
        
        logger.info("TurtleBot3 controller initialized")
        
    def _handle_communication_error(self, error, operation="operation"):
        """
        Handle communication errors with optional reconnection
        
        Args:
            error: The exception that occurred
            operation: Description of the operation that failed
        """
        if isinstance(error, SerialConnectionError):
            if not self.connection_lost_notified:
                logger.error(f"Connection lost during {operation}: {error}")
                self.connection_lost_notified = True
            
            if self.auto_reconnect and self.dxl.reconnect():
                self.connection_lost_notified = False
                logger.info(f"Reconnected, retrying {operation}")
                return True
        else:
            logger.warning(f"Communication error during {operation}: {error}")
        
        return False
    
    def send_heartbeat(self):
        """
        Send heartbeat to maintain connection
        
        Returns:
            bool: True if heartbeat sent successfully
        """
        current_time = time.time()
        if current_time - self.last_heartbeat_time >= 0.4:  # Send every 400ms
            self.heartbeat_counter = (self.heartbeat_counter + 1) % 256
            
            for attempt in range(2):  # Two attempts for heartbeat
                try:
                    self.dxl.write_data(ControlTableAddr.HEARTBEAT, 
                                       struct.pack('B', self.heartbeat_counter))
                    self.last_heartbeat_time = current_time
                    return True
                except CommunicationError as e:
                    if not self._handle_communication_error(e, "heartbeat"):
                        if attempt == 1:  # Last attempt
                            logger.error("Failed to send heartbeat")
                        continue
                    # If reconnected, try again
                    continue
            
            return False
        return True
        
    def get_torque_state(self) -> Optional[bool]:
        """
        Read current torque enable state
        
        Returns:
            bool: True if torque enabled, False if disabled, None if read fails
        """
        try:
            data = self.dxl.read_data(ControlTableAddr.MOTOR_TORQUE, 1)
            if data:
                return struct.unpack('B', data)[0] != 0
            return None
        except CommunicationError as e:
            self._handle_communication_error(e, "get_torque_state")
            return None
        except struct.error as e:
            logger.error(f"Error unpacking torque state data: {e}")
            return None

    def set_velocity(self, velocity_set):
        """
        Set velocity command for the robot
        
        Args:
            linear_x: Linear velocity in m/s (forward/backward)
            angular_z: Angular velocity in rad/s (rotation)
            
        Returns:
            bool: True if command sent successfully
            
        Raises:
            ValueError: If velocity values are out of range
        """
        # Validate inputs
        MAX_LINEAR = 0.22  # m/s (Burger max)
        MAX_ANGULAR = 2.84  # rad/s
        
        linear_x, angular_z = velocity_set

        if abs(linear_x) > MAX_LINEAR:
            logger.warning(f"Linear velocity {linear_x} exceeds max {MAX_LINEAR}, clamping")
            linear_x = max(-MAX_LINEAR, min(MAX_LINEAR, linear_x))
        
        if abs(angular_z) > MAX_ANGULAR:
            logger.warning(f"Angular velocity {angular_z} exceeds max {MAX_ANGULAR}, clamping")
            angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_z))
        
        # Convert to centimeters per second (firmware expects values in cm/s * 100)
        linear_cmd = int(linear_x * 100)  # m/s to cm/s
        angular_cmd = int(angular_z * 100)  # rad/s to (rad/s * 100)
        
        success = False
        
        for attempt in range(2):
            try:
                # Write linear velocity
                self.dxl.write_data(ControlTableAddr.CMD_VEL_LINEAR_X, 
                                   struct.pack('<i', linear_cmd))
                
                # Write angular velocity
                self.dxl.write_data(ControlTableAddr.CMD_VEL_ANGULAR_Z, 
                                   struct.pack('<i', angular_cmd))
                
                # Send heartbeat
                self.send_heartbeat()
                self.last_command_time = time.time()
                success = True
                break
                
            except CommunicationError as e:
                if not self._handle_communication_error(e, "set_velocity"):
                    if attempt == 1:
                        logger.error(f"Failed to set velocity after retries: {e}")
                        raise
                    continue
                # If reconnected, try again
                continue
        
        return success
    
    def get_motor_status(self) -> Optional[Dict[str, int]]:
        """
        Read current motor status with error handling
        
        Returns:
            dict: Motor status data or None if read fails
        """
        status = {}
        
        try:
            # Read present velocity
            vel_l = self.dxl.read_data(ControlTableAddr.PRESENT_VELOCITY_L, 4)
            vel_r = self.dxl.read_data(ControlTableAddr.PRESENT_VELOCITY_R, 4)
            
            if vel_l and vel_r:
                status['velocity_left'] = struct.unpack('<i', vel_l)[0]
                status['velocity_right'] = struct.unpack('<i', vel_r)[0]
            
            # Read present position
            pos_l = self.dxl.read_data(ControlTableAddr.PRESENT_POSITION_L, 4)
            pos_r = self.dxl.read_data(ControlTableAddr.PRESENT_POSITION_R, 4)
            
            if pos_l and pos_r:
                status['position_left'] = struct.unpack('<i', pos_l)[0]
                status['position_right'] = struct.unpack('<i', pos_r)[0]
            
            return status if status else None
            
        except CommunicationError as e:
            self._handle_communication_error(e, "get_motor_status")
            return None
        except struct.error as e:
            logger.error(f"Error unpacking motor status data: {e}")
            return None
    
    def get_battery_voltage(self) -> Optional[float]:
        """
        Read battery voltage with error handling
        
        Returns:
            float: Battery voltage in volts or None if read fails
        """
        try:
            data = self.dxl.read_data(ControlTableAddr.BATTERY_VOLTAGE, 4)
            if data:
                voltage_x100 = struct.unpack('<I', data)[0]
                return voltage_x100 / 100.0  # Convert back to volts
            return None
        except CommunicationError as e:
            self._handle_communication_error(e, "get_battery_voltage")
            return None
        except struct.error as e:
            logger.error(f"Error unpacking battery voltage data: {e}")
            return None
    
    def is_connected(self) -> bool:
        """
        Check if connected to the robot
        
        Returns:
            bool: True if connected
        """
        return self.dxl.is_connected
    
    def stop(self):
        """
        Stop the robot (emergency stop)
        
        Returns:
            bool: True if stop command sent successfully
        """
        logger.info("Stopping robot")
        try:
            return self.set_velocity((0.0, 0.0))
        except Exception as e:
            logger.error(f"Error during emergency stop: {e}")
            return False
    
    def close(self):
        """Close connection safely"""
        logger.info("Closing TurtleBot3 controller")
        try:
            self.stop()
            time.sleep(0.1)  # Allow stop command to process
        except:
            pass
        finally:
            self.dxl.close()


# Example usage with comprehensive error handling
if __name__ == '__main__':
    robot = None
    
    try:
        # Initialize controller (adjust port as needed)
        print("Initializing TurtleBot3 controller...")
        robot = TurtleBot3Controller(port='/dev/ttyACM0', baudrate=115200, auto_reconnect=True)
        
        print("TurtleBot3 Motor Control Started")
        print("Press Ctrl+C to stop")
        
        # Check connection
        if not robot.is_connected():
            print("ERROR: Not connected to robot")
            exit(1)

        # Check battery
        voltage = robot.get_battery_voltage()
        if voltage:
            print(f"Battery Voltage: {voltage:.2f}V")
            if voltage < 11.0:
                print("WARNING: Low battery!")
        else:
            print("WARNING: Could not read battery voltage")
    
        # Example 1: Move forward for ?? seconds
        # All these need to be redone to look like this

        is_done = False
        vel_tuple = (0.0,0.0)
        seconds_to_run = 3
        current_state = "start"
        end_time = time.time() + seconds_to_run
        while not is_done:
            # UPDATE STATE - SENSE
            time_now = time.time()

            # MAKE DECISIONS - DECIDE
            # Transition betweens states.
            if current_state == "start": 
                current_state = "forward"                    
                print(f"\nMoving forward for {seconds_to_run} seconds...")
                vel_tuple=(0.1, 0.0)
                end_time = time_now + seconds_to_run
            if time_now > end_time:
                if current_state == "forward":
                    current_state = "rotate"                    
                    print(f"Rotating for {seconds_to_run} seconds...")
                    vel_tuple=(0.0, 0.5)
                    end_time = time_now + seconds_to_run
                elif current_state == "rotate":
                    current_state = "curve"                    
                    print(f"Moving in curve {seconds_to_run} seconds...")
                    vel_tuple=(0.1, 0.3)
                    end_time = time_now + seconds_to_run
                elif current_state == "curve":
                    current_state = "stop"                    
                    print("Stopping...")
                    vel_tuple=(0.0, 0.0)
                    end_time = time_now + seconds_to_run
                else:
                    is_done=True
                    robot.stop()

            # TAKE ACTION - ACT
            # Set the velocity based on the state the robot is in.
            if robot.set_velocity(vel_tuple):  # Only change the velocity in one place in the code.
                pass
            else:
                print("ERROR: Failed to set velocity")
        # End of the big while loop 
       
        # Read motor status
        print("\nReading motor status...")
        status = robot.get_motor_status()
        if status:
            print(f"Motor Status:")
            print(f"  Left - Position: {status.get('position_left', 'N/A')}, "
                  f"Velocity: {status.get('velocity_left', 'N/A')}")
            print(f"  Right - Position: {status.get('position_right', 'N/A')}, "
                  f"Velocity: {status.get('velocity_right', 'N/A')}")
        else:
            print("WARNING: Could not read motor status")
        
        # Final battery check
        voltage = robot.get_battery_voltage()
        if voltage:
            print(f"\nFinal Battery Voltage: {voltage:.2f}V")
        
    except SerialConnectionError as e:
        print(f"\nSerial connection error: {e}")
        print("Please check:")
        print("  1. Robot is powered on")
        print("  2. USB cable is connected")
        print("  3. Correct port is specified")
        print("  4. User has permission to access the port (add user to dialout group)")
        
    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt detected, stopping robot...")
        
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        logger.exception("Unexpected error in main")
        
    finally:
        if robot:
            print("Shutting down...")          
            robot.close()
        print("Connection closed")