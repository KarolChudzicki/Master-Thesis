# This script handles communication with a gripper using Modbus RTU over serial. 
# It includes functions to connect to the gripper, activate it, check its status, 
# and control its position, speed, and force. The gripper's status can be checked
# using a function check status

import serial
import time
import logging

logging.basicConfig(level=logging.INFO)



class Gripper:
    def __init__(self, port='COM5', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=8) -> None:
        self.default_speed = 50 # 50% speed
        self.default_force = 1 # Lowest force
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                parity=parity,
                stopbits=stopbits,
                bytesize=bytesize
            )
            logging.info(f"Gripper initialized on port: {port}, with baudrate: {baudrate}")
        except serial.SerialException as e:
            logging.error(f"Failed to connect to the gripper on port: {port}, Error: {e}")
            raise
    
    def connect(self) -> None:
        """Check if the gripper is connected"""
        if self.ser.is_open:
            logging.info("Gripper connection established")
        else:
            logging.error("Failed to extablish gripper connection")
    
    def activate(self) -> None:
        """Function used to activate the gripper"""
        
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
        data = self.ser.readline()
        time.sleep(0.01)
            
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00\x72\xE1')
        data = self.ser.readline()
        time.sleep(0.01)

        while True:
            self.ser.write(b'\x09\x03\x07\xD0\x00\x01\x85\xCF')
            data = self.ser.readline()
            time.sleep(0.5)
            if data == b'\x09\x03\x02\x31\x00\x4C\x15':
                break
            
        logging.info("Gripper activated")
        
        
    def check_status(self) -> dict:
        """
        Checks the gripper status.

        Returns
        -------
        dict:
            - "ACTIVATION_STATUS": 0 - reset, 1 - activated
            - "ACTION_STATUS": 0 - stopped, 1 - moving to position
            - "GRIPPER_STATUS": 0 - reset, 1 - activating, 2 - unused, 3 - activated
            - "OBJECT_DETECTION_STATUS": 0 - moving, 1 - stopped (opening), 2 - stopped (closing), 3 - at position
            - "POSITION": Current finger gap in mm (0 = fully open, 85 = fully closed)
            - "FINGER_CURRENT": Finger current in mA
        """
    
        data = self.ser.readline()
        
        SLAVE_ID = 0x09
        FUNCTION_CODE = 0x03 # Read holding registers
        FIRST_REGISTER_ADDRESS_HIGH = 0x07
        FIRST_REGISTER_ADDRESS_LOW = 0xD0
        REGISTERS_WRITTEN_TO_HIGH = 0x00
        REGISTERS_WRITTEN_TO_LOW = 0x03
        
        command = [SLAVE_ID, 
                FUNCTION_CODE,
                FIRST_REGISTER_ADDRESS_HIGH, 
                FIRST_REGISTER_ADDRESS_LOW,
                REGISTERS_WRITTEN_TO_HIGH,
                REGISTERS_WRITTEN_TO_LOW]
        
        CRCL, CRCH, _ =  self.modbusCrc(command)
        command.append(CRCH)
        command.append(CRCL)
        self.ser.write(command)

        data = self.ser.readline()
        
        if len(data) != 11:
            logging.error("Invalid data received from gripper")
            return None
        else:
            data = [hex(byte) for byte in data]
            
            # Get useful values of the gripper
            GRIPPER_STATUS_BYTE = bin(int(data[3],16))[2:].zfill(8)
            GRIPPER_STATUS_PER_BIT = [int(bit) for bit in GRIPPER_STATUS_BYTE]
            POSITION = int(data[6],16)
            FINGER_CURRENT = (int(data[8],16) << 8) | (int(data[9],16)) # Merge two hex values together
            
            
            # Explanation of each bit
            ACTIVATION_STATUS = GRIPPER_STATUS_PER_BIT[7] # 0 - gripper reset, 1 - gripper activation
            ACTION_STATUS = GRIPPER_STATUS_PER_BIT[4] # 0 - stopped, 1 - go to position request
            GRIPPER_STATUS = GRIPPER_STATUS_PER_BIT[3] + GRIPPER_STATUS_PER_BIT[2] * 2 # 0 - gripper is in reset, 1 - activation in progress, 2 - not used, 3 - activation completed
            OBJECT_DETECTION_STATUS = GRIPPER_STATUS_PER_BIT[1] + GRIPPER_STATUS_PER_BIT[0] * 2 # 0 - fingers are in motion towards position, 1 - fingers stopped due to contact during opening, 2 - fingers stopped due to contact during closing, 3 - fingers are at requested position
            
            # Returns a dictionary with all parameters of the gripper
            return {
                "ACTIVATION_STATUS": ACTIVATION_STATUS,
                "ACTION_STATUS": ACTION_STATUS,
                "GRIPPER_STATUS": GRIPPER_STATUS,
                "OBJECT_DETECTION_STATUS": OBJECT_DETECTION_STATUS,
                "POSITION": POSITION,
                "FINGER_CURRENT": f"{FINGER_CURRENT*10} mA"
            }
    
    
    def open_close(self, POSITION_REQUEST: int, SPEED: int = None, FORCE: int = None):
        """
        Controls the gripper by opening or closing its fingers.
        
        Parameters
        ----------
        - POSITION_REQUEST (int): Desired gripper position in millimeters. 
                                Valid range: 0 (fully closed) to 85 mm (fully open).
        - SPEED (int): Speed of the fingers' movement as a percentage. 
                    Valid range: 1 (slowest) to 100 (fastest).
        - FORCE (int): Force applied by the fingers as a percentage. 
                    Valid range: 1 (lowest force) to 100 (maximum force).
        """
        
        # Use default values if SPEED and FORCE are not provided
        SPEED = SPEED if SPEED is not None else self.default_speed
        FORCE = FORCE if FORCE is not None else self.default_force
        
        SLAVE_ID = 0x09
        FUNCTION_CODE = 0x10
        FIRST_REGISTER_ADDRESS_HIGH = 0x03
        FIRST_REGISTER_ADDRESS_LOW = 0xE8
        REGISTERS_WRITTEN_TO_HIGH = 0x00
        REGISTERS_WRITTEN_TO_LOW = 0x03
        NUMBER_OF_DATA_BYTES = 0x06
        ACTION_REQUEST = 0x09
        GRIPPER_OPTIONS1 = 0x00
        GRIPPER_OPTIONS2 = 0x00
        POSITION_REQUEST = self.position_to_command(POSITION_REQUEST)
        SPEED = self.speed_to_command(SPEED)
        FORCE = self.force_to_command(FORCE)
        
        command = [SLAVE_ID, 
                FUNCTION_CODE,
                FIRST_REGISTER_ADDRESS_HIGH, 
                FIRST_REGISTER_ADDRESS_LOW,
                REGISTERS_WRITTEN_TO_HIGH,
                REGISTERS_WRITTEN_TO_LOW, 
                NUMBER_OF_DATA_BYTES, 
                ACTION_REQUEST, 
                GRIPPER_OPTIONS1, 
                GRIPPER_OPTIONS2, 
                POSITION_REQUEST, 
                SPEED, 
                FORCE]
        
        CRCL, CRCH, _ =  self.modbusCrc(command)
        command.append(CRCH)
        command.append(CRCL)
        self.ser.write(command)
    
    def position_to_command(self, POSITION: int) -> int:
        """Convert the given gripper position (in mm) to a corresponding command value that can be sent to the robot."""
        if POSITION > 85 or POSITION < 0:
            logging.error(f"Invalid position: {POSITION}, the valid range is 0-85")
            # Get the current position of the gripper
            status = self.check_status()
            return status["POSITION"] if status else None
        else:
            return int(round((1 - POSITION / 85.0) * 255.0))
    
    def speed_to_command(self, SPEED: int) -> int:
        """Convert the given gripper speed (in %) to a corresponding command value that can be sent to the robot."""
        if SPEED > 100 or SPEED <= 0:
            logging.error(f"Invalid speed: {SPEED}, the valid range is 0-100, using the default speed instead: {self.default_speed}")
            # Return default speed
            return int((self.default_speed - 1) * (255 - 1) / (100 - 1) + 1)
        else:
            return int((SPEED - 1) * (255 - 1) / (100 - 1) + 1)
    
    def force_to_command(self, FORCE: int) -> int:
        """Convert the given gripper force (in %) to a corresponding command value that can be sent to the robot."""
        if FORCE > 100 or FORCE <= 0:
            logging.error(f"Invalid force: {FORCE}, the valid range is 0-100, using the default force instead: {self.default_force}")
            return int((self.default_force - 1) * (255 - 1) / (100 - 1) + 1)
        else:
            return int((FORCE - 1) * (255 - 1) / (100 - 1) + 1)
        
    def modbusCrc(self, msg: str) -> int:
        """Calculate cyclic redundancy check for given modbus command"""
        crc = 0xFFFF
        for n in range(len(msg)):
            crc ^= msg[n]
            for i in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        
        crcl = (crc >> 8) & 0xFF
        crch = crc & 0xFF
        return crcl, crch, crc

g = Gripper()
g.activate()
g.open_close(51, 100, 1)