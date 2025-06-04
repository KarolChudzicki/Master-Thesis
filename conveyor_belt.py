from pymodbus.client import ModbusSerialClient


# ==================== Communication addresses ====================
main_control_bit_address = 0x0200
given_frequency_address = 0x0201


operation_run_or_stop = 0x0000       # 0 - stop, 1 - operating
jog = 0x0001                         # 0 - invalid, 1 - jog
forward_reverse = 0x0002             # 0 - forward, 1 - reverse


class conveyorBelt:
    # ==================== Connect to the inverter ====================
    def __init__(self):
        self.client = ModbusSerialClient(port='COM4', baudrate=19200, stopbits=1, bytesize=8, parity='N', timeout=3)
        
        if self.client.connect():
            print("Connected to the inverter")
        else:
            print("Failed to connect to the inverter")
            self.client.close()
    
    # ==================== Function for starting ====================
    def start(self):
        self.client.write_register(address=main_control_bit_address,value=0x0001)
        print("Conveyor started")
    
    # ==================== Function for stoping ====================
    def stop(self):
        self.client.write_register(address=main_control_bit_address, value=(operation_run_or_stop | 0x0000))
        print("Conveyor stopped")
    
    # ==================== Function for direction control ====================
    def setDirection(self, direction):
        if direction == 0:
            self.client.write_register(address=main_control_bit_address, value=(0x0002))
            print("Direction set to forward")
        elif direction == 1:
            self.client.write_register(address=main_control_bit_address, value=(0x0004))
            print("Direction set to reverse")
        else:
            print("Incorrect direction value")
    
    # ==================== Function for speed control ====================  
    def setSpeed(self, speed):
        if speed >= 0 and speed <= 1000:
            self.client.write_register(address=given_frequency_address, value=speed, unit=1)
            print(f"Speed set to {speed}")
        else:
            print("Incorrect speed value")





    




