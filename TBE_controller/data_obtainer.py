from utilities import *
import smbus2
import time
import struct

# Class to obtain the data from the sensors and store it in a structured format
class SensorData():

    def __init__(self, logger: Logger):

        # Logger for logging the sensor data
        self.logger = logger

        # Heel and toe sensor data
        self.heel_fsr = 0.0
        self.toe_fsr = 0.0

        # Torque input data
        self.torque_input = 0.0

        # Bus to read sensor data
        self.bus = smbus2.SMBus(I2C_BUS)

        self.logger.logger.info("Sensor stream channel opened.")
    
    # Function to read the sensors data
    def readSensors(self):

        # Reading FSR data
        self.toe_fsr = self.read_channel(self.bus, CFG_AIN0)
        self.heel_fsr = self.read_channel(self.bus, CFG_AIN1)

        v0 = max(self.toe_fsr, 0) * VOLTS_PER_COUNT
        v1 = max(self.heel_fsr, 0) * VOLTS_PER_COUNT

        print(f"{self.toe_fsr:>12d}  {v0:>10.4f}  {self.heel_fsr:>12d}  {v1:>10.4f}")

        # Readin torque data
        self.torque_input = 0.0

    def sendTorqueData(self, torque: float):
        self.logger.logger.info(f"Torque sent: {torque}")

    def read_channel(self, bus: smbus2.SMBus, config: list[int]) -> int:
        """Trigger a single-shot conversion and return the signed 16-bit raw count."""
        # Write config register to start conversion
        bus.write_i2c_block_data(ADC_ADDR, REG_CONFIG, config)
        # Wait for conversion to complete
        time.sleep(CONV_DELAY)
        # Read 2 bytes from conversion register
        data = bus.read_i2c_block_data(ADC_ADDR, REG_CONVERSION, 2)
        # Big-endian signed 16-bit
        raw = struct.unpack(">h", bytes(data))[0]
        return raw
    
 


        