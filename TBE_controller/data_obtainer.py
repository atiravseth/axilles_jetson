from utilities import *
import numpy as np
import smbus2
import can          # ADDED: for CAN bus communication
import struct
import time

# ── CAN / Motor Constants ─────────────────────────────────────────────────── # ADDED
CAN_INTERFACE = "can0"
MOTOR_ID      = 0x68        # Change if your motor ID differs

# MIT mode limits for AK80-9
MIT_P_MIN  = -12.56
MIT_P_MAX  =  12.56
MIT_V_MIN  = -65.0
MIT_V_MAX  =  65.0
MIT_T_MIN  = -100.0
MIT_T_MAX  =  100.0
MIT_KP_MIN =   0.0
MIT_KP_MAX = 500.0
MIT_KD_MIN =   0.0
MIT_KD_MAX =   5.0

# CAN mode IDs
MODE_MIT      = 8
MODE_VELOCITY = 3

# ── CAN Helpers (from ak_80_test.py) ──────────────────────────────────────── # ADDED

def _clamp(val, lo, hi):
    return max(lo, min(hi, val))

def _float_to_uint(x, x_min, x_max, bits):
    x = _clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) / span))


# Class to obtain the data from the sensors and store it in a structured format
class SensorData():

    def __init__(self, logger: Logger):

        # Logger for logging the sensor data
        self.logger = logger

        # Heel and toe sensor data
        self.heel_fsr = 0.0
        self.toe_fsr = 0.0

        # Filtered FSR data for heel and toe
        self.filtered_heel_fsr = 0.0
        self.filtered_toe_fsr = 0.0
        self.alpha = 2 * np.pi * FSR_FILTER_CUTOFF * DT / (2 * np.pi * FSR_FILTER_CUTOFF * DT + 1)

        # Torque output (torque sent to the motor)
        self.torque_input = 0.0

        # Encoder position and velocity
        self.encoder_data = 0.0
        self.encoder_velocity = 0.0
        self._prev_encoder_data = -np.inf
        self._prev_encoder_time = time.perf_counter()
        self.filtered_encoder_velocity = 0.0
        self.alpha_enc = 2 * np.pi * ENC_VEL_CUTOFF * DT / (2 * np.pi * ENC_VEL_CUTOFF * DT + 1)

        # Bus to read FSR sensor data
        self.bus = smbus2.SMBus(I2C_BUS)

        # ADDED: CAN bus for motor communication
        try:
            self.can_bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
            self.logger.logger.info(f"CAN bus opened on {CAN_INTERFACE}")
        except Exception as e:
            self.logger.logger.error(f"Failed to open CAN bus: {e}")
            self.logger.logger.error("Run: sudo ip link set can0 up type can bitrate 1000000")
            self.can_bus = None

        self.logger.logger.info("Sensor stream channel opened.")
    
    # Function to read the sensors data
    def readSensors(self):

        # Reading FSR data
        self.toe_fsr = self.read_channel(self.bus, CFG_AIN0)
        self.heel_fsr = self.read_channel(self.bus, CFG_AIN1)

        # Passing the fsr data through a low pass filter
        self.lowPassFilter()

        # Reading encoder data
        self.readEncoder()

    # Function to filter the data using a low-pass filter (for FSR data)
    def lowPassFilter(self):
        # Simple low-pass filter using exponential moving average
        self.filtered_heel_fsr = self.alpha * self.heel_fsr + (1 - self.alpha) * self.filtered_heel_fsr 
        self.filtered_toe_fsr = self.alpha * self.toe_fsr + (1 - self.alpha) * self.filtered_toe_fsr 

    # ADDED: Send torque command to motor via CAN (MIT impedance mode, pure torque)
    def sendTorqueData(self):

        if self.can_bus is None:
            self.logger.logger.warning("CAN bus not available. Torque not sent.")
            return

        # Clamp torque to motor limits
        self.torque_input = ASSISTANCE_LEVEL * _clamp(self.torque_input, MIT_T_MIN, MIT_T_MAX)
        self.logger.logger.info(f"Sending torque command: {self.torque_input:.2f} Nm")
        # MIT mode with kp=0, kd=0, pos=0, vel=0 → pure feedforward torque
        kp = 0.0
        kd = 0.0
        pos = 0.0
        vel = 0.0

        kp_int = _float_to_uint(kp,     MIT_KP_MIN, MIT_KP_MAX, 12)
        kd_int = _float_to_uint(kd,     MIT_KD_MIN, MIT_KD_MAX, 12)
        p_int  = _float_to_uint(pos,    MIT_P_MIN,  MIT_P_MAX,  16)
        v_int  = _float_to_uint(vel,    MIT_V_MIN,  MIT_V_MAX,  12)
        t_int  = _float_to_uint(self.torque_input, MIT_T_MIN,  MIT_T_MAX,  12)

        buf = [0] * 8
        buf[0] =  kp_int >> 4
        buf[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8)
        buf[2] =  kd_int & 0xFF
        buf[3] =  p_int  >> 8
        buf[4] =  p_int  & 0xFF
        buf[5] =  v_int  >> 4
        buf[6] = ((v_int  & 0xF) << 4) | (t_int >> 8)
        buf[7] =  t_int  & 0xFF

        arb_id = (MODE_MIT << 8) | MOTOR_ID
        msg = can.Message(arbitration_id=arb_id, data=buf, is_extended_id=True)
        # self.can_bus.send(msg)
  
        # Reseting the torque value to zero
        self.torque_input = 0.0

    # ADDED: Stop motor safely (zero velocity command)
    def stopMotor(self):
        if self.can_bus is None:
            return

        # Send zero torque via MIT mode
        self.torque_input = 0.0
        self.sendTorqueData()
        self.logger.logger.info("Motor stopped (zero torque sent).")

    # ADDED: Shut down CAN bus cleanly
    def shutdown(self):
        self.stopMotor()
        if self.can_bus is not None:
            self.can_bus.shutdown()
            self.logger.logger.info("CAN bus shut down.")
        self.bus.close()
        self.logger.logger.info("I2C bus closed.")

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
    
    def readEncoder(self) -> None:
        """Read 12-bit filtered angle from AS5600, return degrees 0–360."""
        try:
            # Get encoder angle
            data = self.bus.read_i2c_block_data(AS5600_ADDR, AS5600_REG_ANGLE, 2)
            raw = ((data[0] & 0x0F) << 8) | data[1]
            self.encoder_data = raw * (360.0 / 4096.0) - ENCODER_OFFSET 

            now = time.perf_counter()
            dt = now - self._prev_encoder_time

            # Get encoder velocity
            if self._prev_encoder_data == -np.inf:
                self._prev_encoder_data = self.encoder_data 

            delta = self.encoder_data - self._prev_encoder_data
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0
            
            self.encoder_velocity = delta / dt

            # Update filtered encoder velocity
            self.filtered_encoder_velocity = self.alpha_enc * (delta / dt) + (1 - self.alpha_enc) * self.filtered_encoder_velocity
            self._prev_encoder_time = now
            self._prev_encoder_data = self.encoder_data

        except OSError:
            return None