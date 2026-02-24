# file for all the parameters and utility functions of the TBE controller
import logging
import time

# Creating a logger class for controller
class Logger():

    def __init__(self):
        self.logger = logging.getLogger("TBEController")
        self._init_time = time.perf_counter()
        self.logger.handlers.clear()    # prevent duplicate handlers on re-init
        self.setFormat()

        self.logger.info("Logger initialized.")

    def getCurrentTime(self):
        return time.perf_counter() - self._init_time

    def _get_timestamp(self):
        elapsed_time = self.getCurrentTime()
        minutes = int(elapsed_time // 60)
        seconds = int(elapsed_time % 60)
        milliseconds = int((elapsed_time - int(elapsed_time)) * 1000)
        return f"{minutes:02d}:{seconds:02d}.{milliseconds:03d}"

    def setFormat(self):

        # Custom formatter that uses our elapsed time
        logger_ref = self  # capture reference for the inner class

        class ElapsedFormatter(logging.Formatter):
            def format(self, record):
                timestamp = logger_ref._get_timestamp()
                level = record.levelname.ljust(4)
                name = record.name
                msg = record.getMessage()
                return f"[{level}][{timestamp}][{name}]: {msg}"

        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        console.setFormatter(ElapsedFormatter())
        self.logger.addHandler(console)
        self.logger.setLevel(logging.DEBUG)


# The number of strides over which TBE takes average
NUM_STRIDES = 5
ADAPTIVE_STRIDE_WINDOW = 2

# THresholds for heel strike and toe off detection
HEEL_STRIKE_THRESHOLD = 0.5
TOE_OFF_THRESHOLD = 0.5

# ── Hardware ──────────────────────────────────────────────────────────────────
I2C_BUS      = 7
ADC_ADDR     = 0x48   # ADS1115 default address (ADDR pin → GND)

# ── ADS1115 Registers ─────────────────────────────────────────────────────────
REG_CONVERSION = 0x00
REG_CONFIG     = 0x01

# ── Config register templates (high byte, low byte) ──────────────────────────
# High byte:  OS=1 | MUX[2:0] | PGA=001 (±4.096 V) | MODE=1 (single‑shot)
# Low byte:   DR=100 (128 SPS) | COMP_MODE=0 | COMP_POL=0 | COMP_LAT=0 | COMP_QUE=11 (disabled)
_CFG_LO  = 0x83   # 128 SPS, comparator disabled

CFG_AIN0 = [0xC3, _CFG_LO]   # OS=1, MUX=100 (AIN0 vs GND)
CFG_AIN1 = [0xD3, _CFG_LO]   # OS=1, MUX=101 (AIN1 vs GND)

# ── Scale ─────────────────────────────────────────────────────────────────────
VOLTS_PER_COUNT = 4.096 / 32767   # ~0.125 mV per count

# ── Sample interval ───────────────────────────────────────────────────────────
MOTOR_CONTROL_FREQ = 200
CONV_DELAY = 1 / 128 + 0.002   # >1 conversion period @ 128 SPS + margin

# Duty time of motor control loop
DT = 1.0 / MOTOR_CONTROL_FREQ

# cut off frequency for low-pass fsr filtering (Hz)
FSR_FILTER_CUTOFF = 10.0