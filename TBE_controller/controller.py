# Importing libraries
import time
import numpy as np
from typing import Optional
from scipy.interpolate import PchipInterpolator

from utilities import *
from data_obtainer import SensorData


# Creating a controller class
class TBEController:

    def __init__(self, logger: Logger):

        # median stride time
        self.stride_time = float('nan')

        # time params
        self._init_time = time.perf_counter()   

        # Torque profile
        self.torque_profile: Optional[PchipInterpolator] = None

        # Logger for logging controller data
        self.logger = logger

        # Track last heel strike time for phase calculation in TBEActivation
        self.last_heel_strike_time = float('nan')

        self.logger.logger.info("TBE Controller initialized.") 
    
# Creating the calibration class for calibrating the thresholds for heel strike and toe off detection
class TBECalibration():

    def __init__(self, controller: TBEController, data: SensorData, logger: Logger):
        self.controller = controller
        self.data = data
        self.logger = logger

        # heel strike and toe off time points
        self.heel_strike_times = np.zeros(NUM_STRIDES)   
        self.toe_off_times = np.zeros(NUM_STRIDES)       

        # Threshold for detecting heel strike and toe off
        self.heel_strike_threshold = HEEL_STRIKE_THRESHOLD
        self.toe_off_threshold = TOE_OFF_THRESHOLD

        # To store previous states
        self._prev_heel_state = False
        self._prev_toe_state = False

        # Torque profile parameters
        self.current_torque_profile_points = []
        self.torque_profile_points = []
        self.stride_saved = False

        # TO check whether calibration is done or not
        self.calibrated = False

        # Checking phase of calibration
        self._stride_times_collected = False

        # start time of current stride for torque profile collection
        self._torque_stride_start_time = None

    # Detect heel strike from heel FSR rising above threshold
    def detectHeelGround(self) -> bool:
        return self.data.filtered_heel_fsr > self.heel_strike_threshold

    # Detect toe off from toe FSR falling below threshold
    def detectToeOff(self) -> bool:
        return self.data.filtered_toe_fsr < self.toe_off_threshold

    # Checks rising threshold of heel strike
    def detectHeelStrikeEdge(self) -> bool:
        current = self.detectHeelGround()
        rising_edge = current and not self._prev_heel_state
        self._prev_heel_state = current
        return rising_edge

    # Checks falling threshold of toe off
    def detectToeOffEdge(self) -> bool:
        current = self.detectToeOff()
        falling_edge = current and not self._prev_toe_state
        self._prev_toe_state = current
        return falling_edge

    # Detect the phase of the gait cycle based on heel strike and toe off detections
    def detectStancePhase(self) -> bool:
        if self.detectHeelGround() or not self.detectToeOff():
            return True
        
        return False
    
    # Calculate the median stride time from the stride time array
    def getStanceTime(self) -> None:

        # Detect transitions
        heel_strike_edge = self.detectHeelStrikeEdge()
        toe_off_edge = self.detectToeOffEdge()

        # Record heel strike timestamp on rising edge, along with no noise check
        if heel_strike_edge:
            self.logger.logger.info("Heel strike detected. Entering stance phase.")
            self.heel_strike_times = np.roll(self.heel_strike_times, -1)
            self.heel_strike_times[-1] = self.logger.getCurrentTime()
        
        # Record toe off timestamp on falling edge, along with no noise check
        if toe_off_edge:
            self.logger.logger.info("Toe off detected. Entering swing phase.")
            self.toe_off_times = np.roll(self.toe_off_times, -1)
            self.toe_off_times[-1] = self.logger.getCurrentTime()
       
        # Once we have NUM_STRIDES worth of timestamps, compute stride time
        if (self.heel_strike_times[0] > 0) and (self.toe_off_times[0] > 0):
            self.controller.stride_time = float(np.median(self.toe_off_times - self.heel_strike_times))
            self._stride_times_collected = True    
            self.logger.logger.info(f"Median stride time calculated: {self.controller.stride_time:.2f} seconds.")                                                

    
    # Calculate the torque profile based on the stance phase percent and the stride time
    def calculateTorqueProfile(self) -> None:

        common_grid = np.linspace(0, 1, 100)    
        resampled = []

        for stride in self.torque_profile_points:
            original_phases = np.linspace(0, 1, len(stride))                  
            resampled.append(np.interp(common_grid, original_phases, stride))

        mean_torque = np.mean(resampled, axis=0)   

        self.controller.torque_profile = PchipInterpolator(common_grid, mean_torque) 
    
    # Getting the torque profile points for each stride
    def getTorqueProfilePoints(self) -> None:
        
        # On new stride, reset timer and list
        if self._torque_stride_start_time is None:
            if self.detectHeelStrikeEdge():
                self._torque_stride_start_time = self.logger.getCurrentTime()
                self.current_torque_profile_points = []
                self.stride_saved = False
            return  

        elapsed = self.logger.getCurrentTime() - self._torque_stride_start_time

        # If less time than stride, keep updating into list
        if elapsed < self.controller.stride_time:    
            self.current_torque_profile_points.append(self.data.torque_input)
            self.stride_saved = False   

        # Once exceeded stride time, save list and reset for next stride
        elif not self.stride_saved:
            self.torque_profile_points.append(self.current_torque_profile_points)
            self.current_torque_profile_points = []
            self.stride_saved = True
            self._torque_stride_start_time = None    

    # The actual calibration process
    def calibrate(self) -> None:

        # Phase 1: Collect stride times
        if not self._stride_times_collected:
            self.getStanceTime()
            return                          

        # Phase 2: Collect torque profiles for NUM_STRIDES
        if len(self.torque_profile_points) < NUM_STRIDES:
            self.getTorqueProfilePoints()
            return                          
        
        # Phase 3: Build the torque profile interpolator
        if self.controller.torque_profile is None:
            self.calculateTorqueProfile()
            self.calibrated = True           


# Class for activating TBE controller
class TBEActivation():

    def __init__(self, controller: TBEController, data: SensorData, calibration: TBECalibration, logger: Logger):
        self.controller = controller
        self.data = data
        self.calibration = calibration
        self.logger = logger

        # TO store the recent strides for adaptive stride time calculation
        self._recent_strides = []

    # Function to provide torque output based on the current time and the torque profile
    def giveTorqueOutput(self, phase: float) -> None:
        if self.controller.torque_profile is not None:
            self.data.sendTorqueData(float(self.controller.torque_profile(phase)))

    def activate(self) -> None:

        # Detect heel strike to reset phase reference
        if self.calibration.detectHeelStrikeEdge():
            current_time = self.logger.getCurrentTime()

            # Compute stride duration from previous heel strike before overwriting
            if not np.isnan(self.controller.last_heel_strike_time):       
                latest_stride = current_time - self.controller.last_heel_strike_time
                self._recent_strides.append(latest_stride)
                if len(self._recent_strides) > ADAPTIVE_STRIDE_WINDOW:
                    self._recent_strides.pop(0)
                self.controller.stride_time = float(np.median(self._recent_strides))

            self.controller.last_heel_strike_time = current_time         

        # Check for heel strike reference and stride time to compute phase
        if np.isnan(self.controller.last_heel_strike_time):
            return
        if np.isnan(self.controller.stride_time):
            return

        elapsed = self.logger.getCurrentTime() - self.controller.last_heel_strike_time
        phase = elapsed / self.controller.stride_time
        phase = float(np.clip(phase, 0.0, 1.0))        

        # Command torque during stance 
        if phase < 1.0:
            self.giveTorqueOutput(phase)
        else:
            self.data.sendTorqueData(0.0)     