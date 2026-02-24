import numpy as np
from controller import TBEController, TBECalibration, TBEActivation
from data_obtainer import SensorData
from utilities import *

def main():

    # Start logging
    TBElog = Logger()

    TBElog.logger.info("Opening sensor channels...")
    # Create an instance of the SensorData class to read the sensor data
    sensor_data = SensorData(TBElog)

    TBElog.logger.info("Instantiating TBE Controller...")
    # Create an instance of the TBEController
    controller = TBEController(TBElog)

    TBElog.logger.info("Instantiating TBE Calibration...")
    # Create an instance of the TBECalibration class to calibrate the thresholds for heel strike and toe off detection
    calibration = TBECalibration(controller, sensor_data, TBElog)

    TBElog.logger.info("Instantiating TBE Activation...")
    # Create an instance of the TBEActivation class to activate
    activation = TBEActivation(controller, sensor_data, calibration, TBElog)

    # Start the main loop
    try:
        while True:

            loop_start = TBElog.getCurrentTime()

            # Read the sensor data
            sensor_data.readSensors()

            # If not calibrated, run calibration
            if not calibration.calibrated:
                calibration.calibrate()
            else:
                # If calibrated, run activation
                activation.activate()
            
            elapsed = TBElog.getCurrentTime()- loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        TBElog.logger.info("Shutting down Controller...")
        sensor_data.sendTorqueData(0.0)           
        TBElog.logger.info("Motor Torque set to zero. Exiting.")

     
if __name__ == "__main__":
    main()