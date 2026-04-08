from controller import TBEController, TBECalibration, TBEActivation, TBEImpedanceController
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
    # Create an instance of the TBEActivation class to activate the controller
    activation = TBEActivation(controller, sensor_data, calibration, TBElog)

    # Create an instance of the TBEImpedanceController class to run impedance control
    impedance_controller = TBEImpedanceController(controller, sensor_data, TBElog)
    TBElog.logger.info("Instantiating TBE Impedance Controller...")

    TBElog.logger.info(f"Starting main loop at {MOTOR_CONTROL_FREQ} Hz...")

    # Start the main loop
    try:
        while True:

            loop_start = time.perf_counter()

            # Read the sensor data
            sensor_data.readSensors()

            # If not calibrated, run calibration
            if not calibration.calibrated:
                calibration.calibrate()
            else:

                # Run impedance control
                # impedance_controller.checkLimits()
                # If calibrated, run activation
                activation.activate()
                
            
            elapsed = time.perf_counter() - loop_start
            sleep_time = DT - elapsed

            # To simply print the remaining time for debugging - REMOVE LATER
            TBElog.logger.info(f"Loop time: {elapsed:.4f} s, Sleep time: {sleep_time:.4f} s")
            
            # If the loop is running faster than the desired frequency, sleep for the remaining time
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        TBElog.logger.info("Shutting down Controller...")
        sensor_data.shutdown()          
        TBElog.logger.info("Exiting.")

     
if __name__ == "__main__":
    main()