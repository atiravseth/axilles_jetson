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
            sensor_data.readSensors()

            if not calibration.calibrated:
                sensor_data.system_state = 1  # CALIBRATING
                calibration.calibrate()
                sensor_data.publishDashboard(False, False, 0.0)
            else:
                if sensor_data.motor_alive:
                    sensor_data.system_state = 2  # ACTIVE
                else:
                    sensor_data.system_state = 3  # E-STOP

                impedance_controller.checkLimits()
                activation.activate()

                # Get current state for dashboard
                heel_on = calibration.detectHeelGround()
                toe_off_on = calibration.detectToeOff()
                elapsed = TBElog.getCurrentTime() - controller.last_heel_strike_time
                phase = float(np.clip(elapsed / controller.stride_time, 0.0, 1.0)) if not np.isnan(controller.last_heel_strike_time) else 0.0
                sensor_data.publishDashboard(heel_on, toe_off_on, phase)
                
            sleep_time = DT - elapsed
            # Print the values for elapsed and time left
            # TBElog.logger.info(f"THe time elapsed: {elapsed}, sleep time : {sleep_time}")
            # If the loop is running faster than the desired frequency, sleep for the remaining time
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        TBElog.logger.info("Shutting down Controller...")
        sensor_data.shutdown()          
        TBElog.logger.info("Exiting.")

     
if __name__ == "__main__":
    main()