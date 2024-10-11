"""
routines for incorporating sensehat into donkeycar
"""

import time
import logging
from sense_hat import SenseHat
import numpy as np

logger = logging.getLogger(__name__)

class SensehatController:
    '''
    Driver to operate sense hat
    '''

    def updateQuadratureColor(self):
        if self.quadAccel >= 1:
            # interpolate b/w central and max
            self.dispColor = self.color_central + self.color_iFactor_high * (self.quadAccel - 1)
        if self.quadAccel < 1:
            # interpolate b/w central and min
            self.dispColor = self.color_central + self.color_iFactor_low * (1 - self.quadAccel)

    def setLEDtoQuadColor(self):
        self.theHat.clear(self.dispColor.astype(int).tolist())

    def __init__(self):
        self.theHat = SenseHat()

        

        # perform initial readings for debug
        ### get accelerometer reading
        accel = self.theHat.get_accelerometer_raw()
        ### get gyro reading
        gyro = self.theHat.get_gyroscope_raw()

        self.accel = np.array([ accel['x'],
                                   accel['y'],
                                   accel['z']])
        self.newAccel = np.array([ accel['x'],
                                  accel['y'],
                                  accel['z']])
        self.quadAccel = np.sqrt(np.sum(np.power(self.accel, 2)))
        
        self.gyro = np.array([ gyro['x'],
                              gyro['y'],
                              gyro['z']])
        self.newGyro = np.array([ gyro['x'],
                              gyro['y'],
                              gyro['z']])
        self.deltaGyro = np.array([ gyro['x'],
                              gyro['y'],
                              gyro['z']])
  
        
        self.dispColor = np.array([255,255,191])

        self.color_central = np.array([255,255,191])
        self.color_max = np.array([215,25,28])
        self.color_min = np.array([26,150,65])
        # assuming scale is up to 2gs from accelerometers
        # that means max reading in quadrature is just under 3.5
        # min reading is 0
        # do a shitty interpolation along each RGB axis to pick color
        self.color_iFactor_high = (self.color_max - self.color_central)/2.5
        
        self.color_iFactor_low = (self.color_min - self.color_central)

        # display welcome message
        self.theHat.show_message("HI!!", scroll_speed = 0.05,
                                 text_colour = [255, 0, 0],
                                 back_colour = self.dispColor.astype(int).tolist())
        time.sleep(2)

        self.iterationCounter = 0

        self.isOn = True
        self.poll_delay = 0.01

        logger.info(f"SensehatController::init() startup completed")


    def shutdown(self):

        self.isOn = False

        # should clear LEDs
        self.theHat.clear()
        logger.info(f"SensehatController::shutdown() shutdown completed")

    def poll(self):
        # read the devices
        ### get accelerometer reading
        accel = self.theHat.get_accelerometer_raw()
        ### get gyro reading
        gyro = self.theHat.get_gyroscope_raw()

        # update values for accel, no pre-processing needed
        self.accel[0] = accel['x']
        self.accel[1] = accel['y']
        self.accel[2] = accel['x']

        # update values for gyro
        # in this case, we want to compare to prev reading
        self.newGyro[0] = gyro['x']
        self.newGyro[1] = gyro['y']
        self.newGyro[2] = gyro['z']
        self.deltaGyro = self.newGyro - self.gyro
        self.gyro = self.newGyro

        # update internal record of quadrature accel
        self.quadAccel = np.sqrt(np.sum(np.power(self.accel, 2)))

    def run(self):
        
        self.poll()
        
        # set internal value of color to use
        self.updateQuadratureColor()

        # actually draw it to the LEDs
        self.setLEDtoQuadColor()
        
        # return IMU-like values
        return {"accel": self.accel,
                "gyro": self.gyro,
                "gyroDelta": self.deltaGyro}

    # THREADED IMPLEMENTATION

    def update(self):
        logger.info(f"SensehatController::update() threaded update function called")
        while self.isOn:
            if self.iterationCounter % 1000 == 0:
                logger.info(f"SensehatController::update() threaded update function loop ping")
            
            self.poll()

            if self.iterationCounter % 1000 == 0:
                logger.info(f"SensehatController::update() threaded update function loop ping, poll called")
            self.iterationCounter += 1
            time.sleep(self.poll_delay)

        logger.info(f"SensehatController::update() threaded update function exiting")
    
    def run_threaded(self):
        # set internal value of color to use
        self.updateQuadratureColor()

        # actually draw it to the LEDs
        self.setLEDtoQuadColor()

        # return IMU-like values
        return {"accel": self.accel,
                "gyro": self.gyro,
                "gyroDelta": self.deltaGyro}
