from Controllers.ControllerABC import Controller
import keyboard
import numpy as np
import logging


class KeyboardController(Controller):
    def set_velocity(self):
        if keyboard.is_pressed('up'):
            self.ego_velocity += 1
        elif keyboard.is_pressed('down'):
            self.ego_velocity -= 1
        self.ego_velocity = np.clip(self.ego_velocity, 0, 255)

    def set_steering_angle(self):
        incr = 1
        if keyboard.is_pressed('right'):
            self.ego_steering_angle += incr
        elif keyboard.is_pressed('left'):
            self.ego_steering_angle -= incr
        logging.debug("Steering Angle is {}".format(self.ego_steering_angle))



