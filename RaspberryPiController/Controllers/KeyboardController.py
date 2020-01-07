from Controllers.ControllerABC import Controller
import keyboard
import numpy as np

class KeyboardController(Controller):
    def set_velocity(self):
        if keyboard.is_pressed('up'):
            self.ego_velocity += 1
        elif keyboard.is_pressed('down'):
            self.ego_velocity -= 1
        self.ego_velocity = np.clip(self.ego_velocity, 0, 255)

    def set_steering_angle(self):
        if keyboard.is_pressed('right'):
            self.ego_steering_angle += 1
        elif keyboard.is_pressed('left'):
            self.ego_steering_angle -= 1
        self.ego_steering_angle = np.clip(self.ego_steering_angle, -30, 30)