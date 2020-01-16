import threading
from CONSTANTS import *
from LOCAL_CONSTANTS import *
from NetworkCommunicator import NetworkReader, NetworkPublisher
# from Controllers import *
import Controllers.KeyboardController as KeyboardController
import Controllers.PlatoonController as PlatoonController
import warnings
import time
#import cv2
from threading import Lock
import numpy as np
import logging

########################################################################################################################
# START VEHICLE OBJECT
########################################################################################################################
#Control Scheme Constants
CONTROL_SCHEME_DICT = {'keyboard': KeyboardController.KeyboardController,
                       'platoon_follower': PlatoonController.PlatoonController}

class Vehicle(object):
    def __init__(self, controller_type, **kwargs):
        # vehicle controller
        controller_constructor = CONTROL_SCHEME_DICT[controller_type]
        steering_angle = kwargs['steering_angle'] if 'steering_angle' in kwargs else 0
        velocity = kwargs['velocity'] if 'velocity' in kwargs else 0
        self.controller = controller_constructor(steering_angle, velocity, communicate_to_arduino=COMMUNICATE_TO_ARDUINO)

        self.do_networking = kwargs['do_networking'] if 'do_networking' in kwargs else True

        #network publisher
        if self.do_networking:
            ip_to_publish = kwargs['ip_to_publish'] if 'ip_to_publish' in kwargs else DEFAULT_IP
            self.network_publisher = NetworkPublisher(ip_to_publish)
            self.network_publisher_thread = self._prepare_thread(target=self._network_publish_thread, name = 'network_publisher')

            #network reader
            ip_to_read = kwargs['ip_to_read'] if 'ip_to_read' in kwargs else DEFAULT_IP
            self.network_reader = NetworkReader(ip_to_read)
            self.network_read_thread = self._prepare_thread(target=self._network_read_thread, name = 'network_reader')


    @property
    def ego_velocity(self):
        return self.controller.ego_velocity

    @property
    def ego_steering_angle(self):
        return self.controller.ego_steering_angle

    @property
    def state_message_dictionary(self):
        message = {'velocity': str(self.ego_velocity), 'steering_angle': str(self.ego_steering_angle)}
        return message

    def _prepare_thread(self, target, name = None):
        t = threading.Thread(target=target, name = name, daemon=True)
        return t

    def _network_publish_thread(self):
        while True:
            self.network_publisher.send_dictionary(self.state_message_dictionary)
            time.sleep(0.5)

    def _network_read_thread(self):
        while True:
            logging.debug("reading")
            self.network_reader.read()
            time.sleep(0.5)

    def start(self):
        if self.do_networking:
            self.network_publisher_thread.start()
            self.network_read_thread.start()
        self.controller.run()
        while True:
            pass




########################################################################################################################
# END VEHICLE OBJECT
########################################################################################################################

if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
                        datefmt='%Y-%m-%d:%H:%M:%S',
                        level=logging.DEBUG)
    vehicle = Vehicle('keyboard', do_networking= DO_NETWORKING, ip_to_publish=PUBLISHING_IP, ip_to_read=READING_IP)
    # vehicle = Vehicle('keyboard', do_networking= True)
    vehicle.start()
