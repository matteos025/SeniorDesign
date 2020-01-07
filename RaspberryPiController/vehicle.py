import threading
from CONSTANTS import *
from NetworkCommunicator import NetworkReader, NetworkPublisher
from Controllers import *
import warnings
import time
import cv2
from threading import Lock
import numpy as np
import logging

########################################################################################################################
# START VEHICLE OBJECT
########################################################################################################################
class Vehicle(object):
    def __init__(self, controller_type, **kwargs):
        #network publisher
        start = time.time()
        ip_to_publish = kwargs['ip_to_publish'] if 'ip_to_publish' in kwargs else DEFAULT_IP
        self.network_publisher = NetworkPublisher(ip_to_publish)
        self.network_publisher_thread = NetworkPublisherThread(self.network_publisher)
        # self.network_publisher_thread.start()
        end = time.time()
        logging.debug("Network Publisher Launched in {}s".format(end-start))

        #network reader
        start = time.time()
        ip_to_read = kwargs['ip_to_read'] if 'ip_to_read' in kwargs else DEFAULT_IP
        self.network_reader = NetworkReader(ip_to_read)
        self.network_reader_thread = NetworkReaderThread(self.network_reader)
        # self.network_reader_thread.start()
        end = time.time()
        logging.debug("Network Reader Launched in {}s".format(end - start))

        #vehicle controller
        start = time.time()
        controller_constructor = CONTROL_SCHEME_DICT[controller_type]
        steering_angle = kwargs['steering_angle'] if 'steering_angle' in kwargs else 0
        velocity = kwargs['velocity'] if 'velocity' in kwargs else 0
        self.controller = controller_constructor(steering_angle, velocity, communicate_to_arduino = False)
        self.control_thread = ControlThread(self.controller)
        self.control_thread.run()
        end = time.time()
        logging.debug("Controller Launched in {}s".format(end - start))


########################################################################################################################
# END VEHICLE OBJECT
########################################################################################################################



########################################################################################################################
# START NETWORK PUBLISHER THREAD
########################################################################################################################
class NetworkPublisherThread(threading.Thread):
    """
    Network Publisher Thread for UDP Communication To Other Cars
    """
    def __init__(self, network_publisher):
        super().__init__(name = 'network_publisher')
        self.network_publisher = network_publisher
        self.setDaemon(True)

    def run(self):
        ctr = 0
        while True:
            # TODO make this send useful messages
            self.network_publisher.send_message()
            time.sleep(0.1)
            ctr += 1
########################################################################################################################
# END NETWORK PUBLISHER THREAD
########################################################################################################################

########################################################################################################################
# START NETWORK READER THREAD
########################################################################################################################
class NetworkReaderThread(threading.Thread):
    """
    Network Reader Thread for UDP Communication To Other Cars
    """
    def __init__(self, network_reader):
        super().__init__(name = 'network_reader')
        self.network_reader = network_reader
        self.setDaemon(True)

    def run(self):
        self.network_reader.run()
########################################################################################################################
# END NETWORK READER THREAD
########################################################################################################################

########################################################################################################################
# START CONTROL THREAD
########################################################################################################################
class ControlThread(threading.Thread):
    """
    Network Reader Thread for UDP Communication To Other Cars
    """
    def __init__(self, controller):
        super().__init__(name = 'controller')
        self.setDaemon(True)
        self.controller = controller

    def run(self):
        self.controller.run()

########################################################################################################################
# END NETWORK READER THREAD
########################################################################################################################



if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    vehicle = Vehicle('keyboard')
