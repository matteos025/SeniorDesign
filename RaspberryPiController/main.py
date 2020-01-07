import threading
from RaspberryPiController.CONSTANTS import *
from RaspberryPiController.NetworkCommunicator import NetworkReader, NetworkPublisher
import warnings
import time
import cv2
from threading import Lock
import numpy as np


########################################################################################################################
# START Vehicle Control Thread THREAD
########################################################################################################################
class VehicleControlThread(threading.Thread):
    """
    The velocity control thread. This thread allows for communication between the Arduino and the Pi
    """
    def run(self):
        while True:
            pass


########################################################################################################################
# END Vehicle Control Thread THREAD
########################################################################################################################

########################################################################################################################
# START NETWORK PUBLISHER THREAD
########################################################################################################################
class NetworkPublisherThread(threading.Thread):
    """
    Network Publisher Thread for UDP Communication To Other Cars
    """
    def __init__(self, ip_to_publish = DEFAULT_IP):
        super().__init__()
        self.network_publisher = NetworkPublisher(ip_to_publish)

    def run(self):
        while True:
            # TODO make this send useful messages
            self.network_publisher.send_message()
            time.sleep(0.1)
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
    def __init__(self, ip_to_read = DEFAULT_IP):
        super().__init__()
        self.ip_to_read = ip_to_read
        self.network_reader = NetworkReader(ip_to_read)

    def run(self):
        self.network_reader.run()
########################################################################################################################
# END NETWORK READER THREAD
########################################################################################################################



if __name__ == "__main__":
    print("running main")
    # You must run this in order to control the car
    comThread = ArduinoComThread()
    comThread.setDaemon(True)
    comThread.start()

    # PROJECT 1 FOLLOW THE CAR
    # ribThread = RibbonTrackThread()
    # ribThread.setDaemon(True)
    # ribThread.start()

    # PROJECT 2: NAVIGATE IN THE PARK
    # mapThread = MapThread()
    # mapThread.setDaemon(True)
    # mapThread.start()
    # camThread = CameraThread()
    # camThread.setDaemon(True)
    # camThread.start()

    # keep main thread alive
    while True:
        pass
