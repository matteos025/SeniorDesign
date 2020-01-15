import socket
from CONSTANTS import *
import logging
import time
import json

class NetworkPublisher(object):
    def __init__(self, ip_to_publish = DEFAULT_IP):
        self.UDP_IP_RECIPIENT = ip_to_publish
        self.UDP_PORT_RECIPIENT = 5005
        self.socket = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
    def send_dictionary(self, message = {'velocity': 0, 'steering_angle': 0}):
        message_string = json.dumps(message)
        self.send_string(message_string)

    def send_string(self, message ="Hello, World!"):
        self.socket.sendto(message.encode(ENCODING), (self.UDP_IP_RECIPIENT, self.UDP_PORT_RECIPIENT))

    def run(self, message_dictionary):
        #TODO fill in to what we want
        ctr = 0
        while True:
            message = {'velocity': ctr, 'steering_angle': ctr}
            self.send_dictionary(message_dictionary)
            # time.sleep(0.1)
            ctr += 1

class NetworkReader(object):
    def __init__(self, ip_to_read = None):
        self.UDP_IP = DEFAULT_IP if ip_to_read is None else ip_to_read
        self.UDP_PORT = 5005

        self.socket = socket.socket(socket.AF_INET,  # Internet
                             socket.SOCK_DGRAM)  # UDP
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.UDP_IP, self.UDP_PORT))

    def run(self):
        # TODO make this actually do more than print the message
        while True:
            self.read()
            time.sleep(0.5)

    def read(self):
        try:
            data, addr = self.socket.recvfrom(1024)  # buffer size is 1024 bytes
            logging.debug("received message {} from address {}\n".format(data.decode(ENCODING), addr))
        except:
            self.socket.shutdown()
            self.socket.close()



