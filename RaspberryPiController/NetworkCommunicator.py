import socket
from CONSTANTS import *

class NetworkPublisher(object):
    def __init__(self, ip_to_publish = DEFAULT_IP):
        self.UDP_IP_RECIPIENT = ip_to_publish
        self.UDP_PORT_RECIPIENT = 5005
        self.socket = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
    def send_message(self, message = "Hello, World!"):
        # print("message:", message)
        self.socket.sendto(message.encode(ENCODING), (self.UDP_IP_RECIPIENT, self.UDP_PORT_RECIPIENT))

class NetworkReader(object):
    def __init__(self, ip_to_read = None):
        self.UDP_IP = DEFAULT_IP if ip_to_read is None else ip_to_read
        self.UDP_PORT = 5005

        self.socket = socket.socket(socket.AF_INET,  # Internet
                             socket.SOCK_DGRAM)  # UDP
        self.socket.bind((self.UDP_IP, self.UDP_PORT))

    def run(self):
        # TODO make this actually do more than print the message
        while True:
            data, addr = self.socket.recvfrom(1024)  # buffer size is 1024 bytes
            print("received message {} from address {}\n".format(data.decode(ENCODING), addr))



