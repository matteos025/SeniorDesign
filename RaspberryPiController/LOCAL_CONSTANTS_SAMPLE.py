#LOCAL CONSTANTS ONLY
PUBLISHING_IP = '192.168.0.165' #This should be the IP of the machine we are sending data to
READING_IP = '192.168.0.100' #This should be the local machine's IP

CONTROL_SCHEME = 'keyboard' #see vehicle class for available control types (can't put that dictionary in here because of import loops

COMMUNICATE_TO_ARDUINO = False #Whether the Pi communicates to the Arduino. Set to False on your computer
DO_CONTROL = False #Whether the Pi attempts to do control. Useful when debugging network
DO_NETWORKING = True #Whether the Pi should take over the network at all

