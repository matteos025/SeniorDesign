import RaspberryPiController.NetworkCommunicator as NetworkCommunicator
import threading
import time

publisher = NetworkCommunicator.NetworkPublisher()
reader = NetworkCommunicator.NetworkReader()

class ReaderThread(threading.Thread):
    def run(self):
        reader.run()

readerThread = ReaderThread()
readerThread.setDaemon(True)
readerThread.start()

for i in range(0, 10):
    message = "Message number {}".format(i)
    publisher.send_message(message)
    time.sleep(0.1)



