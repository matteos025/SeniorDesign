from smbus import SMBus
import traceback


class ArduinoCommunicator:
    def __init__(self, arduino_address = 0x8):
        self.address = arduino_address
        self.bus = SMBus(1)
        self.write_attempt = 0
        self.data_from_arduino = dict.fromkeys(["IMU data"])
        self.UPDATE_SEND_REGISTER = 11

        self.data_to_arduino_register = {
            "velocity": 1,
            "steering_angle": 2
        }

        self.SIZE_OF_ARDUINO_SEND_REGISTERS = 8

    def write_float_to_register(self, num, register):
        """
        Writes a float into the Arduino's receive register
        """

        if register < 0 or register > 7:
            raise Exception("Error in write float: register must be in [0,7]")
        data = list(bytearray(str(num), 'utf8'))
        self._write(register, data)

    def _write(self, register, data):
        timeout = 10
        try:
            self.bus.write_i2c_block_data(self.address, register, data)
            self.write_attempt = 0
        except Exception as e:
            self.write_attempt += 1
            if self.write_attempt < timeout:
                print("Failed to write due to Exception " + str(e) + ". Trying again")
                self._write(register, data)
            else:
                print("Timed out writing")
                traceback.print_exc()

