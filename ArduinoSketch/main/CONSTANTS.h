//Communication Constants
const int RECEIVE_REGISTER_SIZE = 8;
float receive_registers[RECEIVE_REGISTER_SIZE];

const int SEND_REGISTER_SIZE = 8;
float send_registers[SEND_REGISTER_SIZE];


int UPDATE_SEND_REGISTER = 11;
int current_send_register = 3;

//Hardware Constants
float SERVOANGLENEUT = 60;
float gyroZbias = 0.8; //degrees per second


//PYTHON GLOBALS THESE MUST MATCH CONSTANTS.py
const int VELOCITY_REGISTER = 1;
const int STEERING_ANGLE_REGISTER = 2;
