#include <Servo.h>
#include "CONSTANTS.h"
//#include </.h>
//#include <Adafruit_Sensor/.h>
#include <Wire.h>


#define servoPin 7 // pin for servo signalj

#define motorPin 8 // PWM for motor


//ping sensor define
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

//IMU sensor define
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega


//
// tell sensor library which pins for accel & gyro data
//
//Adafruit_LSM9DS1 ls/m = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
Servo steeringServo; //servo name variable

// This is the code that is run when first downloaded to the Arduino. It is called once and sets up many
// important environments and variables for the Arduino
void setup() {
  //Initiate Serial communication. It's good practice to always do this in Arduino
  Serial.begin(9600);

  // join i2c bus with address #8
  Wire.begin(0x8);     

  // tell the slave device what it should do when the master sends it data
  // receiveEvent needs to be a method with a void return type and a single parameter
  // of type int that indicates how many bytes are being written (this includes the command byte)        
  Wire.onReceive(receiveEvent); 

  // tell the slave device what it should do when the master asks for data. Since the master won't send
  // a command saying what data it wants, you will need to get creative in order to send the master
  // different kinds of data
  Wire.onRequest(sendData);  
  //servo setup
  steeringServo.attach(servoPin); //initalize servo to pin
  steeringServo.write(SERVOANGLENEUT);//set servo to straight line

  //initialize empty registers
  for(int i = 0; i < SEND_REGISTER_SIZE; i++)
  {
    send_registers[i] = 0;
  }
  for(int i = 0; i < RECEIVE_REGISTER_SIZE; i++)
  {
    receive_registers[i] = 0;
  }
}


// This function is looped continuously. 
void loop() {
  unsigned long time = millis();
  setSpeed(receive_registers[VELOCITY_REGISTER]);
  setSteeringAngle(receive_registers[STEERING_ANGLE_REGISTER]);

  int DELAY = 20; //amount we wish to delay in miliseconds
  while(millis() <= time + DELAY){
        
  }
}
