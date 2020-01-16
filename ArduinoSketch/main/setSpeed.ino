float setSpeed(float val) {
  //
  //Return the the value of the speed that we wish to drive at
  //
  byte motorPWM = constrain(val,0,250);
  analogWrite(motorPin, motorPWM);
  return motorPWM;


  //Legacy code from ESE 421
  /*
  getPingDistanceCM(); //get distance to furthest object
  int Kspeed = 5;
  int Kint = 0.5;
  float deltaTms = 20*0.001; //dt
  
  float error = pingDistanceCM - 30;
  float intError = error*deltaTms;
  byte val = Kspeed*constrain(error+intError,0,255);
  Serial.println(val);
  motorPWM = constrain(val,0,250);
  analogWrite(motorPin, motorPWM);
  return 0.002069*val+.2824 //motor parameters
  */
}
