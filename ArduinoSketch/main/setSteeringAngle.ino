float setSteeringAngle(float val)
{
  static int ctr = 0;
  float bound = 25;
  float servoAngleDeg = constrain(SERVOANGLENEUT - val, -SERVOANGLENEUT-bound, SERVOANGLENEUT+bound);
  steeringServo.write(servoAngleDeg);
  if(ctr % 100000 == 0)
  {
    Serial.print("Steering Angle is ");
    Serial.println(servoAngleDeg);
  }
  ctr++;
  return servoAngleDeg;
}
