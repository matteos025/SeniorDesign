float setSteeringAngle(float angle)
{
  float bound = 30;
  float servoCommand = angleToServoCommand(angle);
  servoCommand = constrain(servoCommand, -SERVOANGLENEUT-bound, SERVOANGLENEUT+bound);
  steeringServo.write(servoCommand);
  Serial.print("Steering Angle is ");
  Serial.println(angle);
  return angle;
}

float angleToServoCommand(float angle)
{
  return SERVOANGLENEUT-angle;
}
