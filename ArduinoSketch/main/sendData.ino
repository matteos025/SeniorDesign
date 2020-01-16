
// function that exectures whenever the master requests it. This function cannot have any parameters, and so
// you will need to take care in order to have the function write different types of information to the master
// one suggestion is to send a command from the master to the slave telling it what data you want and then having the Arduino write it
void sendData() {
  // send the data to the master
  char data[8];
  dtostrf(send_registers[current_send_register],8, 4, data);
  Serial.println(data);
  Wire.write(data);
 
}

void kalman_update() {
  Serial.println("doing kalman update");
}
