// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  String full_datastring = "";
  
  while (Wire.available()) { // loops through all the bytes that are sent over the wire
    // read in the information on the wire one byte at a time.
    // c can be set to any one byte primitive type such as char, int, byte. This type will cause c to be treated differently
    char c = Wire.read(); 
    full_datastring = full_datastring + c;
  }
  // extract the command byte based on the command byte have different behavior
  byte command = full_datastring.charAt(0);

  // read in a string and interpret data as string
//  if(command == STRING_COMMAND)
//  {
//    Serial.println("received data string");
//    String data = full_datastring.substring(1);
//    Serial.println(data);
//  }

  // called before the Pi reads from a register in send register. Updates which register the
  // Pi will read from.
  if(command == UPDATE_SEND_REGISTER){
    int data = full_datastring.substring(1).toInt();
    current_send_register = data;
    //Serial.println("updating send register");
    //Serial.println(current_send_register);
  }


  // If the command is inside the default write register range then write to the write register
  if(command >= 0 && command <= RECEIVE_REGISTER_SIZE)
  {
    // received a float and therefore write to a register
    //Serial.println("Received Data Float. Writing to register " + String(command));
    float data = full_datastring.substring(1).toFloat();
    //Serial.println(data);
    receive_registers[command] = data;
  }
   
  // Take care not to continue calling Wire.read() if there is nothing on the wire as you will get nonsense typically
}
