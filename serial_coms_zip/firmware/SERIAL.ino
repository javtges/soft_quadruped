/* This file contains methods for handeling the reception and transmission
 *  of serial data. It is intended for use in moderate reliability projects
 *  and should be modified before inclusion in a final build. 
 *  
 *  Reception: Data is assumed to be recieved in serial "frames" each frame
 *  should be in the format given below. Each time a new byte is poped 
 *  out of the incomming serial buffer the last n (n = frame length) bytes 
 *  are analyzed to determine if they comprise a valid frame. If so, the
 *  command and associated data are routed to the appropriate responce
 *  function. 
 *  
 *  [Start Byte][Data Bytes...][Checksum Byte (if implimented)][End Byte]
 *  
 *  Maintainer: Jake Ketchum
 *  Last Edited: 6/18/2019
 */


#define SERIAL_FRAME_LENGTH 20 //Start byte + end byte + command byte + data bytes (inc checksum). 
#define SERIAL_INDEX_START SERIAL_FRAME_LENGTH - 1
#define SERIAL_INDEX_COMMAND SERIAL_INDEX_START - 1
#define SERIAL_INDEX_DATA_START SERIAL_INDEX_COMMAND - 1
#define SERIAL_INDEX_DATA_END 2 //the last byte in the array to contain payload data. 
#define SERIAL_INDEX_CHECKSUM 1 //the byte containing the checksum if used. 

#define SERIAL_START        'A'
#define SERIAL_END          'Z'
#define SERIAL_PING         'a'
#define SERIAL_ECHO         'b'
#define SET_SERVOS          'c' // Set the pwm servo board outputs.
#define SET_LED             'd' // Set the onboard LED state

#define SERIAL_ID           "CRB"

void process_serial(){
  static byte bytes[SERIAL_FRAME_LENGTH]; //stores the incomming command data. 
  while(Serial.available()){
    byte incomming = Serial.read();

    for (int i = SERIAL_FRAME_LENGTH - 1; i > 0; i--){
      bytes[i] = bytes[i-1];
    }
    bytes[0] = incomming; 
    if (bytes[SERIAL_FRAME_LENGTH - 1] == SERIAL_START && bytes[0] == SERIAL_END){
      parse_command(bytes);
    }
  }
}

/*
 * Responcible for parsing command strings into actions. It is 
 * assumed that all provided command strins are valid. Invalid 
 * command bytes will result in the command being ignored. However,
 * there is no checking for reasonableness of paramiter values. 
 * 
 * command - byte array in the form [start][command][bytes...][stop]
 *           data is sent MSB first. 
 */
void parse_command(byte command[]){
  switch(command[SERIAL_INDEX_COMMAND]){
    case SERIAL_PING:
      //Reply with a pre-programed responce. 
      Serial.println(SERIAL_ID);
      break;
    case SERIAL_ECHO: 
      //Reply with exactly the payload that was sent. 
      for(int i = SERIAL_INDEX_DATA_START; i >=SERIAL_INDEX_DATA_END; i--){
        Serial.write(command[i]);
      }
      break;
    case SET_SERVOS:
      int angles[16];
      int b = 0;
      for(int i = SERIAL_INDEX_DATA_START; i >=SERIAL_INDEX_DATA_END; i--){
        angles[b] = command[i];
        b++;
      }
      update_servos(angles);

    case SET_LED:
      update_led(command[SERIAL_INDEX_DATA_START]);
      
    default: 
      //Likely an invalid command, prints command on serial. 
      Serial.print("default: ");
      Serial.write(command[SERIAL_INDEX_COMMAND]);
      Serial.println("");
      break;
  }
}
