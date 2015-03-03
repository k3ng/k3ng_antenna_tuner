
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "k3ng_rig_control.h"


uint8_t _rigtype;
SoftwareSerial *serialport;
uint8_t rig_status = PORT_CLOSED;
uint8_t receive_buffer_bytes = 0;
unsigned long receive_buffer_last_rx_time;
uint8_t receive_buffer[RIG_RECEIVE_BUFFER_BYTES];
uint8_t timeouts = 0;
uint8_t mode;
unsigned int frequency = 0;
uint8_t command_submitted = NONE;
unsigned long last_action_time = 0; 
unsigned int _baud;


//-----------------------------------------------------------------------------------------------------

Rig::Rig(SoftwareSerial *serialportin, uint8_t rigtype) {

  _rigtype = rigtype;
  serialport = serialportin;


}
//-----------------------------------------------------------------------------------------------------

void Rig::remove_rig_receive_buffer_byte(){
 
  if (receive_buffer_bytes) {
    receive_buffer_bytes--;
    for (int x = 0;x < receive_buffer_bytes;x++) {
      receive_buffer[x] = receive_buffer[x+1];
    }
  }
  
}
//-----------------------------------------------------------------------------------------------------


uint8_t Rig::bcd_low_value(uint8_t byte_in){

  return(byte_in & B00001111); 
  
}

//-----------------------------------------------------------------------------------------------------

uint8_t Rig::bcd_high_value(uint8_t byte_in){

  byte_in = byte_in >> 4;
  return(byte_in & B00001111); 
  
}


//-----------------------------------------------------------------------------------------------------

void Rig::initialize(int baud,uint8_t connection_type){

  serialport->begin(baud);
  rig_status = PORT_OPEN;
  _baud = baud;
  #ifdef DEBUG_RIG
  Serial.print(F("Rig::initialize: PORT_OPEN"));
  #endif //DEBUG_RIG

}

//-----------------------------------------------------------------------------------------------------
void Rig::write_byte(uint8_t byte_to_write){

  serialport->write(byte_to_write);
  #ifdef DEBUG_RIG
  Serial.print(F("Rig::write: "));
  Serial.println(byte_to_write,HEX);
  #endif //DEBUG_RIG

}


//-----------------------------------------------------------------------------------------------------

void Rig::write_bytes(uint8_t byte_to_write,uint8_t number_of_bytes){

  for (uint8_t x = 0;x < number_of_bytes;x++){
    write_byte(byte_to_write);
  }


}



//-----------------------------------------------------------------------------------------------------

void Rig::service(){


  // service the radio port and do housekeeping
  
  // ------  check for incoming rig serial port bytes and puts them in each rig receive buffer array
  
  uint8_t incoming_byte = 0;
  
  if ( (rig_status == PORT_OPEN) && (serialport->available()) ){     
    if (command_submitted == CONTROL_PASS_THROUGH) { // if we're in rig command pass through, don't buffer it just send it out serial port
      Serial.write(serialport->read());
      last_action_time = millis(); 
    } else {     
      incoming_byte = serialport->read();
      if (receive_buffer_bytes < RIG_RECEIVE_BUFFER_BYTES){
        receive_buffer[receive_buffer_bytes] = incoming_byte;
        receive_buffer_bytes++;
        receive_buffer_last_rx_time = millis();
        #ifdef DEBUG_RIG

          Serial.println(F("rig::service: rig: "));
          Serial.print(F("byte: "));
          Serial.print(incoming_byte,HEX);
          Serial.print(F(" rx_buffer_bytes: "));
          Serial.println(receive_buffer_bytes);

        #endif //DEBUG_RIG
      } else {
        #ifdef DEBUG_RIG

          Serial.println(F("rig::service: buffer full rig ")); 

        #endif //DEBUG_RIG 
      }
    }
  }
    
  if ((rig_status == PORT_CLOSED) && ((millis()-last_action_time) >= (RIG_PORT_RETRY_SECS*1000))){
    #ifdef DEBUG_RIG
    Serial.println(F("rig::service: retry rig port "));    
    #endif //DEBUG_RIG      
    serialport->begin(_baud);
    rig_status = PORT_OPEN;
  }    
        

 
  //  -------- service the rig receive buffer -------------

  uint8_t found_terminator = 0;


  if (receive_buffer_bytes > 0){
    switch(_rigtype){
      #ifdef YAESU_SUPPORT
      case YAESU:
        switch(command_submitted){
          case PTT: // Yaesu returns one byte for PTT command which we discard
            if (receive_buffer_bytes){
              timeouts = 0;
              remove_rig_receive_buffer_byte();
            }
            break; //PTT
          case FREQUENCY_REQUEST:  // Yaesu returns five bytes for a frequency request command and returns freq and mode
            if (receive_buffer_bytes == 5) {
              timeouts = 0;              
              /*frequency = ((bcd_low_value(receive_buffer[0]))*10000) + (bcd_high_value(receive_buffer[1])*1000) + 
                (bcd_low_value(receive_buffer[1])*100) + (bcd_high_value(receive_buffer[2])*10) + (bcd_low_value(receive_buffer[2]));*/

              frequency = ((receive_buffer[0]&B00001111)*10000) + ((receive_buffer[1]>>4)*1000) + 
                ((receive_buffer[1]&B00001111)*100) + ((receive_buffer[2]>>4)*10) + (receive_buffer[2]&B00001111);

                
              #ifdef DEBUG_RIG
              for (int x =0;x < 5;x++){
                Serial.print(F("rig::service: yaesu: frequency: byte: "));
                Serial.print(x);
                Serial.print(": ");
                Serial.println(receive_buffer[x],HEX);
              }
              Serial.print(F("rig::service: yaesu: frequency: "));
              Serial.println(frequency);
              #endif //DEBUG_RIG 

              switch (receive_buffer[4]&B00001111){
                case 0x00: mode = LSB; break; 
                case 0x01: mode = USB; break;
                case 0x02: mode = CW; break;
                case 0x03: mode = CWR; break;
                case 0x04: mode = AM; break;
                case 0x06: mode = WFM; break;
                case 0x08: mode = FM; break;
                case 0x0A: mode = DIG; break;
                case 0x0C: mode = PKT; break;
                default: mode = UNDEF; break;
              }
              command_submitted = NONE;
              receive_buffer_bytes = 0;
              last_action_time = millis();
            } //(receive_buffer_bytes == 5)
            break;  //FREQUENCY_REQUEST
        } //switch(command_submitted)
        // check for a timeout condition
        if ((receive_buffer_bytes < 5) && ((millis() - receive_buffer_last_rx_time) > RIG_RX_BUFFER_TIMEOUT_MS)) {
          receive_buffer_bytes = 0;
          timeouts++;
        }
        break; //YAESU
        #endif //YAESU_SUPPORT
        
      #ifdef KENWOOD_SUPPORT
      case KENWOOD:   
        if (receive_buffer_bytes){
          // if we have bytes in the buffer, see if we got a terminating ';'
          for (int y = 0;y < receive_buffer_bytes;y++) {
            if (receive_buffer[y] == ';') {found_terminator = 1;}
          }
          if (found_terminator){  //we have something to process
            switch(command_submitted){
              case FREQUENCY_REQUEST:
                timeouts = 0;              
                frequency = ((receive_buffer[5]*10000) + (receive_buffer[6]*1000) + 
                  (receive_buffer[7]*100) + (receive_buffer[8]*10) + receive_buffer[9]);                
                break; //FREQUENCY_REQUEST
            
            } // switch(command_submitted)
            command_submitted = NONE;
            receive_buffer_bytes = 0;
            last_action_time = millis();             
          }  //if (found_terminator)
          
          //check for a timeout condition
          if (((millis() - receive_buffer_last_rx_time) > RIG_RX_BUFFER_TIMEOUT_MS)) {
            receive_buffer_bytes = 0;
            timeouts++;
          }                
        } //if (receive_buffer_bytes)
        break; //KENWOOD
        #endif //KENWOOD_SUPPORT
    }
    
  }
    

  if ((rig_status == PORT_OPEN) && (command_submitted == CONTROL_PASS_THROUGH)  
  && ((millis() - last_action_time) >= RIG_CONTROL_PASS_THROUGH_CLEAR_MS)){ //clear control pass through command if there's been no activity for awhile
     command_submitted = NONE;
     receive_buffer_bytes = 0;
     last_action_time = millis();
  }
    

  if ((rig_status == PORT_OPEN) && (command_submitted != NONE) && ((millis() - last_action_time) >= RIG_COMMAND_TIMEOUT_MS)){ //a command has timed out
    timeouts++;
    last_action_time = millis();
    command_submitted = NONE; 
    receive_buffer_bytes = 0;
    #ifdef DEBUG_RIG
    Serial.println(F("service_rig_receive_buffers: command timeout rig "));
    #endif //DEBUG_RIG      
  }
     
    
  if ((rig_status == PORT_OPEN) && (timeouts >= RIG_TIMEOUTS_PORT_CLOSE)){ //have we had too many commands time out?
    rig_status = PORT_CLOSED;
    last_action_time = millis();
    #ifdef DEBUG_RIG
    Serial.println(F("service_rig_receive_buffers: rig timeouts port closed rig "));
    #endif //DEBUG_RIG
  }
    

 
}

//-----------------------------------------------------------------------------------------------------

uint8_t Rig::request_frequency(){

  #ifdef DEBUG_RIG
  Serial.println(F("Rig::request_frequency"));
  #endif //DEBUG_RIG

  switch(_rigtype) {
    #ifdef YAESU_SUPPORT
    case YAESU:
      write_bytes(0x00,4);
      write_byte(0x03);     
      break;
      #endif // YAESU_SUPPORT
    #ifdef KENWOOD_SUPPORT
    case KENWOOD:
      write_byte('I');
      write_byte('F');
      write_byte(';');
      break;
      #endif // KENWOOD_SUPPORT
    default:
      return 3;
      break;
	
  }

  command_submitted = FREQUENCY_REQUEST;
  last_action_time = millis(); 
  frequency = 0;
  last_freq_request_time = millis(); 

}

//-----------------------------------------------------------------------------------------------------

uint8_t Rig::ptt(uint8_t on_off){


  switch(_rigtype) {
    #ifdef YAESU_SUPPORT
    case YAESU:
      write_bytes(0x00,4);
      if (on_off) {
        write_byte(0x08);     
      } else {
        write_byte(0x88);
      }
      break;
      #endif //YAESU_SUPPORT
      default: return 3; break;
  }
  last_action_time = millis(); 
  command_submitted = PTT;
}      

//-----------------------------------------------------------------------------------------------------

uint8_t Rig::change_mode(uint8_t switch_to_mode){

  switch(_rigtype) {
    #ifdef YAESU_SUPPORT
    case YAESU:
      switch(switch_to_mode) {
        case LSB: write_byte(0x00); break;
        case USB: write_byte(0x01); break;
        case CW:  write_byte(0x02); break;
        case CWR: write_byte(0x03); break;
        case AM:  write_byte(0x04); break;
        case FM:  write_byte(0x08); break;
        case DIG: write_byte(0x0A); break;
        case PKT: write_byte(0x0C); break;
        case FMN: write_byte(0x88); break;
        default: return 5; break;
      }
      write_bytes(0x00,3);        
      write_byte(0x07);          
      break;
      #endif //YAESU_SUPPORT
      default: return 3; break;
  }
  last_action_time = millis(); 


}

//-----------------------------------------------------------------------------------------------------

void Rig::activate_pass_through(){

  command_submitted = CONTROL_PASS_THROUGH;
  last_action_time = millis(); 

}

