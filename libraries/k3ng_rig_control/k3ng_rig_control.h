#ifndef k3ng_rig_control_h
#define k3ng_rig_control_h

//#define DEBUG_RIG

#define YAESU_SUPPORT
//#define KENWOOD_SUPPORT

enum rig_type {UNDEFINED, YAESU, KENWOOD};
enum rig_status_type {PORT_CLOSED, PORT_OPEN};
enum rig_command_type {NONE, INITIALIZE, FREQUENCY_REQUEST, PTT, CONTROL_PASS_THROUGH, MODE};
enum rig_mode_type {UNDEF, LSB, USB, CW, CWR, AM, WFM, FM, DIG, PKT, FMN, TUNE_MODE, FSK, FSKR};

#define PORT_CLOSED 0
#define PORT_OPEN 1

#define RIG_RECEIVE_BUFFER_BYTES 39          // size of rig serial receive buffer - 10 is fine for Yaesu, use 39 for Kenwood
#define RIG_TIMEOUTS_PORT_CLOSE 3            // close a rig port after this many commmand timeouts
#define RIG_COMMAND_TIMEOUT_MS 5000          // register a timeout when not receiving a response from a rig in this many mS
#define RIG_CONTROL_PASS_THROUGH_CLEAR_MS 250// clear pass through state in this amount of time
#define RIG_RX_BUFFER_TIMEOUT_MS 1000        // clear out the rig receive buffer if we receive a partial response and nothing more in this amount of time
#define RIG_PORT_RETRY_SECS 30               // if a port times out, reinitialize it this many seconds later

#include <SoftwareSerial.h>


class Rig {

  public:
	  Rig(SoftwareSerial *serialportin, uint8_t rigtype);
        void initialize(int baud,uint8_t connection_type);

        void service();  /* call this frequently - this services the rig port, updates statuses, and does housekeeping */
		
        /* rig commands */
        uint8_t request_frequency();
        uint8_t ptt(uint8_t on_off);
        uint8_t change_mode(uint8_t switch_to_mode);
        void activate_pass_through();
        void write_byte(uint8_t byte_to_write);

        /* commonly accessed parameters */
        unsigned int frequency;
        uint8_t mode;

        /* administrative parameters */       
        uint8_t command_submitted;
        uint8_t rig_status;
        uint8_t receive_buffer_bytes;
        unsigned long last_action_time; 
        unsigned long last_freq_request_time;
        uint8_t timeouts;
        
		
  private:	  
		uint8_t rigtype;
		uint8_t _rigtype;
        SoftwareSerial *serialport;
        void write_bytes(uint8_t byte_to_write,uint8_t number_of_bytes);
        void remove_rig_receive_buffer_byte();
        uint8_t bcd_low_value(uint8_t byte_in);
        uint8_t bcd_high_value(uint8_t byte_in);
        unsigned long receive_buffer_last_rx_time;
        uint8_t receive_buffer[RIG_RECEIVE_BUFFER_BYTES];        
        unsigned int _baud;


};

#endif
