#ifndef tuner_tuner_hardware_h
#define tuner_tuner_hardware_h

// hardware configuration

const byte i2c_expander_addr[] = {0x21, 0x22};  // i2c addresses of I/O expanders # 0, 1, 2, etc...
byte i2c_expander_pins[] = {0, 0};              // stores the current state of the I/O expander pins
#define IO_EXPANDERS 2
/*

 Here's how to define three IO Expanders:

const byte i2c_expander_addr[] = {0x21, 0x22, 0x23};  // i2c addresses of I/O expanders # 0, 1, 2, etc...
byte i2c_expander_pins[] = {0, 0, 0};                 // stores the current state of the I/O expander pins
#define IO_EXPANDERS 3

*/
#define PINS_PER_IO_EXPANDER 8  // number of output on each IO expander
#define IO_EXPANDER_MCP23008    // current the code supports just the Microchip MCP23008

/* quick macro howto:

      ! = activate I2C pin
      . = deactive I2C pin
      + = activate native pin
      - = deactive native pin

*/

//       Inductor definitions           L1  L2  L3  L4    L5    L6    L7    L8   
const unsigned int inductor_values[] = { 8, 16, 32, 64,  130,  260,  520, 1040 };        // inductor values in nH
char* inductor_activate_macros[]   = {"!08","!07","!06","!05","!04","!03","!02","!01"};  // macros to activate inductor relays 
char* inductor_deactivate_macros[] = {".08",".07",".06",".05",".04",".03",".02",".01"};  // macros to deactivate inductor relays
byte inductor_status[]             = {0,0,0,0,0,0,0,0};
#define INDUCTORS 8
#define TOTAL_INDUCTOR_VALUE 2070
//        Capacitor definitions           C1    C2    C3    C4     C5    C6    C7    C8
const unsigned int capacitor_values[] = { 12,   22,   39,   82,   150,  300,  600, 1200 };  // capacitor values in pF
char* capacitor_activate_macros[]     = {"!16","!15","!14","!13","!12","!11","!10","!09"}; // macros to activate capacitor relays
char* capacitor_deactivate_macros[]   = {".16",".15",".14",".13",".12",".11",".10",".09"}; // macros to deactivate capacitor relays
byte capacitor_status[]               = {0,0,0,0,0,0,0,0};
#define CAPACITORS 8
#define TOTAL_CAPACITOR_VALUE 2405
//
char* tuning_mode_names[] =             {"HiZ",   "LoZ"};
char* tuning_mode_activate_macros[] =   {"-04+03","-03+04"};
char* tuning_mode_deactivate_macros[] = {"-03",   "-04"};
/* if using three IO expanders:
char* tuning_mode_activate_macros[] =   {"!24",   "!23"};
char* tuning_mode_deactivate_macros[] = {".24",   ".23"};   */
//                          tuning mode:   1         2
byte tuning_mode_status[] = {0,0};
#define TUNING_MODES 2
//
char* tx_switch_names[]  = {"TX1",   "TX2"};
char* tx_switch_macros[] = {"-09+08","-08+09"};
#define TXS 0                     // in development - transmitter switch functionality
//                                  
char* antenna_switch_names[] =  {"ANT1",  "ANT2"};
char* antenna_switch_macros[] = {".20!19",".19!20"};
#define ANTENNAS 0                // in development - antenna switch functionality


#endif //tuner_tuner_hardware_h