/*

 Arduino Antenna Tuner

 Copyright 2013 Anthony Good, K3NG
 All trademarks referred to in source code and documentation are copyright their respective owners.

 ***************************************************************************************************************

    This program is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License
    
                              http://creativecommons.org/licenses/by-nc-sa/3.0/

                          http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


 ***************************************************************************************************************
   
 Are you a radio artisan ?   
 
*/

#define CODE_VERSION "2013062601"

#define TWI_FREQ 100000L //100000L   // change this if you would like to speed up the I2C bus - this is the bus freq in hertz
#include <Wire.h>                    // used for I2C functionality
#include <LiquidCrystal.h>         // uncomment for FEATURE_DISPLAY and when using a regular LCD in 4 bit mode
#include <FreqCounter.h>             // uncomment for FEATURE_FREQ_COUNTER
#include <avr/sleep.h>               // uncomment for FEATURE_SLEEP_MODE
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>             
#include <Adafruit_MCP23017.h>       // uncomment for FEATURE_DISPLAY in combination with FEATURE_LCD_I2C
#include <Adafruit_RGBLCDShield.h>   // uncomment for FEATURE_DISPLAY in combination with FEATURE_LCD_I2C
#include <SoftwareSerial.h>          // needed for rigs not using native hardware serial ports
#include "k3ng_rig_control.h"


// TODO 
//   - Kenwood, Icom support
//   - Single LED Indicator
//   - LCD menu
//   - LCD status messages
//   - multi antenna & tx support - finish - commands into command buffer
//   - multi antenna & tx support- service_tuning() - match ant and tx when searching for tune buffer entries?
//   - parse macros for native pins that are used and initialize them
//   - handle high voltage in measure_swr()
//   - schematic: ASR disable with cap
//   - hardware: 1:4 balun
//   - read Yaesu SWR?
//   - buffer clean out for one band
//   - overwrite same frequency entries in tune_buffer_add?
//   - CLI rig switching and current_rig switching




// To override the pin settings and hardware configuration below, update the lines below to where your personal pins.h and hardware.h filea are located

//#include "C:\Users\goody\Documents\Arduino Sketchbook\tuner\pins.h"       // In the file, create this line: "#define pins_h" (no quotes)
//#include "C:\Users\goody\Documents\Arduino Sketchbook\tuner\hardware.h"   // In the file, create this line: "#define hardware_h" (no quotes)

// administrative and control pin definitions (not I2C expander pins)
#ifndef pins_h
#define pin_led 13
#define pin_tune_in_progress 0   // indicator - goes high when tuning (0 = disable)
#define pin_tuned 0              // indicator - goes high when tuned (0 = disable)
#define pin_untunable 0          // indicator - goes high when untunable (0 = disable)
#define pin_frequency_counter 5  // input - frequency counter (dummy entry - hard coded in frequency counter library)
#define pin_tune_lock 0          // input - ground to lock tuning (0 = disable)
#define pin_forward_v  A0        // input (analog) - SWR sensor forward voltage
#define pin_reflected_v A1       // input (analog) - SWR sensor reverse voltage
#define pin_voltage_control 7    // output - controls SWR sensor voltage attenuator
#define pin_wakeup 2             // input - use with FEATURE_SLEEP_MODE - low wakes unit up
#define pin_awake 0              // output - use with FEATURE_SLEEP_MODE - goes high when unit is awake (0 = disable)
#define pin_manual_tune 0        // input - ground to initiate tuning (0 = disable)
#define rig_0_control_tx A2      // rig serial port - rig RX line / Arduino TX line
#define rig_0_control_rx A3      // rig serial port - rig TX line / Arduino RX line
#endif //pins_h
// end of adminitrative and control pin definitions


#define FEATURE_DISPLAY
#define FEATURE_LCD_I2C
#define FEATURE_SERIAL
#define FEATURE_SERIAL_HELP
#define FEATURE_COMMAND_LINE_INTERFACE
#define FEATURE_FREQ_COUNTER
//#define FEATURE_RECEIVE_BYPASS
//#define FEATURE_SLEEP_MODE
//#define FEATURE_RIG_INTERFACE
//#define FEATURE_RECEIVE_FREQ_AUTOSWITCH
//#define FEATURE_RIG_CONTROL_PASS_THROUGH   // this works best when serial port and rig port baud are the same
#define FEATURE_LCD_I2C_STATUS_COLOR


// Dependency checking
#ifdef FEATURE_RIG_CONTROL_PASS_THROUGH
#undef FEATURE_COMMAND_LINE_INTERFACE
#endif //FEATURE_RIG_CONTROL_PASS_THROUGH
//-
#ifdef FEATURE_RECEIVE_FREQ_AUTOSWITCH
#ifndef FEATURE_RIG_INTERFACE
#error FEATURE_RECEIVE_FREQ_AUTOSWITCH requires FEATURE_RIG_INTERFACE
#endif
#endif
// End - dependency checking


// Rig definitions
SoftwareSerial rig0serial(rig_0_control_rx,rig_0_control_tx);
Rig rig0(&rig0serial,YAESU);
//SoftwareSerial rig1serial(rig_1_control_rx,rig_1_control_tx);
//Rig rig1(&rig1serial,YAESU);
//SoftwareSerial rig2serial(rig_2_control_rx,rig_2_control_tx);
//Rig rig2(&rig2serial,YAESU);
Rig *rig[] = {&rig0};  /*Rig *rig[] = {&rig0,&rig1,&rig3};*/
#define RIGS 1

// set rig baud rates below, look for rig_baud[]


#define OPTION_WRITE_CONFIG_BEFORE_SLEEP             // write the configuration and tune buffer to EEPROM before going to sleep (probably a good idea)
#define OPTION_TUNE_BUFFER_ENTRY_USE_ACCEPTABLE_SWR  // when trying tune buffer entry, use TARGET_SWR_ACCEPTABLE_SETTING threshold (improves tune times)


// debug switches - don't turn these on unless you know what the hell you are doing or you want to get into the inner guts of the code
//#define DEBUG_I2C_PIN_WRITE
//#define DEBUG_EEPROM
//#define DEBUG_RELAY_TEST
//#define DEBUG_COMMAND_BUFFER
//#define DEBUG_MEASURE_SWR
#define DEBUG_STATUS_DUMP
//#define DEBUG_STATUS_DUMP_SWR_CACHE
#define DEBUG_CHECK_STATE
//#define DEBUG_SERVICE_TUNING
//#define DEBUG_SERVICE_TUNING_VERBOSE_FREQ
//#define DEBUG_RIG
//#define DEBUG_RECEIVE_FREQ_AUTOSWITCH
//#define DEBUG_TUNE_BUFFER
//#define DEBUG_REAL_DEEP_STUFF
//#define DEBUG_DONT_TUNE
//#define DEBUG_NO_FREQ
//#define DEBUG_SERIAL

// various settings
// Some of these are rather obtuse or obscure.  Read the manual to fully understand what these do if you're going to tweak the knobs
//
#define TARGET_SWR_GOOD_SETTING 1.2          // the SWR we would like to get
#define TARGET_SWR_ACCEPTABLE_SETTING 1.8    // the SWR we'll settle for
#define SWR_TRIGGER_TUNE 2.0 //2.4                 // the point at which the tuner will automatically start tuning
#define SWR_TRIGGER_TIME_MS 1000             // how long we have to be above the trigger point to start tuning (milliseconds)
#define SWR_OF_LAST_RESORT_CONSIDER_TUNED 2.5 //2.9 // if we run out of tuning combinations and settle for the best match we found, consider it tuned if better or equal to this SWR
#define SWR_OF_LAST_RESORT_KHZ_TUNED 50      // if we setted for SWR or last resort, consider it tuned with this many khz of the original tune frequency
#define TRANSMIT_STOP_GOTO_IDLE_TIME_MS 2000 // if tuning and the TX stops, wait this long before going back into idle mode
#define PENDING_IDLE_SWR_OK_TIME 1000        // the amount of time in mS we must see an OK SWR in order to go back to IDLE
#define TARGET_SWR_GOOD_TIME 5000            // maximum time in mS to seek TARGET_SWR_GOOD
#define TARGET_SWR_ACCEPTABLE_TIME 20000     // maximum time in mS to seek TARGET_SWR_ACCEPTABLE (essentially maximum tuning time) - must be greater than TARGET_SWR_GOOD_TIME
#define UNTUNABLE_RETRY_TIME 5000            // if we couldn't find acceptablematch wait this many mS to attempt tuning again
#define SERIAL_BAUD_RATE 115200              // baud rate of the native serial interface port
#define LCD_COLUMNS 16                       // number of columns in the LCD display
#define LCD_ROWS 2                           // number of rows in the LCD display
#define DISPLAY_STATIC_SCREEN_UPDATE_MS 1000 // how often the LCD is updated, time in mS
#define FREQ_COUNTER_GATE_TIME 1             // amount of time in mS to sample the frequency
#define I2C_LCD_N_BUTTONS_ADDRESS 0x20       // I2C address of the LCD display
#define EEPROM_MAGIC_NUMBER 74               // first byte of EEPROM; used to determine if we have a valid EEPROM structure
#define EEPROM_WRITE_WAIT_MS 5000 //60000
#define COMMAND_BUFFER_SIZE 56
#define SWR_HISTORY_CACHE_SIZE 10
#define SWR_SAMPLE_TIME_MS 0 //1 //5 //10    // wait this many mS in between taking SWR samples
#define MINIMUM_SWR_SAMPLE_COUNT 3           // take this many SWR samples and average them together before considering it a good SWR reading
#define RELAY_SETTLE_TIME_MS 25 //9               // this is the maximum engage / disengage time of the relays (consult the datasheet for the particular relay you use)
#define FORWARD_V_TX_SENSE_THRESH 6          // above this threshold on the analog forward V line, consider the transmitter on (unit is thhe straight analog reading)
#define RECEIVE_BYPASS_DELAY 200             // use with FEATURE_RECEIVE_BYPASS - the amount of time in mS to wait for the TX to be off to switch to RX bypass
#define GO_TO_SLEEP_TIMER 5000               // use with FEATURE_SLEEP_MODE - the amount of time in mS of inactivity before we go to sleep
#define TUNE_BUFFER_SIZE 18                  // the number of tuning combinations we store in RAM and EEPROM
#define TUNE_BUFFER_ADD_MATCH_THRESH_KHZ 50  // if we don't have a tune match within this many khz, add it to the tune buffer
#define TUNE_BUFFER_MATCH_THRESH_KHZ 300     // when doing a tune, try all tune buffer entries within this many khz of the current tx freq
#define I2C_POST_WRITE_DELAY 1               // wait this many mS after doing an I2C write
#define RIG_FREQ_REFRESH_TIME_MS 5000        // request the freq from a rig every x mS
#define RX_FREQ_AUTOSWITCH_TIME_THRESH 0              // wait this many mS to check again for autoswitch
#define RX_FREQ_AUTOSWITCH_FREQ_THRESH_KHZ 100        // trigger ann autoswitch search if RX freq changes this many khz
#define RX_FREQ_AUTOSWITCH_TUNE_MATCH_KHZ_THRESH 500  // choose the closest match in the tune combination buffer within this many khz of the current RX freq
#define RX_FREQ_AUTOSWITCH_WAIT_TIME_FREQ_CHANGE 500  // wait x mS after freq change to trigger rx tune autoswitch
#define EEPROM_BYTES 1024                    // number of bytes in EEPROM - TODO - get this automagically at compile time?
#define HI_L_C_INCREMENT 100                 // pF
#define HI_L_L_INCREMENT 200                 // uH * 100
#define DEBUG_STATUS_DUMP_FREQ_MS 500        // how often to do a periodic status dump
#define DEBUG_STATUS_DUMP_DELAY 0            // delay this many seconds after doing a status dump (only for debug purposes)



enum transmit_state {IDLE, IDLE_UNTUNABLE, TUNING, PENDING_IDLE_NO_TX, LOCK, PENDING_IDLE_SWR_OK};
enum process_command_buffer_state {NORMAL, IN_DELAY};
enum relay_state {RELAY_NORMAL, RELAY_SETTLE};
enum service_tuning_action {TUNE, INITIATE};
enum led_indication_type {TUNE_IN_PROGRESS,TUNED,UNTUNABLE,INDICATOR_IDLE};
enum tuning_phases {PHASE_1_STRAIGHT_THRU, PHASE_2_CHECK_TUNE_BUFFER, PHASE_3_HI_Z_SCAN, PHASE_4_LO_Z_SCAN, PHASE_5_HI_Z_HI_L_SCAN, PHASE_6_LO_Z_HI_L_SCAN,
PHASE_3_L_PROFILE, PHASE_4_C_SCAN, PHASE_7_LAST_RESORT, PHASE_999_THE_END};
enum swr_targets {TARGET_SWR_GOOD, TARGET_SWR_ACCEPTABLE};
enum adjust_component {INDUCTOR, CAPACITOR};
enum adjust_adjustment_type {STEP, ABSOLUTE, RELATIVE, SINGLE};
enum tuning_flag_type {TUNING_FLAG_NONE,TUNING_FLAG_TUNE_BUFFER_SWR_ACCEPT};

// LCD definitions
//LiquidCrystal lcd(12, 11, 10, 9, 8, 7);                 // uncomment this for a regular LCD display
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();      // uncomment this for FEATURE_LCD_I2C (address 0x21)

byte state_tuner = IDLE;
float current_swr = 0;
byte config_dirty = 0;
byte command_buffer[COMMAND_BUFFER_SIZE];
byte command_buffer_pointer = 0;
byte relay_status = RELAY_NORMAL;
byte tuning_phase = PHASE_1_STRAIGHT_THRU;
byte tuning_phase_iteration = 0;
byte swr_target = TARGET_SWR_GOOD;
unsigned long relay_settle_start_time = 0;
unsigned int best_match_C = 0;
unsigned int best_match_L = 0;
unsigned int tuned_freq = 0;
byte best_match_tuning_mode = 0;
float best_match_swr = 999;
byte tried_best_swr = 0;
byte debug_mode = 1;
byte manual_tune_invoke = 0;
byte lock_invoke = 0;
byte unlock_invoke = 0;
unsigned int last_known_good_freq = 0;
byte tuning_flag = TUNING_FLAG_NONE;
byte current_antenna = 1;
byte current_transmitter = 1;

#ifdef DEBUG_SERVICE_TUNING
byte debug_service_tune_buffer_swr_measure_flag = 0;
#endif //DEBUG_SERVICE_TUNING

#ifdef FEATURE_RECEIVE_FREQ_AUTOSWITCH
byte receive_freq_autoswitch_active = 1;
#endif //FEATURE_RECEIVE_FREQ_AUTOSWITCH

#ifdef FEATURE_COMMAND_LINE_INTERFACE
byte cli_command_buffer[50];
int cli_command_buffer_index = 0;

#endif //FEATURE_COMMAND_LINE_INTERFACE

byte periodic_print_status = 0;

struct config_t {
  byte magic_number;
  unsigned int last_L;
  unsigned int last_C;
  byte last_tuning_mode_tx_ant;
  byte receive_bypass; 
  unsigned int tune_buffer_entries; 
  unsigned int tuned_freq;
} configuration;

struct tune_buffer_t {   // tune buffer - this stores succesful tuning combinations
  unsigned int freq;  // frequency in khz
  unsigned int L;     // inductance in nH
  unsigned int C;     // capacitance in pF
  byte tuning_mode;   // bits 87654321:    8 : future, 765 : antenna, 43 : transmitter, 21 : tuning mode
                      //                                 000 = #1        00 = #1             00 = none
                      //                                 001 = #2        01 = #2             01 = HiZ
                      //                                 010 = #3        10 = #3             10 = LoZ
                      //                                 011 = #4        11 = #4             11 = unused/future
                      //                                 100 = #5
                      //                                 101 = #6
                      //                                 110 = #7
                      //                                 111 = #8
                      
  unsigned int swr;   // SWR * 100
} tune_buffer[TUNE_BUFFER_SIZE];


#ifdef FEATURE_RIG_INTERFACE
unsigned long rig_last_freq_request_time[RIGS];
unsigned int rig_baud[] = {9600};    // multiple rig example: unsigned int rig_baud[] = {9600, 4800, 38400};
byte current_rig = 0;                // 0 = rig #1, 1 = rig #2, etc.
#endif //FEATURE_RIG_INTERFACE


#ifdef FEATURE_RECEIVE_BYPASS
byte receive_bypass = 0;  // turn on receive bypass mode here (TODO - user activation / deactivation & EEPROM storage)
byte in_receive_bypass = 0;
#endif //FEATURE_RECEIVE_BYPASS

// hardware configuration
#ifndef hardware_h
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
#endif //hardware_h
// end of hardware configuration

int forward_voltage = 0;
int reverse_voltage = 0;
float forward_power = 0;
float history_swr[SWR_HISTORY_CACHE_SIZE];
float history_forward_power[SWR_HISTORY_CACHE_SIZE];
byte transmit_sense = 0;

//float frequencies[]             = {1.8,   3.5,  7.0, 10.1, 14.0,  18.0,  21.0,  24.5,  28.0};
//float fwd_power_calibarations[] = {50.0, 27.7, 25.0, 25.0,  5.0, 0.833, 0.446, 0.217, 0.053};
//int number_calibrations_frequencies = 9;


#ifdef FEATURE_FREQ_COUNTER
float frequency_counter_calibration = 0.985836;
unsigned int last_measured_frequency = 0;
#endif //FEATURE_FREQ_COUNTER

#ifdef FEATURE_SLEEP_MODE
unsigned long last_activity_time = 0;
byte sleep_disabled = 0;
#endif //FEATURE_SLEEP_MODE



#ifdef FEATURE_DISPLAY
enum lcd_statuses {LCD_CLEAR, LCD_REVERT, LCD_TIMED_MESSAGE, LCD_SCROLL_MSG, LCD_STATIC};
#define default_display_msg_delay 1000
byte lcd_status = LCD_STATIC;
unsigned long lcd_timed_message_clear_time = 0;
byte lcd_previous_status = LCD_STATIC;
byte lcd_scroll_buffer_dirty = 0;
String lcd_scroll_buffer[LCD_ROWS];
byte lcd_scroll_flag = 0;
#ifdef FEATURE_LCD_I2C
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
byte lcdcolor = GREEN;  // default color of I2C LCD display
#define LCD_I2C_STATUS_COLOR_TUNED 0x2     // used by FEATURE_LCD_I2C_STATUS_COLOR
#define LCD_I2C_STATUS_COLOR_UNTUNED 0x1   // used by FEATURE_LCD_I2C_STATUS_COLOR
#define LCD_I2C_STATUS_COLOR_TUNING 0x4    // used by FEATURE_LCD_I2C_STATUS_COLOR
#endif //FEATURE_LCD_I2C
#endif //FEATURE_DISPLAY

/* here's where it all starts */

void setup() {

  #ifdef FEATURE_SERIAL
  initialize_serial();
  #endif //FEATURE_SERIAL

  initialize_native_pins();
  initialize_i2c();

  #ifdef FEATURE_DISPLAY
  initialize_display();
  display_startup_message();
  #endif //FEATURE_DISPLAY
  
  
  #ifdef FEATURE_RIG_INTERFACE
  initialize_rigs();  
  #endif //FEATURE_RIG_INTERFACE

  #ifdef DEBUG_RELAY_TEST
  relay_test();
  #endif //DEBUG_RELAY_TEST
  
  tune_buffer_clear();
  initialize_settings_from_eeprom();

  
  #ifdef FEATURE_FREQ_COUNTER
  initialize_freq_counter();
  #endif //FEATURE_FREQ_COUNTER
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("setup: going into loop"));
  #endif //DEBUG_REAL_DEEP_STUFF
  

}

/* the code was always there, I just removed the excess ones and zeros to form it */

void loop() {
 
  
  #ifndef DEBUG_DONT_TUNE 
  check_state();
  measure_swr();
  check_relay_status();
  check_for_dirty_configuration();
  #endif //DEBUG_DONT_TUNE

  #ifdef FEATURE_DISPLAY
  //check_display_buttons();
  service_display();
  #endif

  process_command_buffer();
  check_serial();
  
  #ifdef FEATURE_RECEIVE_BYPASS
  check_receive_bypass();
  #endif //FEATURE_RECEIVE_BYPASS
  
  #ifdef FEATURE_RIG_INTERFACE
  service_rigs();
  get_rig_frequencies();
  #endif //FEATURE_RIG_INTERFACE

  #ifdef FEATURE_RECEIVE_FREQ_AUTOSWITCH  
  check_receive_freq_autoswitch();
  #endif //FEATURE_RECEIVE_FREQ_AUTOSWITCH

}

//-----------------------------------------------------------------------------------------------------
//
//
//                                            Subroutines
//
//
//-----------------------------------------------------------------------------------------------------


void check_state(){
  
  byte pin_read = 0;
  static unsigned long tuning_start_time = 0;
  static unsigned long swr_trigger_start_time = 0;
  static byte last_relay_status = RELAY_NORMAL;
  static unsigned long no_transmit_sense_start_time = 0;
  static unsigned long swr_target_start_time = 0;
  static unsigned int frequency_at_start = 0;
  byte trigger_tune = 0;

  #ifdef DEBUG_STATUS_DUMP
  static unsigned long last_debug_dump = 0;
  byte debug_printed = 0;
  #endif //DEBUG_STATUS_DUMP

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("check_state: entering"));
  #endif
  
  #ifdef DEBUG_CHECK_STATE
  static unsigned long last_debug_check_state_print = 0;
  #endif


  if (pin_tune_lock) {                        // has tune lock been switched on?
    pin_read = digitalRead(pin_tune_lock);
    if ((pin_read == LOW) && (state_tuner != LOCK)) {
      state_tuner = LOCK;
      #ifdef FEATURE_SLEEP_MODE
      last_activity_time = millis();
      #endif
    }
    if ((pin_read == HIGH) && (!lock_invoke) && (state_tuner == LOCK)) {
      state_tuner = IDLE;           
      #ifdef FEATURE_SLEEP_MODE
      last_activity_time = millis();
      #endif
    }
  }
  
  if ((lock_invoke) && (state_tuner != LOCK)) {
    state_tuner = LOCK;
    #ifdef FEATURE_SLEEP_MODE
    last_activity_time = millis();
    #endif    
  }
  if ((lock_invoke) && (unlock_invoke) && (state_tuner == LOCK)) {
    state_tuner = IDLE;           
    #ifdef FEATURE_SLEEP_MODE
    last_activity_time = millis();
    #endif
    lock_invoke = 0;
    unlock_invoke = 0;
  }  
    
  
  if (((pin_manual_tune) || (manual_tune_invoke)) && (state_tuner != TUNING)) {  // is the manual tune button being pressed?
    pin_read = digitalRead(pin_manual_tune);
    if ((pin_read == LOW) || (manual_tune_invoke)) {
      state_tuner = TUNING;
      swr_target = TARGET_SWR_GOOD;
      swr_target_start_time = millis();
      service_tuning(INITIATE);
      tuning_start_time = millis();
      set_indicators(TUNE_IN_PROGRESS);       
      #ifdef FEATURE_SLEEP_MODE
      last_activity_time = millis();
      #endif
      manual_tune_invoke = 0;   
      #ifdef FEATURE_DISPLAY
      update_static_screen(1);
      #endif //FEATURE_DISPLAY 
    }  
  }

  switch (state_tuner) {
    case IDLE:  
      if (transmit_sense) {
        if (current_swr > SWR_TRIGGER_TUNE) {
          trigger_tune = 1; 
          #ifdef DEBUG_CHECK_STATE
          if ((debug_mode) && ((millis() - last_debug_check_state_print) > 1000)){
            Serial.println(F("check_state: current_swr > SWR_TRIGGER_TUNE"));
            debug_printed = 1;
          }        
          #endif   
        } else {
          
          #ifdef DEBUG_CHECK_STATE
          if ((debug_mode) && ((millis() - last_debug_check_state_print) > 1000)){
            Serial.println(F("check_state: current_swr < SWR_TRIGGER_TUNE"));
            debug_printed = 1;
          }        
          #endif           
        }
          
        /*  experimental - triggered too often
        if (better_swr_in_tune_buffer(current_swr,current_freq())){ 
          trigger_tune = 1; 
          #ifdef DEBUG_CHECK_STATE
          if ((debug_mode) && ((millis() - last_debug_check_state_print) > 1000)){
            Serial.println(F("check_state: better swr in tune buffer"));
          }  
          #endif         
        } 
       */        
          
        if ((SWR_OF_LAST_RESORT_CONSIDER_TUNED > 0) && (current_swr <= SWR_OF_LAST_RESORT_CONSIDER_TUNED)&&(abs(tuned_freq - current_freq()) < SWR_OF_LAST_RESORT_KHZ_TUNED) && 
        ((tuning_phase == PHASE_7_LAST_RESORT)  || (tuning_phase == PHASE_999_THE_END))) {
          #ifdef DEBUG_CHECK_STATE
          if ((debug_mode) && ((millis() - last_debug_check_state_print) > 1000)){
            Serial.println(F("check_state: within SWR_OF_LAST_RESORT_CONSIDER_TUNED"));
            last_debug_check_state_print = millis();
          }  
          #endif          
          trigger_tune = 0;
        }

            
       if (trigger_tune){ 
          if (swr_trigger_start_time == 0) {  //this is the first time we've been above the trigger
            swr_trigger_start_time = millis();
            best_match_swr = 999;
          } else {
            if ((millis() - swr_trigger_start_time) > SWR_TRIGGER_TIME_MS)  {  //we've been above the trigger limit enough time to initiate a tune
              state_tuner = TUNING;
              swr_target = TARGET_SWR_GOOD;
              swr_target_start_time = millis();
              swr_trigger_start_time = millis();
              service_tuning(INITIATE);
              tuning_start_time = millis();
              set_indicators(TUNE_IN_PROGRESS);   
              #ifdef DEBUG_CHECK_STATE
              if (debug_mode) {
                Serial.print(F("check_state: start TUNING swr: "));
                Serial.println(current_swr);
              }
              #endif //DEBUG_CHECK_STATE
              #ifdef FEATURE_DISPLAY
              update_static_screen(1);
              #endif //FEATURE_DISPLAY               
            }
          }           
        } else {
          swr_trigger_start_time = 0;         
        } // (trigger_tune)
          
        #ifdef FEATURE_SLEEP_MODE
        last_activity_time = millis();
        #endif
      } else {
        swr_trigger_start_time = 0;
      } // (transmit_sense)
      
      #ifdef DEBUG_CHECK_STATE
      if (debug_printed) {last_debug_check_state_print = millis();}
      #endif
      break;
  
    case TUNING:
      if ((last_relay_status == RELAY_SETTLE) && (relay_status == RELAY_NORMAL)) {  //we just came out of a relay settle period
        clear_swr_cache();     
        last_relay_status = RELAY_NORMAL;
      }
  
      if (relay_status == RELAY_NORMAL) {
        last_relay_status = RELAY_NORMAL;
        if (transmit_sense) {                           // is the transmitter is on?
          no_transmit_sense_start_time = 0;
          if (current_swr > 0) { // do we have a valid SWR reading yet?
          
            #ifdef DEBUG_SERVICE_TUNING
            if ((debug_mode) && (debug_service_tune_buffer_swr_measure_flag)) {   
                Serial.print(F("check_state: swr:"));
                Serial.println(current_swr);   
                debug_service_tune_buffer_swr_measure_flag = 0;      
            }
            #endif //DEBUG_SERVICE_TUNING
          
            if ((swr_target == TARGET_SWR_GOOD) && (current_swr <= TARGET_SWR_GOOD_SETTING)){
              state_tuner = PENDING_IDLE_SWR_OK; 
              #ifdef DEBUG_CHECK_STATE
              if (debug_mode) {Serial.println(F("check_state: TARGET_SWR_GOOD"));}
              #endif              
            }
             
            if ((swr_target == TARGET_SWR_ACCEPTABLE) && (current_swr <= TARGET_SWR_ACCEPTABLE_SETTING)){
              state_tuner = PENDING_IDLE_SWR_OK; 
              #ifdef DEBUG_CHECK_STATE
              if (debug_mode) {Serial.println(F("check_state: TARGET_SWR_ACCEPTABLE"));}
              #endif                  
            }
            
            
            if ((tuning_phase == PHASE_999_THE_END) && (current_swr <= SWR_OF_LAST_RESORT_CONSIDER_TUNED)){
              state_tuner = PENDING_IDLE_SWR_OK; 
              #ifdef DEBUG_CHECK_STATE
              if (debug_mode) {Serial.println(F("check_state: PHASE_999_THE_END"));}
              #endif                 
            }
            
            if ((tuning_flag == TUNING_FLAG_TUNE_BUFFER_SWR_ACCEPT) && (current_swr <= TARGET_SWR_ACCEPTABLE_SETTING)){
              state_tuner = PENDING_IDLE_SWR_OK;
              #ifdef DEBUG_CHECK_STATE
              if (debug_mode) {Serial.println(F("check_state: TUNING_FLAG_TUNE_BUFFER_SWR_ACCEPT"));}
              #endif              
          
            }
            


           if (state_tuner == PENDING_IDLE_SWR_OK){
              #ifdef DEBUG_CHECK_STATE
              if (debug_mode) {
                Serial.print(F("check_state: PENDING_IDLE_SWR_OK swr:"));
                Serial.print(current_swr);
                Serial.print(F(" secs: "));
                Serial.println((millis()-swr_trigger_start_time)/1000);
              }
              #endif //DEBUG_CHECK_STATE 

              //clear_swr_cache();
              swr_trigger_start_time = millis();

            } else {  // we did not get a target SWR
              if ((swr_target == TARGET_SWR_GOOD) && ((millis() - swr_target_start_time) > TARGET_SWR_GOOD_TIME)) {
                swr_target = TARGET_SWR_ACCEPTABLE;
                swr_target_start_time = millis();               
              }
              if (((swr_target == TARGET_SWR_ACCEPTABLE) && ((millis() - swr_target_start_time) > TARGET_SWR_ACCEPTABLE_TIME)) ||
              ((tuning_phase == PHASE_999_THE_END) && (current_swr > SWR_OF_LAST_RESORT_CONSIDER_TUNED))) {
                state_tuner = IDLE_UNTUNABLE;
                swr_target_start_time = millis();
                set_indicators(UNTUNABLE);
                tuned_freq = 0;
                #ifdef FEATURE_DISPLAY
                lcd_center_print_timed("Sorry",0,4000);
                lcd_center_print_timed("No Match Found",1,4000);
                #endif //FEATURE_DISPLAY                                
                #ifdef DEBUG_CHECK_STATE
                if (debug_mode) {
                  Serial.print(F("check_state: IDLE_UNTUNABLE swr:"));
                  Serial.println(current_swr);
                }
                #endif //DEBUG_CHECK_STATE 
              } else {
                  if (current_swr < best_match_swr) {
                    best_match_swr = current_swr;
                    best_match_C = current_C();
                    best_match_L = current_L();
                    best_match_tuning_mode = current_tuning_mode();  
                  }
                  service_tuning(TUNE);         
              }
            }
          } //(current_swr > 0)
        } else {   // (transmit_sense)                                     // the transmitter stopped
          state_tuner = PENDING_IDLE_NO_TX;
          no_transmit_sense_start_time = millis();
          #ifdef DEBUG_CHECK_STATE
          if (debug_mode) {
            Serial.print(F("check_state: PENDING_IDLE_NO_TX swr:"));
            Serial.println(current_swr);
          }
          #endif //DEBUG_CHECK_STATE           
        }  // (transmit_sense)
      } else {  // (relay_status == RELAY_NORMAL)
        last_relay_status = RELAY_SETTLE;  // we're in relay settle time - don't do anything right now
        if (frequency_at_start == 0){             // while we're waiting, measure the frequency
          frequency_at_start = current_freq();
        }
      } // (relay_status == RELAY_NORMAL)
      break;
      
    case IDLE_UNTUNABLE:
      if ((millis() - swr_target_start_time) >= UNTUNABLE_RETRY_TIME) {
        state_tuner = IDLE;
        swr_trigger_start_time = 0;
        clear_swr_cache();
        set_indicators(INDICATOR_IDLE);
        #ifdef DEBUG_CHECK_STATE
        if (debug_mode) {
          Serial.print(F("check_state: IDLE swr: "));
          Serial.println(current_swr);
        }
        #endif //DEBUG_CHECK_STATE 
      }
      break;
      
    case PENDING_IDLE_NO_TX:
      if (transmit_sense) {
        state_tuner = TUNING;
        clear_swr_cache();
        #ifdef DEBUG_CHECK_STATE
        if (debug_mode) {
          Serial.println(F("check_state: TUNING"));
        }
        #endif //DEBUG_CHECK_STATE         
      } else {
        if ((millis() - no_transmit_sense_start_time) > TRANSMIT_STOP_GOTO_IDLE_TIME_MS) {
          state_tuner = IDLE;
          frequency_at_start = 0;
          swr_trigger_start_time = 0; 
          clear_swr_cache();
          set_indicators(INDICATOR_IDLE);
          #ifdef DEBUG_CHECK_STATE
          if (debug_mode) {
            Serial.print(F("check_state: IDLE swr: "));
            Serial.println(current_swr);
          }
          #endif //DEBUG_CHECK_STATE           
        }
      }
      break;
      
    case PENDING_IDLE_SWR_OK:
      if (((millis() - swr_trigger_start_time) >= PENDING_IDLE_SWR_OK_TIME) /*|| (tuning_phase == PHASE_999_THE_END)*/) {  // have we been in this state long enough?
        tune_buffer_add(frequency_at_start);
        frequency_at_start = 0;  // set it to 0 for the next tune
        config_dirty = 1;
        swr_trigger_start_time = 0;              
        set_indicators(TUNED); 
        state_tuner = IDLE;      
      } else {  // no, check swr again
//zzzz
        if (((swr_target == TARGET_SWR_GOOD) && (current_swr <= TARGET_SWR_GOOD_SETTING)) || 
        ((swr_target == TARGET_SWR_ACCEPTABLE) && (current_swr <= TARGET_SWR_ACCEPTABLE_SETTING)) ||
        ((tuning_flag == TUNING_FLAG_TUNE_BUFFER_SWR_ACCEPT) && (current_swr <= TARGET_SWR_ACCEPTABLE_SETTING)) ||
        ((tuning_phase == PHASE_999_THE_END) && (current_swr <= SWR_OF_LAST_RESORT_CONSIDER_TUNED))) {  
            
        } else {
          state_tuner = TUNING;
          clear_swr_cache();
          #ifdef DEBUG_CHECK_STATE
          if (debug_mode) {
            Serial.println(F("check_state: PENDING_IDLE_SWR_OK->TUNING"));
          }
          #endif //DEBUG_CHECK_STATE         
        }
      }
      break;

  }




  #ifdef FEATURE_SLEEP_MODE  
  if (((millis() - last_activity_time) > GO_TO_SLEEP_TIMER) && (!sleep_disabled)) {
    if (digitalRead(pin_wakeup) == HIGH) {
      if (pin_awake) {
        digitalWrite(pin_awake, LOW);
      }
      attachInterrupt(0, wakeup, LOW);
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      #ifdef DEBUG_CHECK_STATE
      if (debug_mode) {
        Serial.println(F("check_state: entering sleep"));
        delay(1000);
      }
      #endif
      #ifdef OPTION_WRITE_CONFIG_BEFORE_SLEEP
      if (config_dirty) {
        write_settings_to_eeprom(0);
        config_dirty = 0;
      }
      #endif //OPTION_WRITE_CONFIG_BEFORE_SLEE
      sleep_mode();

      // ZZZZZZZZ - shhhhh! we are asleep here !!

      sleep_disable();
      if (pin_awake) {
        digitalWrite(pin_awake, HIGH);
      }
    }
    last_activity_time = millis(); 
  }
  #endif //FEATURE_SLEEP_MODE 

  #ifdef DEBUG_STATUS_DUMP
  if (periodic_print_status && ((millis()-last_debug_dump) > DEBUG_STATUS_DUMP_FREQ_MS)) {
    print_status();
    last_debug_dump = millis();
  }
  #endif //DEBUG_STATUS_DUMP

}
//-----------------------------------------------------------------------------------------------------
void service_tuning(byte action){
  
  static unsigned int workint0 = 0;
  static unsigned int workint1 = 0;
  static unsigned int workint2 = 0;
  static byte phase_init = 0;
  static byte last_tuning_mode = 0;
  static byte tried_best_swr_cleanup = 0;
  static unsigned int last_C = 0;
  static unsigned int last_L = 0;
  unsigned int check_freq = 0;
  static byte c_going_up = 0;
  
  


  if (action == INITIATE) {
    #ifdef DEBUG_SERVICE_TUNING
    if (debug_mode) {Serial.println(F("service_tuning: INITIATE / PHASE_1_STRAIGHT_THRU"));}
    #endif //DEBUG_SERVICE_TUNING
    tuning_phase = PHASE_1_STRAIGHT_THRU;
    tuning_phase_iteration = 0;
    adjust(CAPACITOR,ABSOLUTE,0);
    adjust(INDUCTOR,ABSOLUTE,0);
    deactivate_all_tuning_modes();
    phase_init = 0;
    tried_best_swr_cleanup = 0;
    tried_best_swr = 0;
    tuning_flag = TUNING_FLAG_NONE;
    //tuned = 0;
  }
  

  if (action == TUNE) {

    tuning_flag = TUNING_FLAG_NONE;
    
    if ((tried_best_swr) && (!tried_best_swr_cleanup)) {  // change the tuning mode back to what it was prior to trying best swr
      switch_tuning_mode(last_tuning_mode & B00000011);
      adjust(INDUCTOR, ABSOLUTE, last_L);
      adjust(CAPACITOR, ABSOLUTE, last_C);      
      tried_best_swr_cleanup = 1;
      #ifdef DEBUG_SERVICE_TUNING
      if (debug_mode) {
        Serial.print(F("service_tuning: tried best swr cleanup last_L: "));
        Serial.print(last_L);
        Serial.print(F(" last_C: "));
        Serial.print(last_C);
        Serial.print(F(" last_tuning_mode: "));
        Serial.println(last_tuning_mode);
      }
      #endif 
    }
    
    if ((swr_target == TARGET_SWR_ACCEPTABLE) && (best_match_swr <= TARGET_SWR_ACCEPTABLE_SETTING) && (!tried_best_swr)) {
      last_tuning_mode = current_tuning_mode();
      last_C = current_C();
      last_L = current_L();
      adjust(INDUCTOR, ABSOLUTE, best_match_L);
      adjust(CAPACITOR, ABSOLUTE, best_match_C);
      switch_tuning_mode(best_match_tuning_mode & B00000011);
      tried_best_swr = 1;
      tried_best_swr_cleanup = 0;
      #ifdef DEBUG_SERVICE_TUNING
      if (debug_mode) {
        Serial.print(F("service_tuning: trying best match: L: "));
        Serial.print(best_match_L);
        Serial.print(F(" C: "));
        Serial.print(best_match_C);
        Serial.print(F(" tuning_mode: "));
        Serial.print(best_match_tuning_mode);        
        Serial.print(F(" best_match_swr: "));
        Serial.println(best_match_swr);
      }
      #endif  
      return;
    }
    

    
    
    switch(tuning_phase) {
      case PHASE_1_STRAIGHT_THRU:
        // go to next phase
        //tuning_phase = PHASE_2_HI_Z_SCAN;
        tuning_phase = PHASE_2_CHECK_TUNE_BUFFER;
        phase_init = 0;        
      break;
      case PHASE_2_CHECK_TUNE_BUFFER:
        if (!phase_init) {
          //for (int x = 0;x < 11;x++) {
          //  current_freq();
          //}
          phase_init = 1;
          workint1 = 0;
          #ifdef DEBUG_SERVICE_TUNING
          if (debug_mode) {
            Serial.println(F("service_tuning: PHASE_2_CHECK_TUNE_BUFFER"));
          }
          #endif //DEBUG_SERVICE_TUNING          
        }
        if (workint1 < TUNE_BUFFER_SIZE) {
          
          check_freq = current_freq();
          
          #ifdef DEBUG_SERVICE_TUNING_VERBOSE_FREQ
          if (debug_mode) {
            //Serial.print(F("service_tuning: PHASE_2_CHECK_TUNE_BUFFER: last_measured_frequency: "));
            //Serial.println(last_measured_frequency);
            Serial.print(F("service_tuning: PHASE_2_CHECK_TUNE_BUFFER: freq: "));
            Serial.println(check_freq);            
          }      
          #endif //DEBUG_SERVICE_TUNING_VERBOSE_FREQ    
                
          if ((tune_buffer[workint1].freq > 0) && (abs(tune_buffer[workint1].freq - check_freq) <= TUNE_BUFFER_MATCH_THRESH_KHZ)) {  // we have a match
            adjust(CAPACITOR,ABSOLUTE,tune_buffer[workint1].C);
            adjust(INDUCTOR,ABSOLUTE,tune_buffer[workint1].L);
            switch_tuning_mode(tune_buffer[workint1].tuning_mode & B00000011);
            #ifdef OPTION_TUNE_BUFFER_ENTRY_USE_ACCEPTABLE_SWR
            tuning_flag = TUNING_FLAG_TUNE_BUFFER_SWR_ACCEPT;                       
            #endif //OPTION_TUNE_BUFFER_ENTRY_USE_ACCEPTABLE_SWR
            #ifdef DEBUG_SERVICE_TUNING
            if (debug_mode) {
              Serial.print(F("service_tuning: PHASE_2_CHECK_TUNE_BUFFER: trying tune buffer: "));
              Serial.print(workint1);
              Serial.print(F("\tbuff f: "));
              Serial.print(tune_buffer[workint1].freq);
              Serial.print(F("\tbuff swr: "));
              Serial.println(float(tune_buffer[workint1].swr)/100);
              debug_service_tune_buffer_swr_measure_flag = 1;
            }      
            #endif //DEBUG_SERVICE_TUNING         
          } else {
            #ifdef DEBUG_SERVICE_TUNING
            if (debug_mode) {
              Serial.print(F("service_tuning: PHASE_2_CHECK_TUNE_BUFFER: no match tune buffer: "));
              Serial.print(workint1);
              Serial.print(F("\tbuff f: "));
              Serial.print(tune_buffer[workint1].freq);
              Serial.print(F("\tcheck_freq: "));
              Serial.println(check_freq);
            }      
            #endif //DEBUG_SERVICE_TUNING            
          }
          workint1++;
        } else {
          // go to next phase
          tuning_phase = PHASE_3_HI_Z_SCAN;
          phase_init = 0;
        }
        break;
        
        
      case PHASE_3_HI_Z_SCAN:     
        if (!phase_init) {
          workint0 = 1;
          workint1 = 1;
          workint2 = 1;
          c_going_up = 1;
          switch_tuning_mode(1);
          tuning_phase_iteration = 0;
          phase_init = 1;
          #ifdef DEBUG_SERVICE_TUNING
          if (debug_mode) {
            Serial.println(F("service_tuning: PHASE_3_HI_Z_SCAN"));
          }
          #endif //DEBUG_SERVICE_TUNING          
        }   
        
        if (workint0 < 3) {        
          if (workint1 < CAPACITORS+1) {
            if (c_going_up) {
              adjust(CAPACITOR,SINGLE,workint1);
            } else {
              adjust(CAPACITOR,SINGLE,(CAPACITORS+1-workint1));            }
            workint1++;
            tuning_phase_iteration++;
          } else {
            if (workint2 < INDUCTORS+1) {
              adjust(INDUCTOR,SINGLE,workint2);
              workint2++;
              workint1 = 1;
              if (c_going_up) {
                c_going_up = 0;
              } else {
                c_going_up = 1;
              }
              tuning_phase_iteration++;
            } else {
              workint0++;
              workint1 = 1;
              workint2 = 1;
              switch_tuning_mode(2);
            }
          }     
        } else {
          // go to next phase
          tuning_phase = PHASE_5_HI_Z_HI_L_SCAN;
          phase_init = 0;
        }        
        break;
       
        
/*      case PHASE_4_LO_Z_SCAN:     
        if (!phase_init) {
          tuning_phase_iteration = 0;
          workint1 = 1;
          workint2 = 1;
          c_going_up = 1;
          adjust(CAPACITOR,ABSOLUTE,0);
          adjust(INDUCTOR,ABSOLUTE,0);
          switch_tuning_mode(2);
          phase_init = 1;
          #ifdef DEBUG_SERVICE_TUNING
          if (debug_mode) {
            Serial.println(F("service_tuning: PHASE_3_LO_Z_SCAN"));
          }
          #endif //DEBUG_SERVICE_TUNING            
        }   
        if (workint1 < CAPACITORS+1) {
          if (c_going_up) {
            adjust(CAPACITOR,SINGLE,workint1);
          } else {
            adjust(CAPACITOR,SINGLE,(CAPACITORS+1-workint1));
          }          
          workint1++;
          tuning_phase_iteration++;
        } else {
          if (workint2 < INDUCTORS+1) {
            adjust(INDUCTOR,SINGLE,workint2);
            workint2++;
            workint1 = 1;
            if (c_going_up) {
              c_going_up = 0;
            } else {
              c_going_up = 1;
            }            
            tuning_phase_iteration++;
          } else {
            // go to next phase
            tuning_phase = PHASE_5_HI_Z_HI_L_SCAN;
            phase_init = 0;
          }
        }         
      break;   */   
      case PHASE_5_HI_Z_HI_L_SCAN: 
        if (!phase_init) {
          workint0 = 1;
          workint2 = inductor_values[INDUCTORS-1];
          workint1 = capacitor_values[0];
          switch_tuning_mode(1);
          tuning_phase_iteration = 0;
          phase_init = 1;
          #ifdef DEBUG_SERVICE_TUNING
          if (debug_mode) {
            Serial.println(F("service_tuning: PHASE_5_HI_Z_HI_L_SCAN"));
          }
          #endif //DEBUG_SERVICE_TUNING            
        }   
        
        if (workint0 < 3) {          
          if (workint1 <= TOTAL_CAPACITOR_VALUE) {
            adjust(CAPACITOR,ABSOLUTE,workint1);
            workint1 = workint1 + HI_L_C_INCREMENT;
            tuning_phase_iteration++;
          } else {
            if (workint2 <= TOTAL_INDUCTOR_VALUE) {
              adjust(INDUCTOR,ABSOLUTE,workint2);        
              workint2 = workint2 + HI_L_L_INCREMENT;
              if ((workint2 > TOTAL_INDUCTOR_VALUE) && (workint2 < (TOTAL_INDUCTOR_VALUE + HI_L_L_INCREMENT))) {
                workint2 = TOTAL_INDUCTOR_VALUE;
              }
        
              //workint2 = workint2 + HI_L_L_INCREMENT;
              workint1 = capacitor_values[0];
              tuning_phase_iteration++;
            } else {
              workint0++;
              workint2 = inductor_values[INDUCTORS-1];
              workint1 = capacitor_values[0];
              switch_tuning_mode(2);              
            }
          }   
        } else {
            // go to next phase
            tuning_phase = PHASE_7_LAST_RESORT;              
            phase_init = 0;   
        }       
        break; //PHASE_5_HI_Z_HI_L_SCAN   
        
 /*     case PHASE_6_LO_Z_HI_L_SCAN: 
        if (!phase_init) {
          workint2 = inductor_values[INDUCTORS-1];
          workint1 = capacitor_values[0];
          switch_tuning_mode(2);
          tuning_phase_iteration = 0;
          phase_init = 1;
          #ifdef DEBUG_SERVICE_TUNING
          if (debug_mode) {
            Serial.println(F("service_tuning: PHASE_6_LO_Z_HI_L_SCAN"));
          }
          #endif //DEBUG_SERVICE_TUNING            
        }   
        if (workint1 <= TOTAL_CAPACITOR_VALUE) {
          adjust(CAPACITOR,ABSOLUTE,workint1);
          workint1 = workint1 + HI_L_C_INCREMENT;
          tuning_phase_iteration++;
        } else {
          if (workint2 <= TOTAL_INDUCTOR_VALUE) {
            adjust(INDUCTOR,ABSOLUTE,workint2);
            workint2 = workint2 + HI_L_L_INCREMENT;
            if ((workint2 > TOTAL_INDUCTOR_VALUE) && (workint2 < (TOTAL_INDUCTOR_VALUE + HI_L_L_INCREMENT))) {
              workint2 = TOTAL_INDUCTOR_VALUE;
            }
            workint1 = capacitor_values[0];
            tuning_phase_iteration++;
          } else {
            // go to next phase
            tuning_phase = PHASE_7_LAST_RESORT;
            phase_init = 0;
          #ifdef DEBUG_SERVICE_TUNING
          if (debug_mode) {
            Serial.println(F("service_tuning: PHASE_999_THE_END"));
          }
          #endif //DEBUG_SERVICE_TUNING              
            //phase_init = 0;
          }
        }                
        break; //PHASE_6_LO_Z_HI_L_SCAN*/
        
      case PHASE_7_LAST_RESORT: 
        if (!phase_init) {   
          adjust(INDUCTOR, ABSOLUTE, best_match_L);
          adjust(CAPACITOR, ABSOLUTE, best_match_C);
          switch_tuning_mode(best_match_tuning_mode & B00000011);
          tried_best_swr = 1;
          tuning_phase = PHASE_999_THE_END;
        }
        break;
          
      case PHASE_999_THE_END:
        phase_init = 1;
        break;
      
    }  //switch(tuning_phase) 
  }  //(action == TUNE)

  
  
  
}

//-----------------------------------------------------------------------------------------------------


void switch_tuning_mode(byte tuning_mode_to_switch_to){
  
  // start with 1
  
  if (tuning_mode_to_switch_to <= TUNING_MODES) {
    deactivate_all_tuning_modes();
    add_to_command_buffer(tuning_mode_activate_macros[(tuning_mode_to_switch_to-1)]);
    tuning_mode_status[(tuning_mode_to_switch_to-1)] = 1;
  }
  
  
}
//-----------------------------------------------------------------------------------------------------
byte current_tuning_mode(){
 
  for (byte x = 0;x < TUNING_MODES;x++) {
    if (tuning_mode_status[x] == 1) {
      return (x+1);
    }
  }  
  
  return 0;
}
//-----------------------------------------------------------------------------------------------------
void deactivate_all_tuning_modes(){
  
  for (byte x = 0;x < TUNING_MODES;x++) {
    add_to_command_buffer(tuning_mode_deactivate_macros[x]);
    tuning_mode_status[x] = 0;
  }
}
//-----------------------------------------------------------------------------------------------------
byte adjust(byte component, byte adjustment_type, unsigned int amount){
 
  //enum adjust_component {INDUCTOR, CAPACITOR};
  //enum adjust_adjustment_type {STEP, ABSOLUTE, RELATIVE, SINGLE};
  
  // return value:
   
  //   STEP
  //     0 = success
  //     1 = reached upper limit
  //     2 = reached lower limit
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("adjust: entering"));
  #endif  
  
  #ifndef DEBUG_DONT_TUNE 
  
  unsigned int component_bit_map_before = 0;
  unsigned int component_bit_map_after = 0;
  unsigned int component_bit_differences = 0;
  unsigned int workint = 0;
  unsigned int workfloat = 0;
  
  byte component_array_size = 0;
  
  //Serial.print(F("adjust: amount: "));
  //Serial.println(amount);
  
  if (component == INDUCTOR) {
    component_array_size = INDUCTORS;
  } else {
    component_array_size = CAPACITORS;
  }


  for (byte x = component_array_size;x > 0 ;x--) {              // throw the component statuses into a binary number
    component_bit_map_before = component_bit_map_before << 1;
    if (component == INDUCTOR) {
      if (inductor_status[x-1]) {
        component_bit_map_before |= 1;
      }
    } else {
      if (capacitor_status[x-1]) {
        component_bit_map_before |= 1;
      }        
    }  
  }

  switch (adjustment_type) {
    case ABSOLUTE:
      if (amount == 0) {
        component_bit_map_after = 0;
      } else {
        workfloat = amount;       
        if (component == INDUCTOR) {   // TODO - complete this
          for (byte x = INDUCTORS;x > 0;x--) {
            component_bit_map_after = component_bit_map_after << 1;
            if ((workfloat > 0) && (inductor_values[(x-1)]) <= workfloat) {
              workfloat = workfloat - inductor_values[(x-1)];
              component_bit_map_after |= 1;
            }
          }
        } else {
          for (byte x = CAPACITORS;x > 0;x--) {
            component_bit_map_after = component_bit_map_after << 1;
            if ((workfloat > 0) && (capacitor_values[(x-1)]) <= workfloat) {
              workfloat = workfloat - capacitor_values[(x-1)];
              component_bit_map_after |= 1;
            }
          }          
        }
        
      }
      break;
    case SINGLE:
      component_bit_map_after = 1;
      for (byte x = (amount-1);x > 0;x--) {
        component_bit_map_after = component_bit_map_after << 1;
      }
      break;
    case STEP:
      
      // blow out with an error if we're going to rollover
//      if (amount > 0) {
//        if ((component_bit_map_before + amount) >= (2 ^ component_array_size)) { return 1; }
//      } else {
//        if ((component_bit_map_before + amount) < 0) { return 2; }
//      }
      
      component_bit_map_after = component_bit_map_before + amount;  // for a step operation, we just add to the binary number      
      break; 
  }
  
  // push the binary number back into the component status array
  workint = component_bit_map_after;
  for (byte x = 0;x < component_array_size;x++){
     if (component == INDUCTOR) {
       if (workint & 1) {
         inductor_status[x] = 1;
       } else {
         inductor_status[x] = 0;
       }
     } else {
       if (workint & 1) {
         capacitor_status[x] = 1;
       } else {
         capacitor_status[x] = 0;
       }       
     }
     workint = workint >> 1;
  }
  
  // send the changed component macros to the command buffer
  component_bit_differences = component_bit_map_before ^ component_bit_map_after;  // XOR to find what bits changed 
  
  //  Serial.print("before: ");
  //  Serial.print(component_bit_map_before,BIN);
  //  Serial.print(" after: ");
  //  Serial.print(component_bit_map_after,BIN);
  //  Serial.print(" dif: ");
  //  Serial.println(component_bit_differences,BIN);
  
  for (byte x = 0;x < component_array_size;x++) {
    if (component_bit_differences & 1) {  // did we have a component bit change
      if (component == INDUCTOR) {
         if (component_bit_map_after & 1) {
           add_to_command_buffer(inductor_activate_macros[x]);
         } else {
           add_to_command_buffer(inductor_deactivate_macros[x]);
         }
      } else {
         if (component_bit_map_after & 1) {
           add_to_command_buffer(capacitor_activate_macros[x]);
         } else {
           add_to_command_buffer(capacitor_deactivate_macros[x]);
         }            
      }
    }
    component_bit_map_after = component_bit_map_after >> 1;
    component_bit_differences = component_bit_differences >> 1;
  }
  
  return 0;
  
  #endif //#ifndef DEBUG_DONT_TUNE 
}
//-----------------------------------------------------------------------------------------------------

void check_relay_status(){
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("check_relay_status: entering"));
  #endif
  
  if ((relay_status == RELAY_SETTLE) && ((millis() - relay_settle_start_time) >= RELAY_SETTLE_TIME_MS)){
    relay_status = RELAY_NORMAL;
  }
  
}
//-----------------------------------------------------------------------------------------------------
void initiate_relay_settle(){

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("initiate_relay_settle: entering"));
  #endif
  
  relay_status = RELAY_SETTLE;
  relay_settle_start_time = millis();
}

//-----------------------------------------------------------------------------------------------------
unsigned int current_C(){
 
  unsigned int return_value = 0;
  
  for (byte x = 0;x < CAPACITORS;x++) {
    if (capacitor_status[x]) {
      return_value = return_value + capacitor_values[x];
    }
  }
  
  return return_value;
}

//-----------------------------------------------------------------------------------------------------
unsigned int current_L(){
  
  unsigned int return_value = 0;
  
  for (byte x = 0;x < INDUCTORS;x++) {
    if (inductor_status[x]) {
      return_value = return_value + inductor_values[x];
    }
  }
  
  return return_value;
}  
//-----------------------------------------------------------------------------------------------------
void initialize_freq_counter(){
  
  #ifndef DEBUG_NO_FREQ
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("initialize_freq_counter: doing freq reads"));
  #endif  //DEBUG_REAL_DEEP_STUFF
  
  for (byte x = 0;x < 3; x++) {  // do a few reads just to blow the dust out
    current_freq();
  }
  #endif //DEBUG_NO_FREQ

}  

//-----------------------------------------------------------------------------------------------------
void initialize_native_pins(){

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("initialize_native_pins: entering"));
  #endif

  digitalWrite(pin_forward_v, LOW);
  pinMode(pin_forward_v, INPUT);
  digitalWrite(pin_reflected_v, LOW);
  pinMode(pin_reflected_v, INPUT);  
  if (pin_voltage_control) {
    pinMode(pin_voltage_control, OUTPUT);
    digitalWrite(pin_voltage_control, LOW);
  }


  if (pin_tune_lock) {    
    pinMode(pin_tune_lock, INPUT);
    digitalWrite(pin_tune_lock, HIGH);
  }
  
  if (pin_untunable) {pinMode(pin_untunable, OUTPUT);}
  if (pin_tune_in_progress) {pinMode(pin_tune_in_progress, OUTPUT);}  
  if (pin_tuned) {pinMode(pin_tuned, OUTPUT);}
  
  if (pin_manual_tune) {
    pinMode(pin_manual_tune, INPUT);
    digitalWrite(pin_manual_tune, HIGH);
  }
  
  set_indicators(INDICATOR_IDLE);

  #ifdef FEATURE_SLEEP_MODE
  if (pin_awake) {
    pinMode(pin_awake, OUTPUT);
    digitalWrite(pin_awake, HIGH);
  }
  pinMode(pin_wakeup, INPUT);
  digitalWrite(pin_wakeup, HIGH);
  #endif //FEATURE_SLEEP_MODE
  
  
  
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW); 


 
  
  
  
}




//-----------------------------------------------------------------------------------------------------

void initialize_i2c(){

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("initialize_i2c: entering"));
  #endif
  
  Wire.begin();
  for (byte x = 0;x < IO_EXPANDERS;x++) {   
    initialize_io_expander(i2c_expander_addr[x]);
  }

}

//-----------------------------------------------------------------------------------------------------
void initialize_io_expander(byte i2c_addr){
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("initialize_io_expander: entering"));
  #endif

  #ifdef IO_EXPANDER_MCP23008
  Wire.beginTransmission(i2c_addr);
  Wire.write((byte) 0x00);                   // IODIR
  Wire.write((byte) 0x00);                   
  Wire.endTransmission();

  Wire.beginTransmission(i2c_addr);
  Wire.write((byte) 0x09);                   // GPIO
  Wire.write((byte) 0x00);                   
  Wire.endTransmission();
  #endif //IO_EXPANDER_MCP23008

}

//-----------------------------------------------------------------------------------------------------

#ifdef FEATURE_SERIAL
void initialize_serial(){
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.print(F("K3NG Tuner Version "));
  Serial.write(CODE_VERSION);
  Serial.println();
}
#endif //FEATURE_SERIAL

//-----------------------------------------------------------------------------------------------------

void i2c_expander_write(byte expander_number,byte value){


  #ifdef IO_EXPANDER_MCP23008
  Wire.beginTransmission(i2c_expander_addr[expander_number]); 
  Wire.write(0x09);
  Wire.write(value);
  Wire.endTransmission();
  i2c_expander_pins[expander_number] = value;
  #endif //IO_EXPANDER_MCP23008
  #ifdef I2C_POST_WRITE_DELAY
  delay(I2C_POST_WRITE_DELAY);
  #endif //I2C_POST_WRITE_DELAY
}

//-----------------------------------------------------------------------------------------------------
void i2c_pin_write(byte pin_number,byte value){

  // pin numbers go sequential from expander 0, 1, 2, etc.
  // value = 0 or 1 to set or clear bit

  byte pin_value = 1;

  #ifdef IO_EXPANDER_MCP23008
  byte expander_number = (pin_number - 1) / PINS_PER_IO_EXPANDER;
  pin_number = pin_number - (expander_number * PINS_PER_IO_EXPANDER);

  pin_value = pin_value << (pin_number-1);

  if (value) {
    i2c_expander_pins[expander_number] = i2c_expander_pins[expander_number] | pin_value;
  } 
  else {
    i2c_expander_pins[expander_number] = i2c_expander_pins[expander_number] &  (~pin_value);
  }

  #ifdef DEBUG_I2C_PIN_WRITE
  Serial.print("i2c_pin_write: expander: ");
  Serial.print(expander_number);
  Serial.print(" i2c_address: ");
  Serial.print(i2c_expander_addr[expander_number]);
  Serial.print(" pin_number: ");
  Serial.print(pin_number);
  Serial.print(" expander_pins: ");
  Serial.print(i2c_expander_pins[expander_number]);
  Serial.print("    pin_value: ");
  Serial.print(pin_value);
  Serial.println();
  unsigned long start_time  = millis();
  #endif //DEBUG_I2C_PIN_WRITE

  Wire.beginTransmission(i2c_expander_addr[expander_number]); 
  Wire.write(0x09);
  Wire.write(i2c_expander_pins[expander_number]);
  Wire.endTransmission();
  
  #ifdef I2C_POST_WRITE_DELAY
  delay(I2C_POST_WRITE_DELAY);
  #endif //I2C_POST_WRITE_DELAY

  #ifdef DEBUG_I2C_PIN_WRITE
  Serial.print(F("i2c_pin_write: "));
  Serial.print(millis()-start_time);
  Serial.println(F(" mS"));
  #endif //DEBUG_I2C_PIN_WRITE 

  #endif //IO_EXPANDER_MCP23008
}

//-----------------------------------------------------------------------------------------------------

//#ifdef FEATURE_DISPLAY
//void lcd_center_print(char* lcd_char_string, byte row) {
//  clear_lcd_row(row);
//  String lcd_string = lcd_char_string;
//  lcd.setCursor(((LCD_COLUMNS-lcd_string.length())/2),row);
//  lcd.print(lcd_string); 
//}
//#endif //FEATURE_DISPLAY

//-----------------------------------------------------------------------------------------------------

#ifdef FEATURE_DISPLAY
void clear_lcd_row(byte row) {

  for (int x = 0;x < LCD_COLUMNS;x++) {
    lcd.setCursor(x,row);
    lcd.print(" ");
  }
}
#endif //FEATURE_DISPLAY
//-----------------------------------------------------------------------------------------------------

#ifdef FEATURE_SLEEP_MODE
void wakeup() {
  detachInterrupt(0);
}
#endif

//-----------------------------------------------------------------------------------------------------

#ifdef FEATURE_FREQ_COUNTER
unsigned int current_freq() { 
  #ifndef DEBUG_NO_FREQ 
  FreqCounter::f_comp=10;
  FreqCounter::start(FREQ_COUNTER_GATE_TIME);  
  while (FreqCounter::f_ready == 0) {
    //last_measured_frequency = (FreqCounter::f_freq * frequency_counter_calibration * 4);
  }
  last_measured_frequency = (FreqCounter::f_freq * frequency_counter_calibration * 4);
  if (last_measured_frequency > 0) {last_known_good_freq = last_measured_frequency;}
  return last_measured_frequency;
  #else
  return 0;
  #endif //#ifndef DEBUG_NO_FREQ
}
#else
unsigned int current_freq() {
   return 0; 
}
#endif

//-----------------------------------------------------------------------------------------------------
void clear_swr_cache(){

  for (byte x = 0;x < SWR_HISTORY_CACHE_SIZE;x++) {   
    history_swr[x] = 0;
    history_forward_power[x] = 0;
  } 
  current_swr = 0;
}
//-----------------------------------------------------------------------------------------------------
void measure_swr() {

  static unsigned long last_history_sample_time = 0;
  //static byte last_transmit_state;
  byte swr_sample_count = 0;
  float swr = 0;

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("measure_swr: entering"));
  #endif

  //current_swr = 0;

  if (((millis() - last_history_sample_time) >= SWR_SAMPLE_TIME_MS) && (relay_status == RELAY_NORMAL)) {
    forward_voltage = analogRead(pin_forward_v);
    reverse_voltage = analogRead(pin_reflected_v);
    if (forward_voltage > FORWARD_V_TX_SENSE_THRESH) {
      transmit_sense = 1;
      #ifdef DEBUG_MEASURE_SWR
      Serial.print(F("measure_swr: forward_voltage: "));
      Serial.print(forward_voltage);
      Serial.print(F("  transmit_sense: "));
      Serial.println(transmit_sense);     
      #endif //DEBUG_MEASURE_SWR 
      if (forward_voltage > reverse_voltage) {
        swr = /*abs(  */(float(forward_voltage + reverse_voltage) / float(forward_voltage - reverse_voltage)) /*)*/;
      } 
      else {
        swr = 100;
      }

      //forward_power = correct_power((float(forward_voltage)/50) * forward_voltage);
      //forward_power = (((float(forward_voltage)/50) * forward_voltage)*0.5);
      forward_power = correct_power(((float(forward_voltage)/50) * forward_voltage));

      // TODO - may need to filter out insane results here

      for (byte x = (SWR_HISTORY_CACHE_SIZE-1);x > 0;x--) {
        history_swr[x] = history_swr[x-1];
        history_forward_power[x] = history_forward_power[x-1];
      } 
      history_swr[0] = swr;
      history_forward_power[0] = forward_power;
      last_history_sample_time = millis();

      current_swr = 0;

      for (byte x = 0;x < SWR_HISTORY_CACHE_SIZE;x++) {
        if (history_swr[x] > 0) {
          current_swr = current_swr + history_swr[x];
          swr_sample_count++;
        }
      }

      if (swr_sample_count >= MINIMUM_SWR_SAMPLE_COUNT) {
        current_swr = current_swr / float(swr_sample_count);
      } else {
        current_swr = 0;
      }
    } 
    else { //(forward_voltage > 0)
      transmit_sense = 0;
      #ifdef DEBUG_MEASURE_SWR
      Serial.print(F("measure_swr: forward_voltage: "));
      Serial.print(forward_voltage);
      Serial.print(F("  transmit_sense: "));
      Serial.println(transmit_sense);
      #endif //DEBUG_MEASURE_SWR
    }
  }  //((millis() - last_history_sample_time) >= SWR_SAMPLE_TIME_MS)
  
  #ifdef DEBUG_MEASURE_SWR
  delay(250);
  #endif //DEBUG_MEASURE_SWR

}
//-----------------------------------------------------------------------------------------------------
float correct_power (float power_in) {

  return power_in;

  //  float best_difference = 30;
  //  int best_frequency = -1;
  //  float measured_frequency;
  //  
  //  measured_frequency = (last_measured_frequency / 1000000.0);
  //  for (int x = 0; x < number_calibrations_frequencies; x++) {
  //     if (abs(frequencies[x] - measured_frequency) < best_difference) {
  //       best_difference =  abs(frequencies[x] - measured_frequency);
  //       best_frequency = x;
  //     }
  //  }
  //  
  //  //lcd.setCursor(LCD_COLUMNS-5,1);   // debugging
  //
  //  if (best_frequency == -1) {
  //    //lcd.print("----");   // debugging
  //    return power_in;
  //  } else {
  //    //lcd.print(frequencies[best_frequency]);   // debugging
  //    return (power_in * fwd_power_calibarations[best_frequency]);
  //  }
}


//-----------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------

void check_for_dirty_configuration()
{
 
  static unsigned long config_dirty_detect_time = 0;
  static byte last_config_dirty_state = 0;
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("check_for_dirty_configuration: entering"));
  #endif  
  
  
  if (config_dirty && !last_config_dirty_state) {
    config_dirty_detect_time = millis();
    last_config_dirty_state = 1;
  }

  if (config_dirty && last_config_dirty_state && ((millis()-config_dirty_detect_time)> EEPROM_WRITE_WAIT_MS) && (state_tuner == IDLE)) {
    write_settings_to_eeprom(0);
    last_config_dirty_state = 0;
    #ifdef DEBUG_EEPROM
    if (debug_mode) {
      Serial.println(F("check_for_dirty_configuration: wrote config"));
    }
    #endif
  }

}

//-----------------------------------------------------------------------------------------------------

void write_settings_to_eeprom(int initialize_eeprom) {
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("write_settings_to_eeprom: entering"));
  #endif  

  if (initialize_eeprom) {
    configuration.magic_number = EEPROM_MAGIC_NUMBER;
    configuration.tune_buffer_entries = 0;
  }
  configuration.last_L = current_L();
  configuration.last_C = current_C();
  configuration.last_tuning_mode_tx_ant = composite_tuning_mode();
  configuration.tuned_freq = tuned_freq;
  #ifdef FEATURE_RECEIVE_BYPASS
  configuration.receive_bypass = receive_bypass;
  #endif //FEATURE_RECEIVE_BYPASS
  //EEPROM_writeAnything(0,configuration);

  const byte* p = (const byte*)(const void*)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++){
    EEPROM.write(ee++, *p++);  
  }
  
  write_tune_buffer_to_eeprom();
  config_dirty = 0;
}

//-----------------------------------------------------------------------------------------------------

int read_settings_from_eeprom() {

  // returns 0 if eeprom had valid settings, returns 1 if eeprom needs initialized
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("read_settings_from_eeprom: entering"));
  #endif  

  //EEPROM_readAnything(0,configuration);
  
  byte* p = (byte*)(void*)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++){
    *p++ = EEPROM.read(ee++);  
  }
  
  if (configuration.magic_number == EEPROM_MAGIC_NUMBER) {
    #ifdef DEBUG_EEPROM
    Serial.print(F("read_settings_from_eeprom: changing tune last_L: "));
    Serial.print(configuration.last_L);
    Serial.print(F(" last_C: "));
    Serial.print(configuration.last_C);
    Serial.print(F(" tuning_mode: "));
    Serial.println(configuration.last_tuning_mode_tx_ant);
    #endif //DEBUG_EEPROM
    adjust(INDUCTOR, ABSOLUTE, configuration.last_L);
    adjust(CAPACITOR, ABSOLUTE, configuration.last_C);
    switch_tuning_mode(configuration.last_tuning_mode_tx_ant & B00000011);
    switch_antenna(((configuration.last_tuning_mode_tx_ant & B01110000) >> 4)+1);
    switch_transmitter(((configuration.last_tuning_mode_tx_ant & B00001100) >> 2)+1);
    tuned_freq = configuration.tuned_freq;
    #ifdef FEATURE_RECEIVE_BYPASS
    receive_bypass = configuration.receive_bypass;
    #endif
    read_tune_buffer_from_eeprom();
    config_dirty = 0;
    return 0;
  }
  else {
    return 1;
  }

}

//-----------------------------------------------------------------------------------------------------

void initialize_settings_from_eeprom(){

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("initialize_settings_from_eeprom: entering"));
  #endif
  
  byte eeprom_status = 0;
  
  #ifdef DEBUG_EEPROM
  Serial.println(F("initialize_settings_from_eeprom: called"));
  #endif //DEBUG_EEPROM

  eeprom_status = read_settings_from_eeprom();
  if (eeprom_status) { // EEPROM was never initialized
    #ifdef DEBUG_EEPROM
    Serial.println(F("initialize_settings_from_eeprom: initializing eeprom"));
    #endif //DEBUG_EEPROM  
    write_settings_to_eeprom(1);  // go initialize it
  }

}

//-----------------------------------------------------------------------------------------------------

#ifdef DEBUG_RELAY_TEST
void relay_test() {

  //i2c_pin_write(23,1);
  //i2c_pin_write(24,1);
  //i2c_expander_write(2,255);
  //while(1) {}

  while(1) {

    byte random_num = 0;
  
    for (byte i = 1; i < 17; i++) {
      i2c_pin_write(i,1);
      delay(250);
      i2c_pin_write(i,0);
      delay(250);
    }
  
    for (byte i = 23; i < 25; i++) {
      i2c_pin_write(i,1);
      delay(250);
      i2c_pin_write(i,0);
      delay(250);
    }
  
    for (byte i = 1; i < 25; i++) {
      i2c_pin_write(i,1);
    }
    delay(250);
  
    for (byte i = 1; i < 25; i++) {
      i2c_pin_write(i,0);
    }
    delay(250);  
  
    i2c_expander_write(0,255);
    i2c_expander_write(1,255);
    i2c_expander_write(2,255);
    delay(250);
    i2c_expander_write(0,0);
    i2c_expander_write(1,0);
    i2c_expander_write(2,0);
    delay(250);
  
    for (byte i = 1; i < 201; i++) {
      random_num = byte(random(1,25));
      i2c_pin_write(random_num,1);  
      delay(30);
      i2c_pin_write(random_num,0);
      delay(30);
    }
  
  }

} 
#endif //DEBUG_RELAY_TEST

//-----------------------------------------------------------------------------------------------------
void add_to_command_buffer(char* command){

  byte x = 0;
  while (command[x] && (command_buffer_pointer < COMMAND_BUFFER_SIZE)) {
    command_buffer[command_buffer_pointer] = command[x];
    command_buffer_pointer++;
    x++;
  }
  #ifdef DEBUG_COMMAND_BUFFER
  if (command_buffer_pointer >= COMMAND_BUFFER_SIZE) {
    if (debug_mode) {Serial.println(F("add_to_command_buffer: buffer full"));}
  }
  #endif //DEBUG_COMMAND_BUFFER

}
//-----------------------------------------------------------------------------------------------------
byte get_byte_from_command_buffer(){

  if (command_buffer_pointer) {
    byte got_byte = command_buffer[0];
    command_buffer_pointer--;
    if (command_buffer_pointer) {
      for (byte x = 0;x < command_buffer_pointer;x++) {
        command_buffer[x] = command_buffer[x+1];
      }
    }
    return got_byte;
  } 
  else {
    #ifdef DEBUG_COMMAND_BUFFER
    if (debug_mode) {Serial.println(F("get_byte_from_command_buffer: called with empty buffer"));}
    #endif //DEBUG_COMMAND_BUFFER    
    return 0;
  }

}
//-----------------------------------------------------------------------------------------------------
byte get_command_buffer_two_byte_number(){

  return ((get_byte_from_command_buffer()-48) * 10) + (get_byte_from_command_buffer() - 48);

} 
//-----------------------------------------------------------------------------------------------------
void process_command_buffer(){ 

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("process_command_buffer: entering"));
  #endif
  
  byte byte_from_buffer = 0;
  //byte action = 0;
  static byte command_buffer_status = NORMAL;
  static unsigned long last_relay_action_time = 0;

  if (command_buffer_status == NORMAL) {

    while (command_buffer_pointer) {

      byte_from_buffer = get_byte_from_command_buffer();

      #ifdef DEBUG_COMMAND_BUFFER
      if (debug_mode) {
        Serial.print(F("process_command_buffer byte: "));
        Serial.write(byte_from_buffer);
        Serial.write(command_buffer[0]);
        Serial.write(command_buffer[1]);
        Serial.println();
      }
      #endif //DEBUG_COMMAND_BUFFER

      switch(byte_from_buffer) {
      case '+': // active native pin, no delay
        //Serial.println(get_command_buffer_two_byte_number());
        //pinMode(get_command_buffer_two_byte_number(),OUTPUT);
        digitalWrite(get_command_buffer_two_byte_number(),HIGH);
        break;
      case '-': // deactivate native pin, no delay
        //pinMode(get_command_buffer_two_byte_number(),OUTPUT);
        digitalWrite(get_command_buffer_two_byte_number(),LOW);
        break;
      case '!': // activate i2c pin, no delay
        i2c_pin_write(get_command_buffer_two_byte_number(),1);
        break;
      case '.': // deactivate i2c pin, no delay    
        i2c_pin_write(get_command_buffer_two_byte_number(),0);
        break;   
      }


      initiate_relay_settle();

    }  //if (command_buffer_pointer)  
  } // if (command_buffer_status == NORMAL)


}

//-----------------------------------------------------------------------------------------------------

//void macro_test() {
//
//  for (byte x = 0;x < 8;x++) {
//    add_to_command_buffer(inductor_activate_macros[x]);
//    process_command_buffer();
//    delay(250);
//
//  }
//  for (byte x = 0;x < 8;x++) {
//    add_to_command_buffer(inductor_deactivate_macros[x]);
//    process_command_buffer();
//    delay(250);
//  }   
//  for (byte x = 0;x < 8;x++) {
//    add_to_command_buffer(capacitor_activate_macros[x]);
//    process_command_buffer();
//    delay(250);
//  }
//  for (byte x = 0;x < 8;x++) {
//    add_to_command_buffer(capacitor_deactivate_macros[x]);
//    process_command_buffer();
//    delay(250);
//  } 
//
//}

//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_RECEIVE_BYPASS
void check_receive_bypass() {

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("check_receive_bypass: entering"));
  #endif
  
  static unsigned long last_transmit_sense_time = 0;
  static float last_L = 0;
  static float last_C = 0;
  static byte last_tuning_mode_status[TUNING_MODES];
  static byte last_tuning_mode = 0;

  if (receive_bypass) {  
    if (transmit_sense) {
      last_transmit_sense_time = millis();
      if (in_receive_bypass) {  // come out of receive bypass
        adjust(CAPACITOR,ABSOLUTE,last_C);
        adjust(INDUCTOR,ABSOLUTE,last_L);
        switch_tuning_mode(last_tuning_mode & B00000011); 
        in_receive_bypass = 0;
      } 
    } else {
      if ((!in_receive_bypass) && ((millis() - last_transmit_sense_time) > RECEIVE_BYPASS_DELAY)) {
        last_L = current_L();
        last_C = current_C();
        last_tuning_mode = current_tuning_mode();
        adjust(CAPACITOR,ABSOLUTE,0);
        adjust(INDUCTOR,ABSOLUTE,0);
        deactivate_all_tuning_modes();
        in_receive_bypass = 1;
      }
    }  
  }
  
}
#endif //FEATURE_RECEIVE_BYPASS

//-----------------------------------------------------------------------------------------------------
void tune_buffer_clear(){
  
  for (byte x = 0;x < TUNE_BUFFER_SIZE;x++){
    tune_buffer[x].freq = 0;
    tune_buffer[x].L = 0;
    tune_buffer[x].C = 0;
    tune_buffer[x].tuning_mode = 0;
    tune_buffer[x].swr = 0;
  }
  configuration.tune_buffer_entries = 0;
  config_dirty = 1;
  
  #ifdef DEBUG_TUNE_BUFFER
  Serial.println(F("tune_buffer_clear: buffer cleared"));
  #endif //DEBUG_TUNE_BUFFER
}
//-----------------------------------------------------------------------------------------------------
byte composite_tuning_mode(){
 
  return(current_tuning_mode() | ((current_antenna-1)<<4) | ((current_transmitter-1)<<2));
  
}
//-----------------------------------------------------------------------------------------------------
void tune_buffer_add(unsigned int frequency){
   
  byte found_a_match = 0;
    
  tuned_freq = frequency;
  #ifdef DEBUG_TUNE_BUFFER
  if (debug_mode) {Serial.print(F("tune_buffer_add: tuned_freq:"));Serial.println(tuned_freq);}
  #endif //DEBUG_TUNE_BUFFER
  
  for (byte x = 0;((x < configuration.tune_buffer_entries) && (!found_a_match));x++){  // make sure this isn't a duplicate
    if ((abs(tune_buffer[x].freq - frequency) <= TUNE_BUFFER_ADD_MATCH_THRESH_KHZ)/* && (tune_buffer[x].L == current_L()) && (tune_buffer[x].C == current_C()) && (tune_buffer[x].tuning_mode == current_tuning_mode())*/) {
      //if ((tune_buffer[x].L + tune_buffer[x].C + tune_buffer[x].tuning_mode) == (current_L() + current_C() + current_tuning_mode())) {
      if ((tune_buffer[x].L == current_L()) && (tune_buffer[x].C == current_C()) && 
      (tune_buffer[x].tuning_mode == composite_tuning_mode())) {  
        found_a_match = 1;
        #ifdef DEBUG_TUNE_BUFFER
        if (debug_mode) {
          Serial.print(F("tune_buffer_add: found existing match:"));
          Serial.println(x);
        }
        #endif //DEBUG_TUNE_BUFFER        
      }
    }
  }
  
  if ((tuning_phase != PHASE_2_CHECK_TUNE_BUFFER) && (!found_a_match)) {  // if this tune didn't come from the buffer, go ahead and add it
   
    for (byte x = (TUNE_BUFFER_SIZE-1);x > 0;x--){  // move everything down one
      tune_buffer[x] = tune_buffer[x-1];
    }
    tune_buffer[0].freq = frequency;
    tune_buffer[0].L = current_L();
    tune_buffer[0].C = current_C();
    tune_buffer[0].tuning_mode = composite_tuning_mode();
    tune_buffer[0].swr = current_swr * 100;
    config_dirty = 1;
    if (configuration.tune_buffer_entries < TUNE_BUFFER_SIZE) {
      configuration.tune_buffer_entries++;
    } else {
      #ifdef DEBUG_TUNE_BUFFER
      if (debug_mode) {
        Serial.println(F("tune_buffer_add: tune buffer full"));
      }
      #endif //DEBUG_TUNE_BUFFER               
    }
  }
    
}
//-----------------------------------------------------------------------------------------------------
void update_last_tuning_mode_tx_ant(){

  configuration.last_tuning_mode_tx_ant = composite_tuning_mode();
  config_dirty = 1;
  
}
//-----------------------------------------------------------------------------------------------------
void switch_antenna(byte antenna_number){
  
  current_antenna = antenna_number;
  
 
  
  //TODO: command to switch
}
//-----------------------------------------------------------------------------------------------------
void switch_transmitter(byte transmitter_number){
  
  current_transmitter = transmitter_number;
  
  //TODO: command to switch
  
}
//-----------------------------------------------------------------------------------------------------

void check_serial(){
  
  #ifdef FEATURE_SERIAL
  
  byte incoming_serial_byte = 0;

  #ifdef FEATURE_COMMAND_LINE_INTERFACE
  if (Serial.available() > 0) {
    incoming_serial_byte = Serial.read();
    Serial.write(incoming_serial_byte);
    if (incoming_serial_byte == 10) { return; }  // ignore carriage returns
    if (incoming_serial_byte != 13) {               // if it's not a carriage return, add it to the buffer
      cli_command_buffer[cli_command_buffer_index] = incoming_serial_byte;
      cli_command_buffer_index++;
    } else {                       // we got a carriage return, time to get to work on the command
      if ((cli_command_buffer[0] > 96) && (cli_command_buffer[0] < 123)) { // uppercase it
        cli_command_buffer[0] = cli_command_buffer[0] - 32;
      }
      switch (cli_command_buffer[0]) {                 // look at the first character of the command
        case 'A':
          if (cli_command_buffer_index == 2) {
            switch_antenna((cli_command_buffer[1]-48));
            update_last_tuning_mode_tx_ant();
            Serial.println();
          } else {
            Serial.println("?");
          }
          break;
        case 'X':
          if (cli_command_buffer_index == 2) {
            switch_transmitter((cli_command_buffer[1]-48));
            update_last_tuning_mode_tx_ant();
            Serial.println();
          } else {
            Serial.println("?");
          }
          break;          
      
      
      
        #ifdef FEATURE_SERIAL_HELP
        case '?': print_serial_help(); break;           // ? - print help
        #endif //FEATURE_SERIAL_HELP
        #ifdef FEATURE_RECEIVE_FREQ_AUTOSWITCH
        case 'W':
          Serial.print(F("\n\rRX Autoswitch O"));
          if (receive_freq_autoswitch_active) {
            receive_freq_autoswitch_active = 0;
            Serial.println(F("ff"));
          } else {
            receive_freq_autoswitch_active = 1;
            Serial.println(F("n"));
          }
          break;
        #endif //FEATURE_RECEIVE_FREQ_AUTOSWITCH
        //#ifdef DEBUG_TUNE_BUFFER
        case 'B': print_tune_buffer(); break;
        case 'C': tune_buffer_clear(); break;        
        //#endif //DEBUG_TUNE_BUFFER
        case 'D':                                       // D - toggle debug mode
          Serial.print(F("\n\rDebug O"));
          if (debug_mode) {
            Serial.println(F("ff"));
            debug_mode = 0;
          } else {
            Serial.println(F("n"));
            debug_mode = 1;
          }
          break; 
        case 'L':                                       // L - tune lock
          lock_invoke = 1;
          Serial.println(F("\n\rTune Lock"));
          break;
        case 'M':                                       // M - manual tune
          manual_tune_invoke = 1;  
          Serial.println(F("\n\rManual Tune"));
          break;
        case 'P':                                       // P - periodic print status
          Serial.print(F("\n\rPeriodic Print Status O"));
          if (periodic_print_status) {
            Serial.println(F("ff"));
            periodic_print_status = 0;
          } else {
            Serial.println(F("n"));
            periodic_print_status = 1;
          }
          break;      
        #ifdef FEATURE_RIG_INTERFACE
        case 'T': rig_tune(1); break;
        case 'R': rig_tune(0); break;
        #endif //FEATURE_RIG_INTERFACE
        case 'S': print_status(); break;                // S - print status
        case 'U':                                       // U - tune unlock
          if (state_tuner == LOCK) {unlock_invoke = 1;}
          Serial.println(F("\n\rTune Unlock"));
          break;
        #ifdef DEBUG_EEPROM    
        case 'V': read_tune_buffer_from_eeprom(); break;
        case 'Y': write_tune_buffer_to_eeprom(); break;    
        case 'O': clear_out_eeprom(); break;
        case 'E': print_eeprom(); break;
        #endif //DEBUG_EEPROM
        #ifdef FEATURE_SLEEP_MODE
        case 'Z':                                       // Z - toggle sleep mode
          Serial.print(F("Sleep "));
          if (sleep_disabled) {
            Serial.print(F("En"));
            sleep_disabled = 0;
          } else {
            Serial.print(F("Dis"));
            sleep_disabled = 1;
          }
          Serial.println(F("abled"));
          break;
        #endif //FEATURE_SLEEP_MODE     
        default: 
          Serial.println(F("\n\rError"));
          #ifdef DEBUG_SERIAL
          if (debug_mode) {
            Serial.print(F("check_serial: command_buffer_index: "));
            Serial.println(cli_command_buffer_index);
            for (int debug_x = 0; debug_x < cli_command_buffer_index; debug_x++) {
              Serial.print(F("check_serial: command_buffer["));
              Serial.print(debug_x);
              Serial.print(F("]: "));
              Serial.print(cli_command_buffer[debug_x]);
              Serial.print(F(" "));
              Serial.write(cli_command_buffer[debug_x]);
              Serial.println();
            }
          }
          #endif //DEBUG_SERIAL
      }
      cli_command_buffer_index = 0;
      cli_command_buffer[0] = 0;  
      cli_command_buffer[1] = 0;    
    }
  } //if (Serial.available() > 0)
  #endif //FEATURE_COMMAND_LINE_INTERFACE

  #ifdef FEATURE_RIG_CONTROL_PASS_THROUGH
  if (Serial.available()){
    if (rig[current_rig]->rig_status == PORT_OPEN) {
      switch (rig[current_rig]->command_submitted) {
        case NONE:
          rig[current_rig]->activate_pass_through();
          break;
        case CONTROL_PASS_THROUGH:
          rig[current_rig]->write_byte(Serial.read());
          rig[current_rig]->last_action_time = millis(); 
          break;
      }
    } else {  // if the rig port is closed, just discard the byte
      Serial.read();  
    }
  }
  #endif //FEATURE_RIG_CONTROL_PASS_THROUGH
  
  #endif //FEATURE_SERIAL
  
}

//-----------------------------------------------------------------------------------------------------
byte transmitter_number_from_tuning_mode(byte tuning_mode_in){
  
 return (((tuning_mode_in>>2) & B00000011)+1);
  
}
//-----------------------------------------------------------------------------------------------------
byte antenna_number_from_tuning_mode(byte tuning_mode_in){
  
 return (((tuning_mode_in>>4) & B00000111)+1);
  
}


//-----------------------------------------------------------------------------------------------------
//#ifdef DEBUG_TUNE_BUFFER
#ifdef FEATURE_SERIAL
void print_tune_buffer() {
  
  Serial.println(F("\n\rTune Buffer\n\r#:\tF\tL\tC\tTMode\tTX\tAnt\tSWR"));
  for (byte x = 0;x < configuration.tune_buffer_entries;x++){
    Serial.print(x);
    Serial.print(F(":\t"));
    Serial.print(tune_buffer[x].freq);
    Serial.print(F(" \t"));
    Serial.print(float(tune_buffer[x].L)/100);
    Serial.print(F(" \t"));
    Serial.print(tune_buffer[x].C);
    Serial.print(F(" \t"));
    Serial.print(tuning_mode_names[(tune_buffer[x].tuning_mode & B00000011) -1]);
    Serial.print(F(" \t"));
    Serial.print(transmitter_number_from_tuning_mode(tune_buffer[x].tuning_mode));    
    Serial.print(F(" \t"));
    Serial.print(antenna_number_from_tuning_mode(tune_buffer[x].tuning_mode));
    Serial.print(F(" \t"));
    Serial.println(float(tune_buffer[x].swr)/100);
  }
}
#endif //FEATURE_SERIAL
//#endif //DEBUG_TUNE_BUFFER
//-----------------------------------------------------------------------------------------------------

void print_status(){
  
  #ifdef DEBUG_STATUS_DUMP

        
  Serial.print(F("\n\rStatus "));
  /*
  void* HP = malloc(4);
  if (HP) {
    free (HP);
  }
  unsigned long free = (unsigned long)SP - (unsigned long)HP;
  if (free > 2048) {
    free = 0;
  }
  Serial.print((unsigned long)free,DEC);    
  */
  /*Serial.print(CODE_VERSION);*/
  Serial.print("  ");
  /*Serial.print(F("state: "));*/
  switch (state_tuner) {
    case IDLE: 
      Serial.print(F("IDLE")); 
      break;
    case TUNING: 
      Serial.print(F("TUNING")); 
      break;
    case LOCK: 
      Serial.print(F("LOCK")); 
      break;
    case IDLE_UNTUNABLE:
      Serial.print(F("UNTUNABLE"));
      break;
    case PENDING_IDLE_NO_TX:
      Serial.print(F("PEND_IDLE_NO_TX"));
      break;
    case PENDING_IDLE_SWR_OK:
      Serial.print(F("PEND_IDLE_SWR_OK"));
      break;      
    default: 
      Serial.print(F("undef")); 
      break;
  }
  /*Serial.write(9);
  if (!tuned) {
    Serial.print(F("NOT_"));
  }
  Serial.print(F("TUNED"));*/
  Serial.print("\tTX_O");
  if (transmit_sense) {
    Serial.print(F("N"));
  } 
  else {
    Serial.print(F("FF"));
  }
  Serial.print("\t");
  /*Serial.print(F("tune phase: "));*/
  
  //PHASE_1_STRAIGHT_THRU, PHASE_2_CHECK_TUNE_BUFFER, PHASE_3_HI_Z_SCAN, PHASE_4_LO_Z_SCAN, PHASE_5_HI_Z_HI_L_SCAN, PHASE_6_LO_Z_HI_L_SCAN,
//PHASE_3_L_PROFILE, PHASE_4_C_SCAN, PHASE_999_THE_END
  
  Serial.print(F("PHASE_"));
  switch (tuning_phase) {
    case PHASE_1_STRAIGHT_THRU: 
      Serial.print(F("1_STRAIGHT_THRU")); 
      break;
    case PHASE_2_CHECK_TUNE_BUFFER:
      Serial.print(F("2_CHECK_TUNE_BUFFER"));
      break;
    case PHASE_3_HI_Z_SCAN: 
      Serial.print(F("3_HI_Z_SCAN")); 
      break;   
    case PHASE_4_LO_Z_SCAN: 
      Serial.print(F("4_LO_Z_SCAN")); 
      break;   
    case PHASE_5_HI_Z_HI_L_SCAN:
      Serial.print(F("5_HI_Z_HI_L_SCAN"));
      break;     
    case PHASE_6_LO_Z_HI_L_SCAN:
      Serial.print(F("6_LO_Z_HI_L_SCAN"));
      break;    
    case PHASE_7_LAST_RESORT:
      Serial.print(F("7_LAST_RESORT"));
      break;
    case PHASE_999_THE_END:
      Serial.print(F("999_THE_END"));
      break;          
    default: 
      /*Serial.print(F("undef")); */
      break;
  }
  /*Serial.print("\t\t");
  Serial.print(F(" iter: "));
  Serial.print(tuning_phase_iteration);*/
  Serial.print("\t");
  /*Serial.print(F("relay_status: "));
  switch (relay_status) {
    case RELAY_NORMAL: 
      Serial.println(F("NORMAL"));
      break;
    case RELAY_SETTLE: 
      Serial.println(F("SETTLE"));
      break;
  }  */    
  /*Serial.print(F("  swr_target: "));*/
  Serial.print(F("TARGET_SWR_"));
  switch (swr_target) {
    case TARGET_SWR_GOOD:
      Serial.print(F("GOOD"));
      break;
    case TARGET_SWR_ACCEPTABLE:
      Serial.print(F("ACCEPTABLE"));
      break;
  }
  Serial.print(F(" Tuned F: "));
  Serial.println(tuned_freq);
  Serial.print(F("swr: "));
  Serial.print(current_swr);
  Serial.print(F("\tfor: "));
  Serial.print(forward_voltage);
  //Serial.print(analogRead(pin_forward_v));
  Serial.print(F("\trev: "));
  Serial.print(reverse_voltage);
  #ifdef FEATURE_FREQ_COUNTER
  Serial.print(F("\tfreq: "));
  Serial.print(current_freq());
  Serial.print(F("\ttx: "));
  Serial.print(current_transmitter);
  Serial.print(F("\tant: "));
  Serial.print(current_antenna);
  #endif //FEATURE_FREQ_COUNTER  
  #ifdef DEBUG_STATUS_DUMP_SWR_CACHE  
  Serial.print(F("  swr cache:"));
  for (byte x = 0;x < SWR_HISTORY_CACHE_SIZE;x++) {
    if (history_swr[x]) {     
      Serial.print(x);
      Serial.print(F(":"));
      Serial.print(history_swr[x]);
      Serial.print(F("|"));
    }
  }
  #endif //DEBUG_STATUS_DUMP_SWR_CACHE
  
  Serial.println();
  
  //Serial.println(analogRead(pin_reflected_v));
  Serial.print(F("L: "));
  Serial.print(float(current_L())/100);
  for (byte x = 0;x < INDUCTORS;x++) {
    if (inductor_status[x]) {
      Serial.print("*");
      Serial.print(x+1);
      Serial.print("*");
    } 
    else {
      Serial.print(" ");
      Serial.print(x+1);
      Serial.print(" ");
    }        
  }
  Serial.print(F("\t"));
  
  Serial.print(F("\tC: "));
  Serial.print(current_C());
  for (byte x = 0;x < CAPACITORS;x++) {
    if (capacitor_status[x]) {
      Serial.print(" *");
      Serial.print(x+1);
      Serial.print("*");
    } 
    else {
      Serial.print("  ");
      Serial.print(x+1);
      Serial.print(" ");
    }        
  }
  Serial.print(F("\t"));  
  /*Serial.print(F("\n\rTuning mode: "));*/
  for (byte x = 0;x < TUNING_MODES;x++) {
    if (tuning_mode_status[x]) {
      Serial.print(" *");
      Serial.print(tuning_mode_names[x]);
      Serial.print("* ");
    } 
    else {
      Serial.print("  ");
      Serial.print(tuning_mode_names[x]);
      Serial.print("  ");        
    }
  }
  Serial.print(F("best match: "));
  Serial.print(best_match_swr);
  Serial.print(F(" L:"));
  Serial.print(best_match_L);
  Serial.print(F(" C:"));
  Serial.print(best_match_C);
  Serial.print(F(" "));
  Serial.print(best_match_tuning_mode);
  if (tried_best_swr) {
    Serial.println(F(" TRIED"));
  } else {
    Serial.println(F(" NOT_TRIED"));
  }


  
  #ifdef FEATURE_RIG_INTERFACE
  for (byte x = 0;x < RIGS;x++){
    Serial.print(F("Rig "));
    Serial.print(x);
    switch(rig[x]->rig_status){
      case(PORT_CLOSED):
        Serial.print(F("\tCLOSED"));
        break;
      case(PORT_OPEN):
        Serial.print(F("\tOPEN"));
        break; 
    }
    switch(rig[x]->command_submitted){
      case(NONE):
        Serial.print(F("\t--"));
        break;
      case(INITIALIZE):
        Serial.print(F("\tINITIALIZE"));
        break;
      case(FREQUENCY_REQUEST):
        Serial.print(F("\tFREQ_REQUEST"));
        break;
    }
    Serial.print(F("\ttouts: "));
    Serial.print(rig[x]->timeouts);
    Serial.print(F("\trx_buff: "));
    Serial.print(rig[x]->receive_buffer_bytes);
    Serial.print(F("\tfreq: "));
    Serial.print(rig[x]->frequency);
    Serial.print("\t");
    switch (rig[x]->mode){
      case LSB: Serial.println(F("LSB")); break; 
      case USB: Serial.println(F("USB")); break;
      case CW: Serial.println(F("CW")); break;
      case CWR: Serial.println(F("CWR")); break;
      case AM: Serial.println(F("AM")); break;
      case WFM: Serial.println(F("WFM")); break;
      case FM: Serial.println(F("FM")); break;
      case DIG: Serial.println(F("DIG")); break;
      case PKT: Serial.println(F("PKT")); break;
      default: Serial.println(F("?")); break;
    }
  }
  #endif //FEATURE_RIG_INTERFACE


  
//    Serial.print(F("\n\rio: "));
//    byte work_byte = 0;
//    
//    for (byte x = 0;x < IO_EXPANDERS;x++) {
//      work_byte = i2c_expander_pins[x];
//      for (byte y = 8;y > 0;y--) {
//        if (work_byte & 128) {
//          Serial.print("1");
//        } else {
//          Serial.print("0");
//        }
//        work_byte = work_byte << 1;
//      }
//      Serial.print(" ");
//    }



  //Serial.println();
  //Serial.println();
  /*Serial.print(F("b free\n\r-------------------------------------\n\r"));*/
  delay(DEBUG_STATUS_DUMP_DELAY);      
  #endif //DEBUG_STATUS_DUMP

}  
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_SERIAL_HELP
void print_serial_help(){

  Serial.println(F("\r\nCLI Help\r\n"));
  Serial.println(F("A#\tSwitch Ant"));
  //#ifdef DEBUG_TUNE_BUFFER
  Serial.println(F("B\tTune Buffer"));
  Serial.println(F("C\tClear Tune Buffer"));
  //#endif //DEBUG_TUNE_BUFFER
  Serial.println(F("D\tDebug"));
  Serial.println(F("L\tTune Lock"));
  Serial.println(F("M\tManual Tune"));
  #ifdef DEBUG_STATUS_DUMP
  Serial.println(F("P\tPeriodic Status"));
  Serial.println(F("S\tStatus"));
  #ifdef FEATURE_RIG_INTERFACE
  Serial.println(F("T\tRig Tune"));
  Serial.println(F("R\tRig Tune Off"));
  #endif //FEATURE_RIG_INTERFACE  
  #endif //DEBUG_STATUS_DUMP
  Serial.println(F("U\tTune Unlock"));
  #ifdef FEATURE_RECEIVE_FREQ_AUTOSWITCH
  Serial.println(F("W\tAutoswitch"));
  #endif //FEATURE_RECEIVE_FREQ_AUTOSWITCH  
  Serial.println(F("X#\tSwitch TX"));
  #ifdef DEBUG_EEPROM    
  Serial.println(F("V:\tRead Tune Buffer from EEPROM"));
  Serial.println(F("Y:\tWrite Tune Buffer to EEPROM"));
  Serial.println(F("O:\tClear Out EEPROM"));
  Serial.println(F("E:\tPrint Dump EEPROM"));
  #endif //DEBUG_EEPROM
  #ifdef FEATURE_SLEEP_MODE
  Serial.println(F("Z\tSleep Enable/Disable"));
  #endif //FEATURE_SLEEP_MODE
  
  
}
#endif
//-----------------------------------------------------------------------------------------------------

#ifdef FEATURE_RIG_INTERFACE
void service_rigs(){
  
   for (byte x = 0;x < RIGS;x++){
     if (rig[x]->rig_status == PORT_OPEN){
       rig[x]->service();
     }
   }
  
}
#endif //FEATURE_RIG_INTERFACE
//-----------------------------------------------------------------------------------------------------

#ifdef FEATURE_RIG_INTERFACE
void get_rig_frequencies(){
  
  // periodically request current frequencies from each active rig
 
  
  for(byte x = 0;x < RIGS;x++){
    if (((millis() - rig_last_freq_request_time[x]) >= RIG_FREQ_REFRESH_TIME_MS) && (rig[x]->rig_status == PORT_OPEN) && (rig[x]->command_submitted == NONE)){
      rig[x]->request_frequency();  
      rig_last_freq_request_time[x] = millis();
    }
  }  
  
}
#endif //FEATURE_RIG_INTERFACE
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_RIG_INTERFACE
void initialize_rigs(){
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("initialize_rigs: entering"));
  #endif  
 
  for (byte x = 0;x < RIGS;x++) {
    rig[x]->initialize(rig_baud[x],0);
  }  
  
}
 #endif //FEATURE_RIG_INTERFACE
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_RIG_INTERFACE
#ifdef FEATURE_RECEIVE_FREQ_AUTOSWITCH
void check_receive_freq_autoswitch(){
  
  // see if the rig frequency has changed, and if we have a matching tune in the tune buffer, use it
  
  static unsigned long last_switch_time = 0;
  static unsigned int last_frequency = 0;
  static unsigned long last_freq_change_time = 0;
  int best_tune_buffer_match = -1;
  unsigned int best_tune_buffer_match_diff = 65000;

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("check_receive_freq_autoswitch: entering"));
  #endif

  if (receive_freq_autoswitch_active){
    
    if (transmit_sense) {
      last_freq_change_time = 0;
      return;
    }
  
  
    // has the rig frequency changed?
    if ((rig[current_rig]->rig_status == PORT_OPEN) && (state_tuner == IDLE) && (abs(rig[current_rig]->frequency - tuned_freq) >= RX_FREQ_AUTOSWITCH_FREQ_THRESH_KHZ)
    && ((millis() - last_switch_time) >= RX_FREQ_AUTOSWITCH_TIME_THRESH) && (last_freq_change_time == 0) && (rig[current_rig]->frequency > 0)) {
      last_freq_change_time = millis();
      last_frequency = rig[current_rig]->frequency;
      #ifdef DEBUG_RECEIVE_FREQ_AUTOSWITCH
      if (debug_mode) {
        Serial.print(F("check_receive_freq_autoswitch: freq change detected - diff:")); 
        Serial.println(abs(rig[current_rig]->frequency - tuned_freq));
      }
      #endif //DEBUG_RECEIVE_FREQ_AUTOSWITCH    
    }
    
    //if we previously detected a frequency change threshold, did the freq change again?  if, so reset the timer
    if ((last_freq_change_time > 0) & (last_frequency != rig[current_rig]->frequency)) {
      last_freq_change_time = millis();
      last_frequency = rig[current_rig]->frequency;
    }
    
    // has the frequency changed and has there been enough time since the last freq change and the last tune switch/check?
    if ((rig[current_rig]->rig_status == PORT_OPEN) && (state_tuner == IDLE) && (last_freq_change_time > 0) && ((millis() - last_freq_change_time) >= RX_FREQ_AUTOSWITCH_WAIT_TIME_FREQ_CHANGE)) {
      #ifdef DEBUG_RECEIVE_FREQ_AUTOSWITCH
      if (debug_mode) {
        Serial.println(F("check_receive_freq_autoswitch: autoswitch search triggered")); 
      }
      #endif //DEBUG_RECEIVE_FREQ_AUTOSWITCH
      
      // find best match      
      for (byte x = 0;x < TUNE_BUFFER_SIZE;x++){
        if ((tune_buffer[x].freq > 0) && (abs(tune_buffer[x].freq - rig[current_rig]->frequency) <= RX_FREQ_AUTOSWITCH_TUNE_MATCH_KHZ_THRESH)&& 
          (abs(tune_buffer[x].freq - rig[current_rig]->frequency) < best_tune_buffer_match_diff)) {
            best_tune_buffer_match = x;
            best_tune_buffer_match_diff = abs(tune_buffer[x].freq - rig[current_rig]->frequency);
            #ifdef DEBUG_RECEIVE_FREQ_AUTOSWITCH
            if ((debug_mode) && (tune_buffer[x].freq > 0)) {
              Serial.print(F("check_receive_freq_autoswitch: tune buffer: "));
              Serial.print(x);
              Serial.print(F(" diff: "));
              Serial.println(abs(tune_buffer[x].freq - rig[current_rig]->frequency));
            }
            #endif //DEBUG_RECEIVE_FREQ_AUTOSWITCH
        }
      }      
      if (best_tune_buffer_match != -1) { // did we find a match below the khz diff threshold?
        if ((tune_buffer[best_tune_buffer_match].C != current_C()) || (tune_buffer[best_tune_buffer_match].tuning_mode != current_tuning_mode()) || (tune_buffer[best_tune_buffer_match].L != current_L())) {
          adjust(CAPACITOR,ABSOLUTE,tune_buffer[best_tune_buffer_match].C);
          adjust(INDUCTOR,ABSOLUTE,tune_buffer[best_tune_buffer_match].L);
          switch_tuning_mode(tune_buffer[best_tune_buffer_match].tuning_mode & B00000011);
          tuned_freq = rig[current_rig]->frequency;
          set_indicators(TUNED);
          #ifdef FEATURE_DISPLAY
          update_static_screen(1);
          #endif //FEATURE_DISPLAY
          #ifdef DEBUG_RECEIVE_FREQ_AUTOSWITCH
          if (debug_mode) {
            Serial.print(F("check_receive_freq_autoswitch: switched to tune buffer entry ")); 
            Serial.println(best_tune_buffer_match);
          }
          #endif //DEBUG_RECEIVE_FREQ_AUTOSWITCH
        } else {
          #ifdef DEBUG_RECEIVE_FREQ_AUTOSWITCH
          if (debug_mode) {
            Serial.println(F("check_receive_freq_autoswitch: already in best tune ")); 
          }  
          #endif //DEBUG_RECEIVE_FREQ_AUTOSWITCH          
        }   
      }    
      last_switch_time = millis();
      //last_search_freq = rig[current_rig].frequency;
      last_freq_change_time = 0;
    }
  } // if (receive_freq_autoswitch_active)



}
#endif //FEATURE_RECEIVE_FREQ_AUTOSWITCH
#endif //FEATURE_RIG_INTERFACE
//-----------------------------------------------------------------------------------------------------
void write_tune_buffer_to_eeprom(){
  
  unsigned int eeprom_location = 0;
  unsigned int i = 0;
  
  for (byte x = 0;((x < configuration.tune_buffer_entries) && (x < TUNE_BUFFER_SIZE));x++){
    eeprom_location = (sizeof(configuration)+(sizeof(tune_buffer[0])*x));
    if (eeprom_location + sizeof(tune_buffer[0]) < EEPROM_BYTES) {
      //EEPROM_writeAnything(eeprom_location,tune_buffer[x]);     
      const byte* p = (const byte*)(const void*)&tune_buffer[x];      
      for (i = 0; i < sizeof(tune_buffer[x]); i++){
        EEPROM.write(eeprom_location++, *p++);      
      }     
    } else {
      #ifdef DEBUG_EEPROM
      if (debug_mode){
        Serial.println(F("write_tune_buffer_to_eeprom: hit end of EEPROM"));
      }
      #endif
      x = TUNE_BUFFER_SIZE;
    }
  }
  
  #ifdef DEBUG_EEPROM
  print_eeprom();
  #endif //DEBUG_EEPROM
  
}
//-----------------------------------------------------------------------------------------------------
void read_tune_buffer_from_eeprom(){
  
  unsigned int eeprom_location = 0;
  unsigned int i = 0;
  
  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("read_tune_buffer_from_eeprom: entering"));
  #endif
 
  for (byte x = 0;x < configuration.tune_buffer_entries;x++){ 
    eeprom_location = (sizeof(configuration)+(sizeof(tune_buffer[0])*x));
    if (eeprom_location < EEPROM_BYTES) {
      //EEPROM_readAnything(eeprom_location,tune_buffer[x]);
      byte* p = (byte*)(void*)&tune_buffer[x];
      for (i = 0; i < sizeof(tune_buffer[x]); i++){
        *p++ = EEPROM.read(eeprom_location++);      
      }
    }
  }
  
}
//-----------------------------------------------------------------------------------------------------
#ifdef DEBUG_EEPROM
void print_eeprom(){
 
  byte y = 0;
  unsigned int x = 0;
  
  Serial.print("\n\r00:\t");
  for (x = 0;x < EEPROM_BYTES;x++){
    Serial.print(EEPROM.read(x),HEX);
    Serial.print("\t");
    y++;
    if ((y == 16) && ((x+1) < EEPROM_BYTES))  {
      Serial.print("\n\r");
      Serial.print((x+1),HEX);
      Serial.print(":\t");
      y = 0;
    }   
  }
  Serial.println();
}

#endif //DEBUG_EEPROM

//-----------------------------------------------------------------------------------------------------
#ifdef DEBUG_EEPROM
void clear_out_eeprom(){
  
  for (unsigned int x = 0;x < EEPROM_BYTES;x++){
    EEPROM.write(x,0xFF);
  }
  
  print_eeprom();
  
}
#endif //DEBUG_EEPROM
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
#ifdef FEATURE_RIG_INTERFACE
void update_static_screen_radio_freq(){

              
  lcd.setCursor((LCD_COLUMNS-6)/2,0);
  if (rig[current_rig]->rig_status == PORT_OPEN) {
    if (rig[current_rig]->frequency < 10000) {
      lcd.print(" ");
    }
    lcd.print(rig[current_rig]->frequency);
  } else {
    lcd.print("     ");
  }   
}
#endif //FEATURE_RIG_INTERFACE
#endif //FEATURE_DISPLAY
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void update_static_screen_tune_info(){
  
  char workstring[8] = "";
  
  lcd.setCursor(0,1);
  dtostrf((current_L()/(float)100),4,2,workstring);
  lcd.print(workstring);
  lcd.print("uH ");
  if (current_C() < 100)  {lcd.print(" ");}
  if (current_C() < 10)   {lcd.print(" ");}
  lcd.print(current_C());
  lcd.print("pF  "); 
  lcd.setCursor(LCD_COLUMNS-1,1);
  switch(current_tuning_mode()) {
    case 0: lcd.print("-"); break;
    case 1: lcd.print("h"); break;
    case 2: lcd.print("l"); break;
  }
 
}

#endif //FEATURE_DISPLAY
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void update_static_screen_swr(){
  
  char workstring[5] = "";
 
  lcd.setCursor(0,0);            
  dtostrf(current_swr,4,2,workstring);
  lcd.print(workstring);
}
  
#endif //FEATURE_DISPLAY
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void update_static_screen_tuned_freq(){

  static unsigned int last_tuned_freq = 0;
  
  if (last_tuned_freq != tuned_freq) { 
    if (tuned_freq) {
      lcd.setCursor(LCD_COLUMNS-5,0);
      if (tuned_freq < 10000) {lcd.print(" ");}
      lcd.print(tuned_freq);
      update_static_screen_tune_info();
    } else {
      lcd.setCursor(LCD_COLUMNS-5,0);
      lcd.print(" ----");
    }
    last_tuned_freq = tuned_freq;
  }  
}

#endif //FEATURE_DISPLAY
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void update_static_screen(byte force_update){
   
  static unsigned long last_static_screen_update = 0;
  static byte last_state_tuner = 999;

  
//aaaa

  char workstring[10] = "";

  if (lcd_status == LCD_STATIC){
    if (((millis() - last_static_screen_update) >= DISPLAY_STATIC_SCREEN_UPDATE_MS) || force_update) {
      switch(state_tuner) {
        case IDLE:
          if (last_state_tuner != IDLE){
            lcd.clear();
            update_static_screen_swr();

            #ifdef FEATURE_RIG_INTERFACE
            update_static_screen_radio_freq();
            #endif //FEATURE_RIG_INTERFACE
            
            update_static_screen_tuned_freq();  
            update_static_screen_tune_info();           
           
          }
          if ((last_state_tuner == IDLE) && transmit_sense){
            update_static_screen_swr();
          }
          
          #ifdef FEATURE_RIG_INTERFACE
          if ((last_state_tuner == IDLE) && !transmit_sense){            
            update_static_screen_radio_freq();
            update_static_screen_tuned_freq();             
          }      
          #endif //FEATURE_RIG_INTERFACE 
          
          break; //IDLE
        case TUNING:
            if (last_state_tuner != TUNING){
              lcd.clear();
              lcd.setCursor((LCD_COLUMNS-6)/2,0);
              lcd.print("Tuning");
              lcd.setCursor((LCD_COLUMNS-8)/2,1);
              lcd.print(current_freq());
              lcd.print(" kHz");            
            }
          break;
      }  //switch(state_tuner)
      last_state_tuner = state_tuner;
      last_static_screen_update = millis();     
   
    }  //if (((millis() - last_static_screen_update) >= DISPLAY_STATIC_SCREEN_UPDATE_MS) || force_update)
  }
}

#endif //FEATURE_DISPLAY
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void service_display() {  

  byte x = 0;

  #ifdef DEBUG_REAL_DEEP_STUFF
  Serial.println(F("service_display: entering"));
  #endif


  switch (lcd_status) {
    case LCD_REVERT:
      switch (lcd_previous_status) {
        case LCD_CLEAR: lcd_clear(); break;
        case LCD_SCROLL_MSG:
           lcd.clear();
           for (x = 0;x < LCD_ROWS;x++){
             //clear_display_row(x);
             lcd.setCursor(0,x);
             lcd.print(lcd_scroll_buffer[x]);
           }         
           lcd_scroll_flag = 0; 
           lcd_scroll_buffer_dirty = 0;         
           break;
      } //switch (lcd_previous_status)      
      lcd_status = lcd_previous_status;
      break;  //LCD_REVERT      
            
    case LCD_STATIC:
      update_static_screen(0);
      break; //LCD_STATIC    
    case LCD_CLEAR:
      /*if (lcd_static_buffer_dirty) {lcd_status = LCD_STATIC;}*/
      break; //LCD_CLEAR
    case LCD_TIMED_MESSAGE:
      if (millis() > lcd_timed_message_clear_time) {
        lcd_status = LCD_REVERT;
      }
      break; //LCD_TIMED_MESSAGE
    case LCD_SCROLL_MSG:
      if (lcd_scroll_buffer_dirty) { 
        if (lcd_scroll_flag) {
          lcd.clear();
          lcd_scroll_flag = 0;
        }         
        for (x = 0;x < LCD_ROWS;x++){
          //clear_display_row(x);
          lcd.setCursor(0,x);
          lcd.print(lcd_scroll_buffer[x]);
        }
        lcd_scroll_buffer_dirty = 0;
      }
      break; //LCD_SCROLL_MSG
  } //switch (lcd_status)

}
#endif  //FEATURE_DISPLAY

//-------------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void display_scroll_print_char(char charin){
  
 static byte column_pointer = 0;
 static byte row_pointer = 0;
 byte x = 0;
 
 if (lcd_status != LCD_SCROLL_MSG) {
   lcd_status = LCD_SCROLL_MSG;
   lcd.clear();
 }
 if (column_pointer > (LCD_COLUMNS-1)) {
   row_pointer++;
   column_pointer = 0;
   if (row_pointer > (LCD_ROWS-1)) {
     for (x = 0; x < (LCD_ROWS-1); x++) {
       lcd_scroll_buffer[x] = lcd_scroll_buffer[x+1];
     }
     lcd_scroll_buffer[x] = "";     
     row_pointer--;
     lcd_scroll_flag = 1;
   }    
  } 
 lcd_scroll_buffer[row_pointer].concat(charin);
 column_pointer++;
 lcd_scroll_buffer_dirty = 1; 
}

#endif //FEATURE_DISPLAY


//-------------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void lcd_clear() {

  lcd.clear();
  lcd_status = LCD_CLEAR;
}
#endif
//-------------------------------------------------------------------------------------------------------
#ifdef FEATURE_DISPLAY
void lcd_center_print_timed(String lcd_print_string, byte row_number, unsigned int duration)
{
  if (lcd_status != LCD_TIMED_MESSAGE) {
    lcd_previous_status = lcd_status;
    lcd_status = LCD_TIMED_MESSAGE;
    lcd.clear();
  } else {
    clear_display_row(row_number);
  }
  lcd.setCursor(((LCD_COLUMNS - lcd_print_string.length())/2),row_number);
  lcd.print(lcd_print_string);
  lcd_timed_message_clear_time = millis() + duration;
}
#endif

//-------------------------------------------------------------------------------------------------------

#ifdef FEATURE_DISPLAY
void clear_display_row(byte row_number)
{
  for (byte x = 0; x < LCD_COLUMNS; x++) {
    lcd.setCursor(x,row_number);
    lcd.print(" ");
  }
}
#endif

//-------------------------------------------------------------------------------------------------------

#ifdef FEATURE_DISPLAY
void display_startup_message()
{

  lcd_center_print_timed("K3NG",0,4000);
  lcd_center_print_timed("Antenna Tuner",1,4000);
}
#endif //FEATURE_DISPLAY

//-----------------------------------------------------------------------------------------------------

#ifdef FEATURE_DISPLAY
void initialize_display(){

  lcd.begin(LCD_COLUMNS, 2);


//  byte bar1[8] = {
//    B10000,
//    B10000,
//    B10000,
//    B10000,
//    B10000,
//    B10000,
//    B10000,
//    B10000
//  };
//
//  byte bar2[8] = {
//    B11000,
//    B11000,
//    B11000,
//    B11000,
//    B11000,
//    B11000,
//    B11000,
//    B11000
//  };
//
//  byte bar3[8] = {
//    B11100,
//    B11100,
//    B11100,
//    B11100,
//    B11100,
//    B11100,
//    B11100,
//    B11100
//  };
//
//  byte bar4[8] = {
//    B11110,
//    B11110,
//    B11110,
//    B11110,
//    B11110,
//    B11110,
//    B11110,
//    B11110
//  };
//
//  byte tx[8] = {
//    B11100,
//    B01000,
//    B01000,
//    B00000,
//    B00000,
//    B00101,
//    B00010,
//    B00101
//  };
//  lcd.createChar(0, tx);
//  lcd.createChar(1, bar1);
//  lcd.createChar(2, bar2);  
//  lcd.createChar(3, bar3);
//  lcd.createChar(4, bar4);




  #ifdef FEATURE_LCD_I2C
  lcd.setBacklight(lcdcolor);
  #endif //FEATURE_LCD_I2C

}
#endif  //FEATURE_DISPLAY

//-----------------------------------------------------------------------------------------------------

//#ifdef FEATURE_DISPLAY
//void draw_bar_graph(byte row, byte start_column, float bars) {
//
//  static float last_bars = 0;
//  byte stop_column = LCD_COLUMNS;
//
//  if (last_bars) {   // shortcut if we already have some bars on the screen
//    if (bars > last_bars) {
//      start_column = (last_bars / 5);
//      last_bars = bars;
//      bars = bars - (start_column * 5);
//      stop_column = start_column + (bars / 5) + 1;
//    }  
//    else {
//      stop_column = (last_bars / 5) + 1;
//      last_bars = bars;
//      if (bars > 4) {
//        start_column = (bars / 5)-1;
//        bars = bars - (start_column * 5);
//      }
//    }
//  } 
//  else {
//    last_bars = bars; 
//  }
//
//  lcd.setCursor(start_column,row);
//  for (int x = start_column; x < stop_column; x++) {
//    //lcd.setCursor(x,row);
//    if (int(bars) >= 5) {
//      lcd.write(255);
//    } 
//    else {
//      if (int(bars) > 0) {
//        lcd.write(int(bars));
//      } 
//      else {
//        lcd.print(" ");
//      }
//    } 
//    bars = bars - 5;
//  }
//}
//#endif

//-----------------------------------------------------------------------------------------------------
/*
……………………………………..________
………………………………,.-‘”……………….“~.,
………………………..,.-”……………………………..“-.,
…………………….,/………………………………………..”:,
…………………,?………………………………………………\,
………………./…………………………………………………..,}
……………../………………………………………………,:`^`..}
……………/……………………………………………,:”………/
…………..?…..__…………………………………..:`………../
…………./__.(…..“~-,_…………………………,:`………./
………../(_….”~,_……..“~,_………………..,:`…….._/
……….{.._$;_……”=,_…….“-,_…….,.-~-,},.~”;/….}
………..((…..*~_…….”=-._……“;,,./`…./”…………../
…,,,___.\`~,……“~.,………………..`…..}…………../
…………(….`=-,,…….`……………………(……;_,,-”
…………/.`~,……`-………………………….\……/\
………….\`~.*-,……………………………….|,./…..\,__
,,_……….}.>-._\……………………………..|…………..`=~-,
…..`=~-,_\_……`\,……………………………\
……………….`=~-,,.\,………………………….\
…………………………..`:,,………………………`\…………..__
……………………………….`=-,……………….,%`>–==“
…………………………………._\……….._,-%…….`\
……………………………..,< `.._|_,-&``................`\

*/

//-----------------------------------------------------------------------------------------------------


void set_indicators(byte indication){
//bbbb 
  switch(indication){
   case TUNE_IN_PROGRESS:
     #if pin_tune_in_progress > 0
     digitalWrite(pin_tune_in_progress, HIGH);
     #endif
     #if pin_tuned > 0
     digitalWrite(pin_tuned, LOW);
     #endif
     #ifdef FEATURE_LCD_I2C_STATUS_COLOR
     lcd.setBacklight(LCD_I2C_STATUS_COLOR_TUNING);
     #endif //FEATURE_LCD_I2C_STATUS_COLOR     
     break;
   case TUNED:
     #if pin_tune_in_progress > 0
     digitalWrite(pin_tune_in_progress, LOW);
     #endif
     #if pin_tuned > 0
     digitalWrite(pin_tuned, HIGH);
     #endif
     #ifdef FEATURE_LCD_I2C_STATUS_COLOR
     lcd.setBacklight(LCD_I2C_STATUS_COLOR_TUNED);
     #endif //FEATURE_LCD_I2C_STATUS_COLOR
     break;
   case UNTUNABLE:
     #if pin_untunable > 0
     digitalWrite(pin_untunable, HIGH);
     #endif
     #if pin_tune_in_progress > 0
     digitalWrite(pin_tune_in_progress, LOW);
     #endif
     #ifdef FEATURE_LCD_I2C_STATUS_COLOR
     lcd.setBacklight(LCD_I2C_STATUS_COLOR_UNTUNED);
     #endif //FEATURE_LCD_I2C_STATUS_COLOR     
     break;
   case INDICATOR_IDLE:
     #if pin_untunable > 0
     digitalWrite(pin_untunable, LOW);
     #endif
     #if pin_tune_in_progress > 0
     digitalWrite(pin_tune_in_progress, LOW);
     #endif
     #if pin_tuned > 0
     digitalWrite(pin_tuned, LOW);
     #endif
     break;     
  } 
}

//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_RIG_INTERFACE
byte rig_tune(byte activate){

  static byte previous_rig_mode = UNDEF;


  if (activate) {
    previous_rig_mode = rig[current_rig]->mode;
    //Serial.print(F("rig_tune: previous_rig_mode: "));
    //Serial.println(previous_rig_mode);
    rig[current_rig]->change_mode(AM);
    service_rigs_for_ms(10);
    rig[current_rig]->ptt(1);
    service_rigs_for_ms(10);
  } else {
    rig[current_rig]->ptt(0);
    service_rigs_for_ms(10);
    rig[current_rig]->change_mode(previous_rig_mode);
    service_rigs_for_ms(10);
  }

  
}
#endif //FEATURE_RIG_INTERFACE
//-----------------------------------------------------------------------------------------------------
#ifdef FEATURE_RIG_INTERFACE
void service_rigs_for_ms(byte service_for_ms){

 unsigned long start_time = millis();

 while ((millis() - start_time) < service_for_ms) {
   rig[current_rig]->service();
 }

  
}
#endif //FEATURE_RIG_INTERFACE
//-----------------------------------------------------------------------------------------------------
/*
byte better_swr_in_tune_buffer(float better_than_swr, unsigned int frequency_to_search){
  
  for (byte x = 0;x < TUNE_BUFFER_SIZE;x++){
    if (((abs(tune_buffer[x].freq - frequency_to_search)) <= TUNE_BUFFER_MATCH_THRESH_KHZ) && (current_antenna == antenna_number_from_tuning_mode(tune_buffer[x].tuning_mode)) && 
    (current_transmitter == transmitter_number_from_tuning_mode(tune_buffer[x].tuning_mode))){
      return 1;
    }
  }
  return 0;
  
}
*/
