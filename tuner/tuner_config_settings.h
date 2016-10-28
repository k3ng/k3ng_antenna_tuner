#ifndef tuner_settings_h
#define tuner_settings_h
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
#define UNTUNABLE_RETRY_TIME 5000            // if we couldn't find acceptable match wait this many mS to attempt tuning again
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
#define DEBUG_MODE_DEFAULT 0
#define FREQUENCY_COUNTER_CALIBRATION 0.985836

#endif //tuner_settings_h