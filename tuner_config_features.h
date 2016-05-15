#ifndef tuner_features_h
#define tuner_features_h

#define FEATURE_DISPLAY
#define FEATURE_LCD_I2C
#define FEATURE_SERIAL
#define FEATURE_SERIAL_HELP
#define FEATURE_COMMAND_LINE_INTERFACE
#define FEATURE_FREQ_COUNTER
// #define FEATURE_RECEIVE_BYPASS
// #define FEATURE_SLEEP_MODE
// #define FEATURE_RIG_INTERFACE
// #define FEATURE_RECEIVE_FREQ_AUTOSWITCH
// #define FEATURE_RIG_CONTROL_PASS_THROUGH   // this works best when serial port and rig port baud are the same
#define FEATURE_LCD_I2C_STATUS_COLOR


#define OPTION_WRITE_CONFIG_BEFORE_SLEEP             // write the configuration and tune buffer to EEPROM before going to sleep (probably a good idea)
#define OPTION_TUNE_BUFFER_ENTRY_USE_ACCEPTABLE_SWR  // when trying tune buffer entry, use TARGET_SWR_ACCEPTABLE_SETTING threshold (improves tune times)



#endif //tuner_features_h