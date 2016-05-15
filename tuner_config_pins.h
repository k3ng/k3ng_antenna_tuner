// administrative and control pin definitions (not I2C expander pins)
#ifndef tuner_pins_h
#define tuner_pins_h

#define pin_led 13
#define pin_tune_in_progress 0   // indicator - goes high when tuning (0 = disable)
#define pin_tuned 0              // indicator - goes high when tuned (0 = disable)
#define pin_untunable 0          // indicator - goes high when untunable (0 = disable)
#define pin_frequency_counter 5  // input - frequency counter (dummy entry - hard coded in frequency counter library)
                                 //   Uno: pin 5
                                 //   Mega: pin 47
#define pin_tune_lock 6          // input - ground to lock tuning (0 = disable)
#define pin_forward_v  A0        // input (analog) - SWR sensor forward voltage
#define pin_reflected_v A1       // input (analog) - SWR sensor reverse voltage
#define pin_voltage_control 7    // output - controls SWR sensor voltage attenuator
#define pin_wakeup 2             // input - use with FEATURE_SLEEP_MODE - low wakes unit up
#define pin_awake 0              // output - use with FEATURE_SLEEP_MODE - goes high when unit is awake (0 = disable)
#define pin_manual_tune 0        // input - ground to initiate tuning (0 = disable)
#define rig_0_control_tx A2      // rig serial port - rig RX line / Arduino TX line
#define rig_0_control_rx A3      // rig serial port - rig TX line / Arduino RX line

#endif //tuner_pins_h
