// Frequency Counter Lib example

/*
  Martin Nawrath KHM LAB3
  Kunsthochschule f¸r Medien Kˆln
  Academy of Media Arts
  http://www.khm.de
  http://interface.khm.de/index.php/labor/experimente/	
 */
#include <FreqCounter.h>


unsigned long frq;
int cnt;
int pinLed=13;

void setup() {
  pinMode(pinLed, OUTPUT);

  Serial.begin(115200);        // connect to the serial port

  Serial.println("Frequency Counter");

}



void loop() {

  // wait if any serial is going on
  FreqCounter::f_comp=10;   // Cal Value / Calibrate with professional Freq Counter
  FreqCounter::start(100);  // 100 ms Gate Time

  while (FreqCounter::f_ready == 0) 

  frq=FreqCounter::f_freq;
  Serial.print(cnt++);
  Serial.print("  Freq: ");
  Serial.println(frq);
  delay(20);
  digitalWrite(pinLed,!digitalRead(pinLed));  // blink Led

}  

