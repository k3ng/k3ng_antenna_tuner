/*
  FreqCounter.h -
  Using Counter1 for counting Frequency on T1 / PD5 / digitalPin 5
  Uses Counter5 on the Mega digitalPin 47

  Using Timer2 for Gatetime generation

  Martin Nawrath KHM LAB3
  Kunsthochschule für Medien Köln
  Academy of Media Arts
  http://www.khm.de
  http://interface.khm.de/index.php/labor/experimente/

  History:
    Dec/08 - V0.0
    May/20  modified by mem to support Mega usting T5 /PL2 on digitalPin 47


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/



#include <FreqCounter.h>


unsigned long FreqCounter::f_freq;

volatile unsigned char FreqCounter::f_ready;
volatile unsigned char FreqCounter::f_mlt;
volatile unsigned int FreqCounter::f_tics;
volatile unsigned int FreqCounter::f_period;
volatile unsigned int FreqCounter::f_comp;

// 16 bit timer defines added by mem to enable redifining the timer used
#if defined(__AVR_ATmega2560__)
#define TCCRnA TCCR5A
#define TCCRnB TCCR5B
#define TCNTn  TCNT5
#define TIFRn  TIFR5
#define TOVn   TOV5
#elif defined(__AVR_ATmega1280__)
#define TCCRnA TCCR5A
#define TCCRnB TCCR5B
#define TCNTn  TCNT5
#define TIFRn  TIFR5
#define TOVn   TOV5
#elif defined (__AVR_ATmega168__)
#define TCCRnA TCCR1A
#define TCCRnB TCCR1B
#define TCNTn  TCNT1
#define TIFRn  TIFR1
#define TOVn   TOV1
#endif
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

void FreqCounter::start(int ms) {

  f_period=ms/2;
  if (f_comp ==0) f_comp=1;

	// hardware counter setup ( refer atmega168.pdf chapter 16-bit counter1)
  TCCRnA=0;		     // reset timer/countern control register A
  TCCRnB=0;			  // reset timer/countern control register A
  TCNTn=0;	     		// counter value = 0
  // set timer/counter1 hardware as counter , counts events on pin Tn ( arduino pin 5 on 168,47 on Mega )
  // normal mode, wgm10 .. wgm13 = 0
  sbi (TCCRnB ,CS10);	 // External clock source on Tn pin. Clock on rising edge.
  sbi (TCCRnB ,CS11);
  sbi (TCCRnB ,CS12);


  // timer2 setup / is used for frequency measurement gatetime generation
  // timer 2 presaler set to 256 / timer 2 clock = 16Mhz / 256 = 62500 Hz
  TCCR2A=0;
  TCCR2B=0;
  cbi (TCCR2B ,CS20);
  sbi (TCCR2B ,CS21);
  sbi (TCCR2B ,CS22);

  //set timer2 to CTC Mode
  cbi (TCCR2A ,WGM20);
  sbi (TCCR2A ,WGM21);
  cbi (TCCR2B ,WGM22);
  OCR2A = 124;


  f_ready=0;			    // reset period measure flag
  f_tics=0;		     // reset interrupt counter
  sbi (GTCCR,PSRASY);	 // reset presacler counting
  TCNT2=0;			// timer2=0
  TCNTn=0;			// Countern = 0

  cbi (TIMSK0,TOIE0);	 // disable Timer0  //disable  millis and delay
  sbi (TIMSK2,OCIE2A);	// enable Timer2 Interrupt

  TCCRnB = TCCRnB | 7;	//  Counter Clock source = pin Tn , start counting now

}



//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer2 every 2ms = 500 Hz
//  16Mhz / 256 / 125 = 500 Hz
//  here the gatetime generation for freq. measurement takes place:

ISR(TIMER2_COMPA_vect) {
										// multiple 2ms = gate time = 100 ms
if (FreqCounter::f_tics >= FreqCounter::f_period) {
							    // end of gate time, measurement ready

   										// GateCalibration Value, set to zero error with reference frequency counter
    delayMicroseconds(FreqCounter::f_comp); // 0.01=1/ 0.1=12 / 1=120 sec
    TCCRnB = TCCRnB & ~7;   			// Gate Off  / Counter Tn stopped
    cbi (TIMSK2,OCIE2A);			    // disable Timer2 Interrupt
    sbi (TIMSK0,TOIE0);     			// enable Timer0 again // millis and delay
    FreqCounter::f_ready=1;		 // set global flag for end count period

							    // calculate now frequeny value
    FreqCounter::f_freq=0x10000 * FreqCounter::f_mlt;  // mult #overflows by 65636
    FreqCounter::f_freq += TCNTn;		// add countern value
    FreqCounter::f_mlt=0;

  }
  FreqCounter::f_tics++;			// count number of interrupt events
  if (TIFRn & 1) {				    // if Timer/Counter n overflow flag
    FreqCounter::f_mlt++;		   // count number of Countern overflows
    sbi(TIFRn,TOVn);				  // clear Timer/Counter n overflow flag
  }
  // PORTB = PORTB ^ 32;				  // int activity test
}
