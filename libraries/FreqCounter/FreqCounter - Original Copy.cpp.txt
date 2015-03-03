/*
  FreqCounter.h - 
  Using Counter1 for counting Frequency on T1 / PD5 / digitalPin 5 
  Using Timer2 for Gatetime generation

  Martin Nawrath KHM LAB3
  Kunsthochschule für Medien Köln
  Academy of Media Arts
  http://www.khm.de
  http://interface.khm.de/index.php/labor/experimente/	
  
  History:
  	Dec/08 - V1.0 
  	Oct/10 - V1.1    removed occasional glitches through interference with timer0 
  	                 set intterrupt timebase to 1ms
  	                 works with atmega328 
  	

	

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

void FreqCounter::start(int ms) {

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)


    TIMSK0 &=~(1<<TOIE0);       // disable Timer0  //disable  millis and delay
    delayMicroseconds(50);      // wait if any ints are pending
    
    f_period=ms;
  
  
    if (f_comp ==0) f_comp=1;  // 0 is not allowed in del us
	
    // hardware counter setup ( refer atmega168.pdf chapter 16-bit counter1)
    TCCR1A=0;                  // reset timer/counter1 control register A
    TCCR1B=0;              	   // reset timer/counter1 control register A
    TCNT1=0;           		   // counter value = 0
    // set timer/counter1 hardware as counter , counts events on pin T1 ( arduino pin 5)
    // normal mode, wgm10 .. wgm13 = 0
    
    TCCR1B |=  (1<<CS10) ;// External clock source on T1 pin. Clock on rising edge.
    TCCR1B |=  (1<<CS11) ;
    TCCR1B |=  (1<<CS12) ;
    
    // timer2 setup / is used for frequency measurement gatetime generation
    TCCR2A=0;
    TCCR2B=0;
    
    // timer 2 presaler set to 128 / timer 2 clock = 16Mhz / 256 = 62500 Hz
    TCCR2B |=  (1<<CS20) ;
    TCCR2B &= ~(1<<CS21) ;
    TCCR2B |=  (1<<CS22) ;

    //set timer2 to CTC Mode with OCR2A is top counter value
    TCCR2A &= ~(1<<WGM20) ;
    TCCR2A |=  (1<<WGM21) ;
    TCCR2A &= ~(1<<WGM22) ;
    OCR2A = 124;                // CTC divider by 125
   
    f_ready=0;                  // reset period measure flag
    f_tics=0;                   // reset interrupt counter
    GTCCR = (1<<PSRASY);        // reset presacler counting
    TCNT2=0;                    // timer2=0
    TCNT1=0;                    // Counter1 = 0
    
    TIMSK2 |=(1<<OCIE2A);       // enable Timer2 Interrupt
    
                                // External clock source on T1 pin. Clock on rising edge.
    TCCR1B |= (1<<CS12) | (1<<CS11) | (1<<CS10);        //   start counting now     
    #endif
}
//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer2 every 1ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz
//  here the gatetime generation for freq. measurement takes place: 

ISR(TIMER2_COMPA_vect) {
										// multiple 2ms = gate time = 100 ms
if (FreqCounter::f_tics >= FreqCounter::f_period) {         	
                            			// end of gate time, measurement ready

   										// GateCalibration Value, set to zero error with reference frequency counter
    //  delayMicroseconds(FreqCounter::f_comp); // 0.01=1/ 0.1=12 / 1=120 sec 
    delayMicroseconds(FreqCounter::f_comp);
    TCCR1B = TCCR1B & ~7;   			// Gate Off  / Counter T1 stopped 
    TIMSK2 &= ~(1<<OCIE2A);    			// disable Timer2 Interrupt
    TIMSK0 |=(1<<TOIE0);     			// enable Timer0 again // millis and delay
    FreqCounter::f_ready=1;             // set global flag for end count period
    
                                        // calculate now frequeny value
    FreqCounter::f_freq=0x10000 * FreqCounter::f_mlt;  // mult #overflows by 65636
    FreqCounter::f_freq += TCNT1;      	// add counter1 value
    FreqCounter::f_mlt=0;
    
    }
    FreqCounter::f_tics++;            	// count number of interrupt events
    if (TIFR1 & 1) {          			// if Timer/Counter 1 overflow flag
    FreqCounter::f_mlt++;               // count number of Counter1 overflows
    TIFR1 =(1<<TOV1);        			// clear Timer/Counter 1 overflow flag
    }
    // PORTB = PORTB ^ 32;  				// int activity test
}
