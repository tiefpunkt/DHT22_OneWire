//owdevice - A small 1-Wire emulator for AVR Microcontroller
//
//Copyright (C) 2012  Tobias Mueller mail (at) tobynet.de
//
//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
// any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
//VERSION 1.2 DS18B20  ATTINY13 (AD input) and ATTINY25 (internal sensor)

//FOR MAKE by hand
/*
avr-gcc -mmcu=attiny13 -O2 -c [name].c
avr-gcc.exe -mmcu=attiny13  [name].o -o [name].elf
avr-objcopy -O ihex  [name].elf [name].hex
*/



#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "am2302.h"

#if defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny13__)
// OW_PORT Pin 6  - PB1
//Analog input PB2

//OW Pin
#define OW_PORT PORTB //1 Wire Port
#define OW_PIN PINB //1 Wire Pin as number
#define OW_PORTN (1<<PINB1)  //Pin as bit in registers
#define OW_PINN (1<<PINB1)
#define OW_DDR DDRB  //pin direction register
#define SET_LOW OW_DDR|=OW_PINN;OW_PORT&=~OW_PORTN;  //set 1-Wire line to low
#define RESET_LOW {OW_DDR&=~OW_PINN;}  //set 1-Wire pin as input
//Pin interrupt	
#define EN_OWINT {GIMSK|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt 
#define DIS_OWINT  GIMSK&=~(1<<INT0);  //disable interrupt
#define SET_RISING MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
#define SET_FALLING MCUCR=(1<<ISC01); //set interrupt at falling edge
#define CHK_INT_EN (GIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
#define PIN_INT ISR(INT0_vect)  // the interrupt service routine
//Timer Interrupt
#define EN_TIMER {TIMSK0 |= (1<<TOIE0); TIFR0|=(1<<TOV0);} //enable timer interrupt
#define DIS_TIMER TIMSK0  &= ~(1<<TOIE0); // disable timer interrupt
#define TCNT_REG TCNT0  //register of timer-counter
#define TIMER_INT ISR(TIM0_OVF_vect) //the timer interrupt service routine

//Initializations of AVR
#define INIT_AVR CLKPR=(1<<CLKPCE);\
				   CLKPR=0;/*9.6Mhz*/\
				   TIMSK0=0;\
				   GIMSK=(1<<INT0);/*set direct GIMSK register*/\
				   TCCR0B=(1<<CS00)|(1<<CS01); /*9.6mhz /64 couse 8 bit Timer interrupt every 6,666us*/

//Setup Temp Measurement DS18B20
#define INIT_TEMP   DDRB&=~(1<<PINB2); \
					ADMUX=(1<<MUX0); \
					ADCSRA= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
#define CONV_TEMP		ADCSRA|=(1<<ADSC); \
						while ((ADCSRA&(1<<ADSC))!=0) {}\
						scratchpad[0]=ADCL;\
						scratchpad[1]=ADCH;\
						



//Times
#define OWT_MIN_RESET 51 //minimum duration of the Reset impulse

#define OWT_RESET_PRESENCE 4 //time between rising edge of reset impulse and presence impulse
#define OWT_PRESENCE 20 //duration of the presence impulse
#define OWT_READLINE 4  //duration from master low to read the state of 1-Wire line
#define OWT_LOWTIME 4 //length of low 
#endif

#ifdef __AVR_ATtiny25__ 
// OW_PORT Pin 7  - PB2


//OW Pin
#define OW_PORT PORTB //1 Wire Port
#define OW_PIN PINB //1 Wire Pin as number
#define OW_PORTN (1<<PINB2)  //Pin as bit in registers
#define OW_PINN (1<<PINB2)
#define OW_DDR DDRB  //pin direction register
#define SET_LOW OW_DDR|=OW_PINN;OW_PORT&=~OW_PORTN;  //set 1-Wire line to low
#define RESET_LOW {OW_DDR&=~OW_PINN;}  //set 1-Wire pin as input
//Pin interrupt	
#define EN_OWINT {GIMSK|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt 
#define DIS_OWINT  GIMSK&=~(1<<INT0);  //disable interrupt
#define SET_RISING MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
#define SET_FALLING MCUCR=(1<<ISC01); //set interrupt at falling edge
#define CHK_INT_EN (GIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
#define PIN_INT ISR(INT0_vect)  // the interrupt service routine
//Timer Interrupt
#define EN_TIMER {TIMSK |= (1<<TOIE0); TIFR|=(1<<TOV0);} //enable timer interrupt
#define DIS_TIMER TIMSK  &= ~(1<<TOIE0); // disable timer interrupt
#define TCNT_REG TCNT0  //register of timer-counter
#define TIMER_INT ISR(TIM0_OVF_vect) //the timer interrupt service routine


#define OWT_MIN_RESET 51
#define OWT_RESET_PRESENCE 4
#define OWT_PRESENCE 20 
#define OWT_READLINE 3 //for fast master, 4 for slow master and long lines
#define OWT_LOWTIME 3 //for fast master, 4 for slow master and long lines 

//Initializations of AVR
#define INIT_AVR CLKPR=(1<<CLKPCE); \
				   CLKPR=0; /*8Mhz*/  \
				   TIMSK=0; \
				   GIMSK=(1<<INT0);  /*set direct GIMSK register*/ \
				   TCCR0B=(1<<CS00)|(1<<CS01); /*8mhz /64 couse 8 bit Timer interrupt every 8us*/

#endif // __AVR_ATtiny25__ 


#if defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny4313__)
// OW_PORT Pin 14  - PB2


//OW Pin
#define OW_PORT PORTB //1 Wire Port
#define OW_PIN PINB //1 Wire Pin as number
#define OW_PORTN (1<<PINB2)  //Pin as bit in registers
#define OW_PINN (1<<PINB2)
#define OW_DDR DDRB  //pin direction register
#define SET_LOW OW_DDR|=OW_PINN;OW_PORT&=~OW_PORTN;  //set 1-Wire line to low
#define RESET_LOW {OW_DDR&=~OW_PINN;}  //set 1-Wire pin as input
//Pin interrupt	
#define EN_OWINT {GIMSK|=(1<<INT0);EIFR|=(1<<INTF0);}  //enable interrupt 
#define DIS_OWINT  GIMSK&=~(1<<INT0);  //disable interrupt
#define SET_RISING MCUCR|=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
#define SET_FALLING {MCUCR|=(1<<ISC01);MCUCR&=~(1<<ISC00);} //set interrupt at falling edge
#define CHK_INT_EN (GIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
#define PIN_INT ISR(INT0_vect)  // the interrupt service routine
//Timer Interrupt
#define EN_TIMER {TIMSK |= (1<<TOIE0); TIFR|=(1<<TOV0);} //enable timer interrupt
#define DIS_TIMER TIMSK  &= ~(1<<TOIE0); // disable timer interrupt
#define TCNT_REG TCNT0  //register of timer-counter
#define TIMER_INT ISR(TIMER0_OVF_vect) //the timer interrupt service routine


#define OWT_MIN_RESET 51
#define OWT_RESET_PRESENCE 4
#define OWT_PRESENCE 20 
#define OWT_READLINE 3 //for fast master, 4 for slow master and long lines
#define OWT_LOWTIME 3 //for fast master, 4 for slow master and long lines

//Initializations of AVR
#define INIT_AVR CLKPR=(1<<CLKPCE); \
				   CLKPR=0; /*8Mhz*/  \
				   TIMSK=0; \
				   GIMSK=(1<<INT0);  /*set direct GIMSK register*/ \
				   TCCR0B=(1<<CS00)|(1<<CS01); /*8mhz /64 couse 8 bit Timer interrupt every 8us*/

#endif // __AVR_ATtiny2313__ 


volatile uint8_t scratchpad[9]={0x50,0x05,0x0,0x0,0x7f,0xff,0x00,0x10,0x0}; //Initial scratchpad
volatile uint8_t scrc; //CRC calculation

volatile uint8_t cbuf; //Input buffer for a command
const uint8_t owid[8]={0x28, 0xA2, 0xD9, 0x84, 0x00, 0x00, 0x02, 0xEA};  
//set your own ID http://www.tm3d.de/index.php/tools/14-crc8-berechnung
volatile uint8_t bitp;  //pointer to current Byte
volatile uint8_t bytep; //pointer to current Bit

volatile uint8_t mode; //state
volatile uint8_t wmode; //if 0 next bit that send the device is  0
volatile uint8_t actbit; //current
volatile uint8_t srcount; //counter for search rom

//States / Modes
#define OWM_SLEEP 0  //Waiting for next reset pulse
#define OWM_RESET 1  //Reset pulse received 
#define OWM_PRESENCE 2  //sending presence pulse
#define OWM_READ_COMMAND 3 //read 8 bit of command
#define OWM_SEARCH_ROM 4  //SEARCH_ROM algorithms
#define OWM_MATCH_ROM 5  //test number
#define OWM_READ_SCRATCHPAD 6   
#define OWM_WRITE_SCRATCHPAD 7
#define OWM_CHK_RESET 8  //waiting of rising edge from reset pulse

//Write a bit after next falling edge from master
//its for sending a zero as soon as possible 
#define OWW_NO_WRITE 2
#define OWW_WRITE_1 1
#define OWW_WRITE_0 0



PIN_INT {
	uint8_t lwmode=wmode;  //let this variables in registers
	uint8_t lmode=mode;
	if ((lwmode==OWW_WRITE_0)) {SET_LOW;lwmode=OWW_NO_WRITE;}    //if necessary set 0-Bit 
	DIS_OWINT; //disable interrupt, only in OWM_SLEEP mode it is active
	switch (lmode) {
		case OWM_SLEEP:  
			TCNT_REG=~(OWT_MIN_RESET);
			EN_OWINT; //other edges ?
			break;
		//start of reading with falling edge from master, reading closed in timer isr
		case OWM_MATCH_ROM:  //falling edge wait for receive 
		case OWM_WRITE_SCRATCHPAD:
		case OWM_READ_COMMAND:
			TCNT_REG=~(OWT_READLINE); //wait a time for reading
			break;
		case OWM_SEARCH_ROM:   //Search algorithm waiting for receive or send
			if (srcount<2) { //this means bit or complement is writing, 
				TCNT_REG=~(OWT_LOWTIME);					
			} else 
				TCNT_REG=~(OWT_READLINE);  //init for read answer of master 
			break;
		case OWM_READ_SCRATCHPAD:  //a bit is sending 
			TCNT_REG=~(OWT_LOWTIME);
			break;
		case OWM_CHK_RESET:  //rising edge of reset pulse
			SET_FALLING; 
			TCNT_REG=~(OWT_RESET_PRESENCE);  //waiting for sending presence pulse
			lmode=OWM_RESET;
			break;
	}
	EN_TIMER;
	mode=lmode;
	wmode=lwmode;
	
}			

	

TIMER_INT {
	uint8_t lwmode=wmode; //let this variables in registers
	uint8_t lmode=mode;
	uint8_t lbytep=bytep;
	uint8_t lbitp=bitp;
	uint8_t lsrcount=srcount;
	uint8_t lactbit=actbit;
	uint8_t lscrc=scrc;
	//Ask input line sate 
	uint8_t p=((OW_PIN&OW_PINN)==OW_PINN);  
	//Interrupt still active ?
	if (CHK_INT_EN) {
		//maybe reset pulse
		if (p==0) { 
			lmode=OWM_CHK_RESET;  //wait for rising edge
			SET_RISING; 
		}
		DIS_TIMER;
	} else
	switch (lmode) {
		case OWM_RESET:  //Reset pulse and time after is finished, now go in presence state
			lmode=OWM_PRESENCE;
			SET_LOW;
			TCNT_REG=~(OWT_PRESENCE);
			DIS_OWINT;  //No Pin interrupt necessary only wait for presence is done
			break;
		case OWM_PRESENCE:
			RESET_LOW;  //Presence is done now wait for a command
			lmode=OWM_READ_COMMAND;
			cbuf=0;lbitp=1;  //Command buffer have to set zero, only set bits will write in
			break;
		case OWM_READ_COMMAND:
			if (p) {  //Set bit if line high 
				cbuf|=lbitp;
			} 
			lbitp=(lbitp<<1);
			if (!lbitp) { //8-Bits read
				lbitp=1;
				switch (cbuf) {
					case 0x55:lbytep=0;lmode=OWM_MATCH_ROM;break;
					case 0xF0:  //initialize search rom
						lmode=OWM_SEARCH_ROM;
						lsrcount=0;
						lbytep=0;
						lactbit=(owid[lbytep]&lbitp)==lbitp; //set actual bit
						lwmode=lactbit;  //prepare for writing when next falling edge
						break;
					case 0x4E:
						lmode=OWM_WRITE_SCRATCHPAD;
						lbytep=2;scratchpad[2]=0;  //initialize writing position in scratch pad 
						break;
					case 0x44:  //Start Convert 
					case 0x64:  // some tool uses this command
						// START CONVERSATION
						lmode=OWM_SLEEP;
						break;
					case 0xBE:
						lmode=OWM_READ_SCRATCHPAD; //read scratch pad 
						lbytep=0;lscrc=0; //from first position
						lactbit=(lbitp&scratchpad[0])==lbitp;
						lwmode=lactbit; //prepare for send firs bit 
						break;
					default: lmode=OWM_SLEEP;  //all other commands do nothing
				}		
			}			
			break;
		case OWM_SEARCH_ROM:
			RESET_LOW;  //Set low also if nothing send (branch takes time and memory)
			lsrcount++;  //next search rom mode
			switch (lsrcount) {
				case 1:lwmode=!lactbit;  //preparation sending complement
					break;
				case 3:
					if (p!=(lactbit==1)) {  //check master bit
						lmode=OWM_SLEEP;  //not the same go sleep
					} else {
						lbitp=(lbitp<<1);  //prepare next bit
						if (lbitp==0) {
							lbitp=1;
							lbytep++;
							if (lbytep>=8) {
								lmode=OWM_SLEEP;  //all bits processed 
								break;
							}
						}				
						lsrcount=0;
						lactbit=(owid[lbytep]&lbitp)==lbitp;
						lwmode=lactbit;
					}		
					break;			
			}
			break;
		case OWM_MATCH_ROM:
			if (p==((owid[lbytep]&lbitp)==lbitp)) {  //Compare with ID Buffer
				lbitp=(lbitp<<1);
				if (!lbitp) {
					lbytep++;
					lbitp=1;
					if (lbytep>=8) {
						lmode=OWM_READ_COMMAND;  //same? get next command
						
						cbuf=0;
						break;			
					}
				} 
			} else {
				lmode=OWM_SLEEP;
			}
			break;
		case OWM_WRITE_SCRATCHPAD:
			if (p) {
				scratchpad[lbytep]|=lbitp;
			} 
			lbitp=(lbitp<<1);
			if (!lbitp) {		
				lbytep++;
				lbitp=1;
				if (lbytep==5) {
					lmode=OWM_SLEEP;
					break;
				} else scratchpad[lbytep]=0;
			}		
			break;	
		case OWM_READ_SCRATCHPAD:
			RESET_LOW;
			if ((lscrc&1)!=lactbit) lscrc=(lscrc>>1)^0x8c; else lscrc >>=1;
			lbitp=(lbitp<<1);
			if (!lbitp) {		
				lbytep++;
				lbitp=1;
				if (lbytep>=9) {
					lmode=OWM_SLEEP;
					break;			
				} else if (lbytep==8) scratchpad[8]=lscrc;
			}					
			lactbit=(lbitp&scratchpad[lbytep])==lbitp;
			lwmode=lactbit;		
			break;
		}
		if (lmode==OWM_SLEEP) {DIS_TIMER;}
		if (lmode!=OWM_PRESENCE)  { 
			TCNT_REG=~(OWT_MIN_RESET-OWT_READLINE);  //OWT_READLINE around OWT_LOWTIME
			EN_OWINT;
		}
		
		mode=lmode;
		wmode=lwmode;
		bytep=lbytep;
		bitp=lbitp;
		srcount=lsrcount;
		actbit=lactbit;
		scrc=lscrc;
}


int main(void) {
	mode=OWM_SLEEP;
	wmode=OWW_NO_WRITE;
	OW_DDR&=~OW_PINN;
	
	SET_FALLING;
	
	INIT_AVR
	DIS_TIMER;

	DDR_SENSOR &= ~(1 << SENSOR); // define as input
	PORT_SENSOR &= ~(1 << SENSOR);  // disable pullup
	
	sei();
	
	uint16_t temp = 0;
	uint16_t hum = 0;
	uint8_t decimal[10] = { 0x00, 0x02, 0x03, 0x05, 0x06, 0x08, 0x0A, 0x0B, 0x0D, 0x0E};
	while(1){

		if (am2302(&hum, &temp) == 0) {
			temp = ((temp/10) << 4 ) | decimal[temp%10];
			scratchpad[0]=0x00ff&temp;
			scratchpad[1]=0x00ff&(temp>>8);
			hum = ((hum/10) << 4 ) | decimal[hum%10];
			scratchpad[2]=0x00ff&hum;
			scratchpad[3]=0x00ff&(hum>>8);
			
		}
		_delay_ms(1000);
	}
}	
