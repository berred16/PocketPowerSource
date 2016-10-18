/*
Copyright (C) 2015  Bernhard Redemann (bernd.red@b-redemann.de)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Modifications / implementations of functions: 
"set_digit_pattern" from Marcel Langner (marcel.langner@myiq.de)
"toggle" from http://www.avrfreaks.net/forum/turn-onoff-led-same-switch (Bob Gardner)

Controls 7 segment display of the power supply
Toggle switch for on/off function
Detect and display short circuit at output terminals

Version 1.A / 2015, Nov. 21st
Version 1.b / 2016, Jan. 04th change v/I factors

Hardware setup:

Atmega8/88/162/328 (smd package, 32 pins)

  -> SHT_DETECT PD3 |1------U-----32| PD2 <- Button on/off
  <- Out on/off PD4 |2            31| PD1 free
				GND |3            30| PD0 free
				VCC |4            29| (Reset)
				GND |5            28| PC5 I_AN_tens
				VCC |6            27| PC4 I_AN_ones
			(g)	PB6 |7            26| PC3 I_AN_tenth
    	    (d) PB7 |8            25| PC2 V_AN_tens
           free PD5 |9            24| PC1 V_AN_ones
           free PD6 |10           23| PC0 V_AN_tenth
           free PD7 |11           22| ADC7 (Amperes)
            (e) PB0 |12           21| GND 
            (f) PB1 |13           20| AREF 
            (a) PB2 |14           19| ADC6 (Volts)
		    (b) PB3 |15			  18| AVCC
			(c) PB4 |16-----------17| PB5 (dp)


  ---------------\   />--------------- V (output +, red)
                  ---
        ___         
  -----|___|-------------------------- I (output -, blue) 
  GND  0.1R         

*/

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000UL

// 7 segment defines
#define SEG_A 0x04
#define SEG_B 0x08
#define SEG_C 0x10
#define SEG_D 0x80
#define SEG_E 0x01
#define SEG_F 0x02
#define SEG_G 0x40
#define SEG_DP 0x20

#define SEG_PORT PORTB

unsigned char number_patterns[23]={		SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F, //0
										SEG_B|SEG_C, //1
										SEG_A|SEG_B|SEG_D|SEG_E|SEG_G, //2
										SEG_A|SEG_B|SEG_C|SEG_D|SEG_G, //3
										SEG_G|SEG_B|SEG_C|SEG_F, //4
										SEG_A|SEG_G|SEG_C|SEG_D|SEG_F, //5
										SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G, //6
										SEG_A|SEG_B|SEG_C, //7
										SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G, //8
										SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G, //9
										SEG_DP, //dp (10)
										0, //nothing (11)
										SEG_A|SEG_G|SEG_C|SEG_D|SEG_F, //S (12)
										SEG_C|SEG_E|SEG_F|SEG_G, //h (13)
										SEG_G|SEG_C|SEG_D|SEG_E, //o (14)
										SEG_E|SEG_G, //r (15)
										SEG_F|SEG_E|SEG_G|SEG_D, //t (16)										
										SEG_A|SEG_F|SEG_G|SEG_E, //F (17)
										SEG_C|SEG_D|SEG_E, //v (18)
										SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G, // A (19)
										SEG_B|SEG_DP, // ! (20)
										SEG_C|SEG_D|SEG_E|SEG_F|SEG_G, // b (21)
										SEG_D|SEG_E|SEG_G // c (22)
									};

// anode defines, all six anodes
#define I_AN_1    0x20
#define I_AN_01   0x10
#define I_AN_001  0x08
#define V_AN_10   0x04
#define V_AN_1    0x02
#define V_AN_01   0x01

#define IV_AN_PORT PORTC									

unsigned char number_digits[6]={	V_AN_10,  // 10V voltage
									V_AN_1,   // 1V volate
									V_AN_01,  // 100mV voltage
									I_AN_1,	  // 1 Amp current
									I_AN_01,  // 100mA current
									I_AN_001, // 10mA current
								};


// variables for display
uint16_t volts, amps, ix, vx;
uint16_t v_tens, v_ones, v_tenth, v_tmp1, v_tmp2;
uint16_t i_tens, i_ones, i_tenth, i_tmp1, i_tmp2;

// function to set the digits (for common anode)
void set_digit_pattern(unsigned char digit,unsigned char pattern)
	{
		SEG_PORT=~(pattern);
		IV_AN_PORT=~(digit);
	}

void adc_init(void)
	{
		// ADC6 
		ADMUX = (0<<MUX0)|(1<<MUX1)|(1<<MUX2); // first ADC6, AREF=reference=3,3V
		// single mode, adc clock = 1/64 of 8MHz = 125kHz, interrupt enabled 
		ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); 
		/* Datasheet
		Bit 7: 1 ADEN Enable
		Bit 6: 0 ADSC Start Conversation
		Bit 5: 0 Single mode
		Bit 4: 0 Interrupt flag
		Bit 3: 1 Interrupt enable
		Bit 2: 1 PS2 Prescalar
		Bit 1: 1 PS1 Prescalar
		Bit 0: 0 PS0 Prescalar
		*/
	}

void adc_convert_v(void)  // the crazy method
	{
		// adc_convert(V);
		volts=ADC*3; // for 30V at v-out (voltage divider 26,7k - 3k3) .
		volts=volts-ix*0.3; // voltage correction, substract voltage across 0R1 (changed in V1.b)
		// tens:
		v_tens=volts;   
		v_tens>>=10;
		
		// ones:
		v_tmp1=v_tens;
		v_tmp1<<=10;
		v_tmp2=v_ones=(volts-v_tmp1) * 10; // V_tmp2 for tenth calculation
		v_ones>>=10; 
		
		// tenth:
 		v_tmp1=v_ones;
		v_tmp1<<=10; 
		v_tenth=(v_tmp2-v_tmp1) * 10;
		v_tenth>>=10; 
	}

void adc_convert_i(void)  // the crazy method
	{
		// adc_convert(I);		
		ix=ADC;  // used for correction the voltage
		amps=ix*3.15; // for 3A max at 100mOhm resistor (changed in V1.b)
		
		// tens:
		i_tens=amps;   
		i_tens>>=10;
		
		// ones:
		i_tmp1=i_tens;
		i_tmp1<<=10;
		i_tmp2=i_ones=(amps-i_tmp1) * 10; // V_tmp2 for tenth calculation
		i_ones>>=10; 
		
		// tenth:
 		i_tmp1=i_ones;
		i_tmp1<<=10; 
		i_tenth=(i_tmp2-i_tmp1) * 10;
		i_tenth>>=10; 
	}

// toggle variables
unsigned char sw;   		// switch this pass
unsigned char sw_last;     	// switch last pass
unsigned char sw_os; 		// going on one shot
unsigned char sw_on; 		// status

void toggle(void)
	{
		// toggle to switch on/off the power supply
		// PD2 button, PD4 Out
		sw=PIND & 0x04;  			// read switch at PD2
		sw_os=sw && !sw_last; 		// one shot of switch going on
		sw_last=sw;          		// remember last pass;
		if(sw_os) sw_on = !sw_on; 	// toggle state
		sw_on<<=4;					// left shift because PD4
		PORTD=sw_on;				// output toggle
		sw_on>>=4;					// shift right (back zu origin)
	}

// short detect variables
unsigned char sh;   				// variable for PD3
unsigned char sh_detect;     		// short detect

void short_circuit(void)
	{
		// detect pin PD3 if there is a short circuit
		sh=PIND & 0x08;				// read value at PD3
		sh>>=3;						// right shift PD3 to get 1 or 0
		sh_detect=sh;				// variable for case instruction
		sh<<=3;						// shift left (back zu origin)
	}


// variables for the display function
volatile unsigned char seg_count = 0;
char i;	

void display(void)
{
switch(sw_on) // display "v1.b off" 
					{
					case 1: // if PD4 = 1
						{
							switch (seg_count)
							{	
							case 0:
								set_digit_pattern(number_digits[0],number_patterns[(18)]);
								break;
							case 1:
								set_digit_pattern(number_digits[1],number_patterns[(1)]|SEG_DP);
								break;
							case 2:
								set_digit_pattern(number_digits[2],number_patterns[(22)]);
								break;
							case 3:
								set_digit_pattern(number_digits[3],number_patterns[(14)]);
								break;	
							case 4:
								set_digit_pattern(number_digits[4],number_patterns[(17)]);
								break;	
							case 5:
								set_digit_pattern(number_digits[5],number_patterns[(17)]);
								break;
							}
						if ((seg_count++)==6) {seg_count = 0; PORTB = 0xff;}  // start from the beginnig and switch all displays off
						break;
						}
					
					case 0: // if PD4 = 0, powered on
						{
						switch (sh_detect) // display "short"
							{
							case 1: 
								{
								switch (seg_count)
									{	
									case 0:
										set_digit_pattern(number_digits[seg_count],number_patterns[(12)]);
										break;
									case 1:
										set_digit_pattern(number_digits[seg_count],number_patterns[(13)]);
										break;
									case 2:
										set_digit_pattern(number_digits[seg_count],number_patterns[(14)]);
										break;
									case 3:
										set_digit_pattern(number_digits[seg_count],number_patterns[(15)]);
										break;
									case 4:
										set_digit_pattern(number_digits[seg_count],number_patterns[(16)]);
										break;
									case 5:
										set_digit_pattern(number_digits[seg_count],number_patterns[(20)]);
									break;
									}
								if ((seg_count++)==6) {seg_count = 0; PORTB = 0xff;}  // start from the beginnig and switch all displays off
								break;
								}
							
							case 0: // no short circuit + powered on
								{
								switch (seg_count) // display v + i
									{	
									case 0:
										set_digit_pattern(number_digits[seg_count],number_patterns[(v_tens)]);
										break;
									case 1:
										set_digit_pattern(number_digits[seg_count],number_patterns[(v_ones)]|SEG_DP);
										break;
									case 2:
										set_digit_pattern(number_digits[seg_count],number_patterns[(v_tenth)]);
										break;
									case 3:
										set_digit_pattern(number_digits[seg_count],number_patterns[(i_tens)]|SEG_DP);
										break;
									case 4:
										set_digit_pattern(number_digits[seg_count],number_patterns[(i_ones)]);
										break;
									case 5:
										set_digit_pattern(number_digits[seg_count],number_patterns[(i_tenth)]);
										break;
									}
								if ((seg_count++)==6) {seg_count = 0; PORTB = 0xff;}  // start from the beginnig and switch all displays off
								break;
								}
							break;
							}
						}
						break;
					}
				
	// do not start ad converter every ISR
	// prevents flickering the display
	if (i>200) // 200 just tested, is o.k.
	{
	ADCSRA |= (1<<ADSC);
	i = 0;
	}
	i++;
}

void timer_init(void)
	{
		TIMSK |= (1 << TOIE2);
		TCCR2 = (0<<CS22)|(1<<CS21)|(1<<CS20);
	}

ISR (TIMER2_OVF_vect)
	{
	toggle(); // check the on/off button
	short_circuit(); // check signal at PD3
	display(); // acitvate display
	}

unsigned char adc_count;
	
ISR (ADC_vect)
	{
			switch(adc_count)
			{
			case 0:
				adc_convert_v(); 
				ADMUX=(1<<MUX0)|(1<<MUX1)|(1<<MUX2); // switch to ADC7 for case 1
				adc_count = 1;
				break;
			case 1:
				adc_convert_i ();
				ADMUX=(0<<MUX0)|(1<<MUX1)|(1<<MUX2); // switch to ADC6 for case 0
				adc_count = 0;
				break;
			}
	}
	
int main(void)
{

DDRB = 0xff; // output all segments
DDRC = 0x3f; // output all anodes
DDRD = 0xf3; // on/off and short detect

PORTB = 0xff; // first all displays off... 

adc_init();
	
timer_init();
	
sei();

	while(1)
	{
	// thank you and good night...
	}
return 0;
}
