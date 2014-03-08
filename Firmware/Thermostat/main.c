/*
Thermostat
(c) Created by candrian on 3/27/13.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <avr/io.h>
#include <stdint.h>
#define F_CPU 8000000UL
#include <util/delay.h>     // Blocking delay functions
#include <avr/pgmspace.h>   // So we can store the 'font table' in ROM
#include <avr/interrupt.h>	// Interrupts and timers
#include <avr/eeprom.h>		// Date/time/pref backup in permanent EEPROM
#include <avr/sleep.h>      // Power save mode
#include <avr/wdt.h>


#include "main.h"

#include "lcd.h"
#include "lcd.c"

volatile    uint8_t minutes_set=0;
volatile    uint8_t hours_set=0;

volatile    uint8_t in_temp_set=0;
volatile    uint8_t pipe_temp_set=0;

volatile    uint16_t in_temp=0;
volatile    uint16_t pipe_temp=0;

volatile    uint8_t     rotary_state=NOTHING;
volatile    uint16_t    rotary_press_time=0;

volatile    uint8_t     menu_edit=OFF;

volatile    uint8_t     counter=0;
volatile    uint8_t     timer=OFF;

volatile    uint8_t     menu_timeout=0;
volatile    uint8_t     backlight_timeout=0;
volatile    uint8_t     backlight=ON;

volatile    uint8_t     in_temp_buffer[10];
volatile    uint8_t     pipe_temp_buffer[10];

volatile    uint8_t     buffer_pointer=0;

volatile    uint8_t     relay_pipe=OFF;
volatile    uint8_t     relay_in=OFF;


uint16_t adc_read(uint8_t sensor){
    ADMUX &= 0xf0;
    if (sensor==IN) {
        ADMUX   |=  (1<<MUX2);
    }else{
        ADMUX   |=  (1<<MUX2) | (1<<MUX0);
    }
    
    _delay_ms(1);
    ADCSRA  |=  _BV(ADSC);          //Start single conversion
    while(ADCSRA & _BV(ADSC));      //Wait to be completed
    
    return (ADC);
}

uint16_t adc_to_celsius(uint16_t adc){
    return((adc*500UL)/1023);
}

//Real Time Clock
ISR(TIMER2_OVF_vect){
    
    in_temp_buffer[buffer_pointer]=adc_to_celsius(adc_read(IN));
    pipe_temp_buffer[buffer_pointer]=adc_to_celsius(adc_read(PIPE));
    buffer_pointer++;
    if (buffer_pointer==9) buffer_pointer=0;
    
    if (backlight==ON && menu_edit==OFF) {
        backlight_timeout++;
        if (backlight_timeout==120) {
            backlight=OFF;
            backlight_timeout=0;
        }
    }
    
    if (menu_edit!=OFF) {
        menu_timeout++;
        
        if (menu_timeout==10) {
            
            eeprom_write_byte((uint8_t *)EE_IN_TEMP_1, in_temp_set);
            eeprom_write_byte((uint8_t *)EE_IN_TEMP_2, in_temp_set);
            
            eeprom_write_byte((uint8_t *)EE_PIPE_TEMP_1, pipe_temp_set);
            eeprom_write_byte((uint8_t *)EE_PIPE_TEMP_2, pipe_temp_set);
            
            eeprom_write_byte((uint8_t *)EE_SET_TIME_MINUTES_1, minutes_set);
            eeprom_write_byte((uint8_t *)EE_SET_TIME_MINUTES_2, minutes_set);
            
            eeprom_write_byte((uint8_t *)EE_SET_TIME_HOURS_1, hours_set);
            eeprom_write_byte((uint8_t *)EE_SET_TIME_HOURS_2, hours_set);
            
            menu_timeout=0;
            menu_edit=OFF;
            timer=OFF;
            
        }
        
    }else{
        menu_timeout=0;
    }
    
    if (timer==ON) {
        t.second++;
	}else{
        t.second=0;
        t.minute=0;
        t.hour=0;
    }
	//Minute passed
	if (t.second>=60) {
		t.second=0;
		t.minute++;
	}
	
	//Hour passed
	if (t.minute>=60) {
		t.minute=0;
		t.hour++;
	}
	
	//Day passed
	if (t.hour==99 && t.minute==60 && t.second==0) {
		t.hour=0;
        t.minute=0;
        t.second=0;
	}
    
}

//Main System
ISR(TIMER0_OVF_vect){
    
    //Enter setup menu or save
    if (rotary_state==BUTTON_LONG){

        if (menu_edit==OFF) {
            menu_edit=IN_TEMP;
            rotary_state=NOTHING;
        }else{
            //Save data
            eeprom_write_byte((uint8_t *)EE_IN_TEMP_1, in_temp_set);
            eeprom_write_byte((uint8_t *)EE_IN_TEMP_2, in_temp_set);
            
            eeprom_write_byte((uint8_t *)EE_PIPE_TEMP_1, pipe_temp_set);
            eeprom_write_byte((uint8_t *)EE_PIPE_TEMP_2, pipe_temp_set);
            
            eeprom_write_byte((uint8_t *)EE_SET_TIME_MINUTES_1, minutes_set);
            eeprom_write_byte((uint8_t *)EE_SET_TIME_MINUTES_2, minutes_set);
            
            eeprom_write_byte((uint8_t *)EE_SET_TIME_HOURS_1, hours_set);
            eeprom_write_byte((uint8_t *)EE_SET_TIME_HOURS_2, hours_set);
            
            menu_edit=OFF;
            timer=OFF;
            rotary_state=NOTHING;
        }
    }
    
    //Rotate menu setup
    if (rotary_state==BUTTON) {
        if (menu_edit!=OFF) {
            menu_edit++;
            if (menu_edit==5) menu_edit=IN_TEMP;
        }
        rotary_state=NOTHING;
    }
    
    if (menu_edit==IN_TEMP) {
        if (rotary_state==RIGHT){
            if (in_temp_set<140) {
                in_temp_set++;
            }else{
                beep();
            }
        }else if (rotary_state==LEFT){
            if (in_temp_set!=0) {
                in_temp_set--;
            }else{
                beep();
            }
        }
        rotary_state=NOTHING;
    }
    
    if (menu_edit==PIPE_TEMP) {
        if (rotary_state==RIGHT){
            if (pipe_temp_set<140) {
                pipe_temp_set++;
            }else{
                beep();
            }
        }else if (rotary_state==LEFT){
            if (pipe_temp_set!=0) {
                pipe_temp_set--;
            }else{
                beep();
            }
        }
        rotary_state=NOTHING;
    }

    if (menu_edit==SET_TIME) {
        if (rotary_state==RIGHT) {
            if (minutes_set==59) {
                minutes_set=0;
                hours_set++;
            }else{
                minutes_set++;
                if (hours_set==99 && minutes_set==59) {
                    hours_set=0;
                    minutes_set=0;
                }
            }
            rotary_state=NOTHING;
        }else if (rotary_state==LEFT){
            if (minutes_set==0 && hours_set!=0) {
                hours_set--;
                minutes_set=59;
            }else if (minutes_set==0 && hours_set==0){
                beep();
            }else{
                minutes_set--;
            }
            rotary_state=NOTHING;
        }else if (rotary_state==BUTTON){
            rotary_state=NOTHING;
        }
    }
    
}

ISR(TIMER1_OVF_vect) {
    
    counter++;
    if (counter>100) {
        counter=0;
    }
    
    print_temp(0,0,IN);
    print_temp(0,1,PIPE);
    
    if (menu_edit==IN_TEMP) {
        if (counter<50) {
            print_set_in_temp(6,0);
        }else{
            lcd_gotoxy(6,0);
            lcd_puts("   ");
        }
    }else{
        print_set_in_temp(6,0);
    }
    
    if (menu_edit==PIPE_TEMP) {
        if (counter<50) {
            print_set_pipe_temp(6,1);
        }else{
            lcd_gotoxy(6,1);
            lcd_puts("   ");
        }
    }else{
        print_set_pipe_temp(6,1);
    }
    
    if (relay_pipe==OFF) {
        lcd_gotoxy(10,1);
        lcd_putc('F');
        PORT_RELAY  &= ~(1<<RELAY_1);
    }else{
        lcd_gotoxy(10,1);
        lcd_putc('N');
        PORT_RELAY  |= (1<<RELAY_1);
    }
    
    if (relay_in==OFF) {
        lcd_gotoxy(10,0);
        lcd_putc('F');
        PORT_RELAY  &= ~(1<<RELAY_2);
    }else{
        lcd_gotoxy(10,0);
        lcd_putc('N');
        PORT_RELAY  |= (1<<RELAY_2);
    }
    
    if (menu_edit==SET_TIME) {
        if (counter<50) {
            print_set_time(12,1);
        }else{
            lcd_gotoxy(12,1);
            lcd_puts("     ");
        }
    }else{
        print_set_time(12,1);
    }
    
    print_time(12,0);
    
    if (backlight==ON) {
        PORT_BACKLIGHT  |=  (1<<BACKLIGHT);
    }else{
        PORT_BACKLIGHT  &=  ~(1<<BACKLIGHT);
    }
}

ISR(PCINT2_vect) {
    //Disable Pin interrupts
    PCICR       &=  ~(1<<PCIE2);
    //Allow other interrupts
    sei();
    //Debounce
    _delay_ms(DEBOUNCE);
    
    backlight=ON;
    
    if ((PIN_ROTARY & (1<<R_SW))==0) {
        rotary_state=BUTTON;
        menu_timeout=0;
        if (menu_edit!=OFF ) {
            beep();
        }
        while ((PIN_ROTARY & (1<<R_SW))==0) {
            
            if (rotary_press_time<151) {
                rotary_press_time++;
                _delay_ms(5);
            }
            
            if (rotary_press_time==150) {
                beep();
                _delay_ms(100);
                beep();
                rotary_state=BUTTON_LONG;
            }
        }
        rotary_press_time=0;

        PCICR       |=  (1<<PCIE2);
        return;
    }
    //Filter bounce
    if ((PIN_ROTARY & (1<<R_A))!=0) {
        //Enable Pin Interrupts
        PCICR       |=  (1<<PCIE2);
        return;
    }

    if ((PIN_ROTARY & (1<<R_B))==0) {
        //left
        rotary_state=LEFT;
        menu_timeout=0;
    }else{
        //right
        rotary_state=RIGHT;
        menu_timeout=0;
    }
    
    PCICR       |=  (1<<PCIE2);
}

void clock_init(){
    // Turn on the RTC by selecting the external 32khz crystal
	// 32.768 / 128 = 256 which is exactly an 8-bit timer overflow
	ASSR |= (1<<AS2);               //Timer/Counter 2 clocked from a crystal to the Timer Oscillator 1 (TOSC1) pin
	TCCR2B = (1<<CS22) | (1<<CS20); //clkT2S/128 (From prescaler)
    
    //Enable Timer/Counter2 Overflow interrupt
	TIMSK2 |= (1<<TOIE2);
    
}

void ports_init(){
    
    DDR_RELAY       |= (1<<RELAY_1) | (1<<RELAY_2);
    PORT_RELAY      &= ~(1<<RELAY_1) & ~(1<<RELAY_2);
    
    DDR_BEEPER      |=  (1<<BEEPER);
    PORT_BEEPER     &=  ~(1<<BEEPER);
    
    DDR_BACKLIGHT   |=  (1<<BACKLIGHT);
    PORT_BACKLIGHT  |=  (1<<BACKLIGHT);
    
    DDR_ROTARY      &=  ~(1<<R_A) & ~(1<<R_B) & ~(R_SW);
    
    //Enable pin change interrupt PCINT20
    PCMSK2      |=  (1<<PCINT21) | (1<<PCINT16);
    PCICR       |=  (1<<PCIE2); 
    
    TCCR0B  =   (1<<CS01)|(1<<CS00);    //f/64=31.250khz
    TIMSK0  =   (1<<TOIE0);             //Enable overflow interrupt
    
    TCCR1B |=   (1<<CS10);                  //clkI/O/1 (No prescaling)
    TIMSK1 |=   (1<<TOIE1);                 //Timer/Counter1 Overflow interrupt
}

void beep(){
    PORT_BEEPER     |=  (1<<BEEPER);
    _delay_ms(20);
    PORT_BEEPER     &=  ~(1<<BEEPER);

}

void adc_init(){
    DDRC    &=~ (1<<LM35_IN);   //Input for LM35
    DDRC    &=~ (1<<LM35_PIPE);  //Input for LM35

    ADMUX   =   (1<<REFS0);                             //AVCC with external capacitor at AREF pin
    ADMUX   |=  (1<<MUX2);                          //Select ADC1 channel
    
    ADCSRA  =   (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);   //Set prescaler to 128 16MHz/128 = 125kHz
    ADCSRA  |=   (1<<ADEN);                              //Enable ADC
}

void lcd_print_decimal(uint8_t value, uint8_t type, uint8_t digits){
    
    uint8_t temp=0;
    uint8_t hundreds=0;
    uint8_t dozens=0;
    uint8_t units=0;
    
    hundreds=value/0x64;
    temp=value%0x64;
    
    dozens=temp/0x0a;
    units=temp%0x0a;
    
    if (type!=TIME) {
        if (hundreds!=0) {
            lcd_putc(hundreds+0x30);
        }else{
            
            lcd_putc(' ');
        }
    }
    if (digits==1) {
        lcd_putc(units+0x30);
    }else{
        lcd_putc(dozens+0x30);
        lcd_putc(units+0x30);
    }

}

void print_time(uint8_t x, uint8_t y){
    lcd_gotoxy(x,y);
    lcd_print_decimal(t.hour,TIME,1);
    lcd_putc(':');
    lcd_print_decimal(t.minute,TIME,2);    
}

void print_set_time(uint8_t x, uint8_t y){
    lcd_gotoxy(x,y);
    lcd_print_decimal(hours_set,TIME,1);
    lcd_putc(':');
    lcd_print_decimal(minutes_set,TIME,2);
}

void print_set_in_temp(uint8_t x, uint8_t y){
    lcd_gotoxy(x,y);
    lcd_print_decimal(in_temp_set, TEMP,2);
    lcd_putc(DEGREE);
}

void print_set_pipe_temp(uint8_t x, uint8_t y){
    lcd_gotoxy(x,y);
    lcd_print_decimal(pipe_temp_set, TEMP,2);
    lcd_putc(DEGREE);
}

void print_temp(uint8_t x, uint8_t y, uint8_t sensor){
    
    lcd_gotoxy(x,y);
    
    if (sensor==IN) {
        lcd_print_decimal(in_temp, TEMP,2);
    }else{
        lcd_print_decimal(pipe_temp, TEMP,2);
    }
    lcd_putc(DEGREE);
    lcd_putc('C');
}

int main(void){
    
    uint16_t temp1=0, temp2=0, i;
    lcd_init(LCD_DISP_ON);

    _delay_ms(500);

    clock_init();
    ports_init();
    adc_init();
    
    lcd_clrscr();
    sei();
    beep();
    _delay_ms(100);
    beep();
    
    for (i=0;i<10;i++){
        in_temp_buffer[i]=21;
        pipe_temp_buffer[i]=21;
    }
        
    temp1=eeprom_read_byte((uint8_t *)EE_IN_TEMP_1);
    temp2=eeprom_read_byte((uint8_t *)EE_IN_TEMP_2);
    
    if (temp1==temp2) {
        in_temp_set=temp1;
    }else{
        eeprom_write_byte((uint8_t *)EE_IN_TEMP_1, 0);
        eeprom_write_byte((uint8_t *)EE_IN_TEMP_2, 0);
    }
    
    temp1=eeprom_read_byte((uint8_t *)EE_PIPE_TEMP_1);
    temp2=eeprom_read_byte((uint8_t *)EE_PIPE_TEMP_2);
    
    if (temp1==temp2) {
        pipe_temp_set=temp1;
    }else{
        eeprom_write_byte((uint8_t *)EE_PIPE_TEMP_1, 0);
        eeprom_write_byte((uint8_t *)EE_PIPE_TEMP_2, 0);
    }
    
    temp1=eeprom_read_byte((uint8_t *)EE_SET_TIME_MINUTES_1);
    temp2=eeprom_read_byte((uint8_t *)EE_SET_TIME_MINUTES_2);
    
    if (temp1==temp2) {
        minutes_set=temp1;
    }else{
        eeprom_write_byte((uint8_t *)EE_SET_TIME_MINUTES_1, 0);
        eeprom_write_byte((uint8_t *)EE_SET_TIME_MINUTES_2, 0);
    }
    
    temp1=eeprom_read_byte((uint8_t *)EE_SET_TIME_HOURS_1);
    temp2=eeprom_read_byte((uint8_t *)EE_SET_TIME_HOURS_2);
    
    if (temp1==temp2) {
        hours_set=temp1;
    }else{
        eeprom_write_byte((uint8_t *)EE_SET_TIME_HOURS_1, 0);
        eeprom_write_byte((uint8_t *)EE_SET_TIME_HOURS_2, 0);
    }
    
    while (1) {
        
        temp1=0;
        temp2=0;
        
        for (i=0; i<10; i++) {
            temp1+=in_temp_buffer[i];
            temp2+=pipe_temp_buffer[i];
        }
        
        in_temp=temp1/10;
        pipe_temp=temp2/10;
        
        if (timer==OFF) {
            if (pipe_temp>=pipe_temp_set) {
                relay_pipe=ON;
                timer=ON;
            }else{
                relay_pipe=OFF;
            }
        }
        
        if (in_temp<=in_temp_set) {
            relay_in=ON;
        }else{
            relay_in=OFF;
        }
        
    
        if (t.minute==minutes_set && t.hour==hours_set) {
            timer=OFF;
        }
        
    }

    return 0;
}
