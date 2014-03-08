#define BACKLIGHT       PB0
#define PORT_BACKLIGHT  PORTB
#define DDR_BACKLIGHT   DDRB


#define LM35_IN         PC4
#define LM35_PIPE       PC5
#define PORT_LM35       PORTC
#define DDR_LM35        DDRC

#define IN          0
#define PIPE        1

#define DEBOUNCE    0

#define R_A         PD5
#define R_B         PD6
#define R_SW        PD0

#define PORT_ROTARY PORTD
#define PIN_ROTARY  PIND
#define DDR_ROTARY  DDRD

#define PORT_BEEPER PORTD
#define DDR_BEEPER  DDRD
#define BEEPER      PD7

#define PORT_RELAY  PORTD
#define DDR_RELAY   DDRD
#define RELAY_1     PD2
#define RELAY_2     PD1

#define DEGREE      223 //Degree symbol

#define TIME        0
#define TEMP        1

#define NOTHING     0
#define RIGHT       1
#define LEFT        2
#define BUTTON      3
#define BUTTON_LONG 4

#define ON  0
#define OFF 1

#define IN_TEMP     2
#define PIPE_TEMP   3
#define SET_TIME    4

#define EE_IN_TEMP_1            0
#define EE_IN_TEMP_2            1
#define EE_PIPE_TEMP_1          2
#define EE_PIPE_TEMP_2          3
#define EE_SET_TIME_MINUTES_1   4
#define EE_SET_TIME_MINUTES_2   5
#define EE_SET_TIME_HOURS_1     6
#define EE_SET_TIME_HOURS_2     7

//Time struct
typedef struct{
	volatile uint8_t second; 
	volatile uint8_t minute;
	volatile uint8_t hour;
}time;
time t;