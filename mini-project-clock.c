/*
 * Author: Jack Andrews
 * UID: 9238363
 * Date: 11/12/2016
 * Name: mini-project-clock.c
 * Description: 
 * >Timer1 is used in conjunction with 32.768kHz oscillator on board and high-priority interrupts to generate a 1Hz clock for timekeeping
 * 
 * >Timer0 is run from the instruction clock to generate an approximate 1ms delay used for other tasks & millisecond counters such as:
 *      -7-segment display/LEDs multiplexing (happens at around 1ms)
 *      -Millisecond counters for:
 *          >Cycling of display of date/time (ms_count0)
 *          >Debounce delay for push buttons (ms_count1)
 *          >Polling of alarms to check whether they should be sounded (ms_count2)
 *          >Timing of length of alarm tone notes (ms_count3)
 * 
 * >The program has basic error reporting/debugging built in when running on the PIC. Errors are denoted by 'Er' on the display, with the error code displayed in
 *  binary on the LEDs. The error codes are:
 *      -Er (1) - Function Num2Disp has been passed an integer which is outside the range 0<=x<=99 and cannot display it
 *      -Er (2) - The combination of toggle switches does not correspond to a menu option. Correct this to enter a defined mode
 *      -Er (3) - Function CurrentDisplay has been passed an index which is outside the range expected and doesn't have anything to display for that index
 *      -Er (4) - The combination of toggle swithces does not correspond to a setting option for either Alarm1/Alarm2
 * 
 * Notes:
 * [1] C18 Peripheral Library OpenTimer functions have not been used due to incompatibilities between the XC8/C18 library versions in use on the computer used
 *     to develop this program. With different version combinations, the compiler will probably work fine.
*/

//Header file include pre-processor directives
#include "18f8722_config_settings.h"
#include "plib/timers.h"
#include "plib/delays.h"

//Various pre-processor directives for global delays used in the program to allow easy editing
//Delays are given in multiples of 10/100/1000/10,000 TCY, unless otherwise stated
#define SET_MENU_FLASH 100          //Rate at which dd/mm/yy hh:mm:ss flashes upon entering set time/date mode
#define ALARM_TOGGLE 150            //Rate at which display toggles between alarm no. (A1/A2) and setting (on/off) in alarm set mode
#define DEBOUNCE_DELAY 25           //(milliseconds) Delay for debouncing push-buttons
#define KEY_REPEAT_DELAY 25         //Rate at which value increments/decrements when a button is held repeatedly
#define DISPLAY_CYCLE_DELAY 3000    //(milliseconds) Rate at which display cycles between dd/mm/yy hh:mm:ss when in normal mode
#define ALARM_POLL_RATE 50          //(milliseconds) How often should the alarms be polled to see if they are equal to the main date/time
#define ALARM_REPEAT_DELAY 100      //Delay between repetition of alarm tone

#define TIMER0_VALUE 63036          //Value loaded into Timer0 to produce ~1ms delay
#define TIMER1_VALUE 32768          //Value loaded into Timer1 to produce 1 second delay (for RTC)

//Define bit patterns to display the following on LEDs or to take inputs from the switches
#define HRS 0x04
#define MINS 0x02
#define SECS 0x01
#define DAY 0x20
#define MONTH 0x10
#define YEAR 0x08
#define ALARM1 0x80
#define ALARM2 0x40

//Define notes from C4 (middle C) to C6
//These are given as half the no. of 10*TCYs required to generate the frequency of the note,
//to avoid overflowing the Delay10KTCYx function in the pre-processor macro below
//Notes with an 'S' in them are sharps
#define D6  53
#define	C6	60
#define	B5	63
#define	AS5	67
#define	A5	71
#define	GS5	75
#define	G5	80
#define	FS5	84
#define	F5	89
#define	E5	95
#define	DS5	100
#define	D5	106
#define	CS5	113
#define	C5	119
#define	B4	127
#define	AS4	134
#define	A4	142
#define	GS4	150
#define	G4	159
#define	FS4	169
#define	F4	179
#define	E4	190
#define	DS4	201
#define	D4	213
#define	CS4	225
#define	C4	239

//Define the lengths of notes in milliseconds
#define SEMIBREVE 800
#define MINIM (SEMIBREVE / 2)
#define CROTCHET (MINIM / 2)
#define QUAVER  (CROTCHET / 2)
#define SEMIQUAVER (QUAVER /2)

//Pre-processor macro to generate a note for a particular length of time
#define GEN_NOTE(length, note, delay) \
        while(ms_count3 <= length && !PB1pressed() && !PB2pressed()) {  /*Test to see if time for note has elapsed or if PB1/PB2 have been pressed (terminates alarm)*/ \
            LATJbits.LATJ6 = 1;                                         /*If time hasn't elapsed, play note by generating a square wave of a specific frequency*/ \
            Delay10TCYx(note);                                          /*on RJ6 (piezo buzzer), by setting it high, delaying, low, delaying*/ \
            Delay10TCYx(note); \
            LATJbits.LATJ6 = 0; \
            Delay10TCYx(note); \
            Delay10TCYx(note); \
            }                  \
        ms_count3 = 0;         \
        while(ms_count3 <= delay && !PB1pressed() && !PB2pressed());  /*Generate a delay between notes equal to the length of delay passed in*/ \
        ms_count3 = 0; \
        if(PB1pressed() || PB2pressed())                               /*Test to see if PB1/PB2 have been pressed, if so, break from playing alarm tone as alarm*/ \    
            break;                                                     /*has been acknowledged and reset*/
            
#define GEN_PAUSE(length) \
        while(ms_count3 <= length && !PB1pressed() && !PB2pressed()) { \
            LATJbits.LATJ6 = 0; \
            } \
        ms_count3 = 0; \
        if(PB1pressed() || PB2pressed()) \
            break;
            
//Define a type TIME as a struct with 3 members to store times            
typedef struct {
    char hrs;
    char mins;
    char secs;
} TIME;

//Define a type DATE as a struct with 4 members to store dates
typedef struct {
    char day;
    char month;
    char year_short;
    unsigned int year_long;
} DATE;

//Function protoypes for compiler
void interrupt hp_secs_count_isr(void);     //High-priority ISR (1Hz clock)
void interrupt low_priority lp_isr(void);   //Low-priority ISR (1ms clock for system tasks)
void Timer1_isr(void);                      //ISR for Timer1 interrupt source
void Timer0_isr(void);                      //ISR for Timer0 interrupt source
void enable_interrupts_all(void);           //Enable all interrupts (global)
void disable_interrupts_all(void);          //Disable all interrupts (global)

void StartTimer0(void);                     //Configures & starts Timer0
void StartTimer1(void);                     //Configures & starts Timer1

void Num2Disp(volatile char *time);         //Displays the number (0 <= x <= 99) on the 7-segment displays
void CurrentDisplay(char *i);               //Displays the dd/mm/yy hh:mm:ss corresponding to the disp_index, i, on the 7-segment displays
void SetMenu(void);                         //Settings menu to provide set date/time/alarm functionality

char Switches(void);                        //Returns the value of the 8-bit toggle switches on the School IOB
char PB1pressed(void);                      //Returns true (1) if PB1 has been pressed, false (0) if not. Implements de-bouncing
char PB2pressed(void);                      //Returns true (1) if PB2 has been pressed, false (0) if not. Implements de-bouncing

void ConfigureIO(void);                     //Configure the PIC IO pins for IO on the School IOB using TRIS registers
void BootTest(void);                        //Boot test routine to check all 7-segment displays, LEDs and buzzer are working

void CalcTime(void);                        //Calculate the time if multiple minutes have rolled over
void CalcDate(void);                        //Calculate the date (including leap years) if a day has rolled over
char CalcLeapYear(unsigned int year);       //Calculate whether a particular year is a leap year or not. Returns true (1) if it is, false (0) if not

void SetSecs(volatile TIME *ts);            //Set the seconds member of the time struct passed to it
void SetMins(volatile TIME *tm);            //Set the minutes member of the time struct passed to it
void SetHrs(volatile TIME *th);             //Set the hours member of the time struct passed to it
void SetDay(volatile DATE *dd);             //Set the day member of the date struct passed to it
void SetMonth(volatile DATE *dm);           //Set the month member of the date struct passed to it
void SetYear(volatile DATE *dy);            //Set the year member of the date struct passed to it

void SecsFlash(void);                       //Flash 7-segment displays with 'SS' when entering seconds set mode
void MinsFlash(void);                       //Flash 7-segment displays with 'mi' when entering minutes set mode
void HrsFlash(void);                        //Flash 7-segment displays with 'hh' when entering hours set mode
void DayFlash(void);                        //Flash 7-segment displays with 'dd' when entering day set mode
void MonthFlash(void);                      //Flash 7-segment displays with 'mo' when entering month set mode
void YearFlash(void);                       //Flash 7-segment displays with 'yy' when entering year set mode

void Alarm1Flash(void);                     //Flash 7-segment displays with 'A1' when entering Alarm1 set mode
void SetAlarm1(void);                       //Enables/disables Alarm1 and sets the hh:mm:ss that Alarm1 will occur at
void SoundAlarm1(void);                     //Sounds Alarm1 melody and acknowledges it with a press of PB1/PB2
void Alarm2Flash(void);                     //Flash 7-segment displays with 'A2' when entering Alarm2 set mode
void SetAlarm2(void);                       //Enables/disables Alarm2 and sets the dd/mm/yy hh:mm:ss that Alarm2 will occur at
void SoundAlarm2(void);                     //Sounds Alarm2 melody and acknowledges it with a press of PB1/PB2

char CompareTimes(volatile TIME mainTime, volatile DATE *mainDate, volatile TIME *alarmTime, volatile DATE *alarmDate, char args); //Compares the date and/or time members of the structs passed to it, returns true (1) if equal, false (0) if not. Used for Alarm1/2


//CONSTANT GLOBAL VARIABLES
//Array of chars containing bit patterns to display numbers 0->9 on 7-segment displays
const char DispNums[] = { 0x84, 0xF5, 0x4C, 0x64, 0x35, 0x26, 0x06, 0xF4, 0x04, 0x34 };

//Struct containing members which represent all alphabetical characters which can be displayed on 7-segment displays
const struct {
    char A;
    char b;
    char C;
    char c;
    char d;
    char E;
    char F;
    char g;
    char H;
    char h;
    char I;
    char i;
    char J;
    char L;
    char M;
    char n;
    char o;
    char P;
    char r;
    char S;
    char t;
    char U;
    char u;
    char y;
    char uo;
} DispChars = { 0x14, 0x07, 0x8E, 0x4F, 0x45, 0x0E, 0x1E, 0x24, 0x15, 0x17, 0x9F, 0xDF, 0xC5, 0x8F, 0xD6, 0x57, 0x47, 0x1C, 0x5F, 0x26, 0x0F, 0x85, 0xC7, 0x25, 0x3C };  //Initialise struct DispChars with bit patterns corresponding to each character

//Array of chars containing number of days in each month for non-leap years. 0 first element is so that array can be disp_indexed using months of the year, which start at 1
const char DaysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

//Array of chars containing number of days in each month for leap years
const char DaysInMonthLeap[] = {0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

//GLOBAL VARIABLES
char disp_index = 0;         //Display cycle disp_index, used to track what is being shown (dd/mm/yy hh:mm:ss) on 7-segment displays currently. Used in conjunction with CurentDisplay() function
char Alarm1On = 0;      //Flag to enable/disable Alarm1
char Alarm2On = 0;      //Flag to enable/disable Alarm2

//Volatile variables modified in ISRs
volatile char multiplex_index = 1;          //Used to track which display is currently illuminated for multiplexing purposes
volatile unsigned int ms_count0 = 0;        //millisecond counter variables, incremented by Timer0 ISR, reset by functions which use them
volatile unsigned int ms_count1 = 0;
volatile unsigned int ms_count2 = 0;
volatile unsigned int ms_count3 = 0;
volatile char disp_U1, disp_U2, disp_LEDS;  //char variables to hold bit patterns of current output on 7-segment displays/LEDs. These are modified by functions when they change what is displayed
volatile char dp_mask = 0xFF;               //Mask used for decimal point of 7-segment display U1, so that second indicator keeps flashing when in set modes
volatile char day_rollover = 0;             //Flag, set when a day rollover (23:00->00:00HRS) has occurred
volatile char mins_rollover = 0;            //Flag, set when a minute rollover has occurred 

volatile TIME MainTime, Alarm1Time, Alarm2Time;     //Declare structs of type TIME to store the RTC, Alarm1 & Alarm2 times
volatile DATE MainDate, Alarm1Date, Alarm2Date;     //Declare structs of type DATE to store the RTC, Alarm1 & Alarm2 dates

//Main function
void main(void) {
   
    //Initialise all time/date structs
    MainTime.hrs = 0;
    MainTime.mins = 0;
    MainTime.secs = 0;
    
    MainDate.day = 1;
    MainDate.month = 1;
    MainDate.year_short = 16;
    MainDate.year_long = 2016;

    Alarm1Time.hrs = 0;
    Alarm1Time.mins = 0;
    Alarm1Time.secs = 0;
    
    Alarm1Date.day = 0;
    Alarm1Date.month = 0;
    Alarm1Date.year_long = 0;
    Alarm1Date.year_short = 0;

    Alarm2Time.hrs = 0;
    Alarm2Time.mins = 0;
    Alarm2Time.secs = 0;

    Alarm2Date.day = 1;
    Alarm2Date.month = 1;
    Alarm2Date.year_long = 2016;
    Alarm2Date.year_short = 16;
    
    ConfigureIO();              //Configure IO of PIC

    StartTimer0();              //Configure & start Timer0 to allow display multiplexing
    WriteTimer0(TIMER0_VALUE);         //Write initial value to produce ~1ms delay
        
    enable_interrupts_all();    //Enable all interrupts (globally)
    
    BootTest();                 //Run the boot test to check that the 7-segment displays, LEDs & buzzer are working

    StartTimer1();              //Configure & start Timer1 to start the RTC
    WriteTimer1(TIMER1_VALUE);         //Write initial value to produce a 1Hz clock        

    //Main while loop, this supervises the scrolling display of date/time, calls functions to evaluate date/time, triggers alarms & tests toggle switches for input
    while (1) {                         
        
        if (mins_rollover >= 1) {       //Calculates time if minutes has rolled over
            CalcTime();
        }
        if (day_rollover == 1) {        //Calculates date if day has rolled over
            CalcDate();
        }

        if (ms_count0 >= DISPLAY_CYCLE_DELAY) {     //Cycle through dd/mm/yy hh:mm:ss on 7-segment displays by incrementing disp_index
            ms_count0 = 0;
            if (disp_index < 5) {
                disp_index++;
            } else {
                disp_index = 0;
            }
        }

        if (PB1pressed() == 1) {                 //If PB1 has been pressed and is held, cycle through dd/mm/yy hh:mm:ss on display by incrementing disp_index 
            Delay10KTCYx(KEY_REPEAT_DELAY);
            if (PB1pressed() == 1) {
                ms_count0 = 0;
                if (disp_index < 5) {
                    disp_index++;
                } else {
                    disp_index = 0;
                }
            }
        }

        if (PB2pressed() == 1) {                //If PB2 has been pressed and is held, cycle through dd/mm/yy hh:mm:ss on display by incrementing disp_index
            Delay10KTCYx(KEY_REPEAT_DELAY);
            if (PB2pressed() == 1) {
                ms_count0 = 0;
                if (disp_index > 0) {
                    disp_index--;
                } else {
                    disp_index = 5;
                }
            }
        }

        CurrentDisplay(&disp_index);    //Display date/time element corresponding to disp_index on 7-segment display

        if (Switches() != 0x00) {       //Test if any of the toggle switches have been set, if so, enter the setting menu
            SetMenu();
        }
        
        if (ms_count2 >= ALARM_POLL_RATE) {     //Check whether Alarm1/Alarm2 dates/times are equal at polling interval set by ALARM_POLL_RATE
            if((CompareTimes(MainTime, &MainDate, &Alarm1Time, &Alarm1Date, 1) && Alarm1On) == 1) {     //If they are equal and the alarm is enabled,
                SoundAlarm1();                                                                          //sound the relevant alarm
                
            }
            if((CompareTimes(MainTime, &MainDate, &Alarm2Time, &Alarm2Date, 2) && Alarm2On) == 1) {
                SoundAlarm2();
            }
            ms_count2 = 0;
        }

    }

    
}

void interrupt hp_secs_count_isr(void) {     
    if (PIR1bits.TMR1IF == 1) {             //Check interrupt source to see if it came from Timer1
        PIR1bits.TMR1IF = 0;                //Clear interrupt flag
        WriteTimer1(TIMER1_VALUE);                 //Re-load timer to generate next 1 second delay
        Timer1_isr();                       //Call interrupt routine
    }
}

void interrupt low_priority lp_isr(void) {
    if(INTCONbits.TMR0IF == 1) {
        INTCONbits.TMR0IF = 0;
        WriteTimer0(TIMER0_VALUE);
        Timer0_isr();
    }
}

void Timer1_isr(void) {         
    if (MainTime.secs < 59)     //If second count is under a minute, continue to increment
        MainTime.secs++;
    else {                      
        MainTime.secs = 0;     //Else, reset seconds back to 0
        mins_rollover++;       //and set minute rollover flag for main function
    }
    dp_mask ^= (1 << 2);       //Toggle decimal point to provide 1Hz flash for timing
}

void Timer0_isr(void) {
    switch(multiplex_index) {               //Switch case to cycle through display on U1, U2 and LEDs
            case(1) :                       //The current display is kept track of by multiplex_index
                LATHbits.LH0 = 1;           //In each case, outputs are set/cleared to enable/disable U1/U2/LEDs
                LATHbits.LH1 = 1;           //in turn, then the value to be displayed is put onto LATF
                LATAbits.LA4 = 1;
                LATF = disp_LEDS;
                break;
            case(2) :
                LATHbits.LH0 = 0;
                LATHbits.LH1 = 1;
                LATAbits.LA4 = 0;
                LATF = disp_U1 & dp_mask;
                break;
            case(3) :
                LATHbits.LH0 = 1;
                LATHbits.LH1 = 0;
                LATAbits.LA4 = 0;
                LATF = disp_U2;
                multiplex_index = 0;
                break;
            default :
                multiplex_index = 0;        //Reset multiplex_index back to 0 to prevent undefined behaviour
                break;
        }
        multiplex_index++;                  //Increment index & millisecond counters
        ms_count0++;
        ms_count1++;
        ms_count2++;
        ms_count3++;
}

void enable_interrupts_all(void) {
    RCONbits.IPEN = 1;                  //Enable prioritised interrupts
    INTCONbits.PEIE = 1;                //Enable interrupts from peripherals
    INTCONbits.GIE = 1;                 //Enable global interrupts
}

void disable_interrupts_all(void) {
    INTCONbits.PEIE = 0;                //Disable peripheral & global interrupts
    INTCONbits.GIE = 0;
}

void StartTimer0(void) {
    T0CON = 0x08;                   //Configure Timer0 as 16-bit, internal clock source, prescaler disabled, but don't turn it on yet
    TMR0H = 0;                      //Clear timer registers
    TMR0L = 0;
    INTCONbits.TMR0IF = 0;          //Clear interrupt flag
    INTCONbits.TMR0IE = 1;          //Enable Timer0 interrupt
    INTCON2bits.TMR0IP = 0;         //Set as low-priority interrupt
    T0CONbits.TMR0ON = 1;           //Turn on Timer0
}

void StartTimer1(void) {
    T1CON = 0x8A;                   //Configure Timer1 as 16-bit, external clock source, 1:1 prescaler, enable oscillator power, don't synchronise clock, but don't turn it on yet
    TMR1H = 0;                      //Clear timer registers
    TMR1L = 0;
    PIR1bits.TMR1IF = 0;            //Clear interrupt flag
    PIE1bits.TMR1IE = 1;            //Enable Timer1 interrupt
    IPR1bits.TMR1IP = 1;            //Set as high-priority interrupt
    T1CONbits.TMR1ON = 1;           //Turn on Timer1
}

void Num2Disp(volatile char *time) {
    char tens, units;               //Two temporary variables to store use as indexes for DispNums[] array
    if(*time > 99) {
        disp_U1 = DispChars.r;      //Display error code 0x01 on LEDs if value is outside range as numbers greater than this cannot be displayed on the 7-segment displays
        disp_U2 = DispChars.E;
        disp_LEDS = 0x01;
        return;
    }
    else {
        tens = *time / 10;          //If number is in the range, calculate the tens & units components
        units = *time % 10;         //using the modulo operator
        disp_U2 = DispNums[tens];   //and copy the bit patterns from DispNums[] corresponding to these integers
        disp_U1 = DispNums[units];  //to the U1 & U2 current display variables
        return;
    }
}

void CurrentDisplay(char *i) {
    switch(*i) {                                //Display either dd/mm/yy hh:mm:ss on displays & LEDs as dictated by the index, i, passed into it
        case(0) : 
            Num2Disp(&MainDate.day);
            disp_LEDS = DAY;
            break;
        case(1) :
            Num2Disp(&MainDate.month);
            disp_LEDS = MONTH;
            break;
        case(2) :
            Num2Disp(&MainDate.year_short);
            disp_LEDS = YEAR;
            break;
        case(3) :
            Num2Disp(&MainTime.hrs);
            disp_LEDS = HRS;
            break;
        case(4) :
            Num2Disp(&MainTime.mins);
            disp_LEDS = MINS;
            break;
        case(5) :
            Num2Disp(&MainTime.secs);
            disp_LEDS = SECS;
            break;
        default :
            disp_U2 = DispChars.E;
            disp_U1 = DispChars.r;
            disp_LEDS = 0x03;
            break;
    }
}

char Switches(void) {           
    char temp, temp1, temp2; 
    temp1 = PORTC;              //Using bit shifting & masking operations, returns the value of the toggle switches
    temp1 >>= 2;
    temp1 &= 0x0F;
    temp2 = PORTH;
    temp2 &= 0xF0;
    temp = (temp1 | temp2);
    return(temp);
}

void SetMenu(void) {
    while (Switches() != 0x00) {                //This function implements the main setting menu to set date/time & alarms, based upon the combination of toggle
        switch (Switches()) {                   //switches set. For all date/time set operations, the 1Hz RTC is disabled to 'freeze' the time, and is re-enabled
            case(SECS):                         //upon exiting the set routine. Comments are given for the seconds & Alarm1 cases, other cases are similar
                PIE1bits.TMR1IE = 0;            //Disable Timer1 interrupt to 'freeze' time
                SecsFlash();                    //Flash 'SS' on displays to show user seconds set mode has been entered
                Num2Disp(&MainTime.secs);       //Display the current seconds value of the Main RTC time on the displays
                while (Switches() == SECS) {    //Stay in the seconds set routine while toggle switches are set to indicate this
                    SetSecs(&MainTime);         //Set seconds member of MainTime by passing in address of MainTime (saves time & processor resources)
                    Num2Disp(&MainTime.secs);   //Update the display with the new MainTime.secs value as it is changed by the user
                }
                PIE1bits.TMR1IE = 1;            //Re-enable 1Hz RTC interrupt to 'un-freeze' time
                break;
            case(MINS):
                PIE1bits.TMR1IE = 0;
                MinsFlash();
                Num2Disp(&MainTime.mins);
                while (Switches() == MINS) {
                    SetMins(&MainTime);
                    Num2Disp(&MainTime.mins);
                }
                PIE1bits.TMR1IE = 1;
                break;
            case(HRS):
                PIE1bits.TMR1IE = 0;
                HrsFlash();
                Num2Disp(&MainTime.hrs);
                while (Switches() == HRS) {
                    SetHrs(&MainTime);
                    Num2Disp(&MainTime.hrs);
                }
                PIE1bits.TMR1IE = 1;
                break;
            case(DAY):
                PIE1bits.TMR1IE = 0;
                DayFlash();
                Num2Disp(&MainDate.day);
                while (Switches() == DAY) {
                    SetDay(&MainDate);
                    Num2Disp(&MainDate.day);
                }
                PIE1bits.TMR1IE = 1;
                break;
            case(MONTH):
                PIE1bits.TMR1IE = 0;
                MonthFlash();
                Num2Disp(&MainDate.month);
                while (Switches() == MONTH) {
                    SetMonth(&MainDate);
                    Num2Disp(&MainDate.month);
                }
                PIE1bits.TMR1IE = 1;
                break;
            case(YEAR):
                PIE1bits.TMR1IE = 0;
                YearFlash();
                Num2Disp(&MainDate.year_short);
                while (Switches() == YEAR) {
                    SetYear(&MainDate);
                    Num2Disp(&MainDate.year_short);
                }
                PIE1bits.TMR1IE = 1;
                break;
            case(ALARM1):                           //Enter alarm set mode if switches are set accordingly
                Alarm1Flash();                      //Flash 'A1' on displays to show user alarm set mode has been entered
                while ((Switches() >> 7) == 1) {    //While bit 7 of switches remains set, remain in alarm set mode
                    SetAlarm1();                    //Set alarm 1
                }
                break;
            case(ALARM2):
                Alarm2Flash();
                while ((Switches() >> 6) == 1) {
                    SetAlarm2();
                }
                break;
            default:
                disp_U2 = DispChars.E;              //Default case if other switch combinations are used which don't correspond to menu options
                disp_U1 = DispChars.r;              //Display error code 2 to indicate this to user. Clock remains running in background.
                disp_LEDS = 0x02;
                break;
        }
    }
}

char PB1pressed(void) {
    if(PORTJbits.RJ5 == 0) {
        ms_count1 = 0;
        while(ms_count1 < DEBOUNCE_DELAY) {
        }
        if(PORTJbits.RJ0 == 0) {
            return(1);
        }
        else {
            return(0);
        }
    }
    else {
        return(0);
    }
}

char PB2pressed(void) {
    if(PORTBbits.RB0 == 0) {
        ms_count1 = 0;
        while(ms_count1 < DEBOUNCE_DELAY) {
        }
        if(PORTBbits.RB0 == 0) {
            return(1);
        }
        else {
            return(0);
        }
    }
    else {
        return(0);
    }
}

void ConfigureIO(void) {
    ADCON1 = 0x3F;
    TRISF = 0x00;
    TRISH = 0xFC;
    TRISA = 0xEF;
    TRISB = 0xFF;
    TRISC = 0xFF;
    TRISJ = 0xBF; 
}

void BootTest(void) {
    disp_LEDS = 0xFF;
    disp_U1 = 0x00;
    disp_U2 = 0x00;
    while (ms_count3 <= SEMIBREVE) {
        LATJbits.LATJ6 = 1;
        Delay10TCYx(C5);
        Delay10TCYx(C5);
        LATJbits.LATJ6 = 0;
        Delay10TCYx(C5);
        Delay10TCYx(C5);
    }
    disp_LEDS = 0x00;
    disp_U1 = 0xFF;
    disp_U2 = 0xFF;
    Delay10KTCYx(250);
}

void CalcTime(void) {
    char mins_temp = 0; 
    mins_temp = MainTime.mins + mins_rollover;
    if (mins_temp < 59) {
        MainTime.mins = mins_temp;
    }
    else {
        MainTime.mins = mins_temp - 60;
        if (MainTime.hrs < 23) {
            MainTime.hrs++;
        } else {
            MainTime.hrs = 0;
            day_rollover = 1;
        }
    }
    mins_rollover = 0;
}

void CalcDate(void) {
    day_rollover = 0;
    if(CalcLeapYear(MainDate.year_long) == 0) {
        if(MainDate.day < DaysInMonth[MainDate.month]) {
            MainDate.day++;
        }
        else {
            MainDate.day = 1;
            if(MainDate.month < 12) {
                MainDate.month++;
            }
            else {
                MainDate.month = 1;
                if(MainDate.year_long < 99) {
                    MainDate.year_long++;
                    MainDate.year_short++;
                }
                else {
                    MainDate.year_long = 2000;
                    MainDate.year_short = 00;
                }
                
            }
        }
    }
    else {
       if(MainDate.day < DaysInMonthLeap[MainDate.month]) {
            MainDate.day++;
        }
        else {
            MainDate.day = 1;
            if(MainDate.month < 12) {
                MainDate.month++;
            }
            else {
                MainDate.month = 1;
                MainDate.year_long++;
                MainDate.year_short++;
            }
        }
    }
}

char CalcLeapYear(unsigned int year) {
    if((year % 4) == 0) {
        if((year % 100) == 0) {
            if ((year % 400) == 0) {
                return(1);
            }
            else {
                return(0);
            }
        }
        else {
            return(1);
        }
    }
    else {
        return(0);
    }
}

void SetSecs(volatile TIME *ts) {
    if(PB2pressed() && ts->secs < 59) {
        ts->secs++;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB2pressed() && ts->secs == 59) {
        ts->secs =  0;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB1pressed() && ts->secs > 0) {
        ts->secs--;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB1pressed() && ts->secs == 0) {
        ts->secs = 59;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
}

void SetMins(volatile TIME *tm) {
    if(PB2pressed() && tm->mins < 59) {
        tm->mins++;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB2pressed() && tm->mins == 59) {
        tm->mins = 0;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB1pressed() && tm->mins > 0) {
        tm->mins--;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB1pressed() && tm->mins == 0) {
        tm->mins = 59;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
}

void SetHrs(volatile TIME *th) {
    if(PB2pressed() && th->hrs < 23) {
        th->hrs++;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB2pressed() && th->hrs == 23) {
        th->hrs = 0;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB1pressed() && th->hrs > 0) {
        th->hrs--;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if(PB1pressed() && th->hrs == 0) {
        th->hrs = 23;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
}

void SetDay(volatile DATE *dd) {
    if(CalcLeapYear(dd->year_long) == 1) {
        if(PB2pressed() && dd->day < DaysInMonthLeap[dd->month]) {
            dd->day++;
            Delay10KTCYx(KEY_REPEAT_DELAY);
        }
        if(PB2pressed() && dd->day == DaysInMonthLeap[dd->month]) {
            dd->day = 1;
            Delay10KTCYx(KEY_REPEAT_DELAY);
        }
        if(PB1pressed() && dd->day > 1) {
            dd->day--;
            Delay10KTCYx(KEY_REPEAT_DELAY);
        }
        if(PB1pressed() && dd->day == 1) {
            dd->day = DaysInMonth[MainDate.month];
            Delay10KTCYx(KEY_REPEAT_DELAY);
       }
    }
    else {
        if(PB2pressed() && dd->day < DaysInMonthLeap[dd->month]) {
            dd->day++;
            Delay10KTCYx(KEY_REPEAT_DELAY);
        }
        if(PB2pressed() && dd->day == DaysInMonth[dd->month]) {
            dd->day = 1;
            Delay10KTCYx(KEY_REPEAT_DELAY);
        }
        if(PB1pressed() && dd->day > 1) {
            dd->day--;
            Delay10KTCYx(KEY_REPEAT_DELAY);
        }
        if(PB1pressed() && dd->day == 1) {
            dd->day = DaysInMonth[MainDate.month];
            Delay10KTCYx(KEY_REPEAT_DELAY);
       }
    }


}

void SetMonth(volatile DATE *dm) {
    if (PB2pressed() && dm->month < 12) {
        dm->month++;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if (PB2pressed() && dm->month == 12) {
        dm->month = 1;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if (PB1pressed() && dm->month > 1) {
        dm->month--;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if (PB1pressed() && dm->month == 1) {
        dm->month = 12;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
}

void SetYear(volatile DATE *dy) {
    if (PB2pressed() && dy->year_long < 2099) {
        dy->year_long++;
        dy->year_short++;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if (PB2pressed() && dy->year_long == 2099) {
        dy->year_long = 2000;
        dy->year_short = 00;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if (PB1pressed() && dy->year_long > 2000) {
        dy->year_long--;
        dy->year_short--;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
    if (PB1pressed() && dy->year_long == 2000) {
        dy->year_long = 2099;
        dy->year_short = 99;
        Delay10KTCYx(KEY_REPEAT_DELAY);
    }
}

void SecsFlash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS |= SECS;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.S;
    disp_U1 = DispChars.S;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.S;
    disp_U1 = DispChars.S;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
}

void MinsFlash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS |= MINS;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.M;
    disp_U1 = DispChars.i;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.M;
    disp_U1 = DispChars.i;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
}

void HrsFlash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS |= HRS;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.h;
    disp_U1 = DispChars.h;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.h;
    disp_U1 = DispChars.h;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
}

void DayFlash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS |= DAY;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.d;
    disp_U1 = DispChars.d;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.d;
    disp_U1 = DispChars.d;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
}

void MonthFlash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS &= MONTH;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.M;
    disp_U1 = DispChars.o;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.M;
    disp_U1 = DispChars.o;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
}

void YearFlash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS |= YEAR;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.y;
    disp_U1 = DispChars.y;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.y;
    disp_U1 = DispChars.y;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
}

void Alarm1Flash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS |= ALARM1;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.A;
    disp_U1 = DispNums[1];
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.A;
    disp_U1 = DispNums[1];
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH); 
}

void SetAlarm1(void) {
    switch(Switches()) {
        case(0x81):
            SecsFlash();
            while(Switches() == 0x81) {
                SetSecs(&Alarm1Time);
                Num2Disp(&Alarm1Time.secs);
            }
            break;
        case(0x82):
            MinsFlash();
            while(Switches() == 0x82) {
                SetMins(&Alarm1Time);
                Num2Disp(&Alarm1Time.mins);
            }
            break;
        case(0x84):
            HrsFlash();
            while(Switches() == 0x84) {
                SetHrs(&Alarm1Time);
                Num2Disp(&Alarm1Time.hrs);
            }
            break;
        case(0x80):
            disp_LEDS = 0x80;
            while(Switches() == 0x80) {
                disp_U2 = DispChars.A;
                disp_U1 = DispNums[1];
                Delay10KTCYx(ALARM_TOGGLE);
                if(PB2pressed() == 1) {
                    Alarm1On = 1;
                }
                if(PB1pressed() == 1) {
                    Alarm1On = 0;
                }
                if(Alarm1On == 1) {
                    disp_U2 = DispChars.o;
                    disp_U1 = DispChars.n;
                }
                else {
                    disp_U2 = DispChars.o;
                    disp_U1 = DispChars.F;
                }
                Delay10KTCYx(ALARM_TOGGLE);
            }
            break;
        default :
            disp_U2 = DispChars.E;
            disp_U1 = DispChars.r;
            disp_LEDS = 0x04;
            break;
    }
}

void SoundAlarm1(void) {
    disp_U2 = DispChars.A;
    disp_U1 = DispNums[1];
    disp_LEDS = 0xFF;
    while (!PB2pressed() && !PB1pressed()) {
        ms_count3 = 0;
        
        /*
        //Wake Me Up Before You Go Go (Chorus)
        GEN_NOTE(QUAVER, G5, QUAVER);
        GEN_NOTE(CROTCHET, B5, SEMIQUAVER);
        GEN_NOTE(CROTCHET, C6, QUAVER);
        GEN_PAUSE(MINIM);
        //--
        GEN_NOTE(QUAVER, G5, SEMIQUAVER);
        GEN_NOTE(CROTCHET, B5, QUAVER);
        GEN_NOTE(QUAVER, C6, SEMIQUAVER);
        GEN_NOTE(CROTCHET, G5, SEMIQUAVER);
        GEN_NOTE(QUAVER, E5, SEMIQUAVER);
        //--
        GEN_PAUSE(CROTCHET);
        GEN_NOTE(QUAVER, E5, SEMIQUAVER);
        GEN_NOTE(QUAVER, F5, SEMIQUAVER);
        GEN_NOTE(QUAVER, G5, SEMIQUAVER);
        GEN_PAUSE(QUAVER);
        //--
        GEN_NOTE(QUAVER, A5, QUAVER);
        GEN_NOTE(QUAVER, A5, SEMIQUAVER);
        GEN_NOTE(QUAVER, G5, SEMIQUAVER);
        GEN_NOTE(QUAVER, F5, SEMIQUAVER);
        GEN_NOTE(QUAVER, E5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(QUAVER, B5, SEMIQUAVER);
        GEN_NOTE(CROTCHET, A5, SEMIQUAVER);
        GEN_NOTE(QUAVER, F5, QUAVER);
        */
        
        //Jingle Bells
        GEN_NOTE(CROTCHET, C5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, F5, QUAVER);
        //--
        GEN_NOTE(MINIM, C5, CROTCHET);
        GEN_NOTE(QUAVER, C5, SEMIQUAVER);
        GEN_NOTE(QUAVER, C5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, C5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, F5, QUAVER);
        //--
        GEN_NOTE(MINIM, D5, QUAVER);
        GEN_PAUSE(MINIM);
        //--
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        //--
        GEN_NOTE(MINIM, E5, QUAVER);
        GEN_PAUSE(MINIM);
        //--
        GEN_NOTE(CROTCHET, C6, QUAVER);
        GEN_NOTE(CROTCHET, C6, QUAVER);
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        //--
        GEN_NOTE(MINIM, A5, QUAVER);
        GEN_PAUSE(MINIM);
        //--
        GEN_NOTE(CROTCHET, C5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, F5, QUAVER);
        //--
        GEN_NOTE(MINIM, C5, QUAVER);
        GEN_PAUSE(MINIM);
        //-
        GEN_NOTE(CROTCHET, C5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, F5, QUAVER);
        //--
        GEN_NOTE(MINIM, D5, QUAVER);
        GEN_PAUSE(MINIM);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, C6, QUAVER);
        GEN_NOTE(CROTCHET, C6, QUAVER);
        GEN_NOTE(CROTCHET, C6, QUAVER);
        GEN_NOTE(QUAVER, C6, SEMIQUAVER);
        GEN_NOTE(QUAVER, C6, QUAVER);
        //--
        GEN_NOTE(CROTCHET, D6, QUAVER);
        GEN_NOTE(CROTCHET, C6, QUAVER);
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        //--
        GEN_NOTE(MINIM, F5, CROTCHET);
        GEN_NOTE(MINIM, C6, QUAVER);
        //--Chorus        
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(MINIM, A5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(MINIM, A5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, C6, SEMIQUAVER);
        GEN_NOTE(CROTCHET, F5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        //--
        GEN_NOTE(SEMIBREVE, A5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        GEN_NOTE(CROTCHET, AS5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(QUAVER, A5, SEMIQUAVER);
        GEN_NOTE(QUAVER, A5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        //--
        GEN_NOTE(MINIM, G5, QUAVER);
        GEN_NOTE(MINIM, C6, QUAVER);
        Delay10KTCYx(ALARM_REPEAT_DELAY);
    }
    Alarm1On = 0;
}

void Alarm2Flash(void) {
    disp_LEDS &= 0xC0;
    disp_LEDS |= ALARM2;
    dp_mask |= (1 << 2);
    disp_U2 = DispChars.A;
    disp_U1 = DispNums[2];
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = DispChars.A;
    disp_U1 = DispNums[2];
    Delay10KTCYx(SET_MENU_FLASH);
    disp_U2 = 0xFF;
    disp_U1 = 0xFF;
    Delay10KTCYx(SET_MENU_FLASH); 
}

void SetAlarm2(void) {
    switch(Switches()) {
        case(0x41):
            SecsFlash();
            while(Switches() == 0x41) {
                SetSecs(&Alarm2Time);
                Num2Disp(&Alarm2Time.secs);
            }
            break;
        case(0x42):
            MinsFlash();
            while(Switches() == 0x42) {
                SetMins(&Alarm2Time);
                Num2Disp(&Alarm2Time.mins);
            }
            break;
        case(0x44):
            HrsFlash();
            while(Switches() == 0x44) {
                SetHrs(&Alarm2Time);
                Num2Disp(&Alarm2Time.hrs);
            }
            break;
        case(0x48):
            YearFlash();
            while(Switches() == 0x48) {
                SetYear(&Alarm2Date);
                Num2Disp(&Alarm2Date.year_short);
            }
            break;
        case(0x50):
            MonthFlash();
            while(Switches() == 0x50) {
                SetMonth(&Alarm2Date);
                Num2Disp(&Alarm2Date.month);
            }
            break;
        case(0x60):
            DayFlash();
            while(Switches() == 0x60) {
                SetDay(&Alarm2Date);
                Num2Disp(&Alarm2Date.day);
            }
            break;
        case(0x40):
            disp_LEDS = 0x40;
            while(Switches() == 0x40) {
                disp_U2 = DispChars.A;
                disp_U1 = DispNums[2];
                Delay10KTCYx(ALARM_TOGGLE);
                if(PB2pressed() == 1) {
                    Alarm2On = 1;
                }
                if(PB1pressed() == 1) {
                    Alarm2On = 0;
                }
                if(Alarm2On == 1) {
                    disp_U2 = DispChars.o;
                    disp_U1 = DispChars.n;
                }
                else {
                    disp_U2 = DispChars.o;
                    disp_U1 = DispChars.F;
                }
                Delay10KTCYx(ALARM_TOGGLE);
            }
            break;
        default :
            disp_U2 = DispChars.E;
            disp_U1 = DispChars.r;
            disp_LEDS = 0x04;
            break;
    }
}

void SoundAlarm2(void) {
    disp_U2 = DispChars.A;
    disp_U1 = DispNums[2];
    disp_LEDS = 0xFF;
    while (!PB2pressed() && !PB1pressed()) {
        ms_count3 = 0;
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(MINIM, E5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(MINIM, D5, CROTCHET);
        //--
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(QUAVER, FS5, SEMIQUAVER);
        GEN_NOTE(QUAVER, G5, SEMIQUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(QUAVER, FS5, SEMIQUAVER);
        GEN_NOTE(QUAVER, G5, SEMIQUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(MINIM, A5, CROTCHET);
        //--
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, A5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, A5, QUAVER);
        GEN_NOTE(CROTCHET, G5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(CROTCHET, FS5, QUAVER);
        //--
        GEN_NOTE(CROTCHET, E5, QUAVER);
        GEN_NOTE(CROTCHET, D5, QUAVER);
        GEN_NOTE(MINIM, D5, QUAVER);
        Delay10KTCYx(ALARM_REPEAT_DELAY);
    }
    Alarm2On = 0;
}

char CompareTimes(volatile TIME mainTime, volatile DATE *mainDate, volatile TIME *alarmTime, volatile DATE *alarmDate, char args) {
    switch (args) {
        case(1):
            if ((mainTime.hrs == alarmTime->hrs) && (mainTime.mins == alarmTime->mins) && (mainTime.secs == alarmTime->secs)) {
                return (1);
            } else {
                return (0);
            }
            break;
        case(2) :
            if((mainTime.hrs == alarmTime->hrs) && (mainTime.mins == alarmTime->mins) && (mainTime.secs == alarmTime->secs) && (mainDate->day == alarmDate->day) && (mainDate->month == alarmDate->month) && (mainDate->year_short == alarmDate->year_short)) {
                return(1);
            }
            else {
                return(0);
            }
        default :
            return(0);
    }
    return(0);
}