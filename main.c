#include <xc.h>
#include <stdlib.h>
#include <limits.h>
#include <plib/delays.h>

#if defined _18F14K50
// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = IRCCLKOUT // Oscillator Selection bits (Internal RC oscillator, CLKOUT function on OSC2)
#pragma config PLLEN = ON       // 4 X PLL Enable bit (Oscillator multiplied by 4)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal/External Oscillator Switchover bit (Oscillator Switchover mode enabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 19        // Brown-out Reset Voltage bits (VBOR set to 1.9 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up bit (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RA3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Table Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)
#elif defined _16F1503 || defined _16F1508
__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_OFF & CP_ON & BOREN_ON & CLKOUTEN_OFF);
__CONFIG(WRT_OFF & STVREN_ON & BORV_LO & LPBOR_OFF & LVP_OFF); // Fuck! If LVP_ON => MCLRE_OFF fail
#elif defined _16F1823 || defined _16F1824 || defined _16F1825
__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_OFF & MCLRE_OFF & CP_ON & CPD_ON & BOREN_ON & CLKOUTEN_OFF & IESO_ON & FCMEN_ON);
__CONFIG(WRT_OFF & PLLEN_ON & STVREN_ON & BORV_LO & LVP_OFF); // Fuck! If LVP_ON => MCLRE_OFF fail
#elif defined _16F1936 || _16F1937
__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_ON & MCLRE_OFF & CP_ON & CPD_ON & BOREN_ON & CLKOUTEN_OFF & IESO_ON & FCMEN_ON);
__CONFIG(WRT_OFF & VCAPEN_OFF & PLLEN_ON & STVREN_ON & BORV_LO & LVP_OFF); // Fuck! If LVP_ON => MCLRE_OFF fail
#endif

#define ANDREW_BATTERY_BT
#ifdef ANDREW_BATTERY_BT
#define BONA_IO_PIN2_OUT
#define BONA_AN3
#define BONA_FVR
#define BONA_TIMER0
#define BONA_FOUND_BUG
#endif

#define VERSION_NUMBER	0xA

#ifdef _PIC18
#include <plib.h>
#endif

#ifdef BONA_UART
#ifndef _PIC18
#include "uart.h"
#endif
#endif // BONA_UART

#ifdef BONA_INTERNAL_EEPROM
#ifndef _PIC18
#define EEP_V2
#include "EEP.h"
#endif
#endif // BONA_INTERNAL_EEPROM

#if defined BONA_I2C_MASTER || defined BONA_I2C_SLAVE
#ifndef _PIC18
#include "i2c.h"
#endif
#endif // BONA_I2C

#pragma config IDLOC0 = 0, IDLOC1 = 0, IDLOC2 = VERSION_NUMBER, IDLOC3 = 0

#ifndef _XTAL_FREQ
 // Unless already defined assume 16MHz system frequency
 // This definition is required to calibrate __delay_us() and __delay_ms()
#if defined _18F14K50 || defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1936 || defined _16F1937
 #define _XTAL_FREQ 32000000
#elif defined _16F1503 || defined _16F1508
 #define _XTAL_FREQ 16000000
#elif defined _18F67J50
 #define _XTAL_FREQ 48000000
#elif defined __PIC24FJ128GA010__
 #define _XTAL_FREQ 32000000
#else
 #define _XTAL_FREQ 8000000
#endif
#endif

#if defined _PIC18 || defined __18CXX || defined _PIC16 || defined _PIC14 || defined _PIC12 || defined _PIC10
#define INSTRUCTION_CYCLE 4
#elif defined __dsPIC30F__ || defined __dsPIC33F__ || defined __PIC24F__ || defined __PIC24FK__ || defined __PIC24H__ || defined __PIC24E__ || defined __dsPIC33E__
#define INSTRUCTION_CYCLE 2
#elif defined __PIC32MX__ || defined __PIC32__
#define INSTRUCTION_CYCLE 1
#else
#define INSTRUCTION_CYCLE 4
#endif

#ifdef JUST_SHOW_ERROR
#error fuck...
#endif

#ifdef BONA_SW_PWM
#ifndef BONA_TIMER2
#define BONA_TIMER2
#endif
#endif

#define BLINK				LATAbits.LATA5
#define AD_2_4_FLAG	IOCAF5
#define AD_2_4				PORTAbits.RA4

#ifdef BONA_I2C_SLAVE
#define I2C_ADDRESS		0xC8

#define I2C_STATE_ADDRESS	0
#define I2C_STATE_OP		1
#define I2C_STATE_DATA		2
#define I2C_STATE_STOP		3

unsigned char I2CAddress;
unsigned char I2COpAddress, I2COpMode, I2COpData;
unsigned char I2CState;
unsigned char reg[] = {0, 0, 0};
#endif

void initializeI2C() {
#ifdef BONA_I2C_MASTER
	OpenI2C(MASTER, SLEW_ON);
	SSP1ADD = 39; // 100 KHz under 16 MHz
#elif defined BONA_I2C_SLAVE
	OpenI2C(SLAVE_7_STSP_INT, SLEW_ON);
	SSPADD = I2C_ADDRESS; // Slave: Address, Master: BAUD Rate Setting
	SSPCON2bits.SEN = 1; // Clock Stretch to prevent Master send too fast result in data loss
#if defined _16F1503 || defined _16F1508
	SSP1IF = 0; // Clear I2C flag
	SSP1IE = 1; // Enable MSSP interrupt
#else
	SSPIF = 0; // Clear I2C flag
	SSPIE = 1; // Enable MSSP interrupt
#endif
#endif // BONA_I2C_MASTER
}

// To save the usage of Memory
unsigned char globalLocalTemp; // initialize before local function
unsigned long globalLocalVariable; // initialize before local function

#if defined BONA_PWM1_CCP || defined BONA_PWM1 || defined BONA_PWM2 || \
defined BONA_PWM3 || defined BONA_PWM4
/* PWM min 400 Hz, fuck!! postscaler can't use....
 */
#define DYNAMIC_PWM_COUNT
#ifndef TIMER2_MAX_PRESCALER
#define TIMER2_MAX_PRESCALER	64
#endif
#ifndef TIMER2_MAX_TMR2
#define TIMER2_MAX_TMR2		255
#endif
#define MIN_FREQUENCY		_XTAL_FREQ / INSTRUCTION_CYCLE / \
				TIMER2_MAX_PRESCALER / \
				TIMER2_MAX_TMR2
#define MAX_FREQUENCY		_XTAL_FREQ / INSTRUCTION_CYCLE / 2
unsigned char setPWMTMR2(unsigned long frequency) // 32MHz, Fosc/4, Prescaler 1, Postscler 6, 0.75 uS
{
#if defined BONA_PWM1 || defined BONA_PWM2 || defined BONA_PWM3 ||\
	defined BONA_PWM4 || defined BONA_PWM1_CCP

#ifdef DYNAMIC_PWM_COUNT
	globalLocalVariable = _XTAL_FREQ / INSTRUCTION_CYCLE / frequency;
	if(frequency < MIN_FREQUENCY || frequency > MAX_FREQUENCY)
		return -1;

	for(globalLocalTemp = 0; globalLocalTemp < 8; globalLocalTemp += 2) { // Prescaler 1, 4, 16, 64
		if((globalLocalVariable >> globalLocalTemp) < TIMER2_MAX_TMR2) {
			T2CONbits.T2CKPS = globalLocalTemp >> 1;
			PR2 = (globalLocalVariable >> globalLocalTemp);
			break;
		}
	}
#else
#if defined _16F1823 || defined _16F1824 || defined _16F1825 // 32MHz, Fosc/4, Prescaler 1, Postscler 6, 0.75 uS
// Frequency = 130 Hz
	T2CON = 0b00101000; // 1 / (32/4) MHz = 125 nS, Postscaler 6 = 750 nS
	PR2 = 111; // 1 / 120 Hz / 100 Level = 83.33 uS, PR2(111) x 0.75 = 83.25 uS
#elif defined _16F1503 || defined _16F1508 // 16MHz, Fosc/4, Prescaler 4, Postscler 10, 10 uS
// Frequency = 1K Hz
	T2CON = 0b01001001; // 1 / (16/4) MHz = 250 nS, Prescaler 4, Postscaler 10 = 10 uS
	PR2 = 100; // 1 / 1K Hz = 1 mS, PR2(100) x 10 uS
#endif
#endif

#endif
	T2CONbits.TMR2ON = 1;
	return 0;
}
#endif // PWM Timer 2 Setting

void initializePWM() {
#if defined BONA_PWM1_CCP || defined BONA_PWM1 || defined BONA_PWM2 ||\
	defined BONA_PWM3 || defined BONA_PWM4
#if defined _16F1823 || defined _16F1824 || defined _16F1825
#ifdef BONA_PWM1_CCP
	TRISC &= 0b11011111;
	CCP1CON = 0b00001100;
	CCPR1L = 4;
	CCP1CONbits.DC1B = 0b00;
#endif
#elif defined _16F1503 || defined _16F1508
#ifdef BONA_PWM1
	TRISC &= 0b11011111;
	PWM1CON = 0b11100000;
	PWM1DCH = 0;
	PWM1DCL = 0b00000000;
#endif
#ifdef BONA_PWM2
	ANSELC &= 0b11110111;
	TRISC &= 0b11110111;
	PWM2CON = 0b11100000;
	PWM2DCH = 4;
	PWM2DCL = 0b00000000;
#endif
#ifdef BONA_PWM3
	ANSELA &= 0b11111011;
	TRISA &= 0b11111011;
	PWM3CON = 0b11100000;
	PWM3DCH = 4;
	PWM3DCL = 0b00000000;
#endif
#ifdef BONA_PWM4
	ANSELC &= 0b11111101;
	TRISC &= 0b11111101;
	PWM4CON = 0b11100000;
	PWM4DCH = 4;
	PWM4DCL = 0b00000000;
#endif
#endif
#endif

#if defined BONA_PWM1_CCP || defined BONA_PWM1 || defined BONA_PWM2 || \
defined BONA_PWM3 || defined BONA_PWM4
	setPWMTMR2(500);
#endif
}

#ifdef BONA_UART
#define round(x)	(int)(((float)(x))>0?(float)(x)+0.5:(float)(x)-0.5)
#define Fosc		_XTAL_FREQ
#define BAUDRATE	115200
#define BRG_DIV		4
//#define BRGVAL		round((FCY/BAUDRATE/(float)BRG_DIV)-1)
#define BRGVAL		round(((Fosc+BRG_DIV/2)/BAUDRATE/(float)BRG_DIV)-1)
#endif
void initializeUART() {
#ifdef BONA_UART
#if defined _16F1508 || defined _16F1823 || defined _16F1824 || defined _16F1825
	OpenUSART(USART_TX_INT_OFF &
		USART_RX_INT_ON &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_HIGH, BRGVAL);
// BRGVAL round(((FCY+BRG_DIV/2)/BAUDRATE/(float)4)-1)
	baudUSART(BAUD_IDLE_CLK_LOW &
		BAUD_16_BIT_RATE &
		BAUD_WAKEUP_OFF &
		BAUD_AUTO_OFF);
#elif defined _18F14K50
	OpenUSART(USART_TX_INT_OFF &
		USART_RX_INT_OFF &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_HIGH, BRGVAL);
	baudUSART(BAUD_IDLE_CLK_LOW &
		BAUD_16_BIT_RATE &
		BAUD_WAKEUP_OFF &
		BAUD_AUTO_OFF);
#endif
#endif // BONA_UART
}

#if defined BONA_AN0 || defined BONA_AN1 || defined BONA_AN2 ||\
	defined BONA_AN3 || defined BONA_AN4 || defined BONA_AN5 ||\
	defined BONA_AN6 || defined BONA_AN7 || defined BONA_AN8 ||\
	defined BONA_AN9 || defined BONA_AN10 || defined BONA_AN11 ||\
	defined BONA_TEMPERATURE || defined BONA_DAC2ADC || defined BONA_FVR
const unsigned char channelList[] = {
	0xFF
#ifdef BONA_AN0
	,0
#endif
#ifdef BONA_AN1
	,1
#endif
#ifdef BONA_AN2
	,2
#endif
#ifdef BONA_AN3
	,3
#endif
#ifdef BONA_AN4
	,4
#endif
#ifdef BONA_AN5
	,5
#endif
#ifdef BONA_AN6
	,6
#endif
#ifdef BONA_AN7
	,7
#endif
#ifdef BONA_AN8
	,8
#endif
#ifdef BONA_AN9
	,9
#endif
#ifdef BONA_AN10
	,10
#endif
#ifdef BONA_AN11
	,11
#endif
#ifdef BONA_AN12
	,12
#endif
#ifdef BONA_AN13
	,13
#endif
#ifdef BONA_TEMPERATURE
	,29
#endif
#ifdef BONA_DAC2ADC
	,30
#endif
#ifdef BONA_FVR
	,31
#endif
};
#endif
void initializeAD() {
#if defined BONA_AN0 || defined BONA_AN1 || defined BONA_AN2 ||\
	defined BONA_AN3 || defined BONA_AN4 || defined BONA_AN5 ||\
	defined BONA_AN6 || defined BONA_AN7 || defined BONA_AN8 ||\
	defined BONA_AN9 || defined BONA_AN10 || defined BONA_AN11 ||\
	defined BONA_AN12 || defined BONA_AN13 || defined BONA_TEMPERATURE
#ifdef BONA_AN0
	ANSELA |= 0b00000001;
	TRISA |= 0b00000001;
	WPUA &= 0b11111110;
#endif
#ifdef BONA_AN1
	ANSELA |= 0b00000010;
	TRISA |= 0b00000010;
	WPUA &= 0b11111101;
#endif
#ifdef BONA_AN2
	ANSELA |= 0b00000100;
	TRISA |= 0b00000100;
	WPUA &= 0b11111011;
#endif
#ifdef BONA_AN3
#if defined _16F1936 || defined _16F1937
	ANSELA |= 0b00010000;
#elif defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELA |= 0b00010000;
	WPUA &= 0b11101111;
#elif defined _18F14K50
	ANSEL |= 0b00001000;
#endif
	TRISA |= 0b00010000;
#endif
#ifdef BONA_AN4
#if defined _16F1508 || defined _16F1503 || defined _16F1936 || defined _16F1937
	ANSELC |= 0b00000001;
	TRISC |= 0b00000001;
#elif defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELC |= 0b00000001;
	TRISC |= 0b00000001;
	WPUC &= 0b11111110;
#elif defined _18F14K50
	ANSEL |= 0b00010000;
	TRISC |= 0b00000001;
#elif defined _16F1936 || defined _16F1937
	ANSELA |= 0b00100000;
	TRISA |= 0b00100000;
	WPUA &= 0b11011111;
#else
	ANSELC |= 0b00000001;
	TRISC |= 0b00000001;
#endif
#endif
#ifdef BONA_AN5
#if defined _16F1508 || defined _16F1503
	ANSELC |= 0b00000010;
#elif defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELC |= 0b00000010;
	WPUC &= 0b11111101;
#elif defined _18F14K50
	ANSEL |= 0b00100000;
#endif
	TRISC |= 0b00000010;
#endif
#ifdef BONA_AN6
#if defined _16F1508 || defined _16F1503
	ANSELC |= 0b00000100;
#elif defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELC |= 0b00000100;
	WPUC &= 0b11111011;
#elif defined _18F14K50
	ANSEL |= 0b01000000;
#endif
	TRISC |= 0b00000100;
#endif
#ifdef BONA_AN7
#if defined _16F1508 || defined _16F1503
	ANSELC |= 0b00001000;
#elif defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELC |= 0b00001000;
	WPUC &= 0b11110111;
#elif defined _18F14K50
	ANSEL |= 0b10000000;
#endif
	TRISC |= 0b00001000;
#endif
#ifdef BONA_AN8
#if defined _16F1508
	ANSELC |= 0b01000000;
	TRISC |= 0b01000000;
#elif defined _18F14K50
	ANSELH |= 0b00000001;
	TRISC |= 0b01000000;
#elif defined _16F1936 || defined _16F1937
	ANSELB |= 0b00000100;
	TRISB |= 0b00000100;
	WPUB &= 0b11111011;
#else
	ANSELC |= 0b01000000;
	TRISC |= 0b01000000;
#endif
#endif
#ifdef BONA_AN9
#if defined _16F1508
	ANSELC |= 0b10000000;
	TRISC |= 0b10000000;
#elif defined _18F14K50
	ANSELH |= 0b00000010;
	TRISC |= 0b10000000;
#elif defined _16F1936 || defined _16F1937
	ANSELB |= 0b00001000;
	TRISB |= 0b00001000;
	WPUB &= 0b11110111;
#else
	ANSELC |= 0b10000000;
	TRISC |= 0b10000000;
#endif
#endif
#ifdef BONA_AN10
#if defined _16F1508
	ANSELB |= 0b00010000;
	TRISB |= 0b00010000;
	WPUB &= 0b11101111;
#elif defined _18F14K50
	ANSELH |= 0b00000100;
	TRISB |= 0b00010000;
	WPUB &= 0b11101111;
#elif defined _16F1936 || defined _16F1937
	ANSELB |= 0b00000010;
	TRISB |= 0b00000010;
	WPUB &= 0b11111101;
#else
	ANSELB |= 0b00010000;
	TRISB |= 0b00010000;
	WPUB &= 0b11101111;
#endif
#endif
#ifdef BONA_AN11
#if defined _16F1508
	ANSELB |= 0b00100000;
	TRISB |= 0b00100000;
	WPUB &= 0b11011111;
#elif defined _18F14K50
	ANSELH |= 0b00001000;
	TRISB |= 0b01000000;
	WPUB &= 0b10111111;
#elif defined _16F1936 || defined _16F1937
	ANSELB |= 0b00010000;
	TRISB |= 0b00010000;
	WPUB &= 0b11101111;
#else
	ANSELH |= 0b00001000;
	TRISB |= 0b01000000;
	WPUB &= 0b10111111;
#endif
#endif
#ifdef BONA_AN12
#if defined _16F1936 || defined _16F1937
	ANSELB |= 0b00000001;
	TRISB |= 0b00000001;
	WPUB &= 0b11111110;
#else
	ANSELB |= 0b00000001;
	TRISB |= 0b00000001;
	WPUB &= 0b11111110;
#endif
#endif
#ifdef BONA_AN13
#if defined _16F1936 || defined _16F1937
	ANSELB |= 0b00100000;
	TRISB |= 0b00100000;
	WPUB &= 0b11011111;
#else
	ANSELB |= 0b00100000;
	TRISB |= 0b00100000;
	WPUB &= 0b11011111;
#endif
#endif
#ifdef BONA_TEMPERATURE
	FVREN = 1;	// enable fixed voltage reference
	FVRCONbits.ADFVR = 0x2; // ADC Fixed Voltage Reference 2.048V
	TSEN = 1; // enable temperature
	TSRNG = 0; // low range but high accuracy
#endif
#ifdef BONA_FVR
	FVREN = 1;	// enable fixed voltage reference
	FVRCONbits.ADFVR = 0x2; // ADC Fixed Voltage Reference 2.048V
#endif

#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1936 || defined _16F1937
	ADCON1bits.ADCS = 0b010; // 32 MHz fatest, Fosc/32, 1 us
#elif defined _16F1503 || defined _16F1508
	ADCON1bits.ADCS = 0b101; // 16 MHz fatest, Fosc/16, 1 us
#elif defined _18F14K50
	ADCON2bits.ADCS = 0b010; // 16 MHz fatest, Fosc/16, 1 us
#else
	ADCON1bits.ADCS = 0x000; // 1 MHz fatest, Fosc/2, 2 us
#endif

#if defined _16F1508 || defined _16F1503 ||  defined _16F1823 || defined _16F1824 || defined _16F1825
	ADCON1bits.ADPREF = 0x0; // Vref connect to VDD
	ADCON0bits.CHS = channelList[1];	// Channel to AN0
#else
	ADCON1 &= 0xF3; // Vref connect to VDD
	ADCON1 &= 0xFC; // Vref connect to VSS
	ADCON2bits.ADCS = channelList[1];	// Channel to AN0
#endif
	ADCON0bits.ADON = 1;	// turn on the A2D conversion module
	PIR1bits.ADIF = 0; // Clear ADC interrupt flag
	PIE1bits.ADIE = 1; // Enable ADC interrupt
	ADCON0bits.GO_nDONE = 1; // Enable AD interrupt
#endif
}

void initializeIO() {
	CM1CON0 = 0b00000111;		// Comparators off. CxIN pins are configured as digital I/O
#if defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1508 || defined _16F1936 || defined _16F1937
	CM2CON0 = 0b00000111;		// Comparators off. CxIN pins are configured as digital I/O
#endif

#ifdef BONA_FOUND_BUG
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELA = 0b00000000;
	ANSELC = 0b00000000;
	TRISA = 0b00000000;
	TRISC = 0b00000000;
	PORTA = 0b00000000;
	PORTC = 0b00000000;
#if defined _16F1508
	ANSELB = 0b00000000;
	TRISB = 0b00000000;
	PORTB = 0b00000000;
#endif
#elif defined _16F1936 ||  defined _16F1937
	ANSELA = 0b00000000;
	ANSELB = 0b00000000;
	TRISA = 0b00000000;
	TRISB = 0b00000000;
	PORTA = 0b00000000;
	PORTB = 0b00000000;
#elif defined _18F14K50
	ANSEL = 0b00000000;
	ANSELH = 0b00000000;
#endif
#endif

#ifdef BONA_IO_PIN1_IN
#if defined _16F1936
	TRISEbits.TRISE3 = 1;
	WPUEbits.WPUE3 = 1;
#elif BONA_IO_PIN1_OUT
#else
	TRISEbits.TRISE3 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN2_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _18F14K50
	TRISAbits.TRISA5 = 1;
	WPUAbits.WPUA5 = 1;
#elif defined _16F1936 ||  defined _16F1937
	TRISAbits.TRISA0 = 1;
#else
	TRISAbits.TRISA5 = 1;
	WPUAbits.WPUA5 = 1;
#endif
#elif defined BONA_IO_PIN2_OUT
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _18F14K50
	TRISAbits.TRISA5 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISAbits.TRISA0 = 0;
#else
	TRISAbits.TRISA5 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN3_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELAbits.ANSA4 = 0;
	TRISAbits.TRISA4 = 1;
	WPUAbits.WPUA4 = 1;
#elif defined _18F14K50
	TRISAbits.TRISA4 = 1;
	WPUAbits.WPUA4 = 1;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 1;
#else
	ANSELbits.ANS3 = 0;
	TRISAbits.TRISA4 = 1;
	WPUAbits.WPUA4 = 1;
#endif
#elif defined BONA_IO_PIN3_OUT
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELAbits.ANSA4 = 0;
	TRISAbits.TRISA4 = 0;
#elif defined _18F14K50
	TRISAbits.TRISA4 = 0;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 0;
#else
	ANSELbits.ANS3 = 0;
	TRISAbits.TRISA4 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN4_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _18F14K50
	WPUAbits.WPUA3 = 1;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 1;
#else
	WPUAbits.WPUA3 = 1;
#endif
#elif defined BONA_IO_PIN4_OUT
#if defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 0;
#else
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN5_IN
#if defined _16F1508 || defined _16F1503 || defined _18F14K50
	TRISCbits.TRISC5 = 1;
#elif defined _16F1823
	TRISCbits.TRISC5 = 1;
	WPUCbits.WPUC5 = 1;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA3 = 0;
	TRISAbits.TRISA3 = 1;
#endif
#elif defined BONA_IO_PIN5_OUT
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _18F14K50
	TRISCbits.TRISC5 = 0;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA3 = 0;
	TRISAbits.TRISA3 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN6_IN
#if defined _16F1508 || defined _16F1503 || defined _18F14K50
	TRISCbits.TRISC4 = 1;
#elif defined _16F1823
	TRISCbits.TRISC4 = 1;
	WPUCbits.WPUC4 = 1;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA4 = 0;
	TRISAbits.TRISA4 = 1;
#else
	TRISCbits.TRISC4 = 1;
#endif
#elif defined BONA_IO_PIN6_OUT
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _18F14K50
	TRISCbits.TRISC4 = 0;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA4 = 0;
	TRISAbits.TRISA4 = 0;
#else
	TRISCbits.TRISC4 = 0;
#endif
#endif
#ifdef BONA_IO_PIN7_IN
#if defined _16F1508 || defined _16F1503
	ANSELCbits.ANSC3 = 0;
	TRISCbits.TRISC3 = 1;
#elif defined _16F1823
	ANSELCbits.ANSC3 = 0;
	TRISCbits.TRISC3 = 1;
	WPUCbits.WPUC3 = 1;
#elif defined _18F14K50
	TRISCbits.TRISC3 = 1;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA5 = 0;
	TRISAbits.TRISA5 = 1;
#else
	ANSELbits.ANS7 = 0;
	TRISCbits.TRISC3 = 1;
#endif
#elif defined BONA_IO_PIN7_OUT
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELCbits.ANSC3 = 0;
	TRISCbits.TRISC3 = 0;
#elif defined _18F14K50
	TRISCbits.TRISC3 = 0;
#elif defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA5 = 0;
	TRISAbits.TRISA5 = 0;
#else
	ANSELbits.ANS7 = 0;
	TRISCbits.TRISC3 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN8_IN
#if defined _16F1503
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 1;
#elif defined _16F1823 || defined _16F1824 || defined _16F1825
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 1;
	WPUCbits.WPUC2 = 1;
#elif defined _16F1508
	ANSELCbits.ANSC6 = 0;
	TRISCbits.TRISC6 = 1;
#elif defined _18F14K50
	ANSELHbits.ANS8 = 0;
	TRISCbits.TRISC6 = 1;
#else
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 1;
#endif
#elif defined BONA_IO_PIN8_OUT
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503 ||  defined _16F1936 ||  defined _16F1937
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 0;
#elif defined _16F1508
	ANSELCbits.ANSC6 = 0;
	TRISCbits.TRISC6 = 0;
#elif defined _18F14K50
	ANSELHbits.ANS8 = 0;
	TRISCbits.TRISC6 = 0;
#else
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN9_IN
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 1;
#if defined _16F1823 || defined _16F1824 || defined _16F1825
	WPUCbits.WPUC1 = 1;
#endif
#elif defined _16F1508
	ANSELCbits.ANSC7 = 0;
	TRISCbits.TRISC7 = 1;
#elif defined _18F14K50
	ANSELHbits.ANS9 = 0;
	TRISCbits.TRISC7 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISAbits.TRISA7 = 1;
#else
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 1;
#endif
#elif defined BONA_IO_PIN9_OUT
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 0;
#elif defined _16F1508
	ANSELCbits.ANSC7 = 0;
	TRISCbits.TRISC7 = 0;
#elif defined _18F14K50
	ANSELHbits.ANS9 = 0;
	TRISCbits.TRISC7 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISAbits.TRISA7 = 0;
#else
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN10_IN
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 1;
#if defined _16F1823 || defined _16F1824 || defined _16F1825
	WPUCbits.WPUC0 = 1;
#endif
#elif defined _16F1508 || defined _18F14K50
	TRISBbits.TRISB7 = 1;
	WPUBbits.WPUB7 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISAbits.TRISA6 = 1;
#else
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 1;
#endif
#elif defined BONA_IO_PIN10_OUT
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 0;
#elif defined _16F1508 || defined _18F14K50
	TRISBbits.TRISB7 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISAbits.TRISA6 = 0;
#else
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN11_IN
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	TRISAbits.TRISA2 = 1;
	WPUAbits.WPUA2 = 1;
#elif defined _16F1508 || defined _18F14K50
	TRISBbits.TRISB6 = 1;
	WPUBbits.WPUB6 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC0 = 1;
#else
	TRISAbits.TRISA2 = 1;
	WPUAbits.WPUA2 = 1;
#endif
#elif defined BONA_IO_PIN11_OUT
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 0;
#elif defined _16F1508 || defined _18F14K50
	TRISBbits.TRISB6 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC0 = 0;
#else
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN12_IN
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 1;
	WPUAbits.WPUA1 = 1;
#elif defined _16F1508
	ANSELBbits.ANSB5 = 0;
	TRISBbits.TRISB5 = 1;
	WPUBbits.WPUB5 = 1;
#elif defined _18F14K50
	ANSELHbits.ANS11 = 0;
	TRISBbits.TRISB5 = 1;
	WPUBbits.WPUB5 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC1 = 1;
#else
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 1;
	WPUAbits.WPUA1 = 1;
#endif
#elif defined BONA_IO_PIN12_OUT
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 0;
#elif defined _16F1508
	ANSELBbits.ANSB5 = 0;
	TRISBbits.TRISB5 = 0;
#elif defined _18F14K50
	ANSELHbits.ANS11 = 0;
	TRISBbits.TRISB5 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC1 = 0;
#else
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN13_IN
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 1;
	WPUAbits.WPUA0 = 1;
#elif defined _16F1508
	ANSELBbits.ANSB4 = 0;
	TRISBbits.TRISB4 = 1;
	WPUBbits.WPUB4 = 1;
#elif defined _18F14K50
	ANSELHbits.ANS10 = 0;
	TRISBbits.TRISB4 = 1;
	WPUBbits.WPUB4 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC2 = 1;
#else
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 1;
	WPUAbits.WPUA0 = 1;
#endif
#elif defined BONA_IO_PIN13_OUT
#if defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1503
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#elif defined _16F1508
	ANSELBbits.ANSB4 = 0;
	TRISBbits.TRISB4 = 0;
#elif defined _18F14K50
	ANSELHbits.ANS10 = 0;
	TRISBbits.TRISB4 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC2 = 0;
#else
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN14_IN
#if defined _16F1508
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 1;
#elif defined _18F14K50
	ANSELbits.ANS6 = 0;
	TRISCbits.TRISC2 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC3 = 1;
#else
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 1;
#endif
#elif defined BONA_IO_PIN14_OUT
#if defined _16F1508
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 0;
#elif defined _18F14K50
	ANSELbits.ANS6 = 0;
	TRISCbits.TRISC2 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC3 = 0;
#else
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN15_IN
#if defined _16F1508
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 1;
#elif defined _18F14K50
	ANSELbits.ANS5 = 0;
	TRISCbits.TRISC1 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC4 = 1;
#else
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 1;
#endif
#elif defined BONA_IO_PIN15_OUT
#if defined _16F1508
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 0;
#elif defined _18F14K50
	ANSELbits.ANS5 = 0;
	TRISCbits.TRISC1 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC4 = 0;
#else
	ANSELCbits.ANSC1 = 0;
	TRISCbits.TRISC1 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN16_IN
#if defined _16F1508
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 1;
#elif defined _18F14K50
	ANSELbits.ANS4 = 0;
	TRISCbits.TRISC0 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC5 = 1;
#else
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 1;
#endif
#elif defined BONA_IO_PIN16_OUT
#if defined _16F1508
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 0;
#elif defined _18F14K50
	ANSELbits.ANS4 = 0;
	TRISCbits.TRISC0 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC5 = 0;
#else
	ANSELCbits.ANSC0 = 0;
	TRISCbits.TRISC0 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN17_IN
#if defined _16F1508
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 1;
	WPUAbits.WPUA2 = 1;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC6 = 1;
#else
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 1;
	WPUAbits.WPUA2 = 1;
#endif
#elif defined BONA_IO_PIN17_OUT
#if defined _16F1508
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 0;
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC6 = 0;
#else
	ANSELAbits.ANSA2 = 0;
	TRISAbits.TRISA2 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN18_IN
#if defined _16F1508
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 1;
	WPUAbits.WPUA1 = 1;
#elif defined _18F14K50
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC7 = 1;
#else
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 1;
	WPUAbits.WPUA1 = 1;
#endif
#elif defined BONA_IO_PIN18_OUT
#if defined _16F1508
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 0;
#elif defined _18F14K50
	#error "Only Input"
#elif defined _16F1936 || defined _16F1937
	TRISCbits.TRISC7 = 0;
#else
	ANSELAbits.ANSA1 = 0;
	TRISAbits.TRISA1 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN19_IN
#if defined _16F1508
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 1;
	WPUAbits.WPUA0 = 1;
#elif defined _18F14K50
#else
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 1;
	WPUAbits.WPUA0 = 1;
#endif
#elif defined BONA_IO_PIN19_OUT
#if defined _16F1508
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#elif defined _18F14K50
	#error "Only Input"
#else
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN20_IN
#if defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#else
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#endif
#elif BONA_IO_PIN20_OUT
#if defined _16F1936 || defined _16F1937
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#else
	ANSELAbits.ANSA0 = 0;
	TRISAbits.TRISA0 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN21_IN
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB0 = 0;
	TRISBbits.TRISB0 = 1;
	WPUBbits.WPUB0 = 1;
#else
	ANSELBbits.ANSB0 = 0;
	TRISBbits.TRISB0 = 1;
	WPUBbits.WPUB0 = 1;
#endif
#elif defined BONA_IO_PIN21_OUT
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB0 = 0;
	TRISBbits.TRISB0 = 0;
#else
	ANSELBbits.ANSB0 = 0;
	TRISBbits.TRISB0 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN22_IN
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB1 = 0;
	TRISBbits.TRISB1 = 1;
	WPUBbits.WPUB1 = 1;
#else
	ANSELBbits.ANSB1 = 0;
	TRISBbits.TRISB1 = 1;
	WPUBbits.WPUB1 = 1;
#endif
#elif defined BONA_IO_PIN22_OUT
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB1 = 0;
	TRISBbits.TRISB1 = 0;
#else
	ANSELBbits.ANSB1 = 0;
	TRISBbits.TRISB1 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN23_IN
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB2 = 0;
	TRISBbits.TRISB2 = 1;
	WPUBbits.WPUB2 = 1;
#else
	ANSELBbits.ANSB2 = 0;
	TRISBbits.TRISB2 = 1;
	WPUBbits.WPUB2 = 1;
#endif
#elif defined BONA_IO_PIN23_OUT
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB2 = 0;
	TRISBbits.TRISB2 = 0;
#else
	ANSELBbits.ANSB2 = 0;
	TRISBbits.TRISB2 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN24_IN
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB3 = 0;
	TRISBbits.TRISB3 = 1;
	WPUBbits.WPUB3 = 1;
#else
	ANSELBbits.ANSB3 = 0;
	TRISBbits.TRISB3 = 1;
	WPUBbits.WPUB3 = 1;
#endif
#elif defined BONA_IO_PIN24_OUT
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB3 = 0;
	TRISBbits.TRISB3 = 0;
#else
	ANSELBbits.ANSB3 = 0;
	TRISBbits.TRISB3 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN25_IN
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB4 = 0;
	TRISBbits.TRISB4 = 1;
	WPUBbits.WPUB4 = 1;
#else
	ANSELBbits.ANSB4 = 0;
	TRISBbits.TRISB4 = 1;
	WPUBbits.WPUB4 = 1;
#endif
#elif defined BONA_IO_PIN25_OUT
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB4 = 0;
	TRISBbits.TRISB4 = 0;
#else
	ANSELBbits.ANSB4 = 0;
	TRISBbits.TRISB4 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN26_IN
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB5 = 0;
	TRISBbits.TRISB5 = 1;
	WPUBbits.WPUB5 = 1;
#else
	ANSELBbits.ANSB5 = 0;
	TRISBbits.TRISB5 = 1;
	WPUBbits.WPUB5 = 1;
#endif
#elif defined BONA_IO_PIN26_OUT
#if defined _16F1936 || defined _16F1937
	ANSELBbits.ANSB5 = 0;
	TRISBbits.TRISB5 = 0;
#else
	ANSELBbits.ANSB5 = 0;
	TRISBbits.TRISB5 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN27_IN
#if defined _16F1936 || defined _16F1937
	TRISBbits.TRISB6 = 1;
	WPUBbits.WPUB6 = 1;
#else
	TRISBbits.TRISB6 = 1;
	WPUBbits.WPUB6 = 1;
#endif
#elif defined BONA_IO_PIN27_OUT
#if defined _16F1936 || defined _16F1937
	TRISBbits.TRISB6 = 0;
#else
	TRISBbits.TRISB6 = 0;
#endif
#else
#endif
#ifdef BONA_IO_PIN28_IN
#if defined _16F1936 || defined _16F1937
	TRISBbits.TRISB7 = 1;
	WPUBbits.WPUB7 = 1;
#else
	TRISBbits.TRISB7 = 1;
	WPUBbits.WPUB7 = 1;
#endif
#elif defined BONA_IO_PIN28_OUT
#if defined _16F1936 || defined _16F1937
	TRISBbits.TRISB7 = 0;
#else
	TRISBbits.TRISB7 = 0;
#endif
#else
#endif

#ifdef BONA_IO_PIN2_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	// IOC Setting
	IOCAPbits.IOCAP5 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN5 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF5 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCAbits.IOCA5 = 1; // Interrupt On Changes Port A pin 5
#else
#error "not defined"
#endif
#endif
#ifdef BONA_IO_PIN3_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	// IOC Setting
	IOCAPbits.IOCAP4 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN4 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF4 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCAbits.IOCA4 = 1; // Interrupt On Changes Port A pin 4
#else
#error "not defined"
#endif
#endif
#ifdef BONA_IO_PIN4_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	// IOC Setting
	IOCAPbits.IOCAP3 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN3 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF3 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCAbits.IOCA3 = 1; // Interrupt On Changes Port A pin 3
#else
#error "not defined"
#endif
#endif
#ifdef BONA_IO_PIN10_IN
#if defined _16F1508
	// IOC Setting
	IOCBPbits.IOCBP7 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN7 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF7 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCBbits.IOCB7 = 1; // Interrupt On Changes Port A pin 5
#else
#endif
#endif
#ifdef BONA_IO_PIN11_IN
#if defined _16F1508
	// IOC Setting
	IOCBPbits.IOCBP6 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN6 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF6 = 0; // Software clear interrupt on change 5 flag
#elif defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	// IOC Setting
	IOCAPbits.IOCAP2 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN2 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF2 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCBbits.IOCB6 = 1; // Interrupt On Changes Port B pin 6
#else
#error "not defined"
#endif
#endif
#ifdef BONA_IO_PIN12_IN
#if defined _16F1508
	// IOC Setting
	IOCBPbits.IOCBP5 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN5 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF5 = 0; // Software clear interrupt on change 5 flag
#elif defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	// IOC Setting
	IOCAPbits.IOCAP1 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN1 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF1 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCBbits.IOCB5 = 1; // Interrupt On Changes Port B pin 5
#else
#error "not defined"
#endif
#endif
#ifdef BONA_IO_PIN13_IN
#if defined _16F1508
	// IOC Setting
	IOCBPbits.IOCBP4 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN4 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF4 = 0; // Software clear interrupt on change 5 flag
#elif defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	// IOC Setting
	IOCAPbits.IOCAP0 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN0 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF0 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCBbits.IOCB4 = 1; // Interrupt On Changes Port B pin 4
#else
#error "not defined"
#endif
#endif
#ifdef BONA_IO_PIN17_IN
#if defined _16F1508
	// IOC Setting
	IOCAPbits.IOCAP2 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN2 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF2 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN18_IN
#if defined _16F1508
	// IOC Setting
	IOCAPbits.IOCAP1 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN1 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF1 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCAbits.IOCA1 = 1; // Interrupt On Changes Port A pin 5
#endif
#endif
#ifdef BONA_IO_PIN19_IN
#if defined _16F1508
	// IOC Setting
	IOCAPbits.IOCAP0 = 1; // Interrupt On Changes Port A pin 5
	IOCANbits.IOCAN0 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCAFbits.IOCAF0 = 0; // Software clear interrupt on change 5 flag
#elif defined _18F14K50
	// IOC Setting
	IOCAbits.IOCA0 = 1; // Interrupt On Changes Port A pin 5
#endif
#endif
#ifdef BONA_IO_PIN21_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP0 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN0 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF0 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN22_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP1 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN1 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF1 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN23_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP2 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN2 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF2 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN24_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP3 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN3 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF3 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN25_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP4 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN4 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF4 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN26_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP5 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN5 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF5 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN27_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP6 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN6 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF6 = 0; // Software clear interrupt on change 5 flag
#endif
#endif
#ifdef BONA_IO_PIN28_IN
#if defined _16F1936
	// IOC Setting
	IOCBPbits.IOCBP7 = 1; // Interrupt On Changes Port A pin 5
	IOCBNbits.IOCBN7 = 1; // Set interrupt on change 5 flag negative stimulate
	IOCBFbits.IOCBF7 = 0; // Software clear interrupt on change 5 flag
#endif
#endif

#if defined BONA_IO_PIN1_IN || defined BONA_IO_PIN2_IN || \
defined BONA_IO_PIN3_IN || defined BONA_IO_PIN4_IN || \
defined BONA_IO_PIN5_IN || defined BONA_IO_PIN6_IN || \
defined BONA_IO_PIN7_IN || defined BONA_IO_PIN8_IN || \
defined BONA_IO_PIN9_IN || defined BONA_IO_PIN10_IN || \
defined BONA_IO_PIN11_IN || defined BONA_IO_PIN12_IN || \
defined BONA_IO_PIN13_IN || defined BONA_IO_PIN14_IN || \
defined BONA_IO_PIN15_IN || defined BONA_IO_PIN16_IN || \
defined BONA_IO_PIN17_IN || defined BONA_IO_PIN18_IN || \
defined BONA_IO_PIN19_IN || defined BONA_IO_PIN20_IN || \
defined BONA_IO_PIN21_IN || defined BONA_IO_PIN22_IN || \
defined BONA_IO_PIN23_IN || defined BONA_IO_PIN24_IN || \
defined BONA_IO_PIN25_IN || defined BONA_IO_PIN26_IN || \
defined BONA_IO_PIN27_IN || defined BONA_IO_PIN28_IN
#if defined _16F1508 || defined _16F1503 ||  defined _16F1823 || defined _16F1824 || defined _16F1825
	OPTION_REGbits.nWPUEN = 0; // Enable Port A weak pull-up
	INTCONbits.IOCIF = 0; // Software clear interrupt on change flag
	INTCONbits.IOCIE = 1; // Enable interrupt on change
#elif defined _18F14K50
	INTCON2bits.RABPU = 0; // Enable Port A weak pull-up
	INTCONbits.RABIF = 0; // Software clear interrupt on change flag
	INTCONbits.RABIE = 1; // Enable interrupt on change
#else
	OPTION_REGbits.nWPUEN = 0; // Enable Port A weak pull-up
	INTCONbits.IOCIF = 0; // Software clear interrupt on change flag
	INTCONbits.IOCIE = 1; // Enable interrupt on change
#endif // _16F1508...
#endif // BONA_IO_PIN1_IN...
}

void initializeDA() {
#if defined BONA_DACOUT || defined BONA_DACOUT1 || defined BONA_DACOUT2
#if defined _16F1823 || defined _16F1824 || defined _16F1825
#ifdef BONA_DACOUT
	WPUA &= 0b11111110;
	DACCON0bits.DACOE = 1; // DACOUT Open
	DACCON0bits.DACPSS = 0b00; // Reference = Vdd
	DACCON0bits.DACLPS = 1; // Positive reference source selected
#endif
#elif defined _16F1503 || defined _16F1508
#ifdef BONA_DACOUT1
	WPUA &= 0b11111110;
	DACCON0bits.DACOE1 = 1; // DACOUT 1 Open
#endif
#ifdef BONA_DACOUT2
	WPUA &= 0b11111011;
	DACCON0bits.DACOE2 = 1; // DACOUT 2 Open
#endif
	DACCON0bits.DACPSS = 0; // Reference = Vdd
#else
#endif

	DACCON0bits.DACEN = 1;
#endif
}

void initializeMCU() {
#ifdef _18F67J50
	OSCCONbits.SCS = 0b00; // determined by FOSC<2:0> 8MHz => 48 MHz
#elif defined _18F14K50
	OSCCONbits.IRCF = 0x6; // 8 MHz => 32 MHz
	OSCCONbits.SCS = 0b00; // determined by FOSC<2:0> 8MHz => 32 MHz
#elif defined _16F1823 || defined _16F1824 || defined _16F1825 || defined _16F1936 || defined _16F1937
	OSCCONbits.IRCF = 0xE; // 8 MHz => 32 MHz
	OSCCONbits.SCS = 0b00; // determined by FOSC<2:0>
#elif defined _16F1503 || defined _16F1508
	OSCCONbits.IRCF = 0xF; // 16 MHz
	OSCCONbits.SCS = 0b00; // determined by FOSC<2:0>
#else
#error "No Default MCU defined for MCU"
#endif
}

#if defined BONA_TIMER0 || defined BONA_TIMER1 || defined BONA_TIMER2 || defined BONA_TIMER4 || defined BONA_TIMER6
#define uS_IN_S			1000000
#endif

#ifdef BONA_TIMER6
#define DYNAMIC_TIMER6_COUNT
#define TIMER6_uS_INSTRUCTIONS	(_XTAL_FREQ / INSTRUCTION_CYCLE / uS_IN_S)
#ifndef TIMER6_MAX_POSTSCALER_TIMER6_USED
#define TIMER6_MAX_POSTSCALER_TIMER6_USED	16
#endif
#ifndef TIMER6_MAX_TMR6
#define TIMER6_MAX_TMR6		255
#endif
#ifndef TIMER6_MAX_PRESCALER
#define TIMER6_MAX_PRESCALER	64
#endif
#define MAX_TIMER6_uS		TIMER6_MAX_PRESCALER * \
				TIMER6_MAX_TMR6 * \
				(TIMER6_MAX_POSTSCALER_TIMER6_USED / \
				TIMER6_uS_INSTRUCTIONS)
#define PRESCALER_X_TMR6	TIMER6_MAX_TMR6 * TIMER6_MAX_POSTSCALER_TIMER6_USED
char setTimer6(unsigned long uS) {
#ifdef DYNAMIC_TIMER6_COUNT
	globalLocalVariable = TIMER6_uS_INSTRUCTIONS * uS;

	if(uS > MAX_TIMER6_uS) // bug?? MAX_TIMER0_uS == 0
		return -1;

	for(globalLocalTemp = 0; globalLocalTemp < 8; globalLocalTemp += 2) { // Prescaler 1, 4, 16, 64
		if((globalLocalVariable >> globalLocalTemp) <= PRESCALER_X_TMR6) {
			T6CONbits.T6CKPS = globalLocalTemp >> 1;
			globalLocalVariable = (globalLocalVariable >> globalLocalTemp);
			break;
		}
	}
	for(globalLocalTemp = 0; globalLocalTemp < TIMER6_MAX_POSTSCALER_TIMER6_USED; ++globalLocalTemp) {
		if((globalLocalVariable / (globalLocalTemp + 1)) <= TIMER6_MAX_TMR6) {
			T6CONbits.T6OUTPS = globalLocalTemp;
			PR6 = (globalLocalVariable / (globalLocalTemp + 1));
			break;
		}
	}
#else
#if defined _16F1823 || defined _16F1824 || defined _16F1825 // 32MHz, Fosc/4, Prescaler 1, Postscler 6, 0.75 uS
// Frequency = 130 Hz
	T6CON = 0b00101000; // 1 / (32/4) MHz = 125 nS, Postscaler 6 = 750 nS
	PR6 = 111; // 1 / 120 Hz / 100 Level = 83.33 uS, PR2(111) x 0.75 = 83.25 uS
#elif defined _16F1503 // 16MHz, Fosc/4, Prescaler 4, Postscler 10, 10 uS
// Frequency = 1K Hz
	T6CON = 0b01001001; // 1 / (16/4) MHz = 250 nS, Prescaler 4, Postscaler 10 = 10 uS
	PR6 = 100; // 1 / 1K Hz = 1 mS, PR2(100) x 10 uS
#endif
#endif
	PIR3bits.TMR6IF = 0;
	PIE3bits.TMR6IE = 1;
	T6CONbits.TMR6ON = 1;
	return 0;
}
#endif

void initializeTimer6() {
#ifdef BONA_TIMER6
	setTimer6(255);
#endif
}

#ifdef BONA_TIMER4
#define DYNAMIC_TIMER4_COUNT
#define TIMER4_uS_INSTRUCTIONS	(_XTAL_FREQ / INSTRUCTION_CYCLE / uS_IN_S)
#ifndef TIMER4_MAX_POSTSCALER_TIMER4_USED
#define TIMER4_MAX_POSTSCALER_TIMER4_USED	16
#endif
#ifndef TIMER4_MAX_TMR4
#define TIMER4_MAX_TMR4		255
#endif
#ifndef TIMER4_MAX_PRESCALER
#define TIMER4_MAX_PRESCALER	64
#endif
#define MAX_TIMER4_uS		TIMER4_MAX_PRESCALER * \
				TIMER4_MAX_TMR4 * \
				(TIMER4_MAX_POSTSCALER_TIMER4_USED / \
				TIMER4_uS_INSTRUCTIONS)
#define PRESCALER_X_TMR4	TIMER4_MAX_TMR4 * TIMER4_MAX_POSTSCALER_TIMER4_USED
char setTimer4(unsigned long uS) {
#ifdef DYNAMIC_TIMER4_COUNT
	globalLocalVariable = TIMER4_uS_INSTRUCTIONS * uS;

	if(uS > MAX_TIMER4_uS) // bug?? MAX_TIMER4_uS == 0
		return -1;

	for(globalLocalTemp = 0; globalLocalTemp < 8; globalLocalTemp += 2) { // Prescaler 1, 4, 16, 64
		if((globalLocalVariable >> globalLocalTemp) <= PRESCALER_X_TMR4) {
			T4CONbits.T4CKPS = globalLocalTemp >> 1;
			globalLocalVariable = (globalLocalVariable >> globalLocalTemp);
			break;
		}
	}
	for(globalLocalTemp = 0; globalLocalTemp < TIMER4_MAX_POSTSCALER_TIMER4_USED; ++globalLocalTemp) {
		if((globalLocalVariable / (globalLocalTemp + 1)) <= TIMER4_MAX_TMR4) {
			T4CONbits.T4OUTPS = globalLocalTemp;
			PR4 = (globalLocalVariable / (globalLocalTemp + 1));
			break;
		}
	}
#else
#if defined _16F1823 || defined _16F1824 || defined _16F1825 // 32MHz, Fosc/4, Prescaler 1, Postscler 6, 0.75 uS
// Frequency = 130 Hz
	T4CON = 0b00101000; // 1 / (32/4) MHz = 125 nS, Postscaler 6 = 750 nS
	PR4 = 111; // 1 / 120 Hz / 100 Level = 83.33 uS, PR2(111) x 0.75 = 83.25 uS
#elif defined _16F1503 // 16MHz, Fosc/4, Prescaler 4, Postscler 10, 10 uS
// Frequency = 1K Hz
	T4CON = 0b01001001; // 1 / (16/4) MHz = 250 nS, Prescaler 4, Postscaler 10 = 10 uS
	PR4 = 100; // 1 / 1K Hz = 1 mS, PR2(100) x 10 uS
#endif
#endif
	PIR3bits.TMR4IF = 0;
	PIE3bits.TMR4IE = 1;
	T4CONbits.TMR4ON = 1;
	return 0;
}
#endif

void initializeTimer4() {
#ifdef BONA_TIMER4
	setTimer4(255);
#endif
}

#ifdef BONA_TIMER2
#define DYNAMIC_TIMER2_COUNT
#define TIMER2_uS_INSTRUCTIONS	(_XTAL_FREQ / INSTRUCTION_CYCLE / uS_IN_S)
#ifndef TIMER2_MAX_POSTSCALER_TIMER2_USED
#define TIMER2_MAX_POSTSCALER_TIMER2_USED	16
#endif
#ifndef TIMER2_MAX_TMR2
#define TIMER2_MAX_TMR2		255
#endif
#ifndef TIMER2_MAX_PRESCALER
#define TIMER2_MAX_PRESCALER	64
#endif
#define MAX_TIMER2_uS		TIMER2_MAX_PRESCALER * \
				TIMER2_MAX_TMR2 * \
				(TIMER2_MAX_POSTSCALER_TIMER2_USED / \
				TIMER2_uS_INSTRUCTIONS)
#define PRESCALER_X_TMR2	TIMER2_MAX_TMR2 * TIMER2_MAX_POSTSCALER_TIMER2_USED
char setTimer2(unsigned long uS) {
#ifdef DYNAMIC_TIMER2_COUNT
	globalLocalVariable = TIMER2_uS_INSTRUCTIONS * uS;

	if(uS > MAX_TIMER2_uS) // bug?? MAX_TIMER2_uS == 0
		return -1;

	for(globalLocalTemp = 0; globalLocalTemp < 8; globalLocalTemp += 2) { // Prescaler 1, 4, 16, 64
		if((globalLocalVariable >> globalLocalTemp) <= PRESCALER_X_TMR2) {
			T2CONbits.T2CKPS = globalLocalTemp >> 1;
			globalLocalVariable = (globalLocalVariable >> globalLocalTemp);
			break;
		}
	}
	for(globalLocalTemp = 0; globalLocalTemp < TIMER2_MAX_POSTSCALER_TIMER2_USED; ++globalLocalTemp) {
		if((globalLocalVariable / (globalLocalTemp + 1)) <= TIMER2_MAX_TMR2) {
			T2CONbits.T2OUTPS = globalLocalTemp;
			PR2 = (globalLocalVariable / (globalLocalTemp + 1));
			break;
		}
	}
#else
#if defined _16F1823 || defined _16F1824 || defined _16F1825 // 32MHz, Fosc/4, Prescaler 1, Postscler 6, 0.75 uS
// Frequency = 130 Hz
	T2CON = 0b00101000; // 1 / (32/4) MHz = 125 nS, Postscaler 6 = 750 nS
	PR2 = 111; // 1 / 120 Hz / 100 Level = 83.33 uS, PR2(111) x 0.75 = 83.25 uS
#elif defined _16F1503 // 16MHz, Fosc/4, Prescaler 4, Postscler 10, 10 uS
// Frequency = 1K Hz
	T2CON = 0b01001001; // 1 / (16/4) MHz = 250 nS, Prescaler 4, Postscaler 10 = 10 uS
	PR2 = 100; // 1 / 1K Hz = 1 mS, PR2(100) x 10 uS
#endif
#endif
	PIR1bits.TMR2IF = 0;
	PIE1bits.TMR2IE = 1;
	T2CONbits.TMR2ON = 1;
	return 0;
}
#endif

void initializeTimer2() {
#ifdef BONA_TIMER2
	setTimer2(255);
#endif
}

#ifdef BONA_TIMER1
#define DYNAMIC_TIMER1_COUNT
#define TIMER1_uS_INSTRUCTIONS	(_XTAL_FREQ / INSTRUCTION_CYCLE / uS_IN_S)
#define TIMER1_MAX_PRESCALER	8
#define TIMER1_MAX_TMR1		65536
#define MAX_TIMER1_uS		TIMER1_MAX_PRESCALER * \
				(TIMER1_MAX_TMR1 / \
				TIMER1_uS_INSTRUCTIONS)
#ifdef DYNAMIC_TIMER1_COUNT
unsigned int tmr1ResetStart = 0;
#endif
char setTimer1(unsigned long uS) {
#ifdef DYNAMIC_TIMER1_COUNT
	globalLocalVariable = MAX_TIMER1_uS;

	if(uS > globalLocalVariable)
		return -1;

	globalLocalVariable = TIMER1_uS_INSTRUCTIONS * uS;
	for(globalLocalTemp = 0; globalLocalTemp < 4; ++globalLocalTemp) { // Prescaler 1, 2, 4, 8
		if((globalLocalVariable >> globalLocalTemp) <= TIMER1_MAX_TMR1) {
			T1CONbits.T1CKPS = globalLocalTemp;
			tmr1ResetStart = TIMER1_MAX_TMR1 - (globalLocalVariable >> globalLocalTemp);
			TMR1H = tmr1ResetStart >> 8;
			TMR1L = tmr1ResetStart & 0xFF;
			break;
		}
	}
#else
#if defined _16F1823 || defined _16F1824 || defined _16F1825 // 32MHz, Fosc/4, Prescaler 8, 1 uS
	T1CON = 0b00110000; // 1 / (32/4) MHz = 125 nS, Prescaler 8 = 1 uS
#elif defined _16F1503 || defined _16F1508 // 16MHz, Fosc/4, Prescaler 4, Postscler 10, 10 uS
// Frequency = 1K Hz
//	T1CON = 0b01000000; // 1 / 16 MHz = 62.5 nS, Prescaler 1 = 62.5 nS
	T1CON = 0b00100000; // 1 / (16/4) MHz = 250 nS, Prescaler 4 = 1 uS
#endif
#endif
	PIR1bits.TMR1IF = 0;
	PIE1bits.TMR1IE = 1;
	T1CONbits.TMR1ON = 1;

	return 0;
}
#endif
void initializeTimer1() {
#ifdef BONA_TIMER1
	setTimer1(65536);
#endif
}

#ifdef BONA_TIMER0
#define DYNAMIC_TIMER0_COUNT
#define TIMER0_8_BITS_MAX	256
#define TIMER0_MAX_PRESCALER	256
#define TIMER0_uS_INSTRUCTIONS	(_XTAL_FREQ / INSTRUCTION_CYCLE / uS_IN_S)
#define MAX_TIMER0_uS		TIMER0_MAX_PRESCALER * \
				(TIMER0_8_BITS_MAX / \
				TIMER0_uS_INSTRUCTIONS)
unsigned char tmr0ResetStart = 0;
char setTimer0(unsigned long uS) {
#ifdef DYNAMIC_TIMER0_COUNT
	globalLocalTemp = TIMER0_uS_INSTRUCTIONS;
	globalLocalVariable = TIMER0_uS_INSTRUCTIONS * uS;

	if(uS > MAX_TIMER0_uS)
		return -1;

	for(globalLocalTemp = 0; globalLocalTemp < 8; ++globalLocalTemp) { // Prescaler 1, 2, 4, ..., 256
		if(globalLocalTemp == 0 && globalLocalVariable <= TIMER0_8_BITS_MAX) {
			OPTION_REGbits.PSA = 1; // Prescaler is not assigned to Timer 0
			tmr0ResetStart = TIMER0_8_BITS_MAX - globalLocalVariable;
		} else {
			if((globalLocalVariable >> (globalLocalTemp + 1)) <= TIMER0_8_BITS_MAX) {
				OPTION_REGbits.PSA = 0; // Prescaler is assigned to Timer 0
				OPTION_REGbits.PS = globalLocalTemp;
				tmr0ResetStart = TIMER0_8_BITS_MAX - (globalLocalVariable >> (globalLocalTemp + 1));
				break;
			}
		}
	}
#else
#if defined _16F1823 || defined _16F1824 || defined _16F1825 // 32MHz, Fosc/4, Prescaler 1, 8 bits, 32 uS
	OPTION_REGbits.PSA = 1; // Prescaler is not assigned to Timer 0
#elif defined _16F1503 // 16MHz, Fosc/4, Prescaler 16, 8 bits, 512 uS
	OPTION_REGbits.PSA = 0; // Prescaler is assigned to Timer 0
	OPTION_REGbits.PS = 3; // Prescaler 16
#endif
#endif
	OPTION_REGbits.TMR0CS = 0; // Fosc / 4
	INTCONbits.TMR0IF = 0;
	INTCONbits.TMR0IE = 1;

	return 0;
}
#endif
void initializeTimer0() {
#ifdef BONA_TIMER0
	setTimer0(2048);
#endif
}

#ifdef BONA_SW_PWM
#define DUTY_FULL_OR_PERIOD	4
void setSWPWMTMR2(unsigned int frequency) {
	unsigned long long us = uS_IN_S;
	setTimer2(us / frequency / DUTY_FULL_OR_PERIOD);
}
#endif
void initializeSWPWM() {
#ifdef BONA_SW_PWM
	setSWPWMTMR2(500);
#endif
}

void enableInterrupt() {
	INTCONbits.GIE = 1; // Enable all interrupt
	INTCONbits.PEIE = 1; // Enable Peripheral interrupt
}

#define PWM100 0x4
#define PWM75 0x3
#define PWM50 0x2
#define PWM25 0x1
#define PWM0 0x0
unsigned char pwmDuty = PWM0;
unsigned char pwmCount = PWM0;

void XACSetup() {
}

void setup() {
	initializeMCU();
	initializeIO();
#if defined BONA_AN0 || defined BONA_AN1 || defined BONA_AN2 ||\
	defined BONA_AN3 || defined BONA_AN4 || defined BONA_AN5 ||\
	defined BONA_AN6 || defined BONA_AN7 || defined BONA_AN8 ||\
	defined BONA_AN9 || defined BONA_AN10 || defined BONA_AN11 ||\
	defined BONA_AN12 || defined BONA_AN13 || defined BONA_TEMPERATURE
	initializeAD();
#endif
#if defined BONA_PWM1_CCP || defined BONA_PWM1 || defined BONA_PWM2 || \
defined BONA_PWM3 || defined BONA_PWM4
	initializePWM();
#endif
#if defined BONA_DACOUT || defined BONA_DACOUT1 || defined BONA_DACOUT2
	initializeDA();
#endif
#ifdef BONA_UART
	initializeUART();
#endif
#if defined BONA_I2C_MASTER || defined BONA_I2C_SLAVE
	initializeI2C();
#endif
#ifdef BONA_TIMER0
	initializeTimer0();
#endif
#ifdef BONA_TIMER1
	initializeTimer1();
#endif
#ifdef BONA_TIMER2
	initializeTimer2();
#endif
#ifdef BONA_TIMER4
	initializeTimer4();
#endif
#ifdef BONA_TIMER6
	initializeTimer6();
#endif
#ifdef BONA_SW_PWM
	initializeSWPWM();
#endif

//	enableInterrupt();
	XACSetup();
}

int isr2mSAcc = -INT_MIN;
void interrupt isr(void) {
#if defined BONA_IO_PIN1_IN || defined BONA_IO_PIN2_IN ||\
  defined BONA_IO_PIN3_IN || defined BONA_IO_PIN4_IN ||\
  defined BONA_IO_PIN5_IN || defined BONA_IO_PIN6_IN ||\
  defined BONA_IO_PIN7_IN || defined BONA_IO_PIN8_IN ||\
  defined BONA_IO_PIN9_IN || defined BONA_IO_PIN10_IN ||\
  defined BONA_IO_PIN11_IN || defined BONA_IO_PIN12_IN ||\
  defined BONA_IO_PIN13_IN || defined BONA_IO_PIN14_IN ||\
  defined BONA_IO_PIN15_IN || defined BONA_IO_PIN16_IN ||\
  defined BONA_IO_PIN17_IN || defined BONA_IO_PIN18_IN ||\
  defined BONA_IO_PIN19_IN || defined BONA_IO_PIN20_IN ||\
  defined BONA_IO_PIN21_IN || defined BONA_IO_PIN22_IN ||\
  defined BONA_IO_PIN23_IN || defined BONA_IO_PIN24_IN ||\
  defined BONA_IO_PIN25_IN || defined BONA_IO_PIN26_IN ||\
  defined BONA_IO_PIN27_IN || defined BONA_IO_PIN28_IN
	// IO Interrupt
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
	if(INTCONbits.IOCIF) {
		INTCONbits.IOCIF = 0;
#elif defined _18F14K50
	if(INTCONbits.RABIF) {
		INTCONbits.RABIF = 0;
#else
	if(INTCONbits.IOCIF) {
		INTCONbits.IOCIF = 0;
#endif
#ifdef BONA_IO_PIN2_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCAFbits.IOCAF5) {
			if(PORTAbits.RA5) {
				switch(pwmDuty) {
					case PWM100:
						pwmDuty = PWM0;
						WriteUSART(pwmDuty);
						break;
					case PWM75:
						pwmDuty = PWM100;
						break;
					case PWM50:
						pwmDuty = PWM75;
						break;
					case PWM25:
						pwmDuty = PWM50;
						break;
					default: // PWM0
						pwmDuty = PWM25;
				}
			}
			IOCAFbits.IOCAF5 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN3_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCAFbits.IOCAF4) {
			if(PORTAbits.RA4) {
				if(isSystemOn) {
					switch(PWM1DCH) {
						case PWM100:
							PWM1DCH = PWM70;
							break;
						case PWM70:
							PWM1DCH = PWM50;
							break;
						case PWM50:
							PWM1DCH = PWM30;
							break;
						default: // PWM30
							PWM1DCH = PWM100;
					}
				}
				isr2mSAcc = INT_MIN;
			} else {
				isr2mSAcc = 0;
			}
			IOCAFbits.IOCAF4 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN4_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCAFbits.IOCAF3) {
			IOCAFbits.IOCAF3 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN10_IN
#if defined _16F1508
		if(IOCBFbits.IOCBF7) {
			IOCBFbits.IOCBF7 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN11_IN
#if defined _16F1508
		if(IOCBFbits.IOCBF6) {
			IOCBFbits.IOCBF6 = 0;
		}
#elif defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCAFbits.IOCAF2) {
		}
#endif
#endif
#ifdef BONA_IO_PIN12_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCBFbits.IOCBF5) {
			IOCBFbits.IOCBF5 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN13_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCBFbits.IOCBF4) {
			IOCBFbits.IOCBF4 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN17_IN
#if defined _16F1508
		if(IOCAFbits.IOCAF2) {
			IOCAFbits.IOCAF2 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN18_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCAFbits.IOCAF1) {
			IOCAFbits.IOCAF1 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN19_IN
#if defined _16F1508 || defined _16F1503 || defined _16F1823 || defined _16F1824 || defined _16F1825
		if(IOCAFbits.IOCAF0) {
			IOCAFbits.IOCAF0 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN21_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF0) {
			IOCAFbits.IOCAF0 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN22_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF1) {
			IOCBFbits.IOCBF1 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN23_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF2) {
			IOCBFbits.IOCBF2 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN24_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF3) {
			IOCBFbits.IOCBF3 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN25_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF4) {
			IOCBFbits.IOCBF4 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN26_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF5) {
			IOCBFbits.IOCBF = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN27_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF6) {
			IOCBFbits.IOCBF6 = 0;
		}
#endif
#endif
#ifdef BONA_IO_PIN28_IN
#if defined _16F1936 || defined _16F1937
		if(IOCBFbits.IOCBF7) {
			IOCBFbits.IOCBF7 = 0;
		}
#endif
#endif
	}
#endif // BONA_IO_PIN1_IN...

#ifdef BONA_UART
	if(PIR1bits.RCIF) {
		PIR1bits.RCIF = 0;
	}
#endif // BONA_UART

#if defined BONA_AN0 || defined BONA_AN1 || defined BONA_AN2 ||\
	defined BONA_AN3 || defined BONA_AN4 || defined BONA_AN5 ||\
	defined BONA_AN6 || defined BONA_AN7 || defined BONA_AN8 ||\
	defined BONA_AN9 || defined BONA_AN10 || defined BONA_AN11 ||\
	defined BONA_TEMPERATURE
	if(PIR1bits.ADIF) {
		PIR1bits.ADIF = 0;
	}
#endif // BONA_AN0...

#ifdef BONA_I2C_SLAVE
	// I2C Command
#if defined _16F1503 || defined _16F1508 || defined _16F1823
	if(PIR1bits.SSP1IF) {
		PIR1bits.SSP1IF = 0;
#elif defined _18F14K50 || defined _16F1936 || defined _16F1937
	if(PIR1bits.SSPIF) {
		PIR1bits.SSPIF = 0;
#else
	if(PIR1bits.SSP1IF) {
		PIR1bits.SSP1IF = 0;
#endif
		if(SSPSTATbits.BF) {	// Master Write (Address,Write|Read + Operation + Data)
			if(SSPSTATbits.D_nA) {	// Data (Operation + Data)
				if(I2CState == I2C_STATE_OP) {	// Operation
					I2CState = I2C_STATE_DATA;
					I2COpMode = SSPBUF;
				} else {	// Data
					I2CState = I2C_STATE_ADDRESS;
					reg[I2COpMode] = SSPBUF;
				}
			} else {	// Master Write Address (Device Address,Write|Read)
				I2CState = I2C_STATE_OP;
				I2CAddress = SSPBUF; // Read out SSPBUF to satisfy ssp module to clear bf
				if (R_nW)	// Read first data if Read mode
					SSPBUF = reg[I2COpMode];
			}
		} else {	// Master Read (from second Data)
			I2CState = I2C_STATE_ADDRESS;
			SSPBUF = reg[I2COpMode];
		}
		SSPCON1bits.WCOL = 0;
		SSPCON1bits.SSPOV = 0;
		SSPCON1bits.CKP = 1; // Clear Clock Stretch
	}
#endif // BONA_I2C_SLAVE

#ifdef BONA_TIMER0
	if(INTCONbits.TMR0IF) {
		++isr2mSAcc;
		if(isr2mSAcc == -1)
			isr2mSAcc = INT_MIN;
		if(isr2mSAcc == 1000) {
			isr2mSAcc = INT_MIN;
		}
		INTCONbits.TMR0IF = 0;
	}
#endif
#ifdef BONA_TIMER1
	if(PIR1bits.TMR1IF) {
		PIR1bits.TMR1IF = 0;
	}
#endif
#ifdef BONA_TIMER2
	if(PIR1bits.TMR2IF) {
		++pwmCount;
		if(pwmCount > PWM100) {
			pwmCount = 0;
			if(pwmDuty != PWM0) {
				LATC3 = 1;
				LATC5 = 1;
			}
		}
		if(pwmDuty == pwmCount && pwmDuty < PWM100) {
			LATC3 = 0;
			LATC5 = 0;
		}
		PIR1bits.TMR2IF = 0;
	}
#endif
#ifdef BONA_TIMER4
	if(PIR3bits.TMR4IF) {
		PIR3bits.TMR4IF = 0;
	}
#endif
#ifdef BONA_TIMER6
	if(PIR3bits.TMR6IF) {
		PIR3bits.TMR6IF = 0;
	}
#endif
}

#ifdef _PIC18
void interrupt low_priority isrl(void)
{

}
#endif

#define BLINK_NONE 0
#define BLINK_ON 1
unsigned char blinkState = BLINK_NONE;
int main(int argc, char** argv) {
	unsigned char adResult;
	unsigned char reference;
	unsigned char v24Is122;
	unsigned char blink2MsAcc;

	setup();
	blink2MsAcc = 0;
	BLINK = 1;

	while(1) {
		// Get ADC
 		ADCON0bits.CHS = 3;
		Delay10TCY();
		ADCON0bits.GO_nDONE = 1;
		while(ADCON0bits.GO_nDONE);
		adResult = ADRESH;
		ADCON0bits.CHS = 31;
		Delay10TCY();
		ADCON0bits.GO_nDONE = 1;
		while(ADCON0bits.GO_nDONE);
		reference = ADRESH;
		v24Is122 = (unsigned int)adResult * 102 / reference;
		// if ADC < 2.4V
		if(v24Is122 < 122) {
			if(blinkState == BLINK_NONE) {
				blinkState = BLINK_ON;
				BLINK = 0;
				blink2MsAcc = 0;
			} else {
				if(INTCONbits.TMR0IF) {
					++blink2MsAcc;
					INTCONbits.TMR0IF = 0;
				}
				if(blink2MsAcc > 250) {
					BLINK = ~BLINK;
					blink2MsAcc = 0;
				}
			}
		} else {
			blinkState = BLINK_NONE;
			BLINK = 1;
		}
	}

	return (EXIT_SUCCESS);
}
