/* 
 * File:   MCP2004.h
 * Author: Bob
 *
 * Created on June 13, 2018, 2:10 PM
 */

#ifndef MCP2004_H
#define	MCP2004_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* MCP2004_H */

// PIC16F15354 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out reset enable bits (Brown-out reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

//#use fast_io(A)       // this compiler will not allow the #use
//#use fast_io(B)

//#define SER_DAT 		PIN_B0  // for shift reg not needed this version
//#define TX_ENA		PIN_B1  // for shift reg not needed this version

#define call_in			PORTAbits.RA0       // main call button (A/D derived)
#define smoke_in		PORTAbits.RA1       // main Smole input (A/D derived)
#define sw_secuity      PORTAbits.RA2       // new input for this version     
//#define sw_call         PORTAbits.RA3       // Emergency Call Button
#define AUX             PORTAbits.RA4       // new input for this version
#define motion_in		PORTAbits.RA5       // new input for this version
#define bedWet          PORTAbits.RA6       // new input for this version
//#define	SR_CLK		PIN_B3
//#define SR_DATAIN		PIN_B4
//#define sw_call		PORTBbits.RB0
#define TX_ENA          PORTBbits.RB1
#define SMOKE_PWR		PORTBbits.RB4

#define LED_CHK_IN		2
#define LED_CALL		3

//#define LED_CHK_IN		LATBbits.LATB3
//#define LED_CALL		LATBbits.LATB5

//#define sr_shift			output_high(PIN_B2)
//#define sr_latch			output_low(PIN_B2)

//#define TP1					PIN_B7

//#define address_flag		bit_test(data,7)


//#define COMP_SUPER		2		//Vrr = 0 level = 1.6v	allow 10us	for change
//#define COMP_TROUBLE	14		//Vrr = 0 level = 3.6v
//#define HALFBIT_PER		52		// corresponds to ~208 uS 
//#define FULLBIT_PER		99		// corresponds to ~416 uS (2400 baud)

#define MINUTE_WD		0xfe39

#define _XTAL_FREQ          32000000

#define bit_set(var, bitno) ((var) |= 1UL << (bitno))
#define bit_clear(var, bitno) ((var) &= ~(1UL << (bitno)))
#define bit_test(D,i) (D & (0x01 << i))



/*
#byte CMCON=0x1f
#byte VRCON=0x9f
#byte T2CON=0x12
#byte PR2  =0x92
#byte TRISA=0x85
#byte TRISB=0x86
*/

unsigned char ra_shadow = 0;
unsigned char rx_data, tx_data, bit_idx;
unsigned char rx_byte1, rx_byte2, rx_byte3 = 0;			// storage for each received byte to the unit's matching address
unsigned char rx_byte1_hold=0, rx_byte2_hold, rx_byte3_hold = 0;		// storage for each received byte to the unit's matching address
unsigned char tx_byte1, tx_byte2;

//int ra_shadow = 0;
//int rx_data, tx_data, bit_idx;
//int rx_byte1, rx_byte2, rx_byte3 = 0;			// storage for each received byte to the unit's matching address
//int rx_byte1_hold=0, rx_byte2_hold, rx_byte3_hold = 0;		// storage for each received byte to the unit's matching address
//int tx_byte1, tx_byte2;

unsigned char parity;
unsigned char rcv_start_bit_flag = 0, xmit_start_bit_flag = 0;
unsigned char rcv_data_flag = 0, xmit_first_flag = 0, xmit_done_flag = 0;  // is xmit_done_flag needed, or just reset xmit_pend_flag when done?

unsigned char rx_bit;
unsigned char receive_mode;

short int address = 0;			// the six bit address of the device
//short command_rdy_flag=0;		// indicates next byte is command to device
unsigned char command_state = 0;			// replaces command_rdy_flag for more than two bytes received added 5/15/13

unsigned char reset_flag = 0;			// used to reset the controller
unsigned char vref_toggle;
unsigned char timeout_30s = 0, timeout_20s = 0;
int poll_count;

unsigned char stat_smoke_alarm = 0; 
unsigned char stat_smoke_trouble = 0; 
unsigned char stat_call_trouble = 0;
unsigned char stat_security = 0;
unsigned char stat_panel_reset = 1;
unsigned char stat_checkin = 0;
unsigned char stat_ecall = 0;
unsigned char stat_icm = 0;

unsigned char ack_reset = 0;
unsigned char ack_checkin = 0;
unsigned char ack_ecall = 0;
unsigned char ack_icm = 0;
unsigned char ack_security = 0;
unsigned char ind_call = 0;
unsigned char ind_checkin = 0;
unsigned char set_smoke_horn = 0;

// New Program declarations

bit ninth = 0;
unsigned char RxBufTemp;
bit BlinkFlag =  0;
unsigned char TXdata[2];
unsigned char RXdata[3];
unsigned char address_flag = 0;
unsigned char BedCall = 0;
unsigned char Smoke = 0;
unsigned char sw_call = 0;
unsigned char sw_checkin = 0;
unsigned char sw_security = 0;
unsigned char count = 0;
bit rx_bit1 = 0;
bit rx_bit2 = 0;
bit rx_bit3 = 0;

int led_timer = 0x20;    // timer runs from 0x20 to 0xff, for 30 sec period
long minute_timer = MINUTE_WD;		//minute timer runs to 0xffff, for 60 second period

//****************************************
//George's Variables
bit BathInputFlag;
bit BedInputFlag;
bit SmokeInputFlag;
bit CallCancelInputFlag;
bit SmokeTroubleFlag;
bit BedTroubleFlag;
bit returncode;
bit GRC1;
bit BitTest1;
bit BitTest2;
bit PortPin;
bit CallCancelFlag;
unsigned char Step;
unsigned char ADCValue;
unsigned char GRC2;
unsigned short Step1;
unsigned char BathCallAck = 0;
unsigned char BedCallAck = 0;

// End of George's variables
//****************************************/



// Function prototypes
void InitUart();
void getAdress();
void InitADC();
void reset_cpu();
void restart_wdt();
void send (unsigned char data[], unsigned char length );
void txport(unsigned char a);
bit parityChk(unsigned char rxbyte);
void eval_receiver(unsigned char data);
void transmit_status();
void interrupt PortComm(void);
void exec_command();
void GetBedSmoke();
void output();
void ProcessPinInput(unsigned int port);
void ProcessADCValue();
void Transmit_bytes();