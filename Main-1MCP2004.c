/* 
 * File:   Main-1MCP2004.c
 * Author: Bob
 *
 * Created on June 1, 2018, 9:15 AM
 */


#include "MCP2004.h"
//Functions


void output(unsigned char LED)
{
    if(bit_test(LED,3))
    {
        LATBbits.LATB5 = 1;     // call led is on
    }
    else
    {
        LATBbits.LATB5 = 0;     // call led is off
    }
    if(bit_test(LED,2))
    {
        LATBbits.LATB3 = 1;     // check in led is on
    }
    else
    {
        LATBbits.LATB3 = 0;     // check in led is off
    }
}

void InitUart()
{
    RC1STAbits.SPEN = 1;
    TX1STAbits.SYNC = 0;
    RC1STAbits.CREN = 1;
    TX1STAbits.BRGH = 1;
    BAUD1CONbits.BRG16 = 1;
    TX1STAbits.TX9 = 1;     // for parity    
    RC1STAbits.RX9 = 1;
    SP1BRGL = 0x04;
    SP1BRGH = 0x0d;
}

void InitTMR0()
{
PIE0bits.TMR0IE = 1;
 T0CON0 = 0x90;     // 16 bit, Timer 0 is On too
 T0CON1 = 0x44;     // FOSC/4 and no prescale
 TMR0H = 0xa2;      // sets Timer 0 for 3ms
 TMR0L = 0x00;      // writes value to register.
}
//void Timer0Off(){
 //   T0CON0 = 0x00;
//}


unsigned char Getdata(unsigned char regdata)
{
    // regdata should be 0x01 for ADC-1 and 0x05 for ADC-2
    ADCON0 = regdata;
    while (ADCON0bits.GOnDONE);
    return ADRESH;
}
// This function gets the address from the dipswitch
void getAddress()
{
    address = PORTC;        // get contents of PORTC
    address = ~address;     // invert to make correct
    address = address & 0x3f;   // mask the two MSB's since dipswitch is 6bits long
}



void InitADC()
{
    ADCON1 = 0x60;      //ADFM = 0, ADCS = 2, ADREF = 0
}
void reset_cpu()
{
    RESET();
}

//void restart_wdt()
//{
  //  CLRWDT();
//}

// This function is called by TotalTX() twice
void send (unsigned char data[], unsigned char length )
{
    PIE3bits.RC1IE = 0;         // disable interrupts for RX
    RC1STAbits.CREN = 0;        // disable the continuous receive function
    TX_ENA = 1;             // turns on the max485
    TX1STAbits.TXEN = 1;    // enables transmit on UART
	for (int i = 0; i < length; i++)
	{
        txport(data[i]) ;
	}
    TX1STAbits.TXEN = 0;    // disables transmit on UART
    TX_ENA = 0;
    RC1STAbits.CREN = 1;
    PIE3bits.RC1IE = 1;
}
// This function is called by send().   It sends one byte of data
void txport(unsigned char a) 
{
    unsigned char x = 0;
    x = (unsigned char)parityChk(a);
    if (x == 1){
        TX1STAbits.TX9 = 0;         // parity
    }
    if (x == 0){
        TX1STAbits.TX9 = 1;
    }
	TX1REG = a ;                    // send data by transferring to TX1REG
    while (TXSTAbits.TRMT == 0);    // wait for tx buffer is empty
}
// this function is used by both the void interrupt PortComm(void) and txport() 
bit parityChk(unsigned char rxbyte)
{
    unsigned char flag = 1;
    unsigned char count1 = 0;
    for (int i = 0; i < 8; i++)
    {
        if ( rxbyte & (flag << i))
        {
            count1++;
        }
    }
    if (count1 % 2 == 0)
    {    
        return 0;
    }
    if (count1 % 2 > 0){
        return 1;
    }
    return 0;
}

void eval_receiever(unsigned char rx1, unsigned char rx2, unsigned char rx3)
{
    if(command_state == 0)
    {
        if (rx_bit1 == parityChk(rx1))
        {
            if (rx_bit2 == parityChk(rx2))
            {
                if (rx_bit3 == parityChk(rx3))
                {
                    rcv_data_flag = 0;
                    command_state = 1;
                }
                else
                {
                    rcv_data_flag = 0;
                    command_state = 0;
                    return;
                }
            }
            else
            {
                rcv_data_flag = 0;
                command_state = 0;
                return;
            }
        }
        else
        {
            rcv_data_flag = 0;
            command_state = 0;
            return;
        }
    }
    if(command_state == 1)
    {
        if(address == (rx1 & 0x3f))
        {
            command_state = 2;
            rcv_data_flag = 0;
            minute_timer = 0;
            return;
        }
        command_state = 0;
        rcv_data_flag = 0;
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// These functions are from the original program
// All other functions are new to this project


  ////////////////////////////////////////////////
 /// Counts consecutive commands and executes ///
////////////////////////////////////////////////

void exec_command()
{
    // decode command byte 1
    if(rx_byte1 & 0x40)     // Smoke Detector Power
    {
        SMOKE_PWR = 0; 
    }
    else
    {
        SMOKE_PWR = 1;
    }
    
    // decode command byte 2
    if(rx_byte2 & 0x01)     // Reset Event Ack
    {
        
    }
    if(rx_byte2 & 0x02)     // Bath Call Ack
    {
        if(BedCallAck == 2)
        {
            BedInputFlag = 0;
            BedCallAck = 0;
        }
        else
        {
            BedCallAck++;
        }
    }
    if(rx_byte2 & 0x04)     // Bath Call Placed LED
    {
        //LATBbits.LATB5 = 1;
    }
    if(rx_byte2 & 0x08)     // Bed Call Ack
    {
 
    }
    if(rx_byte2 & 0x10)     // Bed Call Placed LED
    {
        
    }
    if(rx_byte2 & 0x20)     // Check in Cancel Ack
    {
        
    }
    if(rx_byte2 & 0x40)     // Check in LED
    {
        
    }    
    // decode command byte 3
    if(rx_byte3 & 0x01)    // Intercom Control Bit
    {
        
    }
    if(rx_byte3 & 0x02)     // PA Control 
    {
        
    } 
    if(rx_byte3 & 0x04)     // Door Bell
    {
        
    } 
    if(rx_byte3 & 0x08)     // Security Ack
    {
        
    } 
    if(rx_byte3 & 0x10)     // Intercom 1 Ack
    {
        
    } 
    if(rx_byte3 & 0x20)     // Intercom 2 Ack
    {
        
    } 
    if(rx_byte3 & 0x40)     // Unused
    {
        
    } 
     
}

void Transmit_bytes()
{
    TXdata[0] = 0;
    TXdata[1] = 0;
    if(BathInputFlag)
    {
        bit_set(TXdata[0], 0);
    }
    if(BedInputFlag)
    {
        bit_set(TXdata[0], 1);
    }
    if(SmokeInputFlag)
    {
        bit_set(TXdata[0],2);
    }
    if(CallCancelFlag)
    {
        bit_set(TXdata[0], 3);
    }
    if(CheckInFlag)
    {
        bit_set(TXdata[0], 4);
    }
    if(SecurityInputFlag)
    {
        bit_set(TXdata[0], 5);
    }
    // 
    if(BedTroubleFlag)
    {
        bit_set(TXdata[1], 1);
    }
    if(SmokeTroubleFlag)
    {
        bit_set(TXdata[1], 2);
    }
    if(MotionInputFlag)
    {
        bit_set(TXdata[1], 3);
    }
    /*
    if(BedExitFlag)
    {
        bit_set(TXdata[1], 4);
    }
    */
    if(BedWetInputFlag)
    {
        bit_set(TXdata[1], 5);
    }
    if(ResetEventFlag)
    {
        bit_set(TXdata[1], 6);
    }  
    __delay_ms(6);
    NOP();
    send(TXdata, 2);   
}

 
void ProcessPinInput(unsigned int port)
{
    GRC1 = 0;
    switch(port)
    {
        case 1:
            BitTest1 = PORTBbits.RB2;
            __delay_ms(5);
            BitTest2 = PORTBbits.RB2;
            break;
        case 2:
            BitTest1 = PORTAbits.RA3;
            __delay_ms(5);
            BitTest2 = PORTAbits.RA3;
            break;
    }
    if(BitTest1 == BitTest2)
    {
        GRC1 =  ~BitTest2;
    }
}

void ProcessADCValue()
{
    GRC2 = 0;
    if(ADCValue > 0xDE)
    {
        GRC2 = 2;
    }
    else
    {
        if(ADCValue <0x63)
        {
            GRC2 = 1;
        }
    }
}



// end of older functions from original program
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void interrupt PortComm(void)
{
    
    if (PIR0bits.TMR0IF == 1)       // for checking timer 0 roll over
    {
        PIR0bits.TMR0IF = 0;
        BlinkFlag = ~BlinkFlag;
        if(led_timer == 0xff)           // control the flash rate of LED's
        {
            led_timer = 0x20; 
            //BlinkFlag = ~BlinkFlag;
        }
        else
        {
            led_timer++;
        }
        if(minute_timer == 0xffff)      // using TMR0 to also be a adhoc minute timer
        {
            reset_flag = 1;         
        }
        else
        {
            minute_timer++;
        }
        //led_timer = !led_timer;
        
        return;
    }

    if(PIR3bits.RC1IF == 1)         // receive data interrupt service routine
    {
        count++;
        PIE3bits.RC1IE = 0;
        ninth = RC1STAbits.RX9D;
        RxBufTemp = RC1REG;

        switch(rcv_data_flag)
        {
            case 0: 
                if (bit_test(RxBufTemp, 7) != 0)
                {
                    rx_byte1 = RxBufTemp;
                    rx_bit1 = RC1STAbits.RX9D;
                    rcv_data_flag = 1;
                }
                break;
            case 1: 
                if (bit_test(RxBufTemp, 7) == 0)
                {
                    rx_byte2 = RxBufTemp;
                    rx_bit2 = RC1STAbits.RX9D;
                    rcv_data_flag = 2;
                }
                else
                {
                    rcv_data_flag = 0;
                }
                break;
            case 2:
                if (bit_test(RxBufTemp, 7) == 0)
                {
                    rx_byte3 = RxBufTemp;
                    rx_bit3 = RC1STAbits.RX9D;
                    rcv_data_flag = 3;
                }
                else
                {
                    rcv_data_flag = 0;
                }
                break;
            default:
                break;
        }

        PIR3bits.RC1IF = 0;
        PIE3bits.RC1IE = 1;
    }
}

void main() 
{
    TRISA =    0b11111111;
    TRISB =    0b00000101;
    TRISC =    0b10111111;     //Portc.RC6 is output all the rest inputs
    ANSELA = 0x03;             // select AN1 nd AN2 for analog inputs
    ANSELB = 0x00;
    ANSELC = 0x00;
    WPUA   = 0x00;
    WPUB   = 0x00;
    WPUC   = 0x00;
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x00;
    RC6PPS = 0x0f;         // selects the output of UART and routes it to TX pin? Really?
    TX1CKPPS = 0x16;
    INTCONbits.GIE = 1;         // enables global interrupts
    INTCONbits.PEIE = 1;        // enables peripheral interrupts
    INTCONbits.INTEDG = 0;
    PIE3bits.RC1IE = 1;         // enables Receive Interrupts
    PIR3bits.RC1IF = 0;         // clears the receive interrupt flag
    PIE0bits.TMR0IE = 0;        // disables interrupt for Timer 0

    InitUart();
    InitADC();
    InitTMR0();
    TX_ENA = 0;
 
// Just temprarily setting these new variable to zero until I further implement them in code
    SecurityInputFlag = 0;
    CheckInFlag = 0;
    MotionInputFlag = 0;
    BedWetInputFlag = 0;
    ResetEventFlag = 0;
    BlinkFlag = 0;
// End of temporary code////////////////////////////////////////////////////////////////////  
    
 receive_mode = 1;
 Step1 = 0;
// main program loop
	do
	{
        
        getAddress();       // gets the address from the dipswitch
        switch(Step1)
        {
            case 3:             // process cancel button
                Step1 = 0;
                if(!CallCancelFlag)
                {
                    ProcessPinInput(1);
                    CallCancelFlag = GRC1;                    
                }
                break;
            case 2:             // process smoke input
                Step1 = 3;
        		ADCValue = Getdata(0x07);
                ProcessADCValue();
                if(GRC2 == 2)
                {
                    SmokeTroubleFlag = 1;
                    SmokeInputFlag = 0;
                }
                else
                {
                    if(GRC2 == 1)
                    {
                    SmokeTroubleFlag = 0;
                    SmokeInputFlag = 1;
                    }
                    else
                    {
                    SmokeTroubleFlag = 0;
                    SmokeInputFlag = 0;                        
                    }
                }
                break;
            case 1:         // process bed call
                Step1 = 2;
        		ADCValue = Getdata(0x03);
                ProcessADCValue();
                if(GRC2 == 2)
                {
                    BedTroubleFlag = 1;
                    BedInputFlag = 0;
                }
                else
                {
                    if(GRC2 == 1)
                    {
                      BedInputFlag = 1;
                      BedTroubleFlag = 0;
                    }
                    else
                    {
                        BedTroubleFlag = 0;
                        BedInputFlag = 0;
                    }
                }
                break;
            case 0:         // process the bath call
                Step1 = 1;
                if(!BathInputFlag)
                {
                    ProcessPinInput(2);
                    BathInputFlag = GRC1;                    
                }
                break;
        }
        ////////////////////////////////////////////////////////
        if (rcv_data_flag == 3)
        {
            eval_receiever(rx_byte1, rx_byte2, rx_byte3);
        }
        
		if (command_state == 2)
		{	
            //eval_receiver(rx_data); 
            exec_command();						// done, so now call function to execute comand conditional on three identical commands
            Transmit_bytes();
            rx_byte1_hold = 0;
            rx_byte2_hold = 0;
            rx_byte3_hold = 0;
            rx_byte1 = 0;
            rx_byte2 = 0;
            rx_byte3 = 0;            
            command_state = 0;
        }
        if(BathInputFlag)
        {
            bit_set(ra_shadow, 3);
        }
        
        
        if (BlinkFlag == 1)
        {
            output(ra_shadow);
        }
        else
        {
            output(0x00);
        }
        
        reset_flag = 0;
	}while(!reset_flag);		// exit program loop when reset flag appears
	reset_cpu();				// and reset and restart
}
        



