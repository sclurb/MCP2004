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
        LATBbits.LATB5 = 1;
    }
    else
    {
        LATBbits.LATB5 = 0;
    }
    if(bit_test(LED,2))
    {
        LATBbits.LATB3 = 1;
    }
    else
    {
        LATBbits.LATB3 = 0;
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

void ProcessADC()
{
    if(BedCall > 0xbe )
    {
        stat_call_trouble = 1;
        stat_ecall = 0;
    }
    if((BedCall < 0xbe) && (BedCall > 0x63 ))
    {
        stat_call_trouble = 0;
        stat_ecall = 0;
    }
    if(BedCall < 0x63 )
    {
        stat_call_trouble = 0;
        stat_ecall = 1;
    }
    
    if(Smoke > 0xbe )
    {
        stat_smoke_trouble = 1;
        stat_smoke_alarm = 0;
    }
    if((Smoke < 0xbe) && (BedCall > 0x63 ))
    {
        stat_smoke_trouble = 0;
        stat_smoke_alarm = 0;
    }
    if(Smoke < 0x63 )
    {
        stat_smoke_trouble = 0;
        stat_smoke_alarm = 1;
    }
}
// This function gets data from the A/D. The specified input in the parameter chooses which A/D

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
            reset_flag = 1;
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
	int anddata, ordata;
	// First byte
	anddata = rx_byte1 & rx_byte1_hold;		// examine first byte record
	ordata = rx_byte1 | rx_byte1_hold;		// needed to confirm zero's
	if ((anddata & 0x40) != 0)
	{	
        SMOKE_PWR = 0;  
    }
	if ((ordata & 0x40) == 0)
	{	
        SMOKE_PWR = 1;  
    }
	
	// Second byte
	anddata = rx_byte2 & rx_byte2_hold;		// examine second byte record
	ordata = rx_byte2 | rx_byte2_hold;
		//reset acknowledge
	if ((anddata & 0x01) != 0)
	{	
        stat_panel_reset = 0;  
    }			// reset acknowledge
	
		// checkin / cancel
	if ((anddata & 0x20) !=0)				// was 0x02
	{	
        ack_checkin = 1;
    }
	
		//bed and bath call ack
	if ((anddata & 0x0a) != 0)				// was 0x04, now two possible sources of acknowledgement
	{	
        ack_ecall = 1;  
    }			// check-in acknowledge
	if (ack_ecall)// && sw_checkin)		// if both call ack and checkin switch
	{	
        stat_ecall = 0; 
        ack_ecall = 0;
    }
		
		// bed call placed	
	if ((anddata & 0x10) !=0)		// call indicator command, was 0x20
	{	
        ind_call = 1;  
    }	
	if ((ordata & 0x10) ==0)
	{	
        ind_call = 0; 	
    }
	
		// check-in indicator	
	if ((anddata & 0x40) !=0)		// check-in indicator command
	{	
        ind_checkin = 1;  
    }	
	if ((ordata & 0x40) == 0)
	{	
        ind_checkin = 0; 	
    }
		
	if (!stat_security)
	{	
        poll_count = 0; 
        ack_security = 0; 
    }
	else 
	{	
		poll_count++;					
		if (poll_count > 1)				// ack after two poll cycles
		{	
            ack_security = 1;  
        }	  
	}
	
	// Third byte
	anddata = rx_byte3 & rx_byte3_hold;		// examine third byte record
	ordata = rx_byte3 | rx_byte3_hold;

	rx_byte1_hold = rx_byte1;
	rx_byte2_hold = rx_byte2;
	rx_byte3_hold = rx_byte3;
	return;
// ack_checkin;
// ack_ecall;
// ack_icm;
// ack_security;
// ind_call;
// ind_checkin;
}

  ////////////////////////
 /// Transmits Status ///
////////////////////////

void transmit_status()
{
	//output_low(TP1);
	tx_byte1 = 0;	
	tx_byte2 = 0;	
	
	if (stat_smoke_alarm)
	{	bit_set(tx_byte1,2);  }
	if (stat_smoke_trouble)
	{	bit_set(tx_byte2,2);  }
	if (stat_call_trouble)
	{	bit_set(tx_byte2,1);  }
	if (stat_security)
	{	bit_set(tx_byte1,5);  }
		
	if (stat_panel_reset)
	{	bit_set(tx_byte2,6);  }	
	if (stat_checkin)
	{	bit_set(tx_byte1,4);  }	
	if (stat_ecall)
	{	bit_set(tx_byte1,1);  }	
	if (stat_icm)
	{	bit_set(tx_byte1,6);  }	
    
    TXdata[0] = tx_byte1;
    TXdata[1] = tx_byte2;

    __delay_ms(6);
    send(TXdata, 2);
	return;
}

void ProcessPinInput(unsigned int port)
{
    RC1 = 0;
    switch(port)
    {
        case 1:
            BitTest1 = PORTBbits.RB2;
            // delay
            BitTest2 = PORTBbits.RB2;
            break;
        case 2:
            BitTest1 = PORTAbits.RA3;
            // delay
            BitTest2 = PORTAbits.RA3;
            break;
    }
    if((BitTest1 == BitTest2)&&(BitTest2 == 1))
    {
        returncode = (unsigned char)BitTest1;
    }
}

void ProcessADCValue()
{
    RC = 0;
    if(ADCValue > 0xDE)
    {
        RC = 1;
    }
    else
    {
        if(ADCValue <0x63)
        {
            RC = 2;
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
        if(led_timer == 0xff)
        {
            led_timer = 0x20;
        }
        else
        {
            led_timer++;
        }
        if(minute_timer == 0xffff)
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
 
 receive_mode = 1;
// main program loop
	do
	{
        switch(Step)
        {
            case 3:             // process cancel button
                Step = 0;
                ProcessPinInput(1);
                CancelCallFlag = returncode;
                NOP();
                break;
            case 2:             // process smoke input
                Step = 3;
        		ADCValue = Getdata(0x07);
                ProcessADCValue();
                if(RC == 2)
                {
                    SmokeTroubleFlag = 1;
                }
                else
                {
                    if(RC == 0)
                    {
                      SmokeInputFlag = 1;
                    }
                }
                break;
            case 1:         // process bed call
                Step = 2;
        		ADCValue = Getdata(0x03);
                ProcessADCValue();
                if(RC == 2)
                {
                    BedTroubleFlag = 1;
                }
                else
                {
                    if(RC == 0)
                    {
                      BedInputFlag = 1;
                    }
                }
                break;
            case 0:         // process the bath call
                Step = 1;
                ProcessPinInput(2);
                BathInputFlag = returncode;
                NOP();
                break;
        }


	}while(!reset_flag);		// exit program loop when reset flag appears
	reset_cpu();				// and reset and restart
}
        
/*
		BedCall = Getdata(0x03);
        Smoke = Getdata(0x07);
        ProcessADC();
        getAddress();

        sw_call = (unsigned char)(bit_test(PORTA, 3));
        if (sw_call == 8)
        {
            sw_call = 0;
        }
        else
        {
            sw_call = 1;
        }
        sw_checkin = (unsigned char)(bit_test(PORTB, 2));
        if(sw_checkin == 4)
        {
            sw_checkin = 0;
        }
        else
        {
            sw_checkin = 1;
        }
        
		//restart_wdt();
        
		// check for received data, if present call evaluation function
			// eval function will determine first or second byte check address and if match, will store command in two byte variables
			// after receiving a second byte, the function will check last to most recent to determine whether to execute, 
			// but in any case will flag a response to the console
        if (rcv_data_flag == 3)
        {
            eval_receiever(rx_byte1, rx_byte2, rx_byte3);
        }
        
		if (command_state == 2)
		{	
            //eval_receiver(rx_data); 
            exec_command();						// done, so now call function to execute comand conditional on three identical commands
            transmit_status();
            rx_byte1_hold = 0;
            rx_byte2_hold = 0;
            rx_byte3_hold = 0;
            rx_byte1 = 0;
            rx_byte2 = 0;
            rx_byte3 = 0;            
            command_state = 0;
        }


        // call indicator decision tree
        //if (stat_ecall || !sw_call)
        if (stat_ecall || sw_call) 			// if either call switch is on, flash 4 Hz
        {	
            if	(bit_test(led_timer,0))
            {	
                bit_clear(ra_shadow,LED_CALL); 
            }
            else 
            {
                bit_set(ra_shadow,LED_CALL); 
            }
        }
        else if (ind_call && sw_checkin)
        {	
            timeout_30s = 1;	// start 30 second timer and blank call LED
            led_timer = 0x20;
        }
        else if (ind_call && !timeout_30s)									// unless call indicator is commanded on
        {	
            bit_set(ra_shadow, LED_CALL);  
        }
        else if (stat_ecall)								// else flash at 1 hz
        {
            if (bit_test(led_timer,2))
            {	
                bit_clear(ra_shadow,LED_CALL); 
            }
            else
            {	
                bit_set(ra_shadow,LED_CALL); 
            }
        }
        else
        {	
            bit_clear(ra_shadow, LED_CALL);  
        }

        // checkin indicator decision tree
        if (ind_checkin & !timeout_20s & !sw_checkin)                
        {
            if	(bit_test(led_timer,1))
            {	
                bit_set(ra_shadow,LED_CHK_IN); 
            }
            else 
            {	
                bit_clear(ra_shadow,LED_CHK_IN); 
            }
        }
        else if (ind_checkin && sw_checkin)
        {	
            timeout_20s = 1;		// start 20 second timer and blank call LED
            led_timer = 0x70;
        }
        else
        {	
            bit_set(ra_shadow,LED_CHK_IN); 
        }

		
		if (led_timer == 0xff)				// end 30sec or 20sec timeout here
		{	
            timeout_30s = 0; 
            timeout_20s = 0;  
        }

		
		
		
		// read serial data from shift register and store
		

		// poll and update status bits
			// supervised inputs, call and smoke
		if (vref_toggle)
		{
			stat_smoke_alarm = bit_test(CMCON,7);		// comparator = 1 for active
			if(!stat_ecall)
			{	stat_ecall = (bit_test(CMCON,6) || sw_call);  }		//event, latch
			setup_vref(VREF_HIGH | COMP_TROUBLE);		// for next time around
		}
		else
		{
			stat_smoke_trouble = !bit_test(CMCON,7);		// comparator = 0 for trouble
			stat_call_trouble = !bit_test(CMCON,6);
			setup_vref(VREF_HIGH | COMP_SUPER);		// for next time around
		}
		vref_toggle=!vref_toggle;
        
        
		
		if (!stat_checkin)
		{	stat_checkin = sw_checkin;  } 		//event, switch
		else if (stat_checkin && ack_checkin)		// 
		{	stat_checkin = 0; ack_checkin = 0;}	
			
		if (!stat_security)
		{	stat_security = sw_security;  }		//event, security
		else if (ack_security && !sw_security)  // ack without level
		{	stat_security = 0;  }
		
		output(ra_shadow);
*/

