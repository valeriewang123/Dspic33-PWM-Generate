/*******************************************************************************
  Code example to generate Pulse Width Modulation(PWM signal) 
  using Output Compare module and Timer module.

  Author: Xiaoyu Wang
 
  Time: 15/11/2019

  NOTE: FOR PROJECT UNICARagil(173124) IN IKA.

  File Name:
    PWM_GEN.c
    
 Summary:
    This file contains following routines
        int  main(void)
        void Init_CLK(void);
        void Init_Timer2(void);
        void Init_PPS(void);
        void Init_OC1(void);
 
        
  Description:
        This is main file for generating a pwm signal of 40us pulse period 
		with 50% duty cycle. User can modify the Period and Duty cycle as 
        desired. Pulse period can be modified by writting to PR2 registe.  
        Duty cycle can be modified by writing to respective OCxR registers. 
 *******************************************************************************/

//*******************************************************************************
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include "p33exxxx.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************
// DSPIC33EP512GM706 Configuration Bit Settings
// Reference: http://ww1.microchip.com/downloads/en/DeviceDoc/70000618d.pdf 
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON           //  (BOR is enabled)
#pragma config ALTI2C1 = OFF        // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF        // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25       // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON          // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON         // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRCPLL        // Oscillator Source Selection (Internal Fast RC (FRC) with PLL)
#pragma config PWMLOCK = OFF        // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF            // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// Functions prototypes
void Init_Timer2 ( void );
void Init_CLK( void );
void Init_PPS( void ); 
void Init_OC1( void ); 


int main(void)
{
    Init_CLK();             // Initialize the Oscillator = 70Mhz
    Init_Timer2();          // Initilise Timer 2 with Period = 2800/70Mhz = 40us
    Init_PPS();             // Map OC1 Ouputs to RP43R
    Init_OC1();             // Initialise OC1 module with duty cycle = 50%
    T2CONbits.TON = 1;      // Start the timer,the clock source for Output Compare Modules

    while( 1 )
    { ;}
}

/******************************************************************************
*    Configure Phase-Locked Loop with  a programmable output divider, or 
*	 an input divider, to scale the input frequency to suit the application:
*	 Here dsPIC33E requires input frequency operating at Fcy = 70MIP
*    Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
*    Fin = FRC = 7.37Mhz
*    Fosc= FRC*76/(2*2)=1400Mhz 
*    Reference: http://ww1.microchip.com/downloads/en/DeviceDoc/70005131a.pdf
*    7.0 - PHASE-LOCKED LOOP (PLL)  - From Page 26
*****************************************************************************/
void Init_CLK(void)
{

    PLLFBD = 74;                        // M=76
    CLKDIVbits.PLLPOST = 0;             // N1=2
    CLKDIVbits.PLLPRE = 0;              // N2=2
    OSCTUN = 0;                         // Tune FRC oscillator, if FRC is used
    RCONbits.SWDTEN = 0;                // Disable Watch Dog Timer

    while( OSCCONbits.LOCK != 1 )       // Wait for PLL to lock    
    {;}
}

/******************************************************************************
*   By setting PR2 = 2800
*   Pluse Period = 2800/Fout = 2800/70Mhz = 40us 
*   Reference: http://ww1.microchip.com/downloads/en/DeviceDoc/S11.pdf
*   11.3 - CONTROL REGISTERS - From Page 11-6
 *****************************************************************************/
void Init_Timer2( void )
{
    T2CON = 0X0000;
    TMR2 = 0X0000;
    PR2 = 2800; // Specify the Pulse period here,is configured for 40 us
}


/******************************************************************************
*   Init_OC1() configures pulse width of OC1 output,
*   Duty cycle = Pulse Width/ Period = 20us / 40us = 50%
*   Reference: http://ww1.microchip.com/downloads/en/DeviceDoc/70005159a.pdf 
*   2.0 - OUTPUT COMPARE REGISTERS - From Page 4
 *****************************************************************************/
void Init_OC1( void )
{
    OC1R = 1400;                // Specify the pulse width of the OC1 Output ,initialised for 20us width, namely 50% Duty Cycle
    OC1RS = 0x000;              // Initialize the secondary compare registe
    OC1CON1 = 0x0;              // Clear all control bits
    OC1CON2 = 0x0;              // Clear all control bits
    OC1CON1bits.OCTSEL = 0x0;   // Select peripheral clock as clock source
    OC1CON2bits.SYNCSEL = 0xC;  // Select Timer2 as sync source
    OC1CON1bits.OCM = 0x6;      // Double compare continuous pulse mode
}

/******************************************************************************
 *  This function is used to initialize the Peripheral Pin Select.
 *  Reference: http://ww1.microchip.com/downloads/en/DeviceDoc/70000598c.pdf
 *  10.4.3.2 - OUTPUT MAPPING - Page 10-11/12
 *****************************************************************************/
void Init_PPS( void )
{
    __builtin_write_OSCCONL( OSCCON & (~(1 << 6)) );

    _RP43R = 0X10;  // RP43 = RB11 pin is mapped as OC1 out

    __builtin_write_OSCCONL( OSCCON | (1 << 6) );
}

/*******************************************************************************
 End of File
 */
