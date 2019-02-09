// PIC18F87J11 Configuration Bit Settings
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on SWDTEN bit))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG1H
#pragma config CP0 = OFF        // Code Protection bit (Program memory is not code-protected)

// CONFIG2L
#pragma config FOSC = HSPLL     // Oscillator Selection bits (HS oscillator, PLL enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit (Two-Speed Start-up disabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler Select bits (1:32768)

// CONFIG3L
#pragma config EASHFT = ON      // External Address Bus Shift Enable bit (Address shifting enabled, address on external bus is offset to start at 000000h)
#pragma config MODE = MM        // External Memory Bus Configuration bits (Microcontroller mode - External bus disabled)
#pragma config BW = 16          // Data Bus Width Select bit (16-bit external bus mode)
#pragma config WAIT = OFF       // External Bus Wait Enable bit (Wait states on the external bus are disabled)

// CONFIG3H
#pragma config CCP2MX = ALTERNATE // ECCP2 MUX bit (ECCP2/P2A is multiplexed with RE7)
#pragma config ECCPMX = DEFAULT // ECCPx MUX bit (ECCP1 outputs (P1B/P1C) are multiplexed with RE6 and RE5; ECCP3 outputs (P3B/P3C) are multiplexed with RE4 and RE3)
#pragma config PMPMX = DEFAULT  // PMP Pin Multiplex bit (PMP port pins connected to EMB (PORTD and PORTE))
#pragma config MSSPMSK = MSK7   // MSSP Address Masking Mode Select bit (7-Bit Address Masking mode enable)

#include <xc.h>
#include "LCD.h"
#include "../src/Motor.h"
#include <stdio.h>

//Connections
//Using ECCP2
// P2A = RE7
// P2B = RE2
// P2C = RE1
// P2D = RE0
// Only need P2B  and P2D for L298

MotorDirection currentDirection = DIR_FWD;
unsigned int ReadPot(void);

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    LCDWriteLine("Starting Motor", 0);
    
    //Set up for ADC
    TRISA = 0b00000001;
    ADCON1 = 0b10111010; //Right justify, No calibration, 20 Tad, FOSC/32
    WDTCONbits.ADSHR = 1; //Switch to alternate address to access ANCON0
    ANCON0 = 0b11111110; //AN0 analog - all others digital
    WDTCONbits.ADSHR = 0; //Back to default address
    
    TRISD = 0;
    LATD = 1;
    TRISBbits.TRISB0 = 1;
    INTCONbits.INT0IF = 0;
    INTCON2bits.INTEDG0 = 0;
    INTCONbits.INT0IE = 1;
    
    InitPWM(25000, 1023);
    
    INTCONbits.GIE = 1;
    
    while (1) {
        unsigned int speed = ReadPot();
        SetSpeed(speed);
        char lcd[17];
        char dir;
        dir = currentDirection == DIR_FWD ? 'F' : 'B';
        sprintf(lcd, "Speed=%u Dir=%c", speed, dir);
        LCDClearLine(1);
        LCDWriteLine(lcd, 1);
        __delay_ms(200);
    }
}


void __interrupt(high_priority) HighIsr(void) {
    if (INT0IF == 1) {
        __delay_ms(20);  //debounce
        if (PORTBbits.RB0 == 0) {
            if (currentDirection == DIR_FWD) {
                currentDirection = DIR_BKWD;
            } else {
                currentDirection = DIR_FWD;
            }
            SetDirection(currentDirection);
            LATDbits.LATD0 ^= 1;
        }
        INT0IF = 0;
    }
}

unsigned int ReadPot(void) {
    ADCON0bits.CHS = 0; //channel 0
    ADCON0bits.ADON = 1;
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1);
    ADCON0bits.ADON = 0;
    return ADRES;
}