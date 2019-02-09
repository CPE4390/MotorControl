#include <xc.h>
#include "Motor.h"

static unsigned int maxSpeed;
static unsigned long int period;
static unsigned int preScale;

void InitPWM(long unsigned int freq, unsigned int maxSpeedValue) {
    maxSpeed = maxSpeedValue;
    TRISE |= 0b10000111; //Set PWM pins as inputs so we can configure ECCP
    T4CONbits.TMR4ON = 0;
    CCP2CONbits.CCP2M = 0; //Off
    CCP2CONbits.P2M = 0b01; //Full-bridge forward 
    CCP2CONbits.DC2B = 0; //Set duty cycle low bits to zero
    CCPR2L = 0; //Duty cycle high bits.
    CCP2CONbits.CCP2M = 0b1100; //PWM mode all active high
    T3CONbits.T3CCP1 = 1; //T3 and T4 used by all CCP modules
    T3CONbits.T3CCP2 = 1;
    if (freq < 7812) {
        T4CONbits.T4CKPS = 0b10; // 1:16 prescale = 500 kHz
        preScale = 16;
        PR4 = (unsigned char) (500000L / freq - 1);
    } else if (freq < 31250) {
        T4CONbits.T4CKPS = 0b01; // 1:4 prescale = 2 MHz
        preScale = 4;
        PR4 = (unsigned char) (2000000L / freq - 1);
    } else {
        T4CONbits.T4CKPS = 0b00; // 1:1 prescale = 8 MHz
        preScale = 1;
        PR4 = (unsigned char) (8000000L / freq - 1);
    }
    period = preScale * 4L * (PR4 + 1);
    PIR3bits.TMR4IF = 0;
    T4CONbits.TMR4ON = 1;
    while (PIR3bits.TMR4IF == 0);
    TRISE &= 0b01111000;
}

void SetSpeed(unsigned int speed) {
    if (speed > maxSpeed) {
        speed = maxSpeed;
    }
    unsigned long dc = (period * speed) / maxSpeed;
    dc /= preScale;
    if (dc > period) {
        dc = period;
    }
    CCP2CONbits.DC2B = (unsigned char)dc;
    CCPR2L = (unsigned char)(((unsigned int)dc) >> 2);
}

void SetDirection(MotorDirection dir) {
    //TODO avoid direction change at near 100% duty cycle to prevent shoot through
    if (dir == DIR_FWD) {
        CCP2CONbits.P2M1 = 0;
    } else {
        CCP2CONbits.P2M1 = 1;
    }
}