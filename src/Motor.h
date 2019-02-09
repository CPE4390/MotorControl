/* 
 * File:   Motor.h
 * Author: bemcg
 *
 * Created on February 8, 2019, 4:24 PM
 */

#ifndef MOTOR_H
#define	MOTOR_H

#ifdef	__cplusplus
extern "C" {
#endif
#define _XTAL_FREQ  32000000L
    typedef enum {DIR_FWD = 1, DIR_BKWD = -1} MotorDirection;
    void InitPWM(long unsigned int freq, unsigned int maxSpeedValue);
    void SetSpeed(unsigned int speed);
    void SetDirection(MotorDirection dir);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_H */

