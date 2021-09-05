#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

class motor {
    
public :
    motor(PinName dright , PinName dleft, PinName pwmpin);
    void setpwm(float inputpwm);
        
private :
    PwmOut pwmpin_;
    DigitalOut dright_;
    DigitalOut dleft_;   
};

#endif
