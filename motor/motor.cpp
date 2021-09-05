#include "motor.h"

motor::motor(PinName dright, PinName dleft, PinName pwmpin): dright_(dright), dleft_(dleft), pwmpin_(pwmpin) {
    pwmpin_.period_us(50);  
}

void motor::setpwm(float inputpwm) {
    
    if (inputpwm >= 0) {
        dright_ = 1;
        dleft_ = 0;
        pwmpin_.write(inputpwm);
    } else {
        dright_ = 0;
        dleft_ = 1;
        pwmpin_.write(-1 * inputpwm);
    }  
}
