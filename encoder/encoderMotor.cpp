//#include "mbed.h"
// Based on the QEI library by Aaron Berk on Mbed.
//=======================================================
/**
 * @author Aaron Berk
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * Quadrature Encoder Interface.
 *
 * A quadrature encoder consists of two code tracks on a disc which are 90
 * degrees out of phase. It can be used to determine how far a wheel has
 * rotated, relative to a known starting position.
 *
 * Only one code track changes at a time leading to a more robust system than
 * a single track, because any jitter around any edge won't cause a state
 * change as the other track will remain constant.
 *
 * Encoders can be a homebrew affair, consisting of infrared emitters/receivers
 * and paper code tracks consisting of alternating black and white sections;
 * alternatively, complete disk and PCB emitter/receiver encoder systems can
 * be bought, but the interface, regardless of implementation is the same.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |  ^  |     |     |     |     |
 *            ---+  ^  +-----+     +-----+     +-----
 *               ^  ^
 *               ^  +-----+     +-----+     +-----+
 * Channel B     ^  |     |     |     |     |     |
 *            ------+     +-----+     +-----+     +-----
 *               ^  ^
 *               ^  ^
 *               90deg
 *
 * The interface uses X2 encoding by default which calculates the pulse count
 * based on reading the current state after each rising and falling edge of
 * channel A.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |     |     |     |     |     |
 *            ---+     +-----+     +-----+     +-----
 *               ^     ^     ^     ^     ^
 *               ^  +-----+  ^  +-----+  ^  +-----+
 * Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
 *            ------+  ^  +-----+  ^  +-----+     +--
 *               ^     ^     ^     ^     ^
 *               ^     ^     ^     ^     ^
 * Pulse count 0 1     2     3     4     5  ...
 *
 * This interface can also use X4 encoding which calculates the pulse count
 * based on reading the current state after each rising and falling edge of
 * either channel.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |     |     |     |     |     |
 *            ---+     +-----+     +-----+     +-----
 *               ^     ^     ^     ^     ^
 *               ^  +-----+  ^  +-----+  ^  +-----+
 * Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
 *            ------+  ^  +-----+  ^  +-----+     +--
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 * Pulse count 0 1  2  3  4  5  6  7  8  9  ...
 *
 * It defaults
 *
 * An optional index channel can be used which determines when a full
 * revolution has occured.
 *
 * If a 4 pules per revolution encoder was used, with X4 encoding,
 * the following would be observed.
 *
 *               +-----+     +-----+     +-----+
 * Channel A     |     |     |     |     |     |
 *            ---+     +-----+     +-----+     +-----
 *               ^     ^     ^     ^     ^
 *               ^  +-----+  ^  +-----+  ^  +-----+
 * Channel B     ^  |  ^  |  ^  |  ^  |  ^  |     |
 *            ------+  ^  +-----+  ^  +-----+     +--
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 *               ^  ^  ^  +--+  ^  ^  +--+  ^
 *               ^  ^  ^  |  |  ^  ^  |  |  ^
 * Index      ------------+  +--------+  +-----------
 *               ^  ^  ^  ^  ^  ^  ^  ^  ^  ^
 * Pulse count 0 1  2  3  4  5  6  7  8  9  ...
 * Rev.  count 0          1           2
 *
 * Rotational position in degrees can be calculated by:
 *
 * (pulse count / X * N) * 360
 *
 * Where X is the encoding type [e.g. X4 encoding => X=4], and N is the number
 * of pulses per revolution.
 *
 * Linear position can be calculated by:
 *
 * (pulse count / X * N) * (1 / PPI)
 *
 * Where X is encoding type [e.g. X4 encoding => X=44], N is the number of
 * pulses per revolution, and PPI is pulses per inch, or the equivalent for
 * any other unit of displacement. PPI can be calculated by taking the
 * circumference of the wheel or encoder disk and dividing it by the number
 * of pulses per revolution.
 */

#include "encoderMotor.h"

encoderMotor::encoderMotor(PinName channelA,
                         PinName channelB,
                         float pulsesPerRev,
                         Encoding encoding) : channelA_(channelA), channelB_(channelB)
{
    pulses_       = 0;
    revolutions_  = 0;
    pulsesPerRev_ = pulsesPerRev;
    encoding_     = encoding;

    //Workout what the current state is.
    int chanA = channelA_.read();
    int chanB = channelB_.read();

    //2-bit state.
    currState_ = (chanA << 1) | (chanB);
    prevState_ = currState_;

    //X2 encoding uses interrupts on only channel A.
    //X4 encoding uses interrupts on      channel A,
    //and on channel B.
    channelA_.rise(this, &encoderMotor::encode);
    channelA_.fall(this, &encoderMotor::encode);

    //If we're using X4 encoding, then attach interrupts to channel B too.
    if (encoding == X4_ENCODING) {
        channelB_.rise(this, &encoderMotor::encode);
        channelB_.fall(this, &encoderMotor::encode);
    }
}

void encoderMotor::reset(void) {

    pulses_      = 0;
    revolutions_ = 0;

}

/*int encoderMotor::getCurrentState(void) {

    return currState_;

}*/

int encoderMotor::getPulses(void) {

    return pulses_;

}

float encoderMotor::getRevolutions(void) {

    revolutions_ = (float) pulses_ / pulsesPerRev_;
    double x = revolutions_ ;
    pulses_       = 0;
    revolutions_  = 0;    
    return x ;
    
}

void encoderMotor::encode(void) {

    int change = 0;
    int chanA  = channelA_.read();
    int chanB  = channelB_.read();

    //2-bit state.
    currState_ = (chanA << 1) | (chanB);

    if (encoding_ == X2_ENCODING) {

        //11->00->11->00 is counter clockwise rotation or "forward".
        if ((prevState_ == 0x3 && currState_ == 0x0) ||
                (prevState_ == 0x0 && currState_ == 0x3)) {
            pulses_++;
        }
        //10->01->10->01 is clockwise rotation or "backward".
        else if ((prevState_ == 0x2 && currState_ == 0x1) ||
                 (prevState_ == 0x1 && currState_ == 0x2)) {
            pulses_--;
        }

    } else if (encoding_ == X4_ENCODING) {

        //Entered a new valid state.
        if (((currState_ ^ prevState_) != INVALID) && (currState_ != prevState_)) {
            //2 bit state. Right hand bit of prev XOR left hand bit of current
            //gives 0 if clockwise rotation and 1 if counter clockwise rotation.
            change = (prevState_ & PREV_MASK) ^ ((currState_ & CURR_MASK) >> 1);
            if (change == 0) {
                change = -1;
            }

            pulses_ -= change;
        }
    
    }

    prevState_ = currState_;

}

void encoderMotor::disableInterrupts(void) 
{
    channelA_.disable_irq();
    channelB_.disable_irq();
}

void encoderMotor::enableInterrupts(void)
{
    channelA_.enable_irq();
    channelB_.enable_irq();    
}
