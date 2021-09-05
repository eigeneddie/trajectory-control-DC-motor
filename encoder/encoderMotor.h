//Code is based on the QEI library by Aaron Berk on Mbed

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

#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */
#define PREV_MASK 0x1 //Mask for the previous state in determining direction
//of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction
//of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.

/**
 * Quadrature Encoder Interface.
 */
class encoderMotor {

public:

    typedef enum Encoding {

        X2_ENCODING,
        X4_ENCODING

    } Encoding;
    
    /** Membuat interface dari encoder    
     *
     * @param inA DigitalIn, out A dari encoder
     * @param inB DigitalIn, out B dari encoder
     */
    encoderMotor(PinName channelA, PinName channelB, float pulsesPerRev, Encoding encoding = X2_ENCODING);
    
    /**
     * Reset encoder.
     *
     * Menset pulse dan revolusi/putaran menjadi 0
     */
    void reset(void);
    
    /**
     * Membaca pulse yang didapat oleh encoder
     *
     * @return Nilai pulse yang telah dilalui.
     */
    int getPulses(void);
    
    /**
     * Membaca putaran yang didapat oleh encoder
     *
     * @return Nilai revolusi/putaran yang telah dilalui.
     */
    float getRevolutions(void);
    
    /**
     * Membaca pulse yang didapat oleh encoder
     *
     * @return Nilai pulse yang telah dilalui.
     */
    //int readDeltaPulses(void);
    
    /**
     * Membaca putaran yang didapat oleh encoder
     *
     * @return Nilai revolusi/putaran yang telah dilalui.
     */
    //int readDeltaRevolutions(void);
    
     void disableInterrupts(void);

    /**
     * Enable encoder interrupt pins
     *
     * @return Enable interrupt pins
     */  
    void enableInterrupts(void);

private:

    /**
     * Menghitung pulse
     *
     * Digunakan setiap rising/falling edge baik channel A atau B
     * Membaca putaran CW atau CCW => mengakibatkan pertambahan/pengurangan pulse
     */
    void encode(void);

    /**
     * Indeks setiap rising edge untuk menghitung putaran
     * Nilai bertambah 1
     */
    //void index(void);

    Encoding encoding_;

    InterruptIn channelA_;
    InterruptIn channelB_;
    //InterruptIn index_;

    float          pulsesPerRev_;
    int          prevState_;
    int          currState_;

    volatile int pulses_;
    volatile float revolutions_;


};

#endif /* encoderMotor_H */
