/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef MICROBIT_QDEC_H
#define MICROBIT_QDEC_H

#include "mbed.h"
#include "MicroBitConfig.h"
#include "MicroBitComponent.h"
#include "MicroBitPin.h"

struct QDecExtraConfig
{
    uint32_t samplePeriod = 128;
    uint8_t LEDDelay = 0;
    bool activeHighLED = true;
    bool useDebounce = false;
};

/**
  * Class definition for MicroBit Quadrature decoder.
  *
  */
class MicroBitQDec : public MicroBitComponent
{
    protected:
    static MicroBitPin pinNC;           // A no-connect pin -- the default for unused LED arguments.
    MicroBitPin&    phaseA,             // Phase A input for decoding
                    phaseB,             // Phase B input for decoding
                    LED;                // LED output to assert while decoding
    uint32_t        samplePeriod;       // Minimum sampling period allowed
    int64_t         position;           // Absolute position
    uint32_t        errors;             // Double-transition counter
    uint8_t         LEDDelay;           // power-up time for LED, in microseconds
    bool            activeHighLED = true;
    bool            useSystemTick = false;
    bool            useDebounce = false;

    public:

    /**
      * Constructor.
      * Create a software abstraction of the quadrature decoder.
      *
      * @param phaseA             Pin connected to quadrature encoder output A
      * @param phaseB             Pin connected to quadrature encoder output B
      * @param LED                The pin for the LED to enable during each quadrature reading
      * @param cfg->samplePeriod  Number of microseconds between samples
      * @param cfg->LEDDelay      Number of microseconds after LED activation before sampling
      * @param cfg->activeHighLED Whether LED is activated on high output (true), or low (false)
      * @param cfg->useDebounce   Use hardware debounce on quadrature inputs
      *
      * @code
      * MicroBitQDec qdec(QDEC_ID, QDEC_PHA, QDEC_PHB, QDEC_LED);
      * @endcode
      */
    MicroBitQDec(MicroBitPin& phaseA, MicroBitPin& phaseB, MicroBitPin& LED = pinNC, QDecExtraConfig const* cfg = NULL);

    /**
      * Automatically call poll() from the systemTick() event.
      *
      * This has the effect of keeping the position up to date to within
      * SYSTEM_TICK_PERIOD_MS milliseconds.  The event is enabled after a call
      * to start() or immediately, if start() has already been called.
      *
      * This should not be used if poll() is being called in response to
      * another regular event.
      */
    void enableSystemTick(void);

    /**
      * Do not automatically call poll() from the systemTick() event (this is the default).
      */
    void disableSystemTick(void);

    /**
      * Configure the hardware to keep this instance up to date.
      *
      * Several instances can exist so long as no more than one of them is
      * attached to the hardware.  This can be a practical way to control
      * several motors with their own encoders if they run only at different
      * times.
      *
      * While the hardware is active, `poll()` must be called
      *
      * @return MICROBIT_OK on success, MICROBIT_BUSY if the hardware is already attached to another instance, or MICROBIT_INVALID_PARAMETER if the configuration is invalid.
      */
    virtual int start(void);

    /**
      * Stop the hardware and make it available for use by other instances.
      */
    virtual void stop(void);

    /** Poll hardware for latest decoder movement and reset the hardware counter to zero.
      *
      * This must be called regularly to prevent the hardware from overflowing.
      * About ten times per second, or less if the attached hardware is
      * guaranteed to count more slowly than 10000 encoder counts per second.
      *
      * This call may be made from systemTick(), or a dedicated motor control ticker interrupt.
      */
    virtual void poll(void);

    /**
      * Read the absolute position of the encoder at last call to `poll()`.
      *
      * @return current decoder position.
      */
    int64_t getPosition(void) { return position; }

    /**
      * Reset the position to a known value.
      *
      * This can be used to zero the counter on detection of an index or end-stop signal.
      *
      * @param value to add to position
      */
    virtual void resetPosition(int64_t position = 0);

    /**
      * Read the number of polling errors since start().
      *
      * This value shows the number of times a sample has encountered a
      * double-transition condition, where the direction cannot be decoded
      * because the relative order of edge transitions was not witnessed.
      *
      * Such errors imply that the sampling period is too long.
      *
      * @return total number of errors.
      */
    int64_t getErrors(void) { return errors; }

    /**
      * Destructor for MicroBitQDec.
      *
      * Makes the necessary call to system_timer_remove_component() if somebody forgot.
      */
    virtual ~MicroBitQDec() override;

    virtual void systemTick(void) override;
};

#endif
