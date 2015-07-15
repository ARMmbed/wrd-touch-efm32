/* mbed
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __LESENSE_API_H__
#define __LESENSE_API_H__


#include "mbed.h"


/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS 16

/* LESENSE number of channels actually in use */
#define NUM_ACTIVE_CHANNELS 5

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT gpioPortC

#define LESENSE_IFS_ALL_CHANNELS 0x0000FFFF

extern FunctionPointer calibrateDoneCallback;

namespace lesense {
    typedef struct CallbackNode
    {
        struct CallbackNode* next;
        FunctionPointer callback;
        bool multipleUpdates;
    } CallbackNode;

    typedef struct
    {
        uint32_t channel;
        FunctionPointer onPress;
        FunctionPointer onRelease;
        uint32_t sensitivity;
        bool updates;
    } params_t;

    void lesenseInit();

    void addChannel(params_t& parameters);

    uint16_t getValue(uint32_t channel);
    uint16_t getMinValue(uint32_t channel);
    uint16_t getMaxValue(uint32_t channel);

    uint32_t getEventTimestamp();
    uint16_t getCalibrationValueForChannel(uint32_t channel);

    bool channelIsActive(uint32_t channel);

    void setIdleFrequency(uint32_t freqHz);
    void setActiveFrequency(uint32_t freqHz);

    void setMaxValue(uint16_t maxValue, uint32_t channel);
    void setMinValue(uint16_t minValue, uint32_t channel);
    void setThreshold(uint16_t threshold, uint32_t channel);

    /*  Power management
    */
    void resume();
    void pause();

    /*  Calibration
    */
    void internalCalibrate(bool forceCalibration, bool useNewValues);

    void calibrate(bool forceCalibration, bool useNewValues, void (*callback)(void));

    template <typename T>
    void calibrate(bool forceCalibration, bool useNewValues, T* object, void (T::*callback)(void))
    {
        calibrateDoneCallback.attach(object, callback);

        internalCalibrate(forceCalibration, useNewValues);
    }

    /*  Private
    */
    bool addCallback(FunctionPointer& newCallback, bool updates, struct CallbackNode** nodes);
    bool removeCallback(FunctionPointer& oldCallback, struct CallbackNode** node);
    void cleanupCallback(struct CallbackNode** node);
}

#endif // __LESENSE_API_H__
