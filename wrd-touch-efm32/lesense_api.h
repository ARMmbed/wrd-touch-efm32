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

#include "core-util/FunctionPointer.h"

#if defined(YOTTA_CFG_HARDWARE_LESENSE)
#define LESENSE_CHANNEL_TOTAL       YOTTA_CFG_HARDWARE_LESENSE_TOTAL_CHANNELS
#define LESENSE_CH_PORT             YOTTA_CFG_HARDWARE_LESENSE_PORT
#define LESENSE_IFS_ALL_CHANNELS    0xFFFF
#else
#error LESENSE module missing. Platform not supported.
#endif

#if defined(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH)
#define LESENSE_SAMPLE_DELAY            YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_SAMPLE_DELAY
#define LESENSE_CHANNELS_IN_USE         YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_CHANNELS_IN_USE
#define LESENSE_SCAN_FREQUENCY_LOW      YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_SCAN_FREQUENCY_LOW
#define LESENSE_SCAN_FREQUENCY_HIGH     YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_SCAN_FREQUENCY_HIGH
#define LESENSE_CALIBRATION_VALUES    YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_CALIBRATION_VALUES
#else
#error Platform not supported.
#endif

using namespace mbed::util;

namespace lesense {
    extern FunctionPointer calibrateDoneCallback;

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

    typedef enum {
        Channel0 = 0,
        Channel1 = 1,
        Channel2 = 2,
        Channel3 = 3,
        Channel4 = 4,
        Channel5 = 5,
        Channel6 = 6,
        Channel7 = 7,
        Channel8 = 8,
        Channel9 = 9,
        Channel10 = 10,
        Channel11 = 11,
        Channel12 = 12,
        Channel13 = 13,
        Channel14 = 14,
        Channel15 = 15,
    } channel_t;

    void addChannel(params_t& parameters);

    void removeChannel(uint32_t lesenseChannel, FunctionPointer& callOnPress, FunctionPointer& callOnRelease);

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
        lesense::calibrateDoneCallback.attach(object, callback);

        internalCalibrate(forceCalibration, useNewValues);
    }

    void calibrateDoneTask(void);
    void cancelCallback(void);
}

#endif // __LESENSE_API_H__
