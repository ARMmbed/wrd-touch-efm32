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

#ifndef __ANALOG_BUTTON_IMPLEMENTATION_H__
#define __ANALOG_BUTTON_IMPLEMENTATION_H__

#include "core-util/FunctionPointer.h"

using namespace mbed::util;

class AnalogButtonImplementation
{
public:
    AnalogButtonImplementation(uint32_t channel, bool multipleUpdatees = false);
    ~AnalogButtonImplementation();

    void fall(FunctionPointer onPress);
    void rise(FunctionPointer onRelease);

    template <typename T>
    void fall(T* object, void (T::*member)(void))
    {
        FunctionPointer callback(object, member);
        fall(callback);
    }

    template <typename T>
    void rise(T* object, void (T::*member)(void))
    {
        FunctionPointer callback(object, member);
        rise(callback);
    }

    void calibrate(FunctionPointer callback);
    void cancelCalibration(void);

    int32_t getValue(void) const;
    int32_t getMinValue(void) const;
    int32_t getMaxValue(void) const;

    bool isPressed(void) const;

    uint32_t getTimestamp(void) const;

    void setIdleFrequency(uint32_t freqHz);
    void setActiveFrequency(uint32_t freqHz);

    void pause(void);
    void resume(void);

private:
    void onPressTask(void);
    void onReleaseTask(void);
    void calibrateDoneTask(void);

    uint32_t channel;
    FunctionPointer onPressHandler;
    FunctionPointer onReleaseHandler;
    FunctionPointer calibrateCallback;
};

#endif // __ANALOG_BUTTON_IMPLEMENTATION_H__
