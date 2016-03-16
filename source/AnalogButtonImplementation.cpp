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

#include "wrd-touch/AnalogButtonImplementation.h"
#include "wrd-touch/lesense_api.h"

#if !defined(YOTTA_CFG_HARDWARE_WRD_TOUCH_SENSITIVITY)
#error platform not supported
#endif

AnalogButtonImplementation::AnalogButtonImplementation(uint32_t _channel, bool multipleUpdates)
    :   channel(_channel)
{
    if (channel < LESENSE_CHANNEL_TOTAL)
    {
        FunctionPointer onPress(this, &AnalogButtonImplementation::onPressTask);
        FunctionPointer onRelease(this, &AnalogButtonImplementation::onReleaseTask);

        /*  Add channel
        */
        lesense::params_t params;

        params.channel = channel;
        params.onPress = onPress;
        params.onRelease = onRelease;
        params.sensitivity = YOTTA_CFG_HARDWARE_WRD_TOUCH_SENSITIVITY;
        params.updates = multipleUpdates;

        lesense::addChannel(params);
    }
}

AnalogButtonImplementation::~AnalogButtonImplementation()
{
    FunctionPointer onPress(this, &AnalogButtonImplementation::onPressTask);
    FunctionPointer onRelease(this, &AnalogButtonImplementation::onReleaseTask);

    lesense::removeChannel(channel, onPress, onRelease);
}

void AnalogButtonImplementation::fall(FunctionPointer callback)
{
    onPressHandler = callback;
}

void AnalogButtonImplementation::rise(FunctionPointer callback)
{
    onReleaseHandler = callback;
}

void AnalogButtonImplementation::calibrate(FunctionPointer callback)
{
    calibrateCallback = callback;
    lesense::calibrate(true, true, this, &AnalogButtonImplementation::calibrateDoneTask);
}

void AnalogButtonImplementation::calibrateDoneTask(void)
{
    if (calibrateCallback)
    {
        calibrateCallback.call();
        calibrateCallback.clear();
    }
}

void AnalogButtonImplementation::cancelCalibration(void)
{
    lesense::cancelCalibration();
}

int32_t AnalogButtonImplementation::getValue(void) const
{
    return lesense::getValue(channel);
}

int32_t AnalogButtonImplementation::getMinValue(void) const
{
    return lesense::getMinValue(channel);
}

int32_t AnalogButtonImplementation::getMaxValue(void) const
{
    return lesense::getMaxValue(channel);
}

bool AnalogButtonImplementation::isPressed(void) const
{
    return lesense::channelIsActive(channel);
}

uint32_t AnalogButtonImplementation::getTimestamp(void) const
{
    return lesense::getTimestamp();
}

void AnalogButtonImplementation::setIdleFrequency(uint32_t freqHz)
{
    lesense::setIdleFrequency(freqHz);
}

void AnalogButtonImplementation::setActiveFrequency(uint32_t freqHz)
{
    lesense::setActiveFrequency(freqHz);
}

void AnalogButtonImplementation::pause(void)
{
    lesense::pause();
}

void AnalogButtonImplementation::resume(void)
{
    lesense::resume();
}

void AnalogButtonImplementation::onPressTask()
{
    if (onPressHandler)
    {
        onPressHandler.call();
    }
}

void AnalogButtonImplementation::onReleaseTask()
{
    if (onReleaseHandler)
    {
        onReleaseHandler.call();
    }
}
