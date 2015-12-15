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

#include "wrd-touch/AnalogButton.h"
#include "wrd-touch-efm32/lesense_api.h"

#if !defined(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_SENSITIVITY)
#error platform not supported
#endif

AnalogButton::AnalogButton(uint32_t _channel)
    :   channel(_channel)
{
    if (channel < LESENSE_CHANNEL_TOTAL)
    {
        FunctionPointer onPress(this, &AnalogButton::onPressTask);
        FunctionPointer onRelease(this, &AnalogButton::onReleaseTask);

        /*  Add channel
        */
        lesense::params_t params;

        params.channel = channel;
        params.onPress = onPress;
        params.onRelease = onRelease;
        params.sensitivity = YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_SENSITIVITY;
        params.updates = false;

        lesense::addChannel(params);
    }
}

AnalogButton::~AnalogButton()
{
    FunctionPointer onPress(this, &AnalogButton::onPressTask);
    FunctionPointer onRelease(this, &AnalogButton::onReleaseTask);

    lesense::removeChannel(channel, onPress, onRelease);
}

int32_t AnalogButton::getValue(void)
{
    return lesense::getValue(channel);
}

int32_t AnalogButton::getMinValue(void)
{
    return lesense::getMinValue(channel);
}

int32_t AnalogButton::getMaxValue(void)
{
    return lesense::getMaxValue(channel);
}
