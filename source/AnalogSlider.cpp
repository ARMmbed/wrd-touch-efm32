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

#include "wrd-touch/AnalogSlider.h"
#include "wrd-touch-efm32/lesense_api.h"

#if 0
#include "swo/swo.h"
#define printf(...) { swoprintf(__VA_ARGS__); }
#else
#define printf(...)
#endif

#if !defined(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_SENSITIVITY)
#error platform not supported
#endif

AnalogSlider::AnalogSlider(const uint32_t* _channelMap, uint32_t _channelsInUse)
    :   channelMap(NULL),
        channelsInUse(_channelsInUse),
        location(0),
        speed(0),
        acceleration(0),
        pressed(false),
        eventTimestamp(0)
{
    /*  Allocate and store channelMap.
    */
    channelMap = new uint32_t[channelsInUse];

    /*  Setup callback functions to calculate slider position.
    */
    FunctionPointer onPress(this, &AnalogSlider::sliderPressTask);
    FunctionPointer onRelease(this, &AnalogSlider::sliderReleaseTask);

    /*  Add channels defined by channel map.
    */
    lesense::params_t params;

    params.onPress = onPress;
    params.onRelease = onRelease;
    params.sensitivity = YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_TOUCH_SENSITIVITY;
    params.updates = true;

    for (std::size_t idx = 0; idx < channelsInUse; idx++)
    {
        channelMap[idx] = _channelMap[idx];
        params.channel = _channelMap[idx];

        lesense::addChannel(params);
    }
}

AnalogSlider::~AnalogSlider()
{
    // remove callbacks for press and release
    FunctionPointer onPress(this, &AnalogSlider::sliderPressTask);
    FunctionPointer onRelease(this, &AnalogSlider::sliderReleaseTask);

    for (std::size_t idx = 0; idx < channelsInUse; idx++)
    {
        lesense::removeChannel(channelMap[idx], onPress, onRelease);
    }

    // if calibration in progress, remove callback
    if (calibrateCallback)
    {
        lesense::cancelCallback();
    }

    delete[] channelMap;
}

/*  This function is executed when one of the analog channels crosses the
    deadzone threshold.
*/
void AnalogSlider::sliderPressTask()
{
    /*  Get the timestamp for this event.
    */
    uint32_t newEventTimestamp = lesense::getEventTimestamp();

    if (newEventTimestamp != eventTimestamp)
    {
        /*  Each analog channel is treated as a "pressure" sensor, where a hard
            press corresponds to a low value and a soft press as a high value.
            The location is calculated as the weighted pressure average across
            all channels, where each channel's location is used as the weight.
        */
        uint32_t pressure[channelsInUse];
        uint32_t pressureNorm[channelsInUse];
        uint32_t total = 0;
        uint32_t totalNorm = 0;

        /*  For each channel, find the pressure as a fraction of the maximum
            (untouched) value and as a relative pressure normalized after the
            observed pressure range (min-max values). All units are multiplied
            by 1000 to avoid rounding errors and increase the accuracy of the
            final location estimate.
        */
        for(uint32_t channelIndex = 0; channelIndex < channelsInUse; channelIndex++)
        {
            uint32_t lesenseChannel = channelMap[channelIndex];
            uint32_t value = lesense::getValue(lesenseChannel);
            uint32_t maxValue = lesense::getMaxValue(lesenseChannel);
            uint32_t minValue = lesense::getMinValue(lesenseChannel);

            printf("%d: %d %d %d\n", (int) channelIndex, (int) minValue, (int) value, (int) maxValue);
            uint32_t temp;

            /*  Do an integrity test to ensure the pressure doesn't become negative.
            */
            temp = value * 1000 / (maxValue + 1);
            pressure[channelIndex] = (temp < 1000) ? 1000 - temp : 0;

            temp = (value - minValue) * 1000 / (maxValue - minValue + 1);
            pressureNorm[channelIndex] = (temp < 1000) ? 1000 - temp : 0;

            /*  Calculate the total sum for the weighted average. */
            total += pressure[channelIndex];
            totalNorm += pressureNorm[channelIndex];
        }

        printf("\n");

        printf("%3d %3d %3d %3d\n", (int) pressure[3], (int) pressure[2], (int) pressure[1], (int) pressure[0]);
        printf("%3d %3d %3d %3d (%3d)\n", (int) pressureNorm[3], (int) pressureNorm[2], (int) pressureNorm[1], (int) pressureNorm[0], (int) (100 - totalNorm));

        uint32_t newLocation = 0;
        uint32_t locationNorm = 0;
        uint32_t maxChannel = 0;

        /*  Calculate the weighted average using the channelIndex as the location.
            The calculation makes room for a fictitious channel at location 1000
            for handling edge cases.
        */
        for (uint32_t channelIndex = 0; channelIndex < channelsInUse; channelIndex++)
        {
            pressure[channelIndex] = pressure[channelIndex] * 1000 / (total + 1);

            newLocation += ((channelIndex+2) * pressure[channelIndex]);
            locationNorm += ((channelIndex + 2) * pressureNorm[channelIndex]);

            /*  Find the channel with the highest pressure (which is where the
                finger is).
            */
            if (pressure[maxChannel] < pressure[channelIndex])
            {
                maxChannel = channelIndex;
            }
        }

        /*  When the finger is sliding off the edges of the touch area we can
            still estimate the position by looking at the normalized pressure
            and normalized location. We detect this cornercase by
            1) the finger (maxChannel) is on the edge and
            2) the total normalized pressure is less than one fully pressed
               channel
            The location is calculated by adding a fictitious extra channel on
            the edge with the pressure: (1000 - totalnorm).
        */
        if ((maxChannel == 0) && (totalNorm < 1000))
        {
            locationNorm += (1000 - totalNorm);
        }
        else if ((maxChannel == (channelsInUse - 1)) && (totalNorm < 1000))
        {
            locationNorm += (channelsInUse + 2) * (1000 - totalNorm);
        }
        else
        {
            locationNorm = 0;
        }

        if (locationNorm > 0)
        {
            newLocation = locationNorm;
        }

        /* Shift location caused by the fictitious channel. */
        newLocation -= 1000;

        /* If the location has changed, calculate speed and acceleration. */
        bool locationChanged = false;
        if (newLocation != location)
        {
            int32_t newSpeed = (location != 0) ? newLocation - location : 0;

            acceleration = newSpeed - speed;
            speed = newSpeed;
            locationChanged = true;
        }
        location = newLocation;

        /* call callback functions if the location or state has changed. */
        if (pressed == false)
        {
            if (callOnPress)
            {
                callOnPress.call();
            }
        }
        else if (locationChanged)
        {
            if (callOnChange)
            {
                callOnChange.call();
            }
        }

        pressed = true;
        eventTimestamp = newEventTimestamp;
    }
}

/*  This block is executed when one of the analog channels crosses the
    deadzone threshold and returns to its original state.
*/
void AnalogSlider::sliderReleaseTask()
{
    /*  Get the timestamp for this event.
    */
    uint32_t newEventTimestamp = lesense::getEventTimestamp();

    if (newEventTimestamp != eventTimestamp)
    {
        /*  Query all channels. The slider has been released when none of the
            channels are pressed.
        */
        bool noChannelsPressed = true;

        for(uint32_t channelIndex = 0; channelIndex < channelsInUse; channelIndex++)
        {
            uint32_t lesenseChannel = channelMap[channelIndex];

            printf("%d %d\n", (int) channelIndex, (int) lesense::channelIsActive(lesenseChannel));

            noChannelsPressed &= !(lesense::channelIsActive(lesenseChannel));
        }

        if (noChannelsPressed)
        {
            pressed = false;

            if (callOnRelease)
            {
                callOnRelease.call();
            }

            /*  Reset the location when the slider is released to avoid
                discontinuity the next time the slider is pressed.
            */
            location = 0;
        }

        eventTimestamp = newEventTimestamp;
    }
}

/*  Set slider settings.
*/
void AnalogSlider::setIdleFrequency(uint32_t freqHz)
{
    lesense::setIdleFrequency(freqHz);
}

void AnalogSlider::setActiveFrequency(uint32_t freqHz)
{
    lesense::setActiveFrequency(freqHz);
}

void AnalogSlider::calibrate(bool calibrateWhenActive, bool useNewValues)
{
    lesense::calibrate(calibrateWhenActive, useNewValues, this, &AnalogSlider::calibrateDoneTask);
}

void AnalogSlider::pause()
{
    lesense::pause();
}

void AnalogSlider::resume()
{
    lesense::resume();
}
