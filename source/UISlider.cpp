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

#include "lesense/UISlider.h"

#if 0
#include "swo/swo.h"
#define printf(...) { swoprintf(__VA_ARGS__); }
#else
#define printf(...)
#endif


UISlider::~UISlider()
{
    FunctionPointer onPress(this, &UISlider::sliderPressTask);
    FunctionPointer onRelease(this, &UISlider::sliderReleaseTask);

    for (std::size_t idx = 0; idx < channelsInUse; idx++)
    {
        lesense::removeChannel(channels[idx], onPress, onRelease);
    }
}

/*  This function is executed when one of the analog channels crosses the
    deadzone threshold.
*/
void UISlider::sliderPressTask()
{
    /*  Get the timestamp for this event.
    */
    uint32_t thisEvent = lesense::getEventTimestamp();

    if (thisEvent != lastEvent)
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
            uint32_t lesenseChannel = channels[channelIndex];
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

        uint32_t location = 0;
        uint32_t locationNorm = 0;
        uint32_t maxChannel = 0;

        /*  Calculate the weighted average using the channelIndex as the location.
            The calculation makes room for a fictitious channel at location 1000
            for handling edge cases.
        */
        for (uint32_t channelIndex = 0; channelIndex < channelsInUse; channelIndex++)
        {
            pressure[channelIndex] = pressure[channelIndex] * 1000 / (total + 1);

            location += ((channelIndex+2) * pressure[channelIndex]);
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
            location = locationNorm;
        }

        /* Shift location caused by the fictitious channel. */
        location -= 1000;

        /* If the location has changed, calculate speed and acceleration. */
        bool locationChanged = false;
        if (location != lastLocation)
        {
            int32_t speed = (lastLocation != 0) ? location - lastLocation : 0;

            lastAcceleration = speed - lastSpeed;
            lastSpeed = speed;
            locationChanged = true;
        }
        lastLocation = location;

        /* call callback functions if the location or state has changed. */
        if (sliderIsPressed == false)
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

        sliderIsPressed = true;
        lastEvent = thisEvent;
    }
}

/*  This block is executed when one of the analog channels crosses the
    deadzone threshold and returns to its original state.
*/
void UISlider::sliderReleaseTask()
{
    /*  Get the timestamp for this event.
    */
    uint32_t thisEvent = lesense::getEventTimestamp();

    if (thisEvent != lastEvent)
    {
        /*  Query all channels. The slider has been released when none of the
            channels are pressed.
        */
        bool noChannelsPressed = true;

        for(uint32_t channelIndex = 0; channelIndex < channelsInUse; channelIndex++)
        {
            uint32_t lesenseChannel = channels[channelIndex];

            printf("%d %d\n", (int) channelIndex, (int) lesense::channelIsActive(lesenseChannel));

            noChannelsPressed &= !(lesense::channelIsActive(lesenseChannel));
        }

        if (noChannelsPressed)
        {
            sliderIsPressed = false;

            if (callOnRelease)
            {
                callOnRelease.call();
            }

            /*  Reset the location when the slider is released to avoid
                discontinuity the next time the slider is pressed.
            */
            lastLocation = 0;
        }

        lastEvent = thisEvent;
    }
}


/*  Set callback functions.
*/
void UISlider::setCallOnPress(void (*_callOnPress)(void))
{
    callOnPress.attach(_callOnPress);
}

void UISlider::setCallOnChange(void (*_callOnChange)(void))
{
    callOnChange.attach(_callOnChange);
}

void UISlider::setCallOnRelease(void (*_callOnRelease)(void))
{
    callOnRelease.attach(_callOnRelease);
}

void UISlider::setCallOnPress(FunctionPointer& _callOnPress)
{
    callOnPress = _callOnPress;
}

void UISlider::setCallOnChange(FunctionPointer& _callOnChange)
{
    callOnChange = _callOnChange;
}

void UISlider::setCallOnRelease(FunctionPointer& _callOnRelease)
{
    callOnRelease = _callOnRelease;
}


/*  Get slider status.
*/
bool UISlider::isPressed()
{
    return sliderIsPressed;
}

uint32_t UISlider::getLocation()
{
    return lastLocation;
}

int32_t UISlider::getSpeed()
{
    return lastSpeed;
}

int32_t UISlider::getAcceleration()
{
    return lastAcceleration;
}


/*  Set slider settings.
*/
void UISlider::setIdleFrequency(uint32_t freqHz)
{
    lesense::setIdleFrequency(freqHz);
}

void UISlider::setActiveFrequency(uint32_t freqHz)
{
    lesense::setActiveFrequency(freqHz);
}

void UISlider::calibrate(bool calibrateWhenActive, bool useNewValues, void (*callback)(void))
{
    lesense::calibrate(calibrateWhenActive, useNewValues, callback);
}

void UISlider::pause()
{
    lesense::pause();
}

void UISlider::resume()
{
    lesense::resume();
}
