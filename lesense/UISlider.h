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

#ifndef __UISLIDER_H__
#define __UISLIDER_H__

#include "mbed.h"

#include "lesense/lesense_api.h"

class UISlider
{
public:

    template <std::size_t C>
    UISlider(LESENSEName (&_channels)[C], uint32_t sensitivity)
        :   channelsInUse(C),
            lastLocation(0),
            lastSpeed(0),
            lastAcceleration(0),
            sliderIsPressed(false),
            lastEvent(0)
    {
        FunctionPointer onPress(this, &UISlider::sliderPressTask);
        FunctionPointer onRelease(this, &UISlider::sliderReleaseTask);

        lesense::params_t params;

        params.onPress = onPress;
        params.onRelease = onRelease;
        params.sensitivity = sensitivity;
        params.updates = true;

        for (std::size_t idx = 0; idx < C; idx++)
        {
            channels[idx] = _channels[idx];
            params.channel = _channels[idx];

            lesense::addChannel(params);
        }
    }

    uint32_t getLocation();

    int32_t getSpeed();
    int32_t getAcceleration();

    bool isPressed();

    void setIdleFrequency(uint32_t freqHz);
    void setActiveFrequency(uint32_t freqHz);

    void setCallOnPress(FunctionPointer& callOnPress);
    void setCallOnChange(FunctionPointer& callOnChange);
    void setCallOnRelease(FunctionPointer& callOnRelease);


    void calibrate(bool calibrateWhenActive, bool useNewValues, void (*callback)(void));

    void pause();
    void resume();

private:

    void sliderPressTask();
    void sliderReleaseTask();

    LESENSEName channels[16];

    uint32_t channelsInUse;

    uint32_t lastLocation;
    int32_t lastSpeed;
    int32_t lastAcceleration;
    bool sliderIsPressed;

    uint32_t lastEvent;

    FunctionPointer callOnPress;
    FunctionPointer callOnChange;
    FunctionPointer callOnRelease;
};


#if 0
@interface YTSlider : NSObject
{
  @protected
  NSArray* buttons;

  @private
}

- (id) initWithButtons:(NSArray*)buttons
           callOnPress:(yt_callback_t)onPress
          callOnChange:(yt_callback_t)onChange
         callOnRelease:(yt_callback_t)onRelease;

- (id) initWithChannels:(uint32_t[])channels
       numberOfChannels:(uint32_t)numberOfChannels
            callOnPress:(yt_callback_t)onPress
           callOnChange:(yt_callback_t)onChange
          callOnRelease:(yt_callback_t)onRelease
            sensitivity:(uint32_t)sensitivity;

@end
#endif

#endif // __UISLIDER_H__

