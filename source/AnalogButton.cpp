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


#include "lesense/AnalogButton.h"
#include "lesense/lesense_api.h"


AnalogButton::AnalogButton(uint32_t _channel)
    :   channel(_channel)
{
    if (_channel < LESENSE_CHANNEL_TOTAL)
    {
        FunctionPointer _onPress(this, &AnalogButton::onPressISR);
        FunctionPointer _onRelease(this, &AnalogButton::onReleaseISR);

        /*  Add channel
        */
        lesense::params_t params;

        params.channel = _channel;
        params.onPress = _onPress;
        params.onRelease = _onRelease;
        params.sensitivity = LESENSE_SENSITIVITY_PERCENT;
        params.updates = false;

        lesense::addChannel(params);
    }
}

AnalogButton::~AnalogButton()
{
    FunctionPointer onPress(this, &AnalogButton::onPressISR);
    FunctionPointer onRelease(this, &AnalogButton::onReleaseISR);

    lesense::removeChannel(channel, onPress, onRelease);
}

void AnalogButton::fall(FunctionPointer& _onPress)
{
    onPressHandler = _onPress;
}

void AnalogButton::rise(FunctionPointer& _onRelease)
{
    onReleaseHandler = _onRelease;
}

void AnalogButton::onPressISR()
{
    if (onPressHandler)
    {
        onPressHandler.call();
    }
}

void AnalogButton::onReleaseISR()
{
    if (onReleaseHandler)
    {
        onReleaseHandler.call();
    }
}


#if 0
AnalogButton::AnalogButton(uint32_t _channel)
    :   channel(_channel),
        onPressHandler(),
        onReleaseHandler(),
        sensitivity(90),
        multipleUpdates(false)
{

}


- (id)initWithChannel:(uint32_t)newChannel
          callOnPress:(yt_callback_t)newCallOnPress
        callOnRelease:(yt_callback_t)newCallOnRelease
          sensitivity:(uint8_t)newSensitivity
      multipleUpdates:(bool)newMultipleUpdates
{
  self = [super init];
  YTError result = ytError(YTSuccess);

  if (self)
  {
    channel = newChannel;
    sensitivity = newSensitivity;
    multipleUpdates = newMultipleUpdates;

    result = [LeSense addChannel:newChannel
                 withCallOnPress:newCallOnPress
                   callOnRelease:newCallOnRelease
                     sensitivity:newSensitivity
                      andUpdates:newMultipleUpdates];

    onPressCache = [[NSMutableArray alloc] init];
    onReleaseCache = [[NSMutableArray alloc] init];

    if (newCallOnPress)
    {
      [onPressCache addObject:newCallOnPress];
    }

    if (newCallOnRelease)
    {
      [onReleaseCache addObject:newCallOnRelease];
    }
  }

  if (ytErrorIsSuccess(result))
  {
    return self;
  }
  else
  {
    return nil;
  }
}

#endif

