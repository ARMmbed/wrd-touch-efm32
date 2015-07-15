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

#ifndef __LESENSE_H__
#define __LESENSE_H__

#include "mbed.h"



class LESENSE
{
private:
    LESENSE();

};


#if 0

typedef void (^calibrate_callback_t)(YTError error);

@interface LeSense : YTStatic

+ (YTError)addChannel:(uint32_t)channel
      withCallOnPress:(yt_callback_t)callOnPress
        callOnRelease:(yt_callback_t)callOnRelease
          sensitivity:(uint32_t)threshold_percent
           andUpdates:(BOOL)multipleUpdates;

+ (void)removeChannel:(uint32_t)channel
          callOnPress:(yt_callback_t)callOnPress
     andCallOnRelease:(yt_callback_t)callOnRelease;

+ (void)calibrateWithForce:(BOOL)forceCalibration
                 andUpdate:(BOOL)useNewValues
                  thenCall:(calibrate_callback_t)callback;

+ (void)pause;
+ (void)resume;

+ (void)setFrequencyWhenIdle:(uint32_t)freqHz;
+ (void)setFrequencyWhenActive:(uint32_t)freqHz;

+ (BOOL)channelIsActive:(uint32_t)channel;

+ (uint16_t)getValueForChannel:(uint32_t)channel;
+ (uint16_t)getMaxValueForChannel:(uint32_t)channel;
+ (uint16_t)getMinValueForChannel:(uint32_t)channel;
+ (void)setMaxValue:(uint16_t)maxValue forChannel:(uint32_t)channel;
+ (void)setMinValue:(uint16_t)minValue forChannel:(uint32_t)channel;
+ (void)setThreshold:(uint16_t)threshold forChannel:(uint32_t)channel;

+ (uint16_t)getCalibrationValueForChannel:(uint32_t)channel;

+ (yt_tick_t)getEventTimestamp;


@end
#endif // __LESENSE_H__

