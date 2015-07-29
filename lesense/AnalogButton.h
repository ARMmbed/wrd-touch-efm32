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

#ifndef __ANALOGBUTTON_H__
#define __ANALOGBUTTON_H__


#include "mbed.h"


class AnalogButton
{
public:
    AnalogButton(uint32_t channel);
    ~AnalogButton();

    void fall(FunctionPointer& onPress);
    void rise(FunctionPointer& onRelease);

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

private:
    void onPressISR();
    void onReleaseISR();

    uint32_t channel;
    FunctionPointer onPressHandler;
    FunctionPointer onReleaseHandler;
};

#endif // __ANALOGBUTTON_H__
