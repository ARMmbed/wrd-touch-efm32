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


#include "wrd-touch-efm32/lesense_api.h"

#include "mbed-drivers/mbed.h"

#include "em_lesense.h"
#include "em_acmp.h"
#include "em_gpio.h"


#if 0
#include "swo/swo.h"
#define printf(...) { swoprintf(__VA_ARGS__); }
#else
#define printf(...)
#endif

namespace lesense {

uint16_t lesenseToActiveMap[LESENSE_CHANNEL_TOTAL];

/*  Setup and State Group
*/
typedef enum {
    STATE_OFF,
    STATE_PAUSE,
    STATE_IDLE,
    STATE_CALIBRATION,
    STATE_CALIBRATION_PAUSE,
    STATE_ACTIVE,
    STATE_ACTIVE_PAUSE
} state_t;

static state_t lesenseState = STATE_OFF;

static void init();

static uint8_t  channelSensitivityPercent[LESENSE_CHANNELS_IN_USE]  = {0};
static bool     alreadyPressed[LESENSE_CHANNELS_IN_USE]             = {0};

static uint32_t scanFrequencyIdle   = LESENSE_SCAN_FREQUENCY_LOW;
static uint32_t scanFrequencyActive = LESENSE_SCAN_FREQUENCY_HIGH;
/* End Setup and State Group */

/*  Linked List Group
    Functions and datastructures for maintaining a linked list of callbacks
    for each channel.
*/
static struct   CallbackNode* onPress[LESENSE_CHANNELS_IN_USE]      = {0};
static struct   CallbackNode* onRelease[LESENSE_CHANNELS_IN_USE]    = {0};
static uint32_t currentUsers[LESENSE_CHANNELS_IN_USE]               = {0};

static bool addCallback(FunctionPointer& newCallback, bool updates, struct CallbackNode** nodes);
static bool removeCallback(FunctionPointer& oldCallback, struct CallbackNode** node);
static void cleanupCallback(struct CallbackNode** node);


/*  End Linked List Group */

/*  Calibration Group
    Functions and datastructures for handling calibration.
*/
static bool     useCalibrationValues = true;
static uint8_t  calibrationValueIndex = 0;
static uint16_t calibrationValue[LESENSE_CHANNELS_IN_USE][LESENSE_CALIBRATION_VALUES];
static uint16_t channelMaxValue[LESENSE_CHANNELS_IN_USE];
static uint16_t channelMinValue[LESENSE_CHANNELS_IN_USE];
static uint16_t channelCalValue[LESENSE_CHANNELS_IN_USE];
static uint16_t GetMedianValue(uint16_t* A, uint16_t N);
FunctionPointer calibrateDoneCallback;
/*  End Calibration Group */

/*  IRQ Group
    Datastructures for transferring data between interrupt handler and tasks.
*/
static uint16_t channelsUsedMask                            = 0;
static uint16_t numChannelsUsed                             = 0;
static uint32_t lastEvent[2]                                = {0};
static uint32_t lastScanres[2]                              = {0};
static uint16_t transferBuffer[2][LESENSE_CHANNELS_IN_USE]  = {0};
static uint8_t  TRANSFER_BUFFER_BANK                        = 0;

static bool calibrationTaskNotPosted    = true;
static bool scanCompleteTaskNotPosted   = true;
static bool buttonWakeupTaskNotPosted   = true;
/*  End IRQ Group */



/*  Private function for initializing the lesense module. */
static void init()
{
    if (lesenseState == STATE_OFF)
    {
        printf("lesense: init\r\n");

        for(uint16_t activeChannel = 0;
            activeChannel < LESENSE_CHANNELS_IN_USE;
            activeChannel++)
        {
            /* Init min and max values for each active channel */
            channelMaxValue[activeChannel] = 0;
            channelCalValue[activeChannel] = 0;
            channelMinValue[activeChannel] = 0xffff;
            channelSensitivityPercent[activeChannel] = 100;
        }

        /*  The lesenseChannel is always less than LESENSE_CHANNEL_TOTAL.
            Use LESENSE_CHANNEL_TOTAL to indicate channel is not in use.
        */
        for (uint32_t lesenseChannel = 0;
             lesenseChannel < LESENSE_CHANNEL_TOTAL;
             lesenseChannel++)
        {
            lesenseToActiveMap[lesenseChannel] = LESENSE_CHANNEL_TOTAL;
        }

        ///////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        /* Setup CMU. */
        /* ACMP */
        CMU_ClockEnable(cmuClock_ACMP0, true);
        CMU_ClockEnable(cmuClock_ACMP1, true);

        /* GPIO */
        CMU_ClockEnable(cmuClock_GPIO, true);

        /* Low energy peripherals
        *   LESENSE
        *   LFXO clock must be enabled prior to enabling
        *   clock for the low energy peripherals */
        CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
        CMU_ClockEnable(cmuClock_CORELE, true);
        CMU_ClockEnable(cmuClock_LESENSE, true);

        /* Disable clock source for LFB clock */
        CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);

        ///////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        /* Setup ACMP. */
        /* Configuration structure for ACMP */
        /* See application note document for description of the different settings. */
        static const ACMP_CapsenseInit_TypeDef acmpInit =
        {
            .fullBias                 = true,            //Configured according to application note
            .halfBias                 = true,            //Configured according to application note
            .biasProg                 = 0x5,             //Configured according to application note
            .warmTime                 = acmpWarmTime512, //LESENSE uses a fixed warmup time
            .hysteresisLevel          = acmpHysteresisLevel5, //Configured according to application note
            .resistor                 = acmpResistor0,   //Configured according to application note
            .lowPowerReferenceEnabled = false,           //LP-reference can introduce glitches with captouch
            .vddLevel                 = 0x30,            //Configured according to application note
            .enable                   = false            //LESENSE enables the ACMP
        };

        /* Initialize ACMP in capsense mode*/
        ACMP_CapsenseInit(ACMP0, &acmpInit);
        ACMP_CapsenseInit(ACMP1, &acmpInit);

        ///////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        /* Setup LESENSE. */
        /* LESENSE configuration structure */
        static const LESENSE_Init_TypeDef initLesense =
        {
            .coreCtrl         =
            {
                .scanStart    = lesenseScanStartPeriodic,
                .prsSel       = lesensePRSCh0,
                .scanConfSel  = lesenseScanConfDirMap,
                .invACMP0     = false,
                .invACMP1     = false,
                .dualSample   = false,
                .storeScanRes = false,
                .bufOverWr    = true,
                .bufTrigLevel = lesenseBufTrigHalf,
                .wakeupOnDMA  = lesenseDMAWakeUpDisable,
                .biasMode     = lesenseBiasModeDutyCycle,
                .debugRun     = false
            },

            .timeCtrl         =
            {
                .startDelay   = 0x0
            },

            .perCtrl            =
            {
                .dacCh0Data     = lesenseDACIfData,
                .dacCh0ConvMode = lesenseDACConvModeDisable,
                .dacCh0OutMode  = lesenseDACOutModeDisable,
                .dacCh1Data     = lesenseDACIfData,
                .dacCh1ConvMode = lesenseDACConvModeDisable,
                .dacCh1OutMode  = lesenseDACOutModeDisable,
                .dacPresc       = 0,
                .dacRef         = lesenseDACRefBandGap,
                .acmp0Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
                .acmp1Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
                .warmupMode     = lesenseWarmupModeNormal
            },

            .decCtrl       =
            {
                .decInput  = lesenseDecInputSensorSt,
                .initState = 0,
                .chkState  = false,
                .intMap    = false,
                .hystPRS0  = false,
                .hystPRS1  = false,
                .hystPRS2  = false,
                .hystIRQ   = false,
                .prsCount  = false,
                .prsChSel0 = lesensePRSCh0,
                .prsChSel1 = lesensePRSCh1,
                .prsChSel2 = lesensePRSCh2,
                .prsChSel3 = lesensePRSCh3
            }
        };

        /* Initialize LESENSE interface _with_ RESET. */
        LESENSE_Init(&initLesense, true);

        /* Set clock divisor for LF clock. */
        LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);

        lesenseState = STATE_PAUSE;
    }
}





/*  Add channel
*/
void addChannel(params_t& params)
{
    //  Make sure module has been initialized
    if (lesenseState == STATE_OFF)
    {
        init(); // init changes state to STATE_IDLE
    }

    uint16_t activeChannel = lesenseToActiveMap[params.channel];

    /*  Sanity check.
    */
    MBED_ASSERT(params.channel < LESENSE_CHANNEL_TOTAL);

    /*  Configure channel first time it is added.
    */
    if (activeChannel == LESENSE_CHANNEL_TOTAL)
    {
        /*  Keep track of channels in use, users per channel, and maintain mapping
        between active channelse and lesense channelse. The usage mask is for
        setting and reading interrupt registers.
        */
        activeChannel = numChannelsUsed;
        numChannelsUsed++;
        lesenseToActiveMap[params.channel] = activeChannel;
        channelsUsedMask |= (1 << params.channel);

        /* Store channel sensitivity for later calibration. */
        channelSensitivityPercent[activeChannel] = params.sensitivity;

        /* Set GPIO pin mode to disabled for all active pins */
        GPIO_PinModeSet(LESENSE_CH_PORT, params.channel, gpioModeDisabled, 0);

        /* Channel configuration */
        static const LESENSE_ChDesc_TypeDef initLesenseCh =
        {
            .enaScanCh     = true,
            .enaPin        = true,
            .enaInt        = true,
            .chPinExMode   = lesenseChPinExDis,
            .chPinIdleMode = lesenseChPinIdleDis,
            .useAltEx      = false,
            .shiftRes      = false,
            .invRes        = false,
            .storeCntRes   = true,
            .exClk         = lesenseClkLF,
            .sampleClk     = lesenseClkLF,
            .exTime        = 0x0,
            .sampleDelay   = LESENSE_SAMPLE_DELAY,
            .measDelay     = 0x0,
            .acmpThres     = 0x0,                   // don't care, configured by ACMPInit
            .sampleMode    = lesenseSampleModeCounter,
            .intMode       = lesenseSetIntLevel,
            .cntThres      = 0x0,                   // Configured later by calibration function
            .compMode      = lesenseCompModeLess
        };

        /* Configure channel */
        LESENSE_ChannelConfig(&initLesenseCh, params.channel);

        /* start calibration */
        calibrate(true, true, NULL);
    }


    /*  The same channel can be added multiple times. This is useful if multiple
        modules want to be notified about user interaction.

        Callbacks are stored as linked lists. See the gpio-efm32 module for details.
    */
    currentUsers[activeChannel]++;

    /* registed call back functions. */
    if (params.onPress)
    {
        addCallback(params.onPress, params.updates, &onPress[activeChannel]);
    }

    if (params.onRelease)
    {
        addCallback(params.onRelease, params.updates, &onRelease[activeChannel]);
    }
}



void removeChannel(uint32_t lesenseChannel, FunctionPointer& callOnPress, FunctionPointer& callOnRelease)
{
    /* Sanity check. */
    if (lesenseChannel < LESENSE_CHANNEL_TOTAL)
    {
        uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

        /*  If lesenseChannel has been configured, remove callbacks (if any)
            and decrement number of users. If no users are left on this lesense
            channel, clean up data structures. If no users are left on any
            channels, pause the lesense module.
        */
        printf("remove channel: %d %d\r\n", lesenseChannel, activeChannel);

        if (activeChannel != LESENSE_CHANNEL_TOTAL)
        {
            if (callOnPress)
            {
                removeCallback(callOnPress, &onPress[activeChannel]);
            }
            if (callOnRelease)
            {
                removeCallback(callOnRelease, &onRelease[activeChannel]);
            }

            currentUsers[activeChannel]--;

            if (currentUsers[activeChannel] == 0)
            {
                // clean up
                cleanupCallback(&onPress[activeChannel]);
                cleanupCallback(&onRelease[activeChannel]);

                lesenseToActiveMap[lesenseChannel] = LESENSE_CHANNEL_TOTAL;

                printf("remove: cleanup\r\n");
            }

            /* check if lesense can be turned off. */
            uint32_t totalUsers = 0;
            for (uint16_t allChannel = 0;
                 allChannel < LESENSE_CHANNELS_IN_USE;
                 allChannel++)
            {
                totalUsers += currentUsers[allChannel];
            }

            /* pause module to save power. */
            if (totalUsers == 0)
            {
                printf("remove: pause\r\n");
                pause();
            }
        }
    }
}


/*  Get status.
*/

uint16_t getValue(uint32_t lesenseChannel)
{
    if (lesenseChannel < LESENSE_CHANNEL_TOTAL)
    {
        uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

        return transferBuffer[TRANSFER_BUFFER_BANK ^ 0x01][activeChannel];
    }
    else
    {
        return 0;
    }
}

uint16_t getMinValue(uint32_t lesenseChannel)
{
    if (lesenseChannel < LESENSE_CHANNEL_TOTAL)
    {
        uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

        return channelMinValue[activeChannel];
    }
    else
    {
        return 0;
    }
}

uint16_t getMaxValue(uint32_t lesenseChannel)
{
    if (lesenseChannel < LESENSE_CHANNEL_TOTAL)
    {
        uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

        return channelMaxValue[activeChannel];
    }
    else
    {
        return 0;
    }
}

uint32_t getEventTimestamp()
{
    return lastEvent[TRANSFER_BUFFER_BANK ^ 0x01];
}

bool channelIsActive(uint32_t lesenseChannel)
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return alreadyPressed[activeChannel];
}



/*  Update the sampling frequency used when the module is in the idle state,
    i.e., none of the channels are above the specified threshold.
*/
void setIdleFrequency(uint32_t freqHz)
{
    if (scanFrequencyIdle != freqHz)
    {
        scanFrequencyIdle = freqHz;

        if (lesenseState == STATE_IDLE)
        {
            LESENSE_ScanFreqSet(0, scanFrequencyIdle);
        }
        else if (lesenseState == STATE_ACTIVE)
        {
            LESENSE_ScanFreqSet(0, scanFrequencyActive);
        }
    }
}

/*  Update the sampling frequency used when the module is in the active state,
    i.e., when at least one of the channels is below the threshold.
*/
void setActiveFrequency(uint32_t freqHz)
{
    if (scanFrequencyActive != freqHz)
    {
        scanFrequencyActive = freqHz;

        if (lesenseState == STATE_IDLE)
        {
            LESENSE_ScanFreqSet(0, scanFrequencyIdle);
        }
        else if (lesenseState == STATE_ACTIVE)
        {
            LESENSE_ScanFreqSet(0, scanFrequencyActive);
        }
    }
}

uint16_t getCalibrationValueForChannel(uint32_t lesenseChannel)
{
    uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

    return channelCalValue[activeChannel];
}

void setMaxValue(uint16_t maxValue, uint32_t lesenseChannel)
{
    uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

    channelMaxValue[activeChannel] = maxValue;
}

void setMinValue(uint16_t minValue, uint32_t lesenseChannel)
{
    uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

    channelMinValue[activeChannel] = minValue;
}

void setThreshold(uint16_t threshold, uint32_t lesenseChannel)
{
    LESENSE_ChannelThresSet(lesenseChannel, 0x0, threshold);
}

/*  Initiate calibration. The force flag will interrupt the current
    activity otherwise calibration will only commence when idle.
    The call back is called when calibration is complete.
*/
void calibrate(bool forceCalibration, bool useNewValues, void (*callback)(void))
{
    calibrateDoneCallback.attach(callback);

    internalCalibrate(forceCalibration, useNewValues);
}

void calibrateDoneTask(void)
{
    if (calibrateDoneCallback)
    {
        calibrateDoneCallback.call();
    }
}

void cancelCallback(void)
{
    calibrateDoneCallback.clear();
}

void internalCalibrate(bool forceCalibration, bool useNewValues)
{
    if ((lesenseState == STATE_PAUSE) ||
        (lesenseState == STATE_IDLE) ||
        ((lesenseState == STATE_ACTIVE) && forceCalibration))
    {
        LESENSE_ScanStop();

        /* Wait for current scan to finish */
        while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

        LESENSE_ResultBufferClear();

        LESENSE_IntDisable(channelsUsedMask);
        LESENSE_IntClear(_LESENSE_IFC_MASK);
        LESENSE_IntEnable(LESENSE_IEN_SCANCOMPLETE);
        NVIC_EnableIRQ(LESENSE_IRQn);

        LESENSE_ScanFreqSet(0, scanFrequencyIdle);
        LESENSE_ScanStart();

        if (lesenseState == STATE_PAUSE)
        {
            lesenseState = STATE_CALIBRATION_PAUSE;
        }
        else
        {
            lesenseState = STATE_CALIBRATION;
        }

        useCalibrationValues = useNewValues;

        printf("lesense: calibration\n");
    }
    else if ((lesenseState == STATE_CALIBRATION) ||
             (lesenseState == STATE_CALIBRATION_PAUSE))
    {
        printf("lesense: calibration in progress\n");

        // retake measurements
        calibrationValueIndex  = 0;

        minar::Scheduler::postCallback(calibrateDoneTask)
            .tolerance(0)
            .getHandle();
    }
    else
    {
        minar::Scheduler::postCallback(calibrateDoneTask)
            .tolerance(0)
            .getHandle();
    }
}



void pause()
{
    // WARNING: should this be in a minar task?
    if (lesenseState == STATE_ACTIVE)
    {
        lesenseState = STATE_ACTIVE_PAUSE;
    }
    else if ((lesenseState == STATE_CALIBRATION) ||
             (lesenseState == STATE_CALIBRATION_PAUSE))
    {
        lesenseState = STATE_CALIBRATION_PAUSE;
    }
    else
    {
        lesenseState = STATE_PAUSE;
        LESENSE_ScanStop();
    }
}

void resume()
{
    if (lesenseState == STATE_ACTIVE_PAUSE)
    {
        lesenseState = STATE_ACTIVE;
    }
    else if (lesenseState == STATE_CALIBRATION_PAUSE)
    {
        lesenseState = STATE_CALIBRATION;
    }
    else if (lesenseState == STATE_PAUSE)
    {
        /* restore interrupts to channel/threshold */
        LESENSE_ScanStop();

        /* Wait for current scan to finish */
        while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

        LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
        LESENSE_IntClear(_LESENSE_IFC_MASK);
        LESENSE_IntEnable(channelsUsedMask);
        NVIC_EnableIRQ(LESENSE_IRQn);

        lesenseState = STATE_IDLE;

        LESENSE_ScanFreqSet(0, scanFrequencyIdle);
        LESENSE_ScanStart();
    }
}




/*  addCallback, removeCallback, and cleanupCallback maintain the linkedlists
    with call back functions.
*/
static bool addCallback(FunctionPointer& newCallback, bool updates, struct CallbackNode** node)
{
    /* insert new node at the end of the linked list */
    while(*node)
    {
        node = &((*node)->next);
    }

    *node = (struct CallbackNode*) calloc(1, sizeof (struct CallbackNode));

    if(!*node)
    {
        return false;
    }

    (*node)->multipleUpdates = updates;
    (*node)->callback = newCallback;

    printf("add: %p %p\r\n", *node, newCallback.get_function());

    return true;
}

static bool removeCallback(FunctionPointer& oldCallback, struct CallbackNode** node)
{
    while(*node)
    {
        if ((*node)->callback == oldCallback)
        {
            printf("remove: %p %p\r\n", *node, oldCallback.get_function());

            void* trashNode = *node;
            *node = (*node)->next;
            free(trashNode);

            return true;
        }

        node = &((*node)->next);
    }

    printf("remove failed: %p\r\n", oldCallback.get_function());

    return false;
}

static void cleanupCallback(struct CallbackNode** node)
{
    while(*node)
    {
        void* trashNode = *node;
        *node = (*node)->next;
        free(trashNode);

        node = &((*node)->next);
    }
}



/****************************************************************************
 * Returns median value in input array of size N
*****************************************************************************/

static inline int compare(const void* one, const void* two)
{
    return (*((uint16_t*)one) < *((uint16_t*)two)) ? -1 : 1;
}

static uint16_t GetMedianValue(uint16_t* A, uint16_t N)
{
#warning Using recursive q-sort. Switch to shellsort.
    qsort(A, N, sizeof(uint16_t), compare);

    return A[N/2];
}

/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/

/*  The IRQ flow is designed around the states: IDLE, ACTIVE, and CALIBRATING.
    In IDLE state the lesense module is in charge of sampling, and interrupts
    are only triggered when a channel's value drops below the preset
    threshold. Once that happens, the buttonWakeupTask is posted. In this task
    the sampling rate is increased and the module enters ACTIVE mode.

    Interrupts in ACTIVE mode and CALIBRATING mode causes all channels to be
    sampled and have their values transferred using the transferBuffer. The
    calibrationTask and scanCompleteTask processes the transferBuffer so the
    interrupt handler can return as soon as possible. The minValue for each
    channel is updated as well.

    In the scanCompleteTask, all channels with values lower than the threshold
    will have their onPress callback's called and those that have previously
    been called but have now returned to values above the threshold have their
    onRelease callbacks called.

    In the calibrationTask, all channels are sampled for
    LESENSE_CALIBRATION_VALUES using multiple rounds of interrupts. Once
    completed, the median value for each channel is found and used as the
    maxValue observed. Each channel's threshold is set based on the
    sensitivity and the maxValue. The minValue is reset to maxValue.
*/

static void calibrationTask()
{
    NVIC_DisableIRQ(LESENSE_IRQn);
    TRANSFER_BUFFER_BANK ^= 0x01;
    NVIC_EnableIRQ(LESENSE_IRQn);

    /* Iterate through all possible channels, but only store the active ones. */
    for(uint32_t lesenseChannel = 0;
        lesenseChannel < LESENSE_CHANNEL_TOTAL;
        lesenseChannel++)
    {
        if((channelsUsedMask >> lesenseChannel) & 0x01)
        {
            uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

            calibrationValue[activeChannel][calibrationValueIndex] =
                transferBuffer[TRANSFER_BUFFER_BANK ^ 0x01][activeChannel];
        }
    }

    /* Wrap around calibrationValueIndex */
    calibrationValueIndex++;
    if(calibrationValueIndex >= LESENSE_CALIBRATION_VALUES)
    {
        /* Calculate max/min-value for each channel and set threshold */
        for(uint32_t lesenseChannel = 0;
            lesenseChannel < LESENSE_CHANNEL_TOTAL;
            lesenseChannel++)
        {
            if((channelsUsedMask >> lesenseChannel) & 0x1)
            {
                uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

                /* Store the median. */
                channelCalValue[activeChannel] = GetMedianValue(calibrationValue[activeChannel], LESENSE_CALIBRATION_VALUES);

                /*  The median can either be used automatically
                    or the thresholds can be set manually.
                */
                if (useCalibrationValues)
                {
                    /* reset the min-max values to use the median calibration value. */
                    uint16_t nominalCount = channelCalValue[activeChannel];
                    channelMaxValue[activeChannel] = nominalCount;
                    channelMinValue[activeChannel] = nominalCount;

                    /* use the new max-value to set threshold based on sensitivity. */
                    uint16_t channelThreshold = ((uint32_t)nominalCount *
                                                (uint32_t)channelSensitivityPercent[activeChannel]) / 100;
                    LESENSE_ChannelThresSet(lesenseChannel, 0x0, channelThreshold);

                    printf("lesense: %d %d\n", (int) lesenseChannel, (int) channelThreshold);
                }
            }
        }

        /* calibration done, switch back to normal mode */
        calibrationValueIndex = 0;

        LESENSE_ScanStop();

        /* If pause has been called during calibration, do not start. */
        if (lesenseState == STATE_CALIBRATION_PAUSE)
        {
            lesenseState = STATE_PAUSE;
        }
        else
        {
            lesenseState = STATE_IDLE;

            /* Wait for current scan to finish */
            while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

            /* restore interrupts to channel/threshold */
            LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
            LESENSE_IntClear(_LESENSE_IFC_MASK);
            LESENSE_IntEnable(channelsUsedMask);
            LESENSE_ScanFreqSet(0, scanFrequencyIdle);
            LESENSE_ScanStart();
        }

        printf("lesense: calibration done\n");

        minar::Scheduler::postCallback(calibrateDoneTask)
            .tolerance(0);
    }
    else
    {
        printf("lesense: calibrating\n");
    }

    NVIC_DisableIRQ(LESENSE_IRQn);
    calibrationTaskNotPosted = true;
    NVIC_EnableIRQ(LESENSE_IRQn);
}


static uint32_t oldCount = 0;

static void buttonWakeupTask()
{
    uint32_t newCount = RTC->CNT;
    printf("lesense: %s %d\r\n", __FUNCTION__, (int)(newCount - oldCount));
    oldCount = newCount;

    /* Wait for current scan to finish */
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

    LESENSE_ResultBufferClear();
    LESENSE_IntClear(_LESENSE_IFC_MASK);
    LESENSE_IntEnable(LESENSE_IEN_SCANCOMPLETE);
    LESENSE_ScanFreqSet(0, scanFrequencyActive);

    NVIC_DisableIRQ(LESENSE_IRQn);
    buttonWakeupTaskNotPosted = true;
    NVIC_EnableIRQ(LESENSE_IRQn);
}


static void scanCompleteTask()
{
    // debug timing measurement
    uint32_t newCount = RTC->CNT;
    printf("lesense: %s %d\r\n", __FUNCTION__, (int)(newCount - oldCount));
    oldCount = newCount;

    NVIC_DisableIRQ(LESENSE_IRQn);
    TRANSFER_BUFFER_BANK ^= 0x01;
    NVIC_EnableIRQ(LESENSE_IRQn);

    uint32_t localScanres = 0;

    /*  Read the latest scan result if "pause" has not been called.
        If "pause" has been requested, ignore the latest scan results which will
        emulate all the channels being released and trigger the appropriate
        callbacks.
    */
    if (lesenseState != STATE_ACTIVE_PAUSE)
    {
        localScanres = lastScanres[TRANSFER_BUFFER_BANK ^ 0x01];
    }

    for(uint32_t lesenseChannel = 0;
        lesenseChannel < LESENSE_CHANNEL_TOTAL;
        lesenseChannel++)
    {
        uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

        if (activeChannel != LESENSE_CHANNEL_TOTAL)
        {
            // threshold reached for this channel, i.e., button was pressed
            if (((localScanres & channelsUsedMask) >> lesenseChannel) & 0x01)
            {
                if (alreadyPressed[activeChannel] == false)
                {
                    alreadyPressed[activeChannel] = true;

                    /* go through all single call callbacks (and other callbacks) */
                    struct CallbackNode* node = onPress[activeChannel];
                    while(node)
                    {
                        minar::Scheduler::postCallback(node->callback)
                            .tolerance(0);

                        node = node->next;
                    }
                }
                else
                {
                    /* only go through callbacks that can be called multiple times */
                    struct CallbackNode* node = onPress[activeChannel];
                    while(node)
                    {
                        if (node->multipleUpdates == true)
                        {
                            minar::Scheduler::postCallback(node->callback)
                                .tolerance(0);
                        }

                        node = node->next;
                    }
                }
            }
            else
            {
                if (alreadyPressed[activeChannel] == true)
                {
                    alreadyPressed[activeChannel] = false;

                    struct CallbackNode* node = onRelease[activeChannel];
                    while(node)
                    {
                        minar::Scheduler::postCallback(node->callback)
                            .tolerance(0);

                        node = node->next;
                    }
                }
            } // end is-button-pressed-or-not
        } // end activeChannel != LESENSE_CHANNEL_TOTAL
    } // end for-loop

    /*  If none of the channels are active, switch state from Active
        to Idle and change the sampling frequency.
    */
    if (!localScanres)
    {
        for (uint32_t lesenseChannel = 0;
             lesenseChannel < LESENSE_CHANNEL_TOTAL;
             lesenseChannel++)
        {
            uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

            if (activeChannel != LESENSE_CHANNEL_TOTAL)
            {
                MBED_ASSERT(!alreadyPressed[activeChannel]);
            }
        }

        LESENSE_ScanStop();

        /* If pause was called during the active period, do not start sampling. */
        if (lesenseState == STATE_ACTIVE_PAUSE)
        {
            lesenseState = STATE_PAUSE;
        }
        else
        {
            lesenseState = STATE_IDLE;

            /* Wait for current scan to finish */
            while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);

            /* restore interrupts to channel/threshold. */
            LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
            LESENSE_IntClear(_LESENSE_IFC_MASK);
            LESENSE_IntEnable(channelsUsedMask);
            LESENSE_ScanFreqSet(0, scanFrequencyIdle);
            LESENSE_ScanStart();
        }
    }

    NVIC_DisableIRQ(LESENSE_IRQn);
    scanCompleteTaskNotPosted = true;
    NVIC_EnableIRQ(LESENSE_IRQn);
}

}

using namespace lesense;

void LESENSE_IRQHandler(void)
{
    /* Get pending and enabled interrupt flags */
    uint32_t interrupt_flags = LESENSE_IntGetEnabled();

    if (interrupt_flags & LESENSE_IFS_ALL_CHANNELS)
    {
        LESENSE_IntDisable(channelsUsedMask);

        if (buttonWakeupTaskNotPosted)
        {
            lesenseState = STATE_ACTIVE;

            buttonWakeupTaskNotPosted = false;
            minar::Scheduler::postCallback(buttonWakeupTask)
                .tolerance(0);
        }
    }
    else if (interrupt_flags & LESENSE_IFS_SCANCOMPLETE)
    {
        /* Get position for first channel data in count buffer from lesense write pointer */
        uint32_t bufferIndex = ((LESENSE->PTR & _LESENSE_PTR_WR_MASK) >> _LESENSE_PTR_WR_SHIFT);

        /* Handle circular buffer wraparound */
        if(bufferIndex >= numChannelsUsed)
        {
            bufferIndex = bufferIndex - numChannelsUsed;
        }
        else
        {
            bufferIndex = LESENSE_CHANNEL_TOTAL + bufferIndex - numChannelsUsed;
        }

        /*  Read data from peripheral buffer into transfer buffer for later
            analysis. The minimum value observed for each channel is updated.
            This value can be used for calculating a normalised pressure.
        */
        for (uint32_t lesenseChannel = 0;
             lesenseChannel < LESENSE_CHANNEL_TOTAL;
             lesenseChannel++)
        {
            if((channelsUsedMask >> lesenseChannel) & 0x01)
            {
                uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

                uint16_t value = LESENSE_ScanResultDataBufferGet(bufferIndex++);
                transferBuffer[TRANSFER_BUFFER_BANK][activeChannel] = value;

                if (value < channelMinValue[activeChannel])
                {
                    channelMinValue[activeChannel] = value;
                }
            }
        }

        /*  The LESENSE module stores in it's status register which channels have
            crossed their thresholds. Transfer status register to task-context.
            Also store the timestamp for the event.
        */
        lastScanres[TRANSFER_BUFFER_BANK] = LESENSE->SCANRES;
        lastEvent[TRANSFER_BUFFER_BANK] = minar::platform::getTime();

        /*  Post the appropriate task to handle the sampled data. */
        if ((lesenseState == STATE_CALIBRATION) ||
            (lesenseState == STATE_CALIBRATION_PAUSE))
        {
            if (calibrationTaskNotPosted)
            {
                calibrationTaskNotPosted = false;

                minar::Scheduler::postCallback(calibrationTask)
                    .tolerance(0);
            }
        }
        else
        {
            if (scanCompleteTaskNotPosted)
            {
                scanCompleteTaskNotPosted = false;

                minar::Scheduler::postCallback(scanCompleteTask)
                    .tolerance(0);
            }
        }
    }

    /* Clear interrupt flag */
    LESENSE_IntClear(_LESENSE_IFC_MASK);
}

