





#if 0

extern "C" {
#import "lesense/LeSense.h"
#import "lesense_config.h"

#import "emlib/em_int.h"
#import "emlib/em_lesense.h"
#import "emlib/em_cmu.h"
#import "emlib/em_gpio.h"
#import "emlib/em_acmp.h"

#import "yottos/yottos.h"
#import "yottos_platform/yottos_platform.h"
#import "Foundation/NSAssert.h"
}


#import <stdlib.h>

#if 0
#import "stdio.h"
#else
#define printf(...) ;
#endif




/* Objective-C static class */

@implementation LeSense

+ (YTError)addChannel:(uint32_t)lesenseChannel
   withCallOnPress:(yt_callback_t)callOnPress
     callOnRelease:(yt_callback_t)callOnRelease
       sensitivity:(uint32_t)sensitivityPercent
        andUpdates:(BOOL)updates
{
  /*  Static class has no constructor. Run lesenseInit once to initialize
      class variables.
  */
  if (runOnce)
  {
    runOnce = false;

    lesenseInit();
  }

  YTError result = ytError(YTSuccess);

  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  /*  Sanity check. activeChannel == NUM_LESENSE_CHANNELS means the
      lesenseChannel has not been configured for use and it can only be
      configured if there are available active channel slots.
  */
  if ((lesenseChannel >= NUM_LESENSE_CHANNELS) ||
      (numChannelsUsed >= NUM_ACTIVE_CHANNELS && activeChannel == NUM_LESENSE_CHANNELS))
  {
    result = ytError(YTBadRequest);
  }
  else
  {
    /*  The same channel can be added multiple times. This is useful if multiple
        modules want to be notified about user interaction.

        Callbacks are stored as linked lists. See the gpio-efm32 module for details.
    */
    if (activeChannel == NUM_LESENSE_CHANNELS)
    {
      /*  Keep track of channels in use, users per channel, and maintain mapping
          between active channelse and lesense channelse. The usage mask is for
          setting and reading interrupt registers.
      */
      activeChannel = numChannelsUsed;
      numChannelsUsed++;
      lesenseToActiveMap[lesenseChannel] = activeChannel;
      currentUsers[activeChannel]++;
      channelsUsedMask |= (1 << lesenseChannel);

      /* Store channel sensitivity for later calibration. */
      channelSensitivityPercent[activeChannel] = sensitivityPercent;

      /* Set GPIO pin mode to disabled for all active pins */
      GPIO_PinModeSet(LESENSE_CH_PORT, lesenseChannel, gpioModeDisabled, 0);

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
        .sampleDelay   = SAMPLE_DELAY,
        .measDelay     = 0x0,
        .acmpThres     = 0x0,                   // don't care, configured by ACMPInit
        .sampleMode    = lesenseSampleModeCounter,
        .intMode       = lesenseSetIntLevel,
        .cntThres      = 0x0,                   // Configured later by calibration function
        .compMode      = lesenseCompModeLess
      };

      /* Configure channel */
      LESENSE_ChannelConfig(&initLesenseCh, lesenseChannel);

      /*  Post initial calibration task. The calibration is postponed if the number
          of channels in use has been changed since the task was posted.
      */
      uint32_t currentCount = numChannelsUsed;
      yt_callback_t initialCalibrationTask = ^{
        if (currentCount == numChannelsUsed)
        {
          [self calibrateWithForce:NO andUpdate:YES thenCall:nil];
        } else
        {
          printf("abort calibration\n");
        }
      };

      ytPost(initialCalibrationTask, ytMilliseconds(0));
    }

    /* registed call back functions. */
    if (callOnPress)
    {
      addCallback(callOnPress, updates, &onPress[activeChannel]);
    }

    if (callOnRelease)
    {
      addCallback(callOnRelease, updates, &onRelease[activeChannel]);
    }
  }

  return result;
}


+ (void)removeChannel:(uint32_t)lesenseChannel
          callOnPress:(yt_callback_t)callOnPress
     andCallOnRelease:(yt_callback_t)callOnRelease
{
  /* Sanity check. */
  if (lesenseChannel < NUM_LESENSE_CHANNELS)
  {
    uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

    /*  If lesenseChannel has been configured, remove callbacks (if any)
        and decrement number of users. If no users are left on this lesense
        channel, clean up data structures. If no users are left on any
        channels, pause the lesense module.
    */
    if (activeChannel != NUM_LESENSE_CHANNELS)
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

        lesenseToActiveMap[lesenseChannel] = NUM_LESENSE_CHANNELS;
      }

      /* check if lesense can be turned off. */
      uint32_t totalUsers = 0;
      for (uint16_t allChannel = 0;
            allChannel < NUM_ACTIVE_CHANNELS;
              allChannel++)
      {
        totalUsers += currentUsers[allChannel];
      }

      /* pause module to save power. */
      if (totalUsers == 0)
      {
        [self pause];
      }
    }
  }
}


/*  Initiate calibration. The force flag will interrupt the current
    activity otherwise calibration will only commence when idle.
    The call back is called when calibration is complete.
*/
+ (void)calibrateWithForce:(BOOL)forceCalibration
                 andUpdate:(BOOL)useNewValues
                  thenCall:(calibrate_callback_t)callback
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

    if (callback)
    {
      calibrateDoneCallback = callback;
    }

    printf("calibration\n");
  }
  else if ((lesenseState == STATE_CALIBRATION) ||
           (lesenseState == STATE_CALIBRATION_PAUSE))
  {
    printf("calibration in progress\n");
    if (callback)
    {
      ytPost(^{callback(ytError(YTUnavailable));}, ytMilliseconds(0));
    }
  }
  else
  {
    if (callback)
    {
      ytPost(^{callback(ytError(YTUnavailable));}, ytMilliseconds(0));
    }
  }
}


+ (void)pause
{
  printf("before: %d\n", lesenseState);

  yt_callback_t delayedPauseTask = ^{

    printf("after: %d\n", lesenseState);

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

    printf("after-after: %d\n", lesenseState);
  };

  ytPost(delayedPauseTask, ytMilliseconds(0));
}

+ (void)resume
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

/*  Update the sampling frequency used when the module is in the idle state,
    i.e., none of the channels are above the specified threshold.
*/
+ (void)setFrequencyWhenIdle:(uint32_t)freqHz
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
+ (void)setFrequencyWhenActive:(uint32_t)freqHz
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

+ (BOOL)channelIsActive:(uint32_t)lesenseChannel
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return alreadyPressed[activeChannel];
}

+ (uint16_t)getValueForChannel:(uint32_t)lesenseChannel
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return transferBuffer[TRANSFER_BUFFER_BANK ^ 0x01][activeChannel];
}

+ (uint16_t)getCalibrationValueForChannel:(uint32_t)lesenseChannel
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return channelCalValue[activeChannel];
}

+ (uint16_t)getMaxValueForChannel:(uint32_t)lesenseChannel
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return channelMaxValue[activeChannel];
}

+ (uint16_t)getMinValueForChannel:(uint32_t)lesenseChannel
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return channelMinValue[activeChannel];
}

+ (void)setMaxValue:(uint16_t)maxValue forChannel:(uint32_t)lesenseChannel
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  channelMaxValue[activeChannel] = maxValue;
}

+ (void)setMinValue:(uint16_t)minValue forChannel:(uint32_t)lesenseChannel
{
  uint16_t activeChannel = lesenseToActiveMap[lesenseChannel];

  channelMinValue[activeChannel] = minValue;
}

+ (void)setThreshold:(uint16_t)threshold forChannel:(uint32_t)lesenseChannel
{
  LESENSE_ChannelThresSet(lesenseChannel, 0x0, threshold);
}

+ (yt_tick_t)getEventTimestamp
{
  return lastEvent[TRANSFER_BUFFER_BANK ^ 0x01];
}

@end


#if 0
/****************************************************************************
 * Returns maximum value in input array of size N
*****************************************************************************/
static uint16_t GetMaxValue(uint16_t* A, uint16_t N)
{
  int32_t i;
  uint16_t max = 0;

  for(i=0; i<N; i++)
  {
    if(max < A[i]){
      max = A[i];
    }
  }
  return max;
}


/****************************************************************************
 * Returns minimum value in input array of size N
*****************************************************************************/
static uint16_t GetMinValue(uint16_t* A, uint16_t N)
{
  int32_t i;
  uint16_t min = 0xffff;

  for(i=0; i<N; i++)
  {
    if(A[i] < min){
      min = A[i];
    }
  }
  return min;
}

#endif

#endif
