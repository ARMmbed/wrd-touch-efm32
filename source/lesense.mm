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
#import "NSAssert/NSAssert.h"
}


#import <stdlib.h>

#if 1
#import "stdio.h"
#else
#define printf(...) ;
#endif

/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS 16

/* LESENSE number of channels actually in use */
#define NUM_ACTIVE_CHANNELS 5

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT gpioPortC

#define LESENSE_IFS_ALL_CHANNELS 0x0000FFFF

static void lesenseInit();


int32_t lesenseToActiveMap[NUM_LESENSE_CHANNELS];

/*  Setup and State Group
*/
typedef enum {
  STATE_PAUSE,
  STATE_IDLE,
  STATE_CALIBRATION,
  STATE_ACTIVE
} state_t;

static state_t lesenseState = STATE_PAUSE;
static bool runOnce = true;

static uint8_t channelSensitivityPercent[NUM_ACTIVE_CHANNELS];
static uint32_t scanFrequencyIdle = LESENSE_SCAN_FREQUENCY_LOW;
static uint32_t scanFrequencyActive = LESENSE_SCAN_FREQUENCY_HIGH;
static bool alreadyPressed[NUM_ACTIVE_CHANNELS] = {0};
/* End Setup and State Group */

/*  Linked List Group 
    Functions and datastructures for maintaining a linked list of callbacks 
    for each channel.
*/
struct CallbackNode
{
    struct CallbackNode* next;
    yt_callback_t callback;
    bool multipleUpdates;
};
static struct CallbackNode* onPress[NUM_ACTIVE_CHANNELS] = {0};
static struct CallbackNode* onRelease[NUM_ACTIVE_CHANNELS] = {0};
static uint32_t currentUsers[NUM_ACTIVE_CHANNELS] = {0};

static bool addCallback(yt_callback_t newCallback, bool updates, struct CallbackNode** nodes);
static bool removeCallback(yt_callback_t oldCallback, struct CallbackNode** node);
static void cleanupCallback(struct CallbackNode** node);
/*  End Linked List Group */

/*  Calibration Group 
    Functions and datastructures for handling calibration.
*/
static calibrate_callback_t calibrateDoneCallback = nil;
static uint8_t calibrationValueIndex = 0;
static uint16_t calibrationValue[NUM_ACTIVE_CHANNELS][NUMBER_OF_CALIBRATION_VALUES];
static uint16_t channelMaxValue[NUM_ACTIVE_CHANNELS];
static uint16_t channelMinValue[NUM_ACTIVE_CHANNELS];
static uint16_t GetMedianValue(uint16_t* A, uint16_t N);
/*  End Calibration Group */

/*  IRQ Group
    Datastructures for transferring data between interrupt handler and tasks.
*/
static uint16_t channelsUsedMask = 0;
static uint8_t numChannelsUsed = 0;
static yt_tick_t lastEvent[2] = {0};
static uint32_t lastScanres[2] = {0};
static uint16_t transferBuffer[2][NUM_ACTIVE_CHANNELS];
static uint8_t TRANSFER_BUFFER_BANK = 0;

static bool calibrationTaskNotPosted = true;
static bool scanCompleteTaskNotPosted = true;
static bool buttonWakeupTaskNotPosted = true;
/*  End IRQ Group */



/* Objective-C static class */

@implementation LeSense

+ (YTError)addChannel:(uint32_t)lesenseChannel 
   withCallOnPress:(yt_callback_t)callOnPress 
     callOnRelease:(yt_callback_t)callOnRelease 
       sensitivity:(uint32_t)sensitivityPercent
        andUpdates:(bool)updates
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

  int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

  /*  Sanity check. activeChannel == -1 means the lesenseChannel has not been
      configured for use and it can only be configured if there are available
      active channel slots. 
  */
  if ((lesenseChannel >= NUM_LESENSE_CHANNELS) ||
      (numChannelsUsed >= NUM_ACTIVE_CHANNELS && activeChannel == -1))
  {
    result = ytError(YTBadRequest);
  }
  else
  {
    /*  The same channel can be added multiple times. This is useful if multiple
        modules want to be notified about user interaction. 

        Callbacks are stored as linked lists. See the gpio-efm32 module for details.
    */
    if (activeChannel == -1)
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

      /* set current state. (removing all channels sets the state to pause.) */
      if (lesenseState == STATE_PAUSE)
      {
        lesenseState = STATE_IDLE;
      }

      /*  Post initial caliration task. The calibration is postponed if the number
          of channels in use has been changed since the task was posted.
      */
      uint32_t currentCount = numChannelsUsed;
      yt_callback_t initialCalibrationTask = ^{ 
        if (currentCount == numChannelsUsed)
        {
          [self calibrateWithForce:NO thenCall:nil];
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
    int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

    /*  If lesenseChannel has been configured, remove callbacks (if any)
        and decrement number of users. If no users are left on this lesense
        channel, clean up data structures. If no users are left on any 
        channels, pause the lesense module.
    */
    if (activeChannel != -1)
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

        lesenseToActiveMap[lesenseChannel] = -1;
      }

      /* check if lesense can be turned off. */
      uint32_t totalUsers = 0;
      for (int32_t allChannel = 0; 
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
+ (void)calibrateWithForce:(bool)forceCalibration
                  thenCall:(calibrate_callback_t)callback
{
  if ((lesenseState == STATE_IDLE) || 
     ((lesenseState != STATE_CALIBRATION) && forceCalibration) )
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

    lesenseState = STATE_CALIBRATION;

    if (callback)
    {
      calibrateDoneCallback = callback;
    }
    printf("calibration\n");
  }
  else if (lesenseState == STATE_CALIBRATION)
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
  if (lesenseState != STATE_PAUSE)
  {
    lesenseState = STATE_PAUSE;    
    LESENSE_ScanStop();
  }  
}

+ (void)resume
{  
  if (lesenseState == STATE_PAUSE)
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

+ (bool)channelIsActive:(uint32_t)lesenseChannel
{
  int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return alreadyPressed[activeChannel];
}

+ (int32_t)getValueForChannel:(uint32_t)lesenseChannel
{
  int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return transferBuffer[TRANSFER_BUFFER_BANK ^ 0x01][activeChannel];
}

+ (int32_t)getMaxValueForChannel:(uint32_t)lesenseChannel
{
  int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return channelMaxValue[activeChannel];
}

+ (int32_t)getMinValueForChannel:(uint32_t)lesenseChannel
{
  int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

  return channelMinValue[activeChannel];
}

+ (void)setMaxValue:(uint16_t)maxValue forChannel:(uint32_t)lesenseChannel
{
  int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

  channelMaxValue[activeChannel] = maxValue;  
}

+ (void)setMinValue:(uint16_t)minValue forChannel:(uint32_t)lesenseChannel
{
  int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

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


/*  Private function for initializing the lesense module. */
static void lesenseInit()
{
  for(int32_t activeChannel = 0; 
        activeChannel < NUM_ACTIVE_CHANNELS; 
          activeChannel++)
  {
    /* Init min and max values for each active channel */
    channelMaxValue[activeChannel] = 0;
    channelMinValue[activeChannel] = 0xffff;
    channelSensitivityPercent[activeChannel] = 100;
  }

  for (uint32_t lesenseChannel = 0; 
        lesenseChannel < NUM_LESENSE_CHANNELS; 
          lesenseChannel++)
  {
    lesenseToActiveMap[lesenseChannel] = -1;
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
      .startDelay     = 0x0
    },

    .perCtrl          =
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

    .decCtrl          =
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

  printf("%s done\n", __FUNCTION__);
}


/*  addCallback, removeCallback, and cleanupCallback maintain the linkedlists
    with call back functions. 
*/
static bool addCallback(yt_callback_t newCallback, bool updates, struct CallbackNode** node)
{
  /* insert new node at the end of the linked list */
  while(*node)
  {
    node = &((*node)->next);
  }

  *node = (struct CallbackNode*) calloc(1, sizeof (struct CallbackNode));

  if(!*node)
  {
    return FALSE;
  }

  (*node)->multipleUpdates = updates;
  (*node)->callback = newCallback;
  
  return TRUE;    
}

static bool removeCallback(yt_callback_t oldCallback, struct CallbackNode** node)
{
  while(*node)
  {
    if ((*node)->callback == oldCallback)
    {            
      void* trashNode = *node;
      *node = (*node)->next;
      free(trashNode);
      
      return TRUE;
    }
    node = &((*node)->next);
  }
  return FALSE;
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
    NUMBER_OF_CALIBRATION_VALUES using multiple rounds of interrupts. Once
    completed, the median value for each channel is found and used as the
    maxValue observed. Each channel's threshold is set based on the 
    sensitivity and the maxValue. The minValue is reset to maxValue.
*/

yt_callback_t calibrationTask = ^{

  NVIC_DisableIRQ(LESENSE_IRQn);
  TRANSFER_BUFFER_BANK ^= 0x01;
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Iterate through all possible channels, but only store the active ones. */
  for(uint32_t lesenseChannel = 0; 
      (lesenseChannel < NUM_LESENSE_CHANNELS); 
        lesenseChannel++)
  {
    if((channelsUsedMask >> lesenseChannel) & 0x01)
    {
      int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

      calibrationValue[activeChannel][calibrationValueIndex] = 
        transferBuffer[TRANSFER_BUFFER_BANK ^ 0x01][activeChannel];
    }
  }
   
  /* Wrap around calibrationValueIndex */
  calibrationValueIndex++;
  if(calibrationValueIndex >= NUMBER_OF_CALIBRATION_VALUES)
  {
    /* Calculate max/min-value for each channel and set threshold */
    for(uint32_t lesenseChannel = 0; 
        (lesenseChannel < NUM_LESENSE_CHANNELS); 
          lesenseChannel++)
    {
      if((channelsUsedMask >> lesenseChannel) & 0x1)
      {
        int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

        /* reset the min-max values to use the median calibration value. */
        channelMaxValue[activeChannel] = GetMedianValue(calibrationValue[activeChannel], NUMBER_OF_CALIBRATION_VALUES);
        channelMinValue[activeChannel] = channelMaxValue[activeChannel];

        /* use the new max-value to set threshold based on sensitivity. */
        uint32_t nominalCount = channelMaxValue[activeChannel];
        LESENSE_ChannelThresSet(lesenseChannel, 0x0,(uint16_t) ((nominalCount * channelSensitivityPercent[activeChannel])/100) ); 

        printf("%d %d\n", lesenseChannel, (nominalCount * channelSensitivityPercent[activeChannel])/100);
      }
    }

    /* calibration done, switch back to normal mode */
    calibrationValueIndex = 0;
    lesenseState = STATE_IDLE;

    /* restore interrupts to channel/threshold */
    LESENSE_ScanStop();

    /* Wait for current scan to finish */
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE); 

    LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
    LESENSE_IntClear(_LESENSE_IFC_MASK);
    LESENSE_IntEnable(channelsUsedMask);
    LESENSE_ScanFreqSet(0, scanFrequencyIdle);
    LESENSE_ScanStart();

    printf("calibration done\n");

    if (calibrateDoneCallback)
    {
      ytPost(^{calibrateDoneCallback(ytError(YTSuccess));}, ytMilliseconds(0));
    }
  }
  else
    printf("calibrating\n");

  NVIC_DisableIRQ(LESENSE_IRQn);
  calibrationTaskNotPosted = true;
  NVIC_EnableIRQ(LESENSE_IRQn);
};


static uint32_t oldCount = 0;

yt_callback_t buttonWakeupTask = ^{
  uint32_t newCount = RTC->CNT;
  printf("%s %d\n", __FUNCTION__, newCount - oldCount);
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
};


yt_callback_t scanCompleteTask = ^{

  // debug timing measurement
  uint32_t newCount = RTC->CNT;
  printf("%s %d\n", __FUNCTION__, newCount - oldCount);
  oldCount = newCount;

  NVIC_DisableIRQ(LESENSE_IRQn);
  TRANSFER_BUFFER_BANK ^= 0x01;
  NVIC_EnableIRQ(LESENSE_IRQn);

  uint32_t localScanres = lastScanres[TRANSFER_BUFFER_BANK ^ 0x01];


  for(uint32_t lesenseChannel = 0; 
      lesenseChannel < NUM_LESENSE_CHANNELS; 
        lesenseChannel++)
  {
    int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

    if (activeChannel != -1)
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
            ytPost(node->callback, ytMilliseconds(0));
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
              ytPost(node->callback, ytMilliseconds(0));
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
            ytPost(node->callback, ytMilliseconds(0));

            node = node->next;
          }
        }
      } // end is-button-pressed-or-not
    } // end activeChannel != -1
  } // end for-loop

  if (!localScanres)
  {
    for(uint32_t lesenseChannel = 0; 
        lesenseChannel < NUM_LESENSE_CHANNELS; 
          lesenseChannel++)
    {
      int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

      NSAssert(!alreadyPressed[activeChannel], "buttons not released\n");
    }

    // restore interrupts to channel/threshold
    LESENSE_ScanStop();

    /* Wait for current scan to finish */
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE); 

    LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
    LESENSE_IntClear(_LESENSE_IFC_MASK);
    LESENSE_IntEnable(channelsUsedMask);
    LESENSE_ScanFreqSet(0, scanFrequencyIdle);
    LESENSE_ScanStart();

    lesenseState = STATE_IDLE;
  }

  NVIC_DisableIRQ(LESENSE_IRQn);
  scanCompleteTaskNotPosted = true;
  NVIC_EnableIRQ(LESENSE_IRQn);
};




void LESENSE_IRQHandler( void )
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
      ytPost(buttonWakeupTask, ytMilliseconds(0));
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
      bufferIndex = NUM_LESENSE_CHANNELS + bufferIndex - numChannelsUsed;
    }

    /*  Read data from peripheral buffer into transfer buffer for later 
        analysis. The minimum value observed for each channel is updated.
        This value can be used for calculating a normalised pressure.        
    */
    for(uint32_t lesenseChannel = 0; 
        (lesenseChannel < NUM_LESENSE_CHANNELS); 
          lesenseChannel++)
    {
      if((channelsUsedMask >> lesenseChannel) & 0x01)
      {
        int32_t activeChannel = lesenseToActiveMap[lesenseChannel];

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
    lastEvent[TRANSFER_BUFFER_BANK] = ytPlatformGetTime();

    /*  Post the appropriate task to handle the sampled data. */
    if (lesenseState == STATE_CALIBRATION)
    {
      if (calibrationTaskNotPosted)
      {
        calibrationTaskNotPosted = false;
        ytPost(calibrationTask, ytMilliseconds(0));
      }
    }
    else 
    {
      if (scanCompleteTaskNotPosted)
      {
        scanCompleteTaskNotPosted = false;  
        ytPost(scanCompleteTask, ytMilliseconds(0));
      }
    }
  }

  /* Clear interrupt flag */
  LESENSE_IntClear(_LESENSE_IFC_MASK);
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

