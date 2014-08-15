extern "C"{
#import "lesense/LESENSE.h"
#import "lesense_config.h"

#import "emlib/em_int.h"
#import "emlib/em_lesense.h"
#import "emlib/em_cmu.h"
#import "emlib/em_gpio.h"
#import "emlib/em_acmp.h"

#import "yottos/yottos.h"
#import "yottos_platform/yottos_platform.h"
#import "NSAssert/NSAssert.h"

#import <stdlib.h>
}

#if 1
#import "stdio.h"
#else
#define printf(...) ;
#endif

/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS    16

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT         gpioPortC

static uint16_t calibration_value[NUM_LESENSE_CHANNELS][NUMBER_OF_CALIBRATION_VALUES];
static uint16_t channel_max_value[NUM_LESENSE_CHANNELS];
static uint16_t channel_min_value[NUM_LESENSE_CHANNELS];

static uint16_t transfer_buffer[2][NUM_LESENSE_CHANNELS];
static uint8_t TRANSFER_BUFFER_BANK = 0;

static uint16_t channels_used_mask;
static uint8_t num_channels_used;

static uint8_t channel_deadzone_percent[NUM_LESENSE_CHANNELS];


#define LESENSE_IFS_ALL_CHANNELS 0x0000FFFF

typedef enum {
  STATE_IDLE,
  STATE_CALIBRATION,
  STATE_ACTIVE
} state_t;

static state_t lesenseState = STATE_IDLE;
static uint32_t scanFrequencyIdle = LESENSE_SCAN_FREQUENCY_LOW;
static uint32_t scanFrequencyActive = LESENSE_SCAN_FREQUENCY_HIGH;

void LETOUCH_Calibration(void);
static uint16_t GetMaxValue(uint16_t* A, uint16_t N);
static uint16_t GetMinValue(uint16_t* A, uint16_t N);

// callback linked-lists
struct CallbackNode
{
    struct CallbackNode* next;
    yt_callback_t callback;
    bool multipleUpdates;
};
static struct CallbackNode* onPress[NUM_LESENSE_CHANNELS] = {0};
static struct CallbackNode* onRelease[NUM_LESENSE_CHANNELS] = {0};
static bool addCallback(yt_callback_t newCallback, bool updates, struct CallbackNode** nodes);
static bool removeCallback(yt_callback_t oldCallback, struct CallbackNode** node);
static void cleanupCallback(struct CallbackNode** node);
static uint32_t currentUsers[NUM_LESENSE_CHANNELS] = {0};

static bool alreadyPressed[NUM_LESENSE_CHANNELS] = {0};

yt_tick_t lastEvent[2] = {0};
uint32_t lastScanres[2] = {0};

static bool runOnce = true;


void channelInit()
{
  if (runOnce)
  {
    runOnce = false;

/*
    GPIO_PinModeSet(gpioPortE, 2, gpioModePushPullDrive, 1);
    GPIO_DriveModeSet(gpioPortE, gpioDriveModeLowest);    
    GPIO_PinOutSet(gpioPortE, 2);
//    GPIO_PinOutClear(gpioPortE, 2);

    GPIO_PinModeSet(gpioPortE, 3, gpioModePushPullDrive, 1);
    GPIO_DriveModeSet(gpioPortE, gpioDriveModeLow);    
    GPIO_PinOutSet(gpioPortE, 3);
//    GPIO_PinOutClear(gpioPortE, 3);
*/

    for(uint32_t idx = 0; idx < NUM_LESENSE_CHANNELS; idx++)
    {
      /* Init min and max values for each channel */
      channel_max_value[idx] = 0;
      channel_min_value[idx] = 0xffff;
      channel_deadzone_percent[idx] = 100;
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
}

void channelAdd(uint32_t channel, 
               yt_callback_t callOnPress, 
               yt_callback_t callOnRelease, 
                uint32_t deadzone_percent,
                        bool updates)
{
  if (channel < NUM_LESENSE_CHANNELS)
  {
    bool result = TRUE;

    /*  The same channel can be added multiple times. This is useful if multiple
        modules want to be notified about user interaction. 

        Callbacks are stored as linked lists. See the gpio-efm32 module for details.
    */
    if (callOnPress)
    {
      result &= addCallback(callOnPress, updates, &onPress[channel]);
    }

    if (callOnRelease)
    {
      result &= addCallback(callOnRelease, updates, &onRelease[channel]);
    }

    if (result)
    {
      if (currentUsers[channel] == 0)
      {
        currentUsers[channel]++;

        /* Add channel sensitivity */
        channel_deadzone_percent[channel] = deadzone_percent;
        channels_used_mask |= (1 << channel);
        num_channels_used++;

        /* Set GPIO pin mode to disabled for all active pins */
        GPIO_PinModeSet(LESENSE_CH_PORT, channel, gpioModeDisabled, 0);

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

        /* Configure channels */
        LESENSE_ChannelConfig(&initLesenseCh, channel);
      }
    }
  }
}

void channelRemove(uint32_t channel, 
                  yt_callback_t callOnPress, 
                  yt_callback_t callOnRelease)
{
  if (channel < NUM_LESENSE_CHANNELS)
  {
    if (currentUsers[channel] > 0)
    {
      if (callOnPress)
      {
        removeCallback(callOnPress, &onPress[channel]);
      }
      if (callOnRelease)
      {
        removeCallback(callOnRelease, &onRelease[channel]);
      }

      currentUsers[channel]--;
    }

    if (currentUsers[channel] == 0)
    {
      // clean up
      cleanupCallback(&onPress[channel]);
      cleanupCallback(&onRelease[channel]);   

      /* FIXME: missing code to disable LESENSE module */
    }
  }
}

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


/****************************************************************************
 * Pause and resume functions
*****************************************************************************/

void lesensePause()
{
  LESENSE_ScanStop();
}

void lesenseResume()
{
  /* restore interrupts to channel/threshold */
  LESENSE_ScanStop();

  /* Wait for current scan to finish */
  while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE); 

  LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
  LESENSE_IntClear(_LESENSE_IFC_MASK);
  LESENSE_IntEnable(channels_used_mask);
  NVIC_EnableIRQ(LESENSE_IRQn);

  LESENSE_ScanFreqSet(0, scanFrequencyIdle);
  LESENSE_ScanStart();
}



/****************************************************************************
 * Calibration function 
*****************************************************************************/
typedef void (^calibrate_callback_t)(YTError error);
static calibrate_callback_t calibrateDoneCallback = nil;

void channelCalibrate(bool forceCalibration, calibrate_callback_t callback)
{
  if ((lesenseState == STATE_IDLE) || forceCalibration)
  {
    LESENSE_ScanStop();

    /* Wait for current scan to finish */
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE); 

    LESENSE_ResultBufferClear();

    LESENSE_IntDisable(channels_used_mask);
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


yt_tick_t getTimestamp()
{
  return lastEvent[TRANSFER_BUFFER_BANK ^ 0x01];
}

int32_t getChannelValue(uint32_t channel)
{
  return transfer_buffer[TRANSFER_BUFFER_BANK ^ 0x01][channel];
}

int32_t getMaxChannelValue(uint32_t channel)
{
  return channel_max_value[channel];
}

int32_t getMinChannelValue(uint32_t channel)
{
  return channel_min_value[channel];
}

bool isPressed(uint32_t channel)
{
  return alreadyPressed[channel];
}

void setIdleFrequency(uint32_t freqHz)
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

void setActiveFrequency(uint32_t freqHz)
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




/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/

static bool calibrationTaskNotPosted = true;
static bool scanCompleteTaskNotPosted = true;
static bool buttonWakeupTaskNotPosted = true;


yt_callback_t calibrationTask = ^{

  static uint8_t calibration_value_index = 0;

  NVIC_DisableIRQ(LESENSE_IRQn);
  TRANSFER_BUFFER_BANK ^= 0x01;
  NVIC_EnableIRQ(LESENSE_IRQn);

  for(uint32_t channel = 0; channel < NUM_LESENSE_CHANNELS; channel++)
  {
    if((channels_used_mask >> channel) & 0x01)
    {
      calibration_value[channel][calibration_value_index] = transfer_buffer[TRANSFER_BUFFER_BANK ^ 0x01][channel];
    }
  }
   
  /* Wrap around calibration_values_index */
  calibration_value_index++;
  if(calibration_value_index >= NUMBER_OF_CALIBRATION_VALUES)
  {
    /* Calculate max/min-value for each channel and set threshold */
    for(uint32_t idx = 0; idx < NUM_LESENSE_CHANNELS; idx++)
    {
      if((channels_used_mask >> idx) & 0x1)
      {
        channel_max_value[idx] = GetMaxValue(calibration_value[idx], NUMBER_OF_CALIBRATION_VALUES);
        channel_min_value[idx] = GetMinValue(calibration_value[idx], NUMBER_OF_CALIBRATION_VALUES);
        
        uint32_t nominal_count = channel_max_value[idx];
        LESENSE_ChannelThresSet(idx, 0x0,(uint16_t) (nominal_count - ((nominal_count * channel_deadzone_percent[idx])/100)) ); 

        printf("%d %d\n", idx, (nominal_count - ((nominal_count * channel_deadzone_percent[idx])/100)));
      }
    }

    /* calibration done, switch back to normal mode */
    calibration_value_index = 0;
    lesenseState = STATE_IDLE;

    /* restore interrupts to channel/threshold */
    LESENSE_ScanStop();

    /* Wait for current scan to finish */
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE); 

    LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
    LESENSE_IntClear(_LESENSE_IFC_MASK);
    LESENSE_IntEnable(channels_used_mask);
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


  for(uint32_t channel = 0; channel < NUM_LESENSE_CHANNELS; channel++)
  {
    // threshold reached for this channel, i.e., button was pressed
    if (((localScanres & channels_used_mask) >> channel) & 0x01)
    {
      if (alreadyPressed[channel] == false) 
      {
        alreadyPressed[channel] = true;

        /* go through all single call callbacks (and other callbacks) */
        struct CallbackNode* node = onPress[channel];
        while(node)
        {
          ytPost(node->callback, ytMilliseconds(0));
          node = node->next;
        }
      }
      else
      {
        /* only go through callbacks that can be called multiple times */
        struct CallbackNode* node = onPress[channel];
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
      if (alreadyPressed[channel] == true)
      {
        alreadyPressed[channel] = false;

        struct CallbackNode* node = onRelease[channel];
        while(node)
        {
          ytPost(node->callback, ytMilliseconds(0));

          node = node->next;
        }
      }
    }
  }

  if (!localScanres)
  {
    for(uint32_t channel = 0; channel < NUM_LESENSE_CHANNELS; channel++)
    {
      NSAssert(!alreadyPressed[channel], "buttons not released\n");
    }

    // restore interrupts to channel/threshold
    LESENSE_ScanStop();

    /* Wait for current scan to finish */
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE); 

    LESENSE_IntDisable(LESENSE_IEN_SCANCOMPLETE);
    LESENSE_IntClear(_LESENSE_IFC_MASK);
    LESENSE_IntEnable(channels_used_mask);
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
//  GPIO_PinOutToggle(gpioPortE, 2);

  /* Get pending and enabled interrupt flags */
  uint32_t interrupt_flags = LESENSE_IntGetEnabled();

  if (interrupt_flags & LESENSE_IFS_ALL_CHANNELS)
  {
//    LESENSE_ScanStop();
//    LESENSE_IntDisable(channels_used_mask);
    LESENSE_IntDisable(channels_used_mask);

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
    if(bufferIndex >= num_channels_used)
    {
      bufferIndex = bufferIndex - num_channels_used;
    }
    else
    {
      bufferIndex = NUM_LESENSE_CHANNELS + bufferIndex - num_channels_used;
    }

    for(uint32_t channel = 0; channel < NUM_LESENSE_CHANNELS; channel++)
    {
      if((channels_used_mask >> channel) & 0x01)
      {
        uint16_t value = LESENSE_ScanResultDataBufferGet(bufferIndex++);
        transfer_buffer[TRANSFER_BUFFER_BANK][channel] = value;

        if (value < channel_min_value[channel])
        {
          channel_min_value[channel] = value;
        } 
      }
    }

    lastScanres[TRANSFER_BUFFER_BANK] = LESENSE->SCANRES;
    lastEvent[TRANSFER_BUFFER_BANK] = ytPlatformGetTime();

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
//  LESENSE_IntClear(interrupt_flags);
  LESENSE_IntClear(_LESENSE_IFC_MASK);
}



/**************************************************************************//**
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


/**************************************************************************//**
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


