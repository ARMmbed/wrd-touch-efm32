#import "yottos/yottos.h"
#import "yterror/YTError.h"
#import "ytstatic/YTStatic.h"


typedef void (^calibrate_callback_t)(YTError error);

@interface LeSense : YTStatic

+ (YTError)addChannel:(uint32_t)channel 
      withCallOnPress:(yt_callback_t)callOnPress 
        callOnRelease:(yt_callback_t)callOnRelease 
          sensitivity:(uint32_t)threshold_percent
           andUpdates:(bool)multipleUpdates;

+ (void)removeChannel:(uint32_t)channel
          callOnPress:(yt_callback_t)callOnPress 
     andCallOnRelease:(yt_callback_t)callOnRelease;

+ (void)calibrateWithForce:(bool)forceCalibration
                  thenCall:(calibrate_callback_t)callback;

+ (void)pause;
+ (void)resume;

+ (void)setFrequencyWhenIdle:(uint32_t)freqHz;
+ (void)setFrequencyWhenActive:(uint32_t)freqHz;

+ (bool)channelIsActive:(uint32_t)channel;

+ (int32_t)getValueForChannel:(uint32_t)channel;
+ (int32_t)getMaxValueForChannel:(uint32_t)channel;
+ (int32_t)getMinValueForChannel:(uint32_t)channel;
+ (void)setMaxValue:(uint16_t)maxValue forChannel:(uint32_t)channel;
+ (void)setMinValue:(uint16_t)minValue forChannel:(uint32_t)channel;
+ (void)setThreshold:(uint16_t)threshold forChannel:(uint32_t)channel;

+ (yt_tick_t)getEventTimestamp;


@end

