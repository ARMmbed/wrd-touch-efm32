#import "yottos/yottos.h"
#import "yterror/YTError.h"

void channelInit();

void channelAdd(uint32_t channel, 
               yt_callback_t callOnPress, 
               yt_callback_t callOnRelease, 
                uint32_t threshold_percent,
                        bool multipleUpdates);

void channelRemove(uint32_t channel,
                  yt_callback_t callOnPress, 
                  yt_callback_t callOnRelease);

typedef void (^calibrate_callback_t)(YTError error);
void channelCalibrate(bool forceCalibration, calibrate_callback_t callback);

yt_tick_t getTimestamp();

int32_t getChannelValue(uint32_t channel);
int32_t getMaxChannelValue(uint32_t channel);
int32_t getMinChannelValue(uint32_t channel);
bool isPressed(uint32_t channel);

void setIdleFrequency(uint32_t freqHz);
void setActiveFrequency(uint32_t freqHz);

void lesensePause();
void lesenseResume();

