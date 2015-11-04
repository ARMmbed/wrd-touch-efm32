#ifndef __LESENSE_CONFIG_H__
#define __LESENSE_CONFIG_H__

typedef enum {
    LESENSE_CHANNEL_0 = 0,
    LESENSE_CHANNEL_1 = 1,
    LESENSE_CHANNEL_2 = 2,
    LESENSE_CHANNEL_3 = 3,
    LESENSE_CHANNEL_4 = 4,
    LESENSE_CHANNEL_5 = 5,
    LESENSE_CHANNEL_6 = 6,
    LESENSE_CHANNEL_7 = 7,
    LESENSE_CHANNEL_8 = 8,
    LESENSE_CHANNEL_9 = 9,
    LESENSE_CHANNEL_10 = 10,
    LESENSE_CHANNEL_11 = 11,
    LESENSE_CHANNEL_12 = 12,
    LESENSE_CHANNEL_13 = 13,
    LESENSE_CHANNEL_14 = 14,
    LESENSE_CHANNEL_15 = 15,
} LESENSEName;

/** These are settings that need to be tuned for different PCB's, overlays and applications. */

/** Scan frequency for LESENSE, how often all the pads are scanned. */
#define LESENSE_SCAN_FREQUENCY_LOW      20
#define LESENSE_SCAN_FREQUENCY_HIGH     20

/** Sample delay, how long the rc-oscillations are sampled. */
#if defined(TARGET_LIKE_WATCH)
#define SAMPLE_DELAY                    90
#define SLIDER_1                        LESENSE_CHANNEL_1
#define SLIDER_2                        LESENSE_CHANNEL_3
#define SLIDER_3                        LESENSE_CHANNEL_0
#define SLIDER_4                        LESENSE_CHANNEL_2
#define BUTTON_BACK                     LESENSE_CHANNEL_4
#else
#define SAMPLE_DELAY                    30
#define SLIDER_1                        LESENSE_CHANNEL_11
#define SLIDER_2                        LESENSE_CHANNEL_10
#define SLIDER_3                        LESENSE_CHANNEL_9
#define SLIDER_4                        LESENSE_CHANNEL_8
#endif

/** Number of calibration events used to calculate threshold. */
#define NUMBER_OF_CALIBRATION_VALUES    10

/** Interval between calibration, in seconds. */
//#define CALIBRATION_INTERVAL            5

#endif // __LESENSE_CONFIG_H__
