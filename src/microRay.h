#ifndef CONFIG_H
#define CONFIG_H

#define MBED_OS_SERIAL

// must have parameters
#define loopCycleTimeUs                            2000
#define CHANNELS_AVAILABLE_COUNT                      9
#define CHANNELS_REQUESTED_COUNT                      5
#define CHANNELS_UNREQUESTED_COUNT                    4
#define PARAMETER_COUNT                               6
#define SPECIAL_COMMANDS_COUNT                        3
#define BAUD_RATE                                115200
#define INT_TYPE                                      1
#define FLOAT_TYPE                                    2
#define RECORD_BUFFER_LENGTH                          1
#define PAUSE_AFTER_RECORD                            0

// All requested channels
#define mR_valvePosition                         (messageOutBuffer.channels[0])
#define mR_valveSetPoint                         (messageOutBuffer.channels[1])
#define mR_e                                     (messageOutBuffer.channels[2])
#define mR_position                              (messageOutBuffer.channels[3])
#define mR_unlimitedSetPoint                     (messageOutBuffer.channels[4])

// All unrequested channels
#define mR_current                               (unrequestedChannels[0])
#define mR_current_derivative                    (unrequestedChannels[1])
#define mR_FIR                                   (unrequestedChannels[2])
#define mR_PT1                                   (unrequestedChannels[3])

// all parameters
#define mR_kp                                    (parameters[0]).valueFloat
#define mR_kd                                    (parameters[1]).valueFloat
#define mR_valveTarget                           (parameters[2]).valueFloat
#define mR_TPT1                                  (parameters[3]).valueFloat
#define mR_KTPT1                                 (parameters[4]).valueFloat
#define mR_VORSTEUER                             (parameters[5]).valueFloat

// all special parameters
#define loopCycleTimeExceededByUs                (specialCommands[0])
#define serialTransmissionLag                    (specialCommands[1])
#define mrRecordModeEnable                       (specialCommands[2])


void microRayInit();
void microRayCommunicate();


#include <stdint.h>


typedef struct MessageIn
{
    int32_t parameterNumber;
    union {
        int32_t parameterValueInt;
        float parameterValueFloat;
    };
} MessageIn;

typedef struct MessageOut
{
    uint32_t loopStartTime;
    uint16_t statusFlags;
    uint16_t parameterNumber;
#if !defined(SUPPRESS_PARAM_CONFIRMATION)
    union {
        int32_t parameterValueInt;
        float parameterValueFloat;
    };
#endif
    float channels[CHANNELS_REQUESTED_COUNT];
} MessageOut;

extern MessageOut messageOutBuffer;



typedef struct Parameter {
    uint8_t dataType;
    union {
        int32_t valueInt;
        float valueFloat;
    };
} Parameter;

extern Parameter parameters[PARAMETER_COUNT];
extern int specialCommands[SPECIAL_COMMANDS_COUNT];


// storage for unrequested channels
// requested channels are stored in messageOutBuffer
extern float unrequestedChannels[CHANNELS_UNREQUESTED_COUNT];

#define RECORD_MODE 0
#define RECORD_TRANSMISSION_MODE 1
#define LIVE_MODE 2
#define RECORD_WAIT_MODE 3
#define WAIT_MODE 4

#define STATUS_BAD_DATA 0
#define STATUS_SKIPPED 1
#endif