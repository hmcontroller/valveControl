#include "microRay.h"

void prepareOutMessage();
void prepareInMessage();
void sendMessage();
void receiveMessage();

unsigned long lastTime = 0;

// storage for unrequested channels
float unrequestedChannels[CHANNELS_UNREQUESTED_COUNT];

#if !defined(SUPPRESS_PARAM_CONFIRMATION)
int parameterSendCounter = 0;
#endif

int receivedBytesCount = 0;
int sendBytesCount = 0;

MessageOut messageOutBuffer;
MessageIn messageInBuffer;

void prepareOutMessage(unsigned long loopStartTime)
{
    // map rotating parameters
    // on each cycle, only one of the "controlled parameters" is send to the pc

    messageOutBuffer.loopStartTime = loopStartTime;

    #if !defined(SUPPRESS_PARAM_CONFIRMATION)
    messageOutBuffer.parameterNumber = parameterSendCounter;
    if (parameterSendCounter < 0) {
        messageOutBuffer.parameterValueFloat = specialCommands[(parameterSendCounter + 1) * -1];
    }
    else {
        switch (parameters[parameterSendCounter].dataType) {
            case INT_TYPE:
                messageOutBuffer.parameterValueInt = parameters[parameterSendCounter].valueInt;
                break;
            case FLOAT_TYPE:
                messageOutBuffer.parameterValueFloat = parameters[parameterSendCounter].valueFloat;
                break;
            default:
                break;
        }
    }

    // increment the counter for sending the "slow parameters"
    parameterSendCounter += 1;
    if (parameterSendCounter >= PARAMETER_COUNT)
    {
        parameterSendCounter = SPECIAL_COMMANDS_COUNT * -1;
    }
#endif
}

void prepareInMessage() {
    if (messageInBuffer.parameterNumber >= 0) {
        switch (parameters[messageInBuffer.parameterNumber].dataType) {
            case INT_TYPE:
                parameters[messageInBuffer.parameterNumber].valueInt = messageInBuffer.parameterValueInt;
                break;
            case FLOAT_TYPE:
                parameters[messageInBuffer.parameterNumber].valueFloat = messageInBuffer.parameterValueFloat;
                break;
            default:
                break;
        }
    }
    else {
        specialCommands[(messageInBuffer.parameterNumber + 1) * -1] = messageInBuffer.parameterValueFloat;
    }
}

void microRayCommunicate()
{
    receiveMessage();
    sendMessage();
}

Parameter parameters[] = {
    { 2, { .valueFloat = 1.0f} },
    { 2, { .valueFloat = 1.0f} },
    { 2, { .valueFloat = 0.0f} }
};

float specialCommands[] = {
    0.0f,
    0.0f
};


#include <mbed.h>

void serialSendCompleteOne(int events);
void serialSendCompleteTwo(int events);
void serialSendCompleteThree(int events);

void receiveMessage();
void readIncomingBytesIntoBuffer();
void appendByteToBuffer(uint8_t inByte);
void shiftGivenPositionToBufferStart(int position);
int seekForFullMessage();
void extractMessage(int messageStartPosition);


Serial mRserial(USBTX, USBRX, BAUD_RATE); // tx, rx
Ticker serialReceiver;

Timer dutyCycleTimer;
Timer debugTimer;
unsigned long timeOfLastSend = 0;
unsigned long timeOfLastCompletedMessage = 0;

#define OUT_START_BYTE (char)7
#define OUT_STOP_BYTE (char)8

#define IN_MESSAGE_SIZE 8
#define IN_BUFFER_SIZE ((IN_MESSAGE_SIZE+2)*2)

#define IN_START_BYTE (char)7
#define IN_STOP_BYTE (char)8

unsigned char startOut = 7;
unsigned char endOut = 8;


event_callback_t serialEventWriteCompleteOne = serialSendCompleteOne;
event_callback_t serialEventWriteCompleteTwo = serialSendCompleteTwo;
event_callback_t serialEventWriteCompleteThree = serialSendCompleteThree;


int debugCounterSend = 0;
void serialSendCompleteOne(int events) {
    mRserial.write((uint8_t *)&messageOutBuffer, sizeof(messageOutBuffer), serialEventWriteCompleteTwo, SERIAL_EVENT_TX_COMPLETE);
}

void serialSendCompleteTwo(int events) {
    mRserial.write((uint8_t *)&endOut, 1, serialEventWriteCompleteThree, SERIAL_EVENT_TX_COMPLETE);
}

void serialSendCompleteThree(int events) {
    timeOfLastCompletedMessage = timeOfLastSend;
}



void microRayInit() {
    dutyCycleTimer.start();
    serialReceiver.attach(&readIncomingBytesIntoBuffer, 0.0001f);
    //serialEventWriteComplete.attach(serialSendComplete);
    //serialEventReceiveComplete.attach(serialReadComplete);

    // start async receive of serial data
    //mRserial.read(&singleInBuffer, 1, serialEventReceiveComplete);

    //mRserial.set_dma_usage_tx(1);
    //mRserial.set_dma_usage_rx(1);
}

int serialTransmissionLagCounter = 0;
void sendMessage() {

    prepareOutMessage((unsigned long)dutyCycleTimer.read_high_resolution_us());

    if(timeOfLastCompletedMessage == timeOfLastSend) {
        timeOfLastSend = (unsigned long)messageOutBuffer.loopStartTime;
        mRserial.write((uint8_t *)&startOut, 1, serialEventWriteCompleteOne, SERIAL_EVENT_TX_COMPLETE);
    }
    else {
        serialTransmissionLagCounter++;
        serialTransmissionLag = (float)serialTransmissionLagCounter;
        timeOfLastSend = (unsigned long)messageOutBuffer.loopStartTime;
    }
}

uint8_t rawMessageInBuffer[IN_BUFFER_SIZE];
uint8_t rawMessageInBufferTemp[IN_BUFFER_SIZE];
int bufferPosition = 0;

void receiveMessage() {
    int foundMessageStartPosition = seekForFullMessage();
    if(foundMessageStartPosition > -1) {
        extractMessage(foundMessageStartPosition);
        prepareInMessage();
    }
}

void readIncomingBytesIntoBuffer() {
    int i = 0;
    for (i = 0; i < IN_MESSAGE_SIZE; i++) {
        if (mRserial.readable ()) {
            appendByteToBuffer(mRserial.getc());
        }
        else {
            break;
        }
    }
}

void appendByteToBuffer(uint8_t inByte) {

    // mRserial.printf("inbyte %c\n", inByte);

    // prevent buffer from overfilling
    if(bufferPosition >= IN_BUFFER_SIZE) {
        // shift whole buffer one to the left to free last position
        shiftGivenPositionToBufferStart(1);
    }
    rawMessageInBuffer[bufferPosition] = inByte;
    bufferPosition += 1;
}

void shiftGivenPositionToBufferStart(int position) {
    // copy and shift
    int i;
    for(i = position; i < bufferPosition; i++) {
        rawMessageInBufferTemp[i - position] = rawMessageInBuffer[i];
    }

    // actualize bufferPosition
    bufferPosition = bufferPosition - position;

    // copy back
    for(i = 0; i < bufferPosition; i++) {
        rawMessageInBuffer[i] = rawMessageInBufferTemp[i];
    }

}

int seekForFullMessage() {
    int i;
    for (i = 0; i < bufferPosition - IN_MESSAGE_SIZE; i++) {
        if (rawMessageInBuffer[i] == IN_START_BYTE) {
            int expectedStopBytePosition = i + IN_MESSAGE_SIZE + 1;
            if (rawMessageInBuffer[expectedStopBytePosition] == IN_STOP_BYTE) {
                return i;
            }
        }
    }
    return -1;
}

void extractMessage(int messageStartPosition) {
    memcpy(&messageInBuffer.parameterNumber, &rawMessageInBuffer[messageStartPosition + 1], 4);
    memcpy(&messageInBuffer.parameterValueInt, &rawMessageInBuffer[messageStartPosition + 1 + 4], 4);
    // mRserial.printf("%i %.2f",messageInBuffer.parameterNumber, messageInBuffer.parameterValueFloat );
    shiftGivenPositionToBufferStart(messageStartPosition + IN_MESSAGE_SIZE + 2);
}
