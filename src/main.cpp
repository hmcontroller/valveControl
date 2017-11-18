#include <mbed.h>
#include "microRay.h"

//#define DEBUG

#if defined DEBUG
#define LOOP_PERIOD 1.0f
#define FIRST_MEASURE_DELAY 0.00001f
#define SAMPLING_PERIOD 0.00001f
#define PWM_PERIOD_LENGTH 0.0002f

#else
#define LOOP_PERIOD 0.0002f
#define FIRST_MEASURE_DELAY 0.00001f
#define SAMPLING_PERIOD 0.00001f
#define PWM_PERIOD_LENGTH 0.0002f
#endif

#define INITIAL_PWM_DUTY_FACTOR 0.5f

void loop();
void slowLoop();
void pwmRising();
void measure();
void setValveTarget(float target);
float getValveTarget();
void calculateMeanCurrent();
float getMeanCurrent();
void calculateCurrentDerivative();
float getMeanCurrentDerivative();
void controlTheValve();

//#if defined DEBUG
//Serial pc(USBTX, USBRX, 115200);
//#endif



Timeout laterDoMeasureTimeout;
PwmOut pwmOut(PE_13);
InterruptIn pwmOutInterrupt(PD_14);
//InterruptIn pwmOutInterrupt(PE_11);
Ticker slowLoopTicker;
AnalogIn valvePosition(A4);
AnalogIn valveCurrent(A0);
AnalogIn valveCurrentDerivative(A2);
Timer measurementTimer;

#define MEASUREMENTS_COUNT 5
int measurementCounter = 0;
bool measurementComplete = false;

volatile unsigned long momentsOfMeasurements[MEASUREMENTS_COUNT] = {0};
volatile float valvePositionsMeasured[MEASUREMENTS_COUNT] = {0.0f};
volatile float valveCurrentMeasured[MEASUREMENTS_COUNT] = {0.0f};
volatile float valveCurrentDerivativeMeasured[MEASUREMENTS_COUNT] = {0.0f};
volatile float valveTarget = 0.0f;
volatile float meanCurrent = 0.0f;
volatile float adjustedCurrentDerivative = 0.0f;

float e = 0.0f;
float valveSetPoint = 0.0f;

DigitalOut yellowLed(LED1);
DigitalOut blueLed(LED2);
DigitalOut redLed(LED3);

Timer loopTimer;

int main() {
    //#if !defined DEBUG
    microRayInit();
    //#endif

    float slowLoopTime = loopCycleTimeUs;
    slowLoopTime = slowLoopTime / 1000000.0f;
    pwmOutInterrupt.rise(&pwmRising);
    pwmOut.period(PWM_PERIOD_LENGTH);
    pwmOut.write(INITIAL_PWM_DUTY_FACTOR);
    slowLoopTicker.attach(&slowLoop, slowLoopTime);
    //setValveTarget(0.2f);

    while(1)
    {
        loop();
    }
}

void loop() {
    if (measurementComplete) {
        measurementComplete = false;
        calculateMeanCurrent();
        calculateCurrentDerivative();
        controlTheValve();
    }
}

void slowLoop() {
    blueLed = !blueLed;
    mR_valvePosition = valvePosition.read();
    mR_current = getMeanCurrent();
    mR_current_derivative = getMeanCurrentDerivative();
    mR_valveSetPoint = valveSetPoint;
    mR_e = e;

    setValveTarget(mR_valveTarget);

    //#if !defined DEBUG
    microRayCommunicate();
    //#endif
}

void pwmRising() {
    redLed = !redLed;
    measurementTimer.reset();
    measurementTimer.start();
    laterDoMeasureTimeout.attach(&measure, FIRST_MEASURE_DELAY);
}

void measure() {
    yellowLed = !yellowLed;

    //__disable_irq();
    momentsOfMeasurements[measurementCounter] = measurementTimer.read_us();
    valvePositionsMeasured[measurementCounter] = valvePosition.read();
    valveCurrentMeasured[measurementCounter] = valveCurrent.read();
    valveCurrentDerivativeMeasured[measurementCounter] = valveCurrentDerivative.read();
    //__enable_irq();


    measurementCounter += 1;
    if(measurementCounter >= MEASUREMENTS_COUNT) {
        measurementCounter = 0;
        measurementComplete = true;
    }
    else {
        laterDoMeasureTimeout.attach(&measure, SAMPLING_PERIOD);
    }
}

void setValveTarget(float target) {
    //__disable_irq();
    valveTarget = target;
    //__enable_irq();
}


float getValveTarget() {
    static float tempValveTarget = 0.0f;
    //__disable_irq();
    tempValveTarget = valveTarget;
    //__enable_irq();
    return tempValveTarget;
}


void calculateMeanCurrent() {
    static float meanCurrentTemp = 0.0f;
    meanCurrentTemp = 0.0f;
    for (int i = 0; i < MEASUREMENTS_COUNT; i++) {
        meanCurrentTemp += valveCurrentMeasured[i];
    }
    meanCurrentTemp = meanCurrentTemp / (float)MEASUREMENTS_COUNT;
    //__disable_irq();
    meanCurrent = meanCurrentTemp;
    //__enable_irq();
}

float getMeanCurrent() {
    static float meanCurrentIRSTemp = 0.0f;
    //__disable_irq();
    meanCurrentIRSTemp = meanCurrent;
    //__enable_irq();
    return meanCurrentIRSTemp;
}

void calculateCurrentDerivative() {
    int i = 0;

    // calculate linear regression
    float nom = 0.0f;
    float denom = 0.0f;

    float xMean = 0.0f;
    float yMean = 0.0f;
    for (i = 0; i < MEASUREMENTS_COUNT; i++) {
        xMean += momentsOfMeasurements[i];
        yMean += valveCurrentMeasured[i];
    }
    xMean = xMean / (float)MEASUREMENTS_COUNT;
    yMean = yMean / (float)MEASUREMENTS_COUNT;

    for (i = 0; i < MEASUREMENTS_COUNT; i++) {
        nom += (momentsOfMeasurements[i] - xMean) * (valveCurrentMeasured[i] - yMean);
        denom += (momentsOfMeasurements[i] - xMean) * (momentsOfMeasurements[i] - xMean);
    }


    float adjustedCurrentDerivativeTEMP = nom / denom;

    //#if defined DEBUG
    //pc.printf("xMean %f, yMean %f\n", xMean, yMean);
    //#endif


    //__disable_irq();
    adjustedCurrentDerivative = adjustedCurrentDerivativeTEMP;
    //__enable_irq();
}


float getMeanCurrentDerivative() {
    static float tempCurrentDerivative = 0.0f;
    //__disable_irq();
    tempCurrentDerivative = adjustedCurrentDerivative;
    //__enable_irq();
    return tempCurrentDerivative;
}


void controlTheValve() {
    // valveSetPoint = KP_VALVE * getMeanCurrentDerivative();
    e = getValveTarget() - getMeanCurrent();
    valveSetPoint = mR_kp * e + mR_kd * getMeanCurrentDerivative();

    if (valveSetPoint < 0.05f) {
        valveSetPoint = 0.05f;
    }
    if (valveSetPoint > 1.0f) {
        valveSetPoint = 1.0f;
    }
    pwmOut.write(valveSetPoint);
    //pwmOut.write(0.6f);
}
