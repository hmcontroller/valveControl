#include <mbed.h>
#include "microRay.h"

// also toggles microRay, so that one can debug with printf's
//#define DEBUG

// frequency of executing control loop
#define LOOP_PERIOD 0.0002f

// frequency of executing measurements
#define PWM_PERIOD_LENGTH 0.001f

// #define FIRST_MEASURE_DELAY 0.0000067f
#define FIRST_MEASURE_DELAY 0.0001f

// duration between single measurement steps
#define SAMPLING_PERIOD 0.0000001f

#define MEASUREMENTS_COUNT 4

// use something greater zero here, otherwise measurements will never be triggered
#define INITIAL_PWM_DUTY_FACTOR 0.5f

#define LOWER_VALVE_CONTROL_LIMIT 0.3f
#define UPPER_VALVE_CONTROL_LIMIT 0.95f




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
float FIRCurrentDerivative(float);
float pt1Filter(float u);

#if defined DEBUG
Serial pc(USBTX, USBRX, 115200);
#endif



Timeout laterDoMeasureTimeout;
PwmOut pwmOut(PE_13);
InterruptIn pwmOutInterrupt(PD_14);
Ticker slowLoopTicker;
AnalogIn valvePosition(A4);
AnalogIn valveCurrent(A0);
AnalogIn valveCurrentDerivative(A2);
Timer measurementTimer;

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
DigitalOut togglePin(PB_11);


Timer loopTimer;

int main() {
    #if !defined DEBUG
    microRayInit();
    #endif

    // loopCycleTimeUs can be set in microRay project settings
    float slowLoopTime = loopCycleTimeUs;
    slowLoopTime = slowLoopTime / 1000000.0f;
    pwmOutInterrupt.rise(&pwmRising);
    pwmOut.period(PWM_PERIOD_LENGTH);
    pwmOut.write(INITIAL_PWM_DUTY_FACTOR);
    slowLoopTicker.attach(&slowLoop, slowLoopTime);

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
    // mainly used for communication with microRay
    blueLed = !blueLed;

    valvePositionsMeasured[measurementCounter] = valvePosition.read();

    mR_valvePosition = valvePosition.read();
    mR_current = getMeanCurrent();
    mR_current_derivative = getMeanCurrentDerivative() * 30.0;
    mR_FIR = FIRCurrentDerivative(mR_current_derivative);
    mR_PT1 = pt1Filter(mR_valvePosition);
    mR_valveSetPoint = valveSetPoint;
    // mR_e = e;

    setValveTarget(mR_valveTarget);

    // pwmOut.write(mR_valveTarget);

    #if !defined DEBUG
    microRayCommunicate();
    #endif
}

void pwmRising() {
    laterDoMeasureTimeout.attach(&measure, FIRST_MEASURE_DELAY);
    redLed = !redLed;
    if (measurementCounter == 0) {
        measurementTimer.reset();
        measurementTimer.start();
    }
}

void measure() {
    yellowLed = !yellowLed;

    //__disable_irq();
    momentsOfMeasurements[measurementCounter] = measurementTimer.read_us();

    togglePin = 1;
    valveCurrentMeasured[measurementCounter] = valveCurrent.read();
    togglePin = 0;
    // valveCurrentDerivativeMeasured[measurementCounter] = valveCurrentDerivative.read();
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

    #if defined DEBUG
    pc.printf("xMean %f, yMean %f\n", xMean, yMean);
    #endif


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

float oldE;

void controlTheValve() {
    // valveSetPoint = KP_VALVE * getMeanCurrentDerivative();
    redLed = !redLed;
    // e = getValveTarget() - getMeanCurrentDerivative();
    // valveSetPoint = mR_kp * e + mR_kd * getMeanCurrentDerivative();

    // Spannung und Spulenwiderstand
    float inductance = (24.0 - 70 * getMeanCurrent()) / getMeanCurrentDerivative();

    e = getValveTarget() - inductance;
    valveSetPoint = mR_kp * e + mR_kd * (e - oldE) / PWM_PERIOD_LENGTH;

    if (valveSetPoint < LOWER_VALVE_CONTROL_LIMIT) {
        valveSetPoint = LOWER_VALVE_CONTROL_LIMIT;
    }
    if (valveSetPoint > UPPER_VALVE_CONTROL_LIMIT) {
        valveSetPoint = UPPER_VALVE_CONTROL_LIMIT;
    }
    pwmOut.write(valveSetPoint);
    mR_e = inductance;
    // pwmOut.write(mR_valveTarget);
}

float oldDerivatives[5] = { 0.0 };
float filtered = 0.0f;
// filter the current derivative please
float FIRCurrentDerivative(float newest) {
    int i = 0;
    for (i=0; i<4; i++) {
        oldDerivatives[i] = oldDerivatives[i+1];
    }
    oldDerivatives[4] = newest;

    filtered = 0.0f;
    i = 0;
    for (i=4; i>-1; i--) {
        filtered += 1.0 * pow(oldDerivatives[i], (-i));
    }

    return filtered / 10000000.0f;
}

float lastY = 0.0;
#define HAHA 0.002
float pt1Filter(float u) {
    float TDurchH = mR_TPT1 / HAHA;
    float filtered =  1 / (TDurchH + 1) * (mR_KTPT1 * u + TDurchH * lastY);
    lastY = filtered;
    return filtered;
}
