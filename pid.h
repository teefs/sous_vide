/*
 * pid.h
 *
 *  Created on: Dec 5, 2018
 *      Author: Nicholas
 */
#ifndef PID_H_
#define PID_H_

//enum Mode {PID_ON = 1, PID_OFF = 0);

typedef struct PIDVars {
    float outLower, outUpper;
    float lastInput, setpoint;
    float kp, ki, kd, integralTerm;
    int computeInterval;
} PIDVariables;

float compute(float input);
int setOutputLimits (float min, float max);
int setComputeInterval (int seconds);
int getComputeInterval (void);
int setGains (float Kp, float Ki, float Kd);
void setInitialInput (float input);
void setSetpoint (float setpoint);
float getSetpoint (void);

#endif /* PID_H_ */
