/*
 * pid.c
 *
 *  Created on: Dec 5, 2018
 *      Author: Nicholas
 */
#include "pid.h"

PIDVariables vars = {.outLower = 0,
                     .outUpper = 0,
                     .lastInput = 0,
                     .setpoint = 0,
                     .kp = 0,
                     .ki = 0,
                     .kd = 0,
                     .integralTerm = 0,
                     .computeInterval = 0};

float compute(float input)
{
    float error = vars.setpoint - input;
    vars.integralTerm += vars.ki * error;
    if (vars.integralTerm > vars.outUpper)
        vars.integralTerm = vars.outUpper;
    else if (vars.integralTerm < vars.outLower)
        vars.integralTerm = vars.outLower;

    float deltaInput = input - vars.lastInput;

    float output = vars.kp * error + vars.integralTerm - vars.kd * deltaInput;
    if (output > vars.outUpper)
        output = vars.outUpper;
    else if (output < vars.outLower)
        output = vars.outLower;

    vars.lastInput = input;
    return output;
}

int setOutputLimits (float min, float max)
{
    if (min > max)
        return -1;
    vars.outUpper = max;
    vars.outLower = min;
    return 0;
}

int setComputeInterval (int seconds)
{
    if (seconds <= 0)
        return -1;
    vars.computeInterval = seconds;
    return 0;
}

int getComputeInterval (void)
{
    return vars.computeInterval;
}

int setGains (float Kp, float Ki, float Kd)
{
    if (vars.computeInterval <= 0)
        return -1;

    if ((Kp >= 0 && Ki >= 0 && Kd >= 0) || (Kp <= 0 && Ki <= 0 && Kd <= 0)){
        vars.kp = Kp / vars.computeInterval;
        vars.ki = Ki / vars.computeInterval;
        vars.kd = Kd / vars.computeInterval;
        return 0;
    }
    return -2;
}

void setInitialInput (float input)
{
    vars.lastInput = input;
}

void setSetpoint (float setpoint)
{
    vars.setpoint = setpoint;
}
float getSetpoint (void)
{
    return vars.setpoint;
}
