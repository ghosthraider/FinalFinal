#include "PID.h"

float PIDController::compute(float setpoint, float input, float dt) {
    float error = setpoint - input;
    
    // Término proporcional
    float pTerm = kp * error;
    
    // Término integral
    integral += error * dt;
    float iTerm = ki * integral;
    
    // Término derivativo
    float derivative = (error - lastError) / dt;
    float dTerm = kd * derivative;
    
    // Calcular salida
    float output = pTerm + iTerm + dTerm;
    
    // Limitar salida
    if (output > maxOutput) output = maxOutput;
    else if (output < -maxOutput) output = -maxOutput;
    
    // Anti-windup para el término integral
    if (output >= maxOutput || output <= -maxOutput) {
        integral -= error * dt;  // Revertir la acumulación del integral
    }
    
    lastError = error;
    
    return output;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
}

void PIDController::setTunings(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void PIDController::setOutputLimits(float maxOut) {
    maxOutput = maxOut;
}