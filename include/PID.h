#ifndef PID_H
#define PID_H

class PIDController {
private:
    float kp, ki, kd;
    float integral;
    float lastError;
    float maxOutput;
    unsigned long lastTime;
    
public:
    PIDController(float p, float i, float d, float maxOut) 
        : kp(p), ki(i), kd(d), maxOutput(maxOut), integral(0), lastError(0), lastTime(0) {}
    
    float compute(float setpoint, float input, float dt);
    void reset();
    void setTunings(float p, float i, float d);
    void setOutputLimits(float maxOut);
};

#endif