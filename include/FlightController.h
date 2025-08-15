#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include "PID.h"
#include "config.h"

struct FlightData {
    float roll, pitch, yaw;
    float rollRate, pitchRate, yawRate;
    float accelX, accelY, accelZ;
    bool armed;
};

struct ControlInputs {
    float throttle;    // 0-100%
    float rollCmd;     // -100 to 100%
    float pitchCmd;    // -100 to 100%
    float yawCmd;      // -100 to 100%
    bool armCmd;
    bool disarmCmd;
};

class FlightController {
private:
    // Sensores
    Adafruit_MPU6050 mpu;
    
    // Servos para ESCs
    Servo esc1, esc2, esc3, esc4;
    
    // Controladores PID
    PIDController pidRoll;
    PIDController pidPitch;
    PIDController pidYaw;
    
    // Variables de estado
    FlightData flightData;
    ControlInputs inputs;
    
    // Timing
    unsigned long lastLoopTime;
    float deltaTime;
    
    // Calibración
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    bool calibrated;
    
    // Métodos privados
    void calibrateSensors();
    void readSensors();
    void calculateAngles();
    void computePID();
    void updateMotors();
    void armMotors();
    void disarmMotors();
    int constrainValue(int value, int min, int max);
    
public:
    FlightController();
    bool init();
    void update();
    void setInputs(ControlInputs& newInputs);
    FlightData getFlightData() const { return flightData; }
    bool isArmed() const { return flightData.armed; }
    void emergencyStop();
};

#endif