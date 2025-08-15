#include "FlightController.h"
#include <Wire.h>

FlightController::FlightController() 
    : pidRoll(PID_P_GAIN_ROLL, PID_I_GAIN_ROLL, PID_D_GAIN_ROLL, PID_MAX_ROLL),
      pidPitch(PID_P_GAIN_PITCH, PID_I_GAIN_PITCH, PID_D_GAIN_PITCH, PID_MAX_PITCH),
      pidYaw(PID_P_GAIN_YAW, PID_I_GAIN_YAW, PID_D_GAIN_YAW, PID_MAX_YAW),
      calibrated(false),
      lastLoopTime(0),
      deltaTime(0) {
    
    // Inicializar datos de vuelo
    memset(&flightData, 0, sizeof(flightData));
    memset(&inputs, 0, sizeof(inputs));
    
    // Configurar offsets iniciales
    gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;
}

bool FlightController::init() {
    Serial.println("Inicializando Flight Controller...");
    
    // Configurar I2C
    Wire.begin(PIN_MPU6050_SDA, PIN_MPU6050_SCL);
    Wire.setClock(400000);
    
    // Inicializar MPU6050
    if (!mpu.begin()) {
        Serial.println("Error: No se pudo inicializar MPU6050!");
        return false;
    }
    
    // Configurar MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Configurar ESCs
    esc1.attach(PIN_ESC_1, ESC_MIN_PULSE, ESC_MAX_PULSE);
    esc2.attach(PIN_ESC_2, ESC_MIN_PULSE, ESC_MAX_PULSE);
    esc3.attach(PIN_ESC_3, ESC_MIN_PULSE, ESC_MAX_PULSE);
    esc4.attach(PIN_ESC_4, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    // Configurar LED de estado
    pinMode(PIN_LED_STATUS, OUTPUT);
    
    // Armar ESCs con pulso mínimo
    Serial.println("Armando ESCs...");
    esc1.writeMicroseconds(ESC_ARM_PULSE);
    esc2.writeMicroseconds(ESC_ARM_PULSE);
    esc3.writeMicroseconds(ESC_ARM_PULSE);
    esc4.writeMicroseconds(ESC_ARM_PULSE);
    
    delay(2000);  // Esperar a que los ESCs se armen
    
    // Calibrar sensores
    calibrateSensors();
    
    Serial.println("Flight Controller inicializado correctamente!");
    return true;
}

void FlightController::calibrateSensors() {
    Serial.println("Calibrando sensores... Mantén el dron inmóvil");
    
    const int numSamples = 1000;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        sumGx += g.gyro.x;
        sumGy += g.gyro.y;
        sumGz += g.gyro.z;
        
        if (i % 100 == 0) {
            Serial.print(".");
        }
        delay(2);
    }
    
    gyroOffsetX = sumGx / numSamples;
    gyroOffsetY = sumGy / numSamples;
    gyroOffsetZ = sumGz / numSamples;
    
    calibrated = true;
    Serial.println("\nCalibración completada!");
    Serial.printf("Offsets - X: %.4f, Y: %.4f, Z: %.4f\n", 
                  gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

void FlightController::readSensors() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Aplicar offsets del giroscopio
    flightData.rollRate = (g.gyro.x - gyroOffsetX) * 180.0 / PI;  // Convertir a grados/s
    flightData.pitchRate = (g.gyro.y - gyroOffsetY) * 180.0 / PI;
    flightData.yawRate = (g.gyro.z - gyroOffsetZ) * 180.0 / PI;
    
    // Leer acelerómetro
    flightData.accelX = a.acceleration.x;
    flightData.accelY = a.acceleration.y;
    flightData.accelZ = a.acceleration.z;
}

void FlightController::calculateAngles() {
    // Calcular ángulos del acelerómetro
    float accelRoll = atan2(flightData.accelY, flightData.accelZ) * 180.0 / PI;
    float accelPitch = atan2(-flightData.accelX, 
                            sqrt(flightData.accelY * flightData.accelY + 
                                 flightData.accelZ * flightData.accelZ)) * 180.0 / PI;
    
    // Filtro complementario
    flightData.roll = GYRO_FILTER_ALPHA * (flightData.roll + flightData.rollRate * deltaTime) +
                      ACCEL_FILTER_ALPHA * accelRoll;
    
    flightData.pitch = GYRO_FILTER_ALPHA * (flightData.pitch + flightData.pitchRate * deltaTime) +
                       ACCEL_FILTER_ALPHA * accelPitch;
    
    // Para yaw solo usamos el giroscopio (sin magnetómetro)
    flightData.yaw += flightData.yawRate * deltaTime;
    
    // Mantener yaw en rango 0-360
    if (flightData.yaw > 360) flightData.yaw -= 360;
    if (flightData.yaw < 0) flightData.yaw += 360;
}

void FlightController::computePID() {
    if (!flightData.armed) return;
    
    // Convertir comandos de entrada a setpoints
    float rollSetpoint = (inputs.rollCmd / 100.0) * MAX_ANGLE_ROLL;
    float pitchSetpoint = (inputs.pitchCmd / 100.0) * MAX_ANGLE_PITCH;
    float yawSetpoint = (inputs.yawCmd / 100.0) * MAX_RATE_YAW;  // Yaw en rate mode
    
    // Calcular salidas PID
    float rollOutput = pidRoll.compute(rollSetpoint, flightData.roll, deltaTime);
    float pitchOutput = pidPitch.compute(pitchSetpoint, flightData.pitch, deltaTime);
    float yawOutput = pidYaw.compute(yawSetpoint, flightData.yawRate, deltaTime);
    
    // Calcular señales para cada motor
    int throttle = (inputs.throttle / 100.0) * (ESC_MAX_PULSE - ESC_MIN_PULSE) + ESC_MIN_PULSE;
    
    int motor1 = throttle + rollOutput - pitchOutput - yawOutput;  // Delantero derecho
    int motor2 = throttle + rollOutput + pitchOutput + yawOutput;  // Trasero derecho
    int motor3 = throttle - rollOutput + pitchOutput - yawOutput;  // Trasero izquierdo
    int motor4 = throttle - rollOutput - pitchOutput + yawOutput;  // Delantero izquierdo
    
    // Constrair valores
    motor1 = constrainValue(motor1, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motor2 = constrainValue(motor2, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motor3 = constrainValue(motor3, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motor4 = constrainValue(motor4, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    // Enviar señales a motores
    esc1.writeMicroseconds(motor1);
    esc2.writeMicroseconds(motor2);
    esc3.writeMicroseconds(motor3);
    esc4.writeMicroseconds(motor4);
}

void FlightController::updateMotors() {
    if (!flightData.armed) {
        esc1.writeMicroseconds(ESC_MIN_PULSE);
        esc2.writeMicroseconds(ESC_MIN_PULSE);
        esc3.writeMicroseconds(ESC_MIN_PULSE);
        esc4.writeMicroseconds(ESC_MIN_PULSE);
    }
}

void FlightController::update() {
    unsigned long currentTime = micros();
    deltaTime = (currentTime - lastLoopTime) / 1000000.0f;
    lastLoopTime = currentTime;
    
    // Leer sensores
    readSensors();
    
    // Calcular ángulos
    calculateAngles();
    
    // Procesar comandos de armado/desarmado
    if (inputs.armCmd && !flightData.armed) {
        armMotors();
        inputs.armCmd = false;  // Resetear comando después de procesar
    } else if (inputs.disarmCmd && flightData.armed) {
        disarmMotors();
        inputs.disarmCmd = false;  // Resetear comando después de procesar
    }
    
    // Calcular PID y actualizar motores
    computePID();
    updateMotors();
    
    // Actualizar LED de estado
    digitalWrite(PIN_LED_STATUS, flightData.armed ? HIGH : LOW);
}

void FlightController::armMotors() {
    if (inputs.throttle <= 5) {  // Permitir hasta 5% de throttle
        flightData.armed = true;
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
        Serial.println("MOTORES ARMADOS!");
        Serial.printf("Throttle actual: %.1f%%\n", inputs.throttle);
    } else {
        Serial.printf("ERROR: Throttle muy alto (%.1f%%) - Baja a 5%% o menos para armar!\n", inputs.throttle);
    }
}

void FlightController::disarmMotors() {
    flightData.armed = false;
    Serial.println("MOTORES DESARMADOS!");
}

void FlightController::setInputs(ControlInputs& newInputs) {
    inputs = newInputs;
}

void FlightController::emergencyStop() {
    disarmMotors();
    esc1.writeMicroseconds(ESC_MIN_PULSE);
    esc2.writeMicroseconds(ESC_MIN_PULSE);
    esc3.writeMicroseconds(ESC_MIN_PULSE);
    esc4.writeMicroseconds(ESC_MIN_PULSE);
    Serial.println("PARADA DE EMERGENCIA ACTIVADA!");
}

int FlightController::constrainValue(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}