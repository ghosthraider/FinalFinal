#ifndef CONFIG_H
#define CONFIG_H

// Configuración de pines ESP32
#define PIN_ESC_1           33  // Motor delantero derecho (CCW)
#define PIN_ESC_2           25  // Motor trasero derecho (CW)
#define PIN_ESC_3           26  // Motor trasero izquierdo (CCW)
#define PIN_ESC_4           27  // Motor delantero izquierdo (CW)

#define PIN_MPU6050_SDA     21  // Pin SDA para MPU6050
#define PIN_MPU6050_SCL     22  // Pin SCL para MPU6050

#define PIN_LED_STATUS      2   // LED interno ESP32

// Configuración de comunicación
#define SERIAL_BAUD         115200

// Configuración de vuelo
#define LOOP_FREQUENCY      250   // Hz - Frecuencia del loop principal
#define LOOP_TIME_US        (1000000 / LOOP_FREQUENCY)

// Configuración PID - Roll
#define PID_P_GAIN_ROLL     1.0f
#define PID_I_GAIN_ROLL     0.01f
#define PID_D_GAIN_ROLL     10.0f
#define PID_MAX_ROLL        400

// Configuración PID - Pitch (igual que Roll)
#define PID_P_GAIN_PITCH    PID_P_GAIN_ROLL
#define PID_I_GAIN_PITCH    PID_I_GAIN_ROLL
#define PID_D_GAIN_PITCH    PID_D_GAIN_ROLL
#define PID_MAX_PITCH       PID_MAX_ROLL

// Configuración PID - Yaw
#define PID_P_GAIN_YAW      2.0f
#define PID_I_GAIN_YAW      0.05f
#define PID_D_GAIN_YAW      0.0f
#define PID_MAX_YAW         400

// Configuración ESC
#define ESC_MIN_PULSE       1000  // Microsegundos
#define ESC_MAX_PULSE       2000  // Microsegundos
#define ESC_ARM_PULSE       1000  // Pulso para armar ESCs

// Limites de ángulos (grados)
#define MAX_ANGLE_ROLL      30.0f
#define MAX_ANGLE_PITCH     30.0f
#define MAX_RATE_YAW        180.0f  // grados/segundo

// Configuración del giroscopio
#define GYRO_SENSITIVITY    131.0f  // LSB/(grados/s) para ±250°/s
#define ACCEL_SENSITIVITY   16384.0f // LSB/g para ±2g

// Filtros
#define GYRO_FILTER_ALPHA   0.98f   // Filtro complementario
#define ACCEL_FILTER_ALPHA  0.02f

#endif