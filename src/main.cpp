#include <Arduino.h>
#include "FlightController.h"
#include "KeyboardController.h"
#include "config.h"

// Instancias globales
FlightController flightController;
KeyboardController keyboardController;

// Variables de timing
unsigned long lastLoopTime = 0;
unsigned long loopCounter = 0;

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(2000);  // Esperar a que se establezca la conexión serial
    
    Serial.println("\n========================================");
    Serial.println("    CONTROLADOR DE VUELO ESP32");
    Serial.println("       Basado en DroneIno");
    Serial.println("========================================");
    
    // Inicializar flight controller
    if (!flightController.init()) {
        Serial.println("ERROR FATAL: No se pudo inicializar el flight controller!");
        while (1) {
            digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
            delay(200);
        }
    }
    
    // Inicializar keyboard controller
    keyboardController.init();
    
    Serial.println("\nSistema listo! Usa 'H' para ver la ayuda.");
    Serial.println("IMPORTANTE: Mantén el throttle en 0% antes de armar!");
    Serial.println("========================================\n");
    
    lastLoopTime = micros();
}

void loop() {
    unsigned long currentTime = micros();
    
    // Control de frecuencia del loop principal
    if (currentTime - lastLoopTime >= LOOP_TIME_US) {
        
        // Procesar entrada del teclado
        if (Serial.available()) {
            char key = Serial.read();
            if (keyboardController.processInput(key)) {
                // Actualizar inputs del flight controller
                ControlInputs inputs = keyboardController.getInputs();
                flightController.setInputs(inputs);
            }
        }
        
        // Actualizar flight controller
        flightController.update();
        
        // Actualizar keyboard controller (para mostrar estado)
        keyboardController.update(flightController.getFlightData());
        
        // Verificar si hay problemas de timing
        unsigned long loopTime = micros() - currentTime;
        if (loopTime > LOOP_TIME_US * 1.2) {  // Si el loop toma más del 120% del tiempo esperado
            Serial.printf("ADVERTENCIA: Loop lento! %lu us (esperado: %lu us)\n", 
                         loopTime, LOOP_TIME_US);
        }
        
        lastLoopTime = currentTime;
        loopCounter++;
        
        // Mostrar estadísticas cada 10 segundos
        if (loopCounter % (LOOP_FREQUENCY * 10) == 0) {
            float avgLoopTime = loopTime / 1000.0;
            Serial.printf("Estadísticas: Loop promedio: %.2f ms | Frecuencia: %d Hz\n", 
                         avgLoopTime, LOOP_FREQUENCY);
        }
    }
    
    // Verificación de seguridad adicional
    static unsigned long lastSafetyCheck = 0;
    if (currentTime - lastSafetyCheck > 1000000) {  // Cada segundo
        
        // Si no hay comunicación serial por mucho tiempo y está armado, desarmar
        static unsigned long lastSerialActivity = 0;
        if (Serial.available()) {
            lastSerialActivity = currentTime;
        }
        
        if (flightController.isArmed() && 
            (currentTime - lastSerialActivity > 10000000)) {  // 10 segundos sin actividad
            Serial.println("ADVERTENCIA: Sin actividad serial - desarmando por seguridad");
            flightController.emergencyStop();
        }
        
        lastSafetyCheck = currentTime;
    }
}