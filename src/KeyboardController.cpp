#include "KeyboardController.h"

KeyboardController::KeyboardController() : helpShown(false) {
    memset(&currentInputs, 0, sizeof(currentInputs));
}

void KeyboardController::init() {
    Serial.println("\n=== CONTROLADOR DE VUELO ESP32 ===");
    showHelp();
}

void KeyboardController::showHelp() {
    Serial.println("\n--- CONTROLES DEL TECLADO ---");
    Serial.println("MOVIMIENTO:");
    Serial.println("  W/S  - Pitch (adelante/atrás)");
    Serial.println("  A/D  - Roll (izquierda/derecha)");
    Serial.println("  Q/E  - Yaw (rotar izq/der)");
    Serial.println("  I/K  - Throttle (subir/bajar)");
    Serial.println("");
    Serial.println("ARMADO:");
    Serial.println("  R    - Armar motores");
    Serial.println("  T    - Desarmar motores");
    Serial.println("  X    - Parada de emergencia");
    Serial.println("");
    Serial.println("OTROS:");
    Serial.println("  H    - Mostrar esta ayuda");
    Serial.println("  C    - Centrar controles");
    Serial.println("  Z    - Mostrar estado");
    Serial.println("");
    Serial.println("Incrementos: ±5% por pulsación");
    Serial.println("ADVERTENCIA: ¡Mantén el throttle bajo para armar!");
    Serial.println("==============================\n");
    helpShown = true;
}

bool KeyboardController::processInput(char key) {
    bool inputChanged = false;
    
    switch (key) {
        // Throttle
        case 'i':
        case 'I':
            currentInputs.throttle += 5;
            if (currentInputs.throttle > 100) currentInputs.throttle = 100;
            Serial.printf("Throttle: %.0f%%\n", currentInputs.throttle);
            inputChanged = true;
            break;
            
        case 'k':
        case 'K':
            currentInputs.throttle -= 5;
            if (currentInputs.throttle < 0) currentInputs.throttle = 0;
            Serial.printf("Throttle: %.0f%%\n", currentInputs.throttle);
            inputChanged = true;
            break;
            
        // Roll (A/D)
        case 'a':
        case 'A':
            currentInputs.rollCmd -= 10;
            if (currentInputs.rollCmd < -100) currentInputs.rollCmd = -100;
            Serial.printf("Roll: %.0f%%\n", currentInputs.rollCmd);
            inputChanged = true;
            break;
            
        case 'd':
        case 'D':
            currentInputs.rollCmd += 10;
            if (currentInputs.rollCmd > 100) currentInputs.rollCmd = 100;
            Serial.printf("Roll: %.0f%%\n", currentInputs.rollCmd);
            inputChanged = true;
            break;
            
        // Pitch (W/S)
        case 'w':
        case 'W':
            currentInputs.pitchCmd += 10;
            if (currentInputs.pitchCmd > 100) currentInputs.pitchCmd = 100;
            Serial.printf("Pitch: %.0f%%\n", currentInputs.pitchCmd);
            inputChanged = true;
            break;
            
        case 's':
        case 'S':
            currentInputs.pitchCmd -= 10;
            if (currentInputs.pitchCmd < -100) currentInputs.pitchCmd = -100;
            Serial.printf("Pitch: %.0f%%\n", currentInputs.pitchCmd);
            inputChanged = true;
            break;
            
        // Yaw (Q/E)
        case 'q':
        case 'Q':
            currentInputs.yawCmd -= 10;
            if (currentInputs.yawCmd < -100) currentInputs.yawCmd = -100;
            Serial.printf("Yaw: %.0f%%\n", currentInputs.yawCmd);
            inputChanged = true;
            break;
            
        case 'e':
        case 'E':
            currentInputs.yawCmd += 10;
            if (currentInputs.yawCmd > 100) currentInputs.yawCmd = 100;
            Serial.printf("Yaw: %.0f%%\n", currentInputs.yawCmd);
            inputChanged = true;
            break;
            
        // Armado/Desarmado
        case 'r':
        case 'R':
            currentInputs.armCmd = true;
            currentInputs.disarmCmd = false;
            Serial.printf("Comando: ARMAR (Throttle: %.1f%%)\n", currentInputs.throttle);
            inputChanged = true;
            break;
            
        case 't':
        case 'T':
            currentInputs.disarmCmd = true;
            currentInputs.armCmd = false;
            Serial.println("Comando: DESARMAR");
            inputChanged = true;
            break;
            
        // Centrar controles
        case 'c':
        case 'C':
            currentInputs.rollCmd = 0;
            currentInputs.pitchCmd = 0;
            currentInputs.yawCmd = 0;
            Serial.println("Controles centrados");
            inputChanged = true;
            break;
            
        // Parada de emergencia
        case 'x':
        case 'X':
            currentInputs.throttle = 0;
            currentInputs.rollCmd = 0;
            currentInputs.pitchCmd = 0;
            currentInputs.yawCmd = 0;
            currentInputs.disarmCmd = true;
            currentInputs.armCmd = false;
            Serial.println("¡PARADA DE EMERGENCIA!");
            inputChanged = true;
            break;
            
        // Ayuda
        case 'h':
        case 'H':
            showHelp();
            break;
            
        // Estado
        case 'z':
        case 'Z':
            Serial.printf("Estado actual - T:%.0f%% R:%.0f%% P:%.0f%% Y:%.0f%%\n",
                         currentInputs.throttle, currentInputs.rollCmd, 
                         currentInputs.pitchCmd, currentInputs.yawCmd);
            break;
            
        default:
            // Tecla no reconocida
            break;
    }
    
    // Resetear comandos de armado después de procesarlos
    if (inputChanged && (currentInputs.armCmd || currentInputs.disarmCmd)) {
        // Los comandos se procesan en el siguiente ciclo
        // NO los reseteamos aquí - se resetean en el FlightController
    }
    
    return inputChanged;
}

void KeyboardController::showStatus(const FlightData& data) {
    Serial.printf("Estado: %s | R:%.1f° P:%.1f° Y:%.1f° | RRate:%.1f°/s PRate:%.1f°/s YRate:%.1f°/s\n",
                  data.armed ? "ARMADO" : "DESARMADO",
                  data.roll, data.pitch, data.yaw,
                  data.rollRate, data.pitchRate, data.yawRate);
}

void KeyboardController::update(const FlightData& flightData) {
    static unsigned long lastStatusTime = 0;
    unsigned long currentTime = millis();
    
    // Mostrar estado cada 2 segundos
    if (currentTime - lastStatusTime > 2000) {
        showStatus(flightData);
        lastStatusTime = currentTime;
    }
}