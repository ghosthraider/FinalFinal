#ifndef KEYBOARD_CONTROLLER_H
#define KEYBOARD_CONTROLLER_H

#include "FlightController.h"

class KeyboardController {
private:
    ControlInputs currentInputs;
    bool helpShown;
    
    void showHelp();
    void showStatus(const FlightData& data);
    
public:
    KeyboardController();
    void init();
    bool processInput(char key);
    ControlInputs getInputs() const { return currentInputs; }
    void update(const FlightData& flightData);
};

#endif