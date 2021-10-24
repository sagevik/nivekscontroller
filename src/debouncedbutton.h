/*
Button class
Used for debouncing buttons to avoid double triggers and noise

debounceDelay is in milliseconds
*/

#include <Arduino.h>

class DebouncedButton
{
public:
    DebouncedButton(int buttonInputPin, unsigned long debounceDelay);
    uint8_t getPinnr();
    int readButton();
 
private:
    unsigned long lastReadTime;
    int buttonReading;
    int prevButtonReading;
    int buttonState;
    int prevButtonState;
    int buttonPin;
    unsigned long debounceDly;
};
