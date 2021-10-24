#include "debouncedbutton.h"

DebouncedButton::DebouncedButton(int buttonInputPin, unsigned long debounceDelay){
    buttonPin = buttonInputPin;
    debounceDly = debounceDelay;
}

uint8_t DebouncedButton::getPinnr(){
    return buttonPin;
}

int DebouncedButton::readButton()
    {
        buttonReading = digitalRead(buttonPin);
 
        // Reset the timer if the state changes
        if (buttonReading != prevButtonReading)
            lastReadTime = millis();
 
        // If this state has lasted longer than the debounce delay parameter,
        // actually count it as a reading
        if ((millis() - lastReadTime) > debounceDly)
        {
            if (buttonReading != buttonState)
                buttonState = buttonReading;
 
            prevButtonState = buttonState;
        }
        prevButtonReading = buttonReading;
        return buttonState;
    }
