#include <Arduino.h>

#include "MIDIUSB.h"
#include "debouncedbutton.h"

// Midi mapping
int midiChannel = 16;

// MidiCC values                // If function button active, this set is used for midi note nr on triggerbuttons
int yellowButtonMidi = 102;     //yellowButtonMidi = 36;
int redButtonMidi = 103;        //redButtonMidi = 37;
int blueButtonMidi = 104;       //blueButtonMidi = 38;
int greenButtonMidi = 105;      //greenButtonMidi = 39;
int pot1MidiCC = 106;
int pot2MidiCC = 107;
int pot3MidiCC = 108;
int touchSliderMidiCC = 109;

// Debug for func button
int fsLast = 0;

// Max/Min values for potensiometer inputs
const int potInputMin = 0;
const int potInputMax = 1023;

// Max/Min values for button inputs
const int buttonInputMin = 0;
const int buttonInputMax = 1;

// Max/Min values for midi data
const int midiMin = 0;
const int midiMax = 127;

//Potentiometer input pins
const int pot1 = A3;
const int pot2 = A2;
const int pot3 = A1;
const int tSlider = A0;

// ---- Function button constants and variables ----
// Function button LED output pin
const int funcLed = 10;
// Function button input pin
const int funcButton = 6;
// Function button state
bool funcButtonActive = false;
// Function button values {previous, current}
int funcVals[] = {0, 0};
// Function button LED blink timer variables
const int funcLedBlinkInterval = 250;
unsigned long lastBlink = 0;
int ledState = HIGH;

//Trigger button input pins
const int yellowInputPin = 2;
const int redInputPin = 3;
const int blueInputPin = 4;
const int greenInputPin = 5;

// Create debounced buttons from the Button class (function button not debounced)
unsigned long buttonDebounceDelayMS = 20;
DebouncedButton yellowButton = DebouncedButton(yellowInputPin, buttonDebounceDelayMS);
DebouncedButton redButton = DebouncedButton(redInputPin, buttonDebounceDelayMS);
DebouncedButton blueButton = DebouncedButton(blueInputPin, buttonDebounceDelayMS);
DebouncedButton greenButton = DebouncedButton(greenInputPin, buttonDebounceDelayMS);

// All button pins and read values
const int numberOfButtons = 4;
const int buttonData = 3;
DebouncedButton buttons[numberOfButtons] = {yellowButton, redButton, blueButton, greenButton};
int btVals[numberOfButtons][buttonData] = {{0, 0, yellowButtonMidi},
                                           {0, 0, redButtonMidi},
                                           {0, 0, blueButtonMidi},
                                           {0, 0, greenButtonMidi}
                                          };

// All potentiometers and read values
const int numberOfPots = 4;
const int potData = 3;
int pots[numberOfPots] = {pot1, pot2, pot3, tSlider};
int potVals[numberOfPots][potData] = {{0, 0, pot1MidiCC},
                                      {0, 0, pot2MidiCC},
                                      {0, 0, pot3MidiCC},
                                      {0, 0, touchSliderMidiCC}
                                     };



void noteOn(byte channel, byte pitch, byte velocity){
    midiEventPacket_t noteOn = {0x09, 0x90 | (channel-1), pitch, velocity};
    MidiUSB.sendMIDI(noteOn);
    MidiUSB.flush();
}

void noteOff(byte channel, byte pitch, byte velocity){
    midiEventPacket_t noteOff = {0x08, 0x80 | (channel-1), pitch, velocity};
    MidiUSB.sendMIDI(noteOff);
    MidiUSB.flush();
}

void controlChange(byte channel, byte control, byte value){
    midiEventPacket_t event = {0x0B, 0xB0 | (channel-1), control, value};
    MidiUSB.sendMIDI(event);
    MidiUSB.flush();
}

// Because INPUT_PULLUP is used when initializing buttons, the button input
// will be HIGH when buttons are NOT pushed
// Buttons scaled "in reverse" to operate more sensible (on when pushed)
int scaleButtonValue(int i){
    return map(i, buttonInputMin, buttonInputMax, midiMax, midiMin);
}

int scalePotValue(int i){
    return map(i, potInputMin, potInputMax, midiMin, midiMax);
}

// --- Func button ---
void printFuncStatus(int state){
    if(state == 1){
        Serial.println("Func button ACTIVATED");
    } else {
        Serial.println("Func button DEACTIVATED");
    }
}

// Switch between Midi Note nr and Midi CC nr for buttons
void setButtonsMidiMapping(int set){
    if(set == 0){
        // set == 0 used for Midi CC
        btVals[0][2] = 102; // yellowButtonMidi CC
        btVals[1][2] = 103; // redButtonMidi CC
        btVals[2][2] = 104; // blueButtonMidi CC
        btVals[3][2] = 105; // greenButtonMidi CC
    } 
    if(set == 1) {
        // set == 1 used for Midi Note numbers
        btVals[0][2] = 36; // yellowButtonMidi note number
        btVals[1][2] = 37; // redButtonMidi note number
        btVals[2][2] = 38; // blueButtonMidi note number
        btVals[3][2] = 39; // greenButtonMidi note number
    }
}

void checkFuncButton(){
    // Read func button
    // change midi mapping according to func button state
    int fBt = digitalRead(funcButton);
    funcButtonActive = fBt;
    if (fBt != fsLast){
        fsLast = fBt;
        printFuncStatus(fBt);
    }
    if (fBt == 1){
        //Func Button ACTIVE
        setButtonsMidiMapping(1);
        //Timer for blinking LED when func button is active
        if (millis()-lastBlink >= funcLedBlinkInterval){
            lastBlink = millis();
            digitalWrite(funcLed, ledState);
            ledState = !ledState;
        }
    } 
    if (fBt == 0){
        //Func Button DISABLED
        setButtonsMidiMapping(0);
        digitalWrite(funcLed, LOW);
    }
}

// --- Button functions ---
void readButtons(){
    // loop through all buttons and read their values

    // read function button (not debounced)
    funcVals[1] = digitalRead(funcButton);

    // read debounced trigger buttons and scale to min/max midi values
    for (int i = 0; i < numberOfButtons; i++){
        int bt = buttons[i].readButton();
        btVals[i][1] = scaleButtonValue(bt);
    }
}

void updateButtonVals(int i){
    // update previous button value with new/current value for button nr i
    btVals[i][0] = btVals[i][1];
}

void sendAndUpdateButtonsData(){
    // loop through all buttons, send midi data and update button values

    // if function button active, send Midi Notes for triggerbuttons
    if(funcButtonActive){
        for(int i=0; i < numberOfButtons; i++){
            if(btVals[i][0] != btVals[i][1]){
                // send note on if button pressed
                // note off if button not pressed
                int velocity = (potVals[3][1] == 0) ? 127 : potVals[3][1];
                if(btVals[i][1]){
                    noteOn(midiChannel, btVals[i][2], velocity);
                } else {
                    noteOff(midiChannel, btVals[i][2], velocity);
                }
                updateButtonVals(i);
            }
        }
    }
    // if function button inactive, send Midi CC for triggerbuttons
    if(!funcButtonActive){
        for (int i = 0; i < numberOfButtons; i++){
            if(btVals[i][0] != btVals[i][1]){
                //send mididata and update btVals
                controlChange(midiChannel, btVals[i][2], btVals[i][1]);
                updateButtonVals(i);
            }
        }
    }
}

// Potentiometer functions
void readPots(){
    // loop through all potentiometers and read their values
    for (int i = 0; i < numberOfPots; i++){
        int pot = analogRead(pots[i]);
        potVals[i][1] = scalePotValue(pot);
    }
}

void updatePotVals(int i){
    // update previous potentiometer value with new/current value for potentiometer nr i
    potVals[i][0] = potVals[i][1];
}

void sendAndUpdatePotsData(){
    // If function button is active, do not send Midi CC for touch slider (used for velocity
    // when trigggering notes with buttons)
    int tSlider = 0;
    if(funcButtonActive){
        tSlider = 1;
    }
    for (int i = 0; i < numberOfPots-tSlider; i++){
        if(potVals[i][0] != potVals[i][1]){
            //send mididata and update potVals
            controlChange(midiChannel, potVals[i][2], potVals[i][1]);
            updatePotVals(i);
        }
    }
}

void setup(){
    // Serial.begin(9600);

    // Initialize func button
    pinMode(funcButton, INPUT_PULLUP);

    // Initialize func led
    pinMode(funcLed, OUTPUT);

    // Initialize buttons
    for (int i = 0; i < numberOfButtons; i++){
        pinMode(buttons[i].getPinnr(), INPUT_PULLUP);
    }
    // Initialize pots
    for (int i = 0; i < numberOfPots; i++){
        pinMode(pots[i], INPUT);
    }
}

void loop(){
    checkFuncButton();
    readButtons();
    readPots();
    sendAndUpdateButtonsData();
    sendAndUpdatePotsData();
}
