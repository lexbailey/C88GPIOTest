#include <shift_74594.h>

#include <shift_74597.h>

#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

#define INPUTSIGN (A2)
#define OUTPUTSIGN (A3)
#define MODE1 (A0)
#define MODE2 (A1)
#define SPEAK (A4)

#define ADC (5)

#define ROTA (2)
#define ROTB (3)

#define MODE_ADC (3)
#define MODE_SWITCHES (1)
#define MODE_ROTARY (2)

#define ROTARY_DELAY (1)

//Pin order: QH, SCK, RCK, SLOAD, SCLR
#define INPUT_SHIFT_PINS 13,11,10,9,12

//Pin order: SER, SCK, RCK, SCLR, RCLR
#define OUTPUT_SHIFT_PINS 8,6,5,7,4

Adafruit_7segment matrixOut = Adafruit_7segment();
Adafruit_7segment matrixIn = Adafruit_7segment();
shift_74597 inputShifter = shift_74597(INPUT_SHIFT_PINS);
shift_74594 outputShifter = shift_74594(OUTPUT_SHIFT_PINS);

void setup() {
  //Serial.begin(9600);
  //Serial.println("7 Segment Backpack Test");
  
  pinMode(INPUTSIGN, INPUT_PULLUP);
  pinMode(OUTPUTSIGN, INPUT_PULLUP);
  pinMode(ROTA, INPUT_PULLUP);
  pinMode(ROTB, INPUT_PULLUP);
  pinMode(MODE1, INPUT_PULLUP);
  pinMode(MODE2, INPUT_PULLUP);
  
  matrixOut.begin(0x70);
  matrixIn.begin(0x71);
  inputShifter.init();
  outputShifter.init();
}

int twosComp(int in, int bits){
  int signBit = 1 << (bits-1);
  if (in & signBit){ return in - (2*signBit); }
  return in;
}

void writeSevenSegs(int in, int out){
  int inSign = digitalRead(INPUTSIGN);
  int outSign = digitalRead(OUTPUTSIGN);
  int inCount, outCount;
  if (inSign == LOW){ inCount = twosComp(in, 8); } else {inCount = in; }
  if (outSign == LOW){ outCount = twosComp(out, 8); } else {outCount = out; }
  matrixOut.println(outCount);
  matrixIn.println(inCount);
  matrixOut.writeDisplay();
  matrixIn.writeDisplay();
}

int lastRotB = 0;
int lastRotA = 0;

unsigned long lastTimeRotary = millis();

int counter = 0;

void loop() {
  int mode;
  inputShifter.clear();
  inputShifter.load();
  char outputData = inputShifter.getByte();
  char switchData = ~inputShifter.getByteReverse();
  char inputPortData = inputShifter.getByte();
  int mode1 = digitalRead(MODE1);
  int mode2 = digitalRead(MODE2);
  
  if (mode2 == LOW) { mode = MODE_ADC; }
  else{if (mode1 == LOW) { mode = MODE_SWITCHES; }
  else{ mode = MODE_ROTARY; }}
  
  if ((mode == MODE_ROTARY) && (millis() - ROTARY_DELAY > lastTimeRotary)){
    int rotA = digitalRead(ROTA);
    int rotB = digitalRead(ROTB);
    if (rotA == HIGH && lastRotA == LOW){ //Rising edge rotA
      if (rotB == LOW) {counter ++;} else {counter--;}
    }
    if (rotA == LOW && lastRotA == HIGH){ //Falling edge rotA
      if (rotB == HIGH) {counter ++;} else {counter--;}
    }
    if (rotB == HIGH && lastRotB == LOW){ //Rising edge rotB
      if (rotA == HIGH) {counter ++;} else {counter--;}
    }
    if (rotB == LOW && lastRotB == HIGH){ //Falling edge rotB
      if (rotA == LOW) {counter ++;} else {counter--;}
    }
    lastRotB = rotB;
    lastRotA = rotA;
    if (counter < 0) {counter = 1023;}
    if (counter > 1023) {counter = 0;}
    lastTimeRotary = millis();
  }

  int inputData;
  switch(mode){
    case MODE_ADC:
      inputData = analogRead(ADC) >> 2;
      break;
    case MODE_ROTARY:
      inputData = counter >> 2;
      break;
    case MODE_SWITCHES:
      inputData = switchData;
      break;
  }  
  writeSevenSegs(inputData, outputData);
  outputShifter.setByte(inputData);
  outputShifter.latch();
  
}
