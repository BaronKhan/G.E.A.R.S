/** ROBOT SKETCH v1.1
 *  BUILD NAME: Globe-Exploring Artificial Robot Son (G.E.A.R.S)
 *  Build Started - 27/07/15
 *  Build v1.0 Finished - 12/08/15
 *  Functions:
 *  - Moves forward, backwards, left and right.
 *  - Green and Red LED to determine whether it is moving.
 *  - Uses distance sensor to detect objects in front.
 *  - Illuminates in the dark with amber LED (pointing forward).
 *  - Speaks using Piezo Speaker.
 *  - Voice and Remote controlled.
 */

/** PIN SETUP **
 *  Distance Sensor:
 *    Trig - 7
 *    Echo - 4
 *  Motor Channel A:
 *    Direction - 12
 *    Speed (PWM) - 3
 *    Brake - 9
 *    Current Sensing - A0
 *  Motor Channel B:
 *    Direction - 13
 *    Speed (PWM) - 11
 *    Brake - 8
 *    Current Sensing - A1
 *  Random Number Generator:
 *    A0
 *  LDR (for Illumination):
 *    Side 1 - GND
 *    Side 2 - A3 and 100k resistor (brown, black, orange)
 *    (Connect other side of resistor to 5V)
 *  Amber LED (for Illumination):
 *    Side 1 (shorter) - GND 
 *    Side 2 (PWM) - 220 Ohm resistor (red, red, brown) to 10
 *  Piezo Speaker:
 *    Plus Side - 2
 *    Minus Side - GND (This is a YELLOW jumper)
 *  Mode Select:
 *    Mode LED - 6 (longer side) to GND
 *    Mode Button Side 1 - 5V
 *    Mode Button Side 2 - 5 and 10k resistor (brown, black, orange) to GND
 *  Braking and Reversing LEDs:
 *    Brake LED - 9 (longer) to GND
 *    Reverse LED - 5 (longer) to GND - REMOVED
 *  Bluetooth Remote Control:
 *    RXD (on outer edge) - 19 (TX)
 *    TXD - 18 (RX)
 */
#include "pitches.h"
#include "NewPing.h"
#include <SoftwareSerial.h>

//Pins
const int randomPin = A2;

const int torchLDRPin = A3;
const int torchPin = 10;

const int trigPin = 2;
const int echoPin = 4;

const int motorADir = 12;
const int motorASpd = 3;
const int motorABrake = 9;
const int motorACurrent = A0;
const int motorBDir = 13;
const int motorBSpd = 11;
const int motorBBrake = 8;
const int motorBCurrent = A1;

const int speakerPin = 7;

const int modeLEDPin = 6;
const int modeButtonPin = 5;

//REMOVED
//const int reverseLEDPin = 5;

const int btRXPin = 18;
const int btTXPin = 19;


//Constants
const int motorA = 0;
const int motorB = 1;
const int stateIdle = 0;
const int stateMoving = 1;
const int stateReversing = 2;
const int modeAuto = 0;
const int modeManual = 1;
const int maxDistance = 200;

const int keyNone = 0;
const int keyUp = 1;
const int keyDown = 2;
const int keyLeft = 3;
const int keyRight = 4;
const int keyGrab = 5;
const int keyRelease = 6;
const int keyRotateRight = 7;
const int keyRotateLeft = 8;
const int keyHello = 9;
const int keyHowAreYou = 10;
const int keyHelloHowAreYou = 11;
const int keyMilkshake = 12;
const int keyFaster = 13;
const int keySlower = 14;
const int keySing = 15;
const int keyName = 16;
const int keyDontKnow = 17;

int maxSpeed = 255;
int currentASpeed = 0;
int currentBSpeed = 0;
int tempSpeed = 0;

int randMotor; //Used to randomise motors
int reverseNum = 0; //Used to randomise reverse direction
int reverseRandNum = 0;

int state = 0; //0=idle, 1=moving, 2=reversing
int mode = 0; //0=auto, 1=manual (remote control)

// For Mode Change:
int modePushCounter = 0;   // counter for the number of button presses
int modeButtonState = 0;         // current state of the button
int modeLastButtonState = 0;     // previous state of the button


//Alarms
int alarm[] = {-1, -1, -1, -1, -1, -1};
const int alarmCount = 6;

//Notes in the start-up voice:
int startUp[] = {
  NOTE_A3, NOTE_G3, NOTE_E3, NOTE_C3, NOTE_D3, NOTE_B3,
  NOTE_F3, NOTE_C4
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int startUpDurations[] = {
  8, 8, 8, 8, 8, 8, 8, 8
};
int startUpLength = 8;


//Notes in the hello voice:
int hello[] = {
  NOTE_A3, NOTE_G3
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int helloDurations[] = {
  4, 4
};
int helloLength = 2;

//Notes in how are you voice:
int howAreYou[] = {
  NOTE_E3, NOTE_E3, NOTE_E3
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int howAreYouDurations[] = {
  4, 4, 4
};
int howAreYouLength = 3;

//Notes in milkshake voice:
int milkshake[] = {
  NOTE_F2, NOTE_E2, NOTE_F2, NOTE_E2
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int milkshakeDurations[] = {
  3, 3, 3, 3
};
int milkshakeLength = 4;

int good[] = {
  NOTE_B3
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int goodDurations[] = {
  4
};
int goodLength = 1;

int dontKnow[] = {
  NOTE_D3, NOTE_A4, NOTE_A3
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int dontKnowDurations[] = {
  4,4,4
};
int dontKnowLength = 3;

int robotName[] = {
  NOTE_D4
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int robotNameDurations[] = {
  2
};
int robotNameLength = 1;

NewPing DistanceSensor(trigPin, echoPin, 200);
unsigned int cm;
unsigned int backDistance;
unsigned int frontDistance;
int distanceCount = 0;

//Change BT Pins
SoftwareSerial BTSerial(btRXPin, btTXPin);

//Initialise Remote Control
int key = 0;
int lastKey;
char ch;

void setup() {
  // put your setup code here, to run once:
  pinMode(torchLDRPin, INPUT);
  pinMode(torchPin, OUTPUT);
  pinMode(motorADir, OUTPUT);
  pinMode(motorASpd, OUTPUT);
  pinMode(motorABrake, OUTPUT);
  pinMode(motorACurrent, INPUT);
  pinMode(motorBDir, OUTPUT);
  pinMode(motorBSpd, OUTPUT);
  pinMode(motorBBrake, OUTPUT);
  pinMode(motorBCurrent, INPUT);
  pinMode(modeButtonPin, INPUT);
  pinMode(modeLEDPin, OUTPUT);
  //pinMode(reverseLEDPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  randomSeed(analogRead(randomPin));
  //startUpSound();
  helloSound();
  //howAreYouSound();
  //milkshakeSound();
  //goodSound();
  //dontKnowSound();
  //robotNameSound();
  Serial.begin(9600);
  BTSerial.begin(9600);
  distanceCount = 0;
  
  state = stateMoving;
  alarm[1] = 1;
}

void loop() {
  // put your main code here, to run repeatedly:
  cm = DistanceSensor.ping_cm();
  //Serial.println(cm);
  /*if (cm < 6 && cm != 0 && state == stateMoving) {
    distanceCount++;
  }
  else {
    distanceCount = 0;
  }*/
  int currentMode = mode;
  updateAlarms();
  activateTorch();
  updateMode(0);
  
  //reset if mode has changed
  if (currentMode != mode) {
    if (mode == modeAuto) {
      state = stateMoving;
      resetAlarms();
      alarm[1] = 300;
    }
    else if (mode == modeManual) {
      state = stateIdle;
      resetAlarms();
      setMotorBrake(motorA, LOW);
      setMotorBrake(motorB, LOW);
      setMotorSpd(motorA, 0);
      setMotorSpd(motorB, 0);
      setMotorDir(motorA, HIGH);
      setMotorDir(motorB, HIGH);
    }
    currentMode = mode;
  }
  


  
  if (mode == modeAuto) {  
    //if (cm < 60) {
      //checkBehind();
      //checkFront();
    //}
    //Serial.println(frontDistance);
    //Check to see if reversing is needed
    //if (cm < (6*(float(maxSpeed/128))) && cm != 0 && state == stateMoving) {
    if (cm < 6 && cm != 0 && state == stateMoving) {
    //if (distanceCount > 8 && state == stateMoving) {
      state = stateIdle;
      alarm[3] = 200;
      //distanceCount = 0;
    }
    //Check to grab bluetooth remote control
    grabRemote();
  }
  else {
    //Check for commands
    remoteControl();
    //Serial.println(cm);
    //if (cm < (6*(float(maxSpeed/128))) && cm != 0 && state == stateMoving) {
    if (cm < 6 && cm != 0 && state == stateMoving) {
    //if (distanceCount > 8 && state == stateMoving) {
      goIdle();
      distanceCount = 0;
    }
  }

  //reset if mode has changed
  if (currentMode != mode) {
    if (mode == modeAuto) {
      state = stateMoving;
      resetAlarms();
      alarm[1] = 300;
    }
    else if (mode == modeManual) {
      state = stateIdle;
      resetAlarms();
      setMotorBrake(motorA, LOW);
      setMotorBrake(motorB, LOW);
      setMotorSpd(motorA, 0);
      setMotorSpd(motorB, 0);
      setMotorDir(motorA, HIGH);
      setMotorDir(motorB, HIGH);
    }
    currentMode = mode;
  }
  
  //Set LED Brake and Reverse Lights
  if (state == stateIdle) {
    setMotorBrake(motorA, HIGH);
    setMotorBrake(motorB, HIGH);
  }
  //else {
    //setMotorBrake(motorA, LOW);
    //setMotorBrake(motorB, LOW);
  //}

  //Update the max speed of the motors
  updateMaxSpeed();

  //Serial.println(mode);
  //Serial.println(state);
  
  /*Reverse Light - REMOVED
  if (state == stateReversing)
    analogWrite(reverseLEDPin, 255);
  else
    analogWrite(reverseLEDPin, 0);
  */
}

void updateAlarms() {
  for (int i=0; i < alarmCount; i++) {
    if (alarm[i] > -1) {
      alarm[i]--;
      if (alarm[i] == 0){
        switch (i) {
          case 0: //Alarm 0 Event: Stop Reversing
          state = stateIdle;
          setMotorBrake(motorA, HIGH);
          setMotorBrake(motorB, HIGH);
          alarm[1] = 100;
          break;
          case 1: //Move Forwards at Full Speed
          goForwards();
          alarm[2] = random(300, 800);
          break;
          case 2: //Change Direction (only if moving forwards)
          if (state == stateMoving) {
            randomiseMotorSpd(motorA);
            randomiseMotorSpd(motorB);
          }
          alarm[2] = random(300, 1000);
          break;
          case 3: //Start Reversing
          setReverse();
          //alarm[0] = int(1200*(128.000/(float(maxSpeed))));
          alarm[0] = random(1000,1500);
          break;
          case 4:
          /*if (cm < 60 && abs(backDistance - cm) < 3 && state == stateReversing) {
            alarm[0] = 1;
          }*/
          break;
          case 5:
          /*if (cm < 60 && abs(frontDistance - cm) < 3 && state == stateMoving) {
            state = stateIdle;
            alarm[3] = 100;
          }*/
          break;
          default:
          break;
        }
      }
    }
  }
}

void resetAlarms(){
  for (int i=0; i<alarmCount; i++) {
    alarm[i] = -1;
  }
}

void activateTorch() {
  int ldrReading = analogRead(torchLDRPin);
  int outputValue = map(ldrReading, 600, 900, 0, 255);
  outputValue = constrain(outputValue, 0, 255);
  analogWrite(torchPin, outputValue);
}

void setMotorSpd(int motor, int spd) {
  spd = constrain(spd, 0, 255);
  //spd = map(spd, 0, 255, 128, 255); 
  if (motor == motorA) {
    if (spd < 100) {
      setMotorBrake(motorA, HIGH);
    }
    else {
      analogWrite(motorASpd, spd);
    }
    currentASpeed = spd;
  }
  else {
    if (spd < 100) {
      setMotorBrake(motorB, HIGH);
    }
    else {
      analogWrite(motorBSpd, spd);
    }
    currentBSpeed = spd;
  }
  //Serial.println(spd);
}

void setMotorDir(int motor, int dir) {
  if (motor == motorA) {
    digitalWrite(motorADir, dir);
  }
  else {
    digitalWrite(motorBDir, dir);
  }
}

void setMotorBrake(int motor, int brake) {
  if (motor == motorA) {
    digitalWrite(motorABrake, brake);
  }
  else {
    digitalWrite(motorBBrake, brake);
  }
}

void randomiseMotorSpd(int motor) {
  randMotor = random(maxSpeed/2, maxSpeed);
  //Serial.println(randMotor);
  setMotorSpd(motor, randMotor);
}

void startUpSound() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < startUpLength; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / startUpDurations[thisNote];
    tone(speakerPin, startUp[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void helloSound() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < helloLength; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / helloDurations[thisNote];
    tone(speakerPin, hello[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void howAreYouSound() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < howAreYouLength; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / howAreYouDurations[thisNote];
    tone(speakerPin, howAreYou[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void milkshakeSound() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < milkshakeLength; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / milkshakeDurations[thisNote];
    tone(speakerPin, milkshake[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void goodSound() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < goodLength; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / goodDurations[thisNote];
    tone(speakerPin, good[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void dontKnowSound() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < dontKnowLength; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / dontKnowDurations[thisNote];
    tone(speakerPin, dontKnow[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void robotNameSound() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < robotNameLength; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / robotNameDurations[thisNote];
    tone(speakerPin, robotName[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speakerPin);
  }
}

void setReverse() {
  setMotorBrake(motorA, LOW);
  setMotorBrake(motorB, LOW);
  state = stateReversing;
  //reverseRandNum = random(1,3);
  //if (reverseRandNum < 2) {
    reverseNum++;
  //}
  if (reverseNum > 4)
    reverseNum = 1;
  if (reverseNum == 3 || reverseNum == 4) {
    //Reverse Right
    setMotorDir(motorA, LOW);
    setMotorDir(motorB, LOW);
    //setMotorBrake(motorB, HIGH);
    setMotorSpd(motorB, maxSpeed/2);
    setMotorSpd(motorA, maxSpeed);
  }
  else {
    //Reverse Left
    setMotorDir(motorA, LOW);
    setMotorDir(motorB, LOW);
    //setMotorBrake(motorA, HIGH);
    setMotorSpd(motorA, maxSpeed/2);
    setMotorSpd(motorB, maxSpeed);
  }
}

void updateMode(bool change) {
  if (change == 1) {
    if (modeButtonState == LOW)
      modeButtonState = HIGH;
    else
      modeButtonState = LOW;
  }
  else {
    // read the pushbutton input pin:
    modeButtonState = digitalRead(modeButtonPin);
  }

  // compare the buttonState to its previous state
  if (modeButtonState != modeLastButtonState) {
    // if the state has changed, increment the counter
    if (modeButtonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      modePushCounter++;
    }
    //Serial.println(mode);
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state,
  //for next time through the loop
  modeLastButtonState = modeButtonState;


  // turns on the LED every two button pushes by
  // checking the modulo of the button push counter.
  // the modulo function gives you the remainder of
  // the division of two numbers:
  if (modePushCounter % 2 == 0) {
    digitalWrite(modeLEDPin, LOW);
    mode = modeAuto;
  } else {
    digitalWrite(modeLEDPin, HIGH);
    mode = modeManual;
  }
}

void goForwards() {
  state = stateMoving;
  setMotorDir(motorA, HIGH);
  setMotorDir(motorB, HIGH);
  setMotorSpd(motorA, maxSpeed);
  setMotorSpd(motorB, maxSpeed);
  setMotorBrake(motorA, LOW);
  setMotorBrake(motorB, LOW);
  Serial.println("going forward");
}

void goBackwards() {
  state = stateReversing;
  setMotorDir(motorA, LOW);
  setMotorDir(motorB, LOW);
  setMotorSpd(motorA, maxSpeed);
  setMotorSpd(motorB, maxSpeed);
  setMotorBrake(motorA, LOW);
  setMotorBrake(motorB, LOW);
}

void goLeft() {
  state = stateMoving;
  setMotorDir(motorA, HIGH);
  setMotorDir(motorB, HIGH);
  setMotorSpd(motorA, maxSpeed);
  setMotorSpd(motorB, maxSpeed/2);
  setMotorBrake(motorA, LOW);
  setMotorBrake(motorB, LOW);
}

void goRight() {
  state = stateMoving;
  setMotorDir(motorA, HIGH);
  setMotorDir(motorB, HIGH);
  setMotorSpd(motorA, maxSpeed/2);
  setMotorSpd(motorB, maxSpeed);
  setMotorBrake(motorA, LOW);
  setMotorBrake(motorB, LOW);
}

void rotateRight() {
  state = stateMoving;
  setMotorDir(motorA, HIGH);
  setMotorDir(motorB, HIGH);
 // setMotorSpd(motorA, 0);
  setMotorSpd(motorB, maxSpeed);
  setMotorBrake(motorA, HIGH);
  setMotorBrake(motorB, LOW);
}

void rotateLeft() {
  state = stateMoving;
  setMotorDir(motorA, HIGH);
  setMotorDir(motorB, HIGH);
  setMotorSpd(motorA, maxSpeed);
  //setMotorSpd(motorB, 0);
  setMotorBrake(motorA, LOW);
  setMotorBrake(motorB, HIGH);
  Serial.println("rotating left");
}

void goIdle() {
  state = stateIdle;
  setMotorDir(motorA, HIGH);
  setMotorDir(motorB, HIGH);
  setMotorSpd(motorA, 0);
  setMotorSpd(motorB, 0);
  setMotorBrake(motorA, HIGH);
  setMotorBrake(motorB, HIGH);
}

void remoteControl() {
  //Serial.println(cm);
  if (getRemoteCommand()) {
    Serial.println(key);
    switch (key) {
      case keyUp:
      goForwards();
      break;
      case keyDown:
      goBackwards();
      break;
      case keyLeft:
      goLeft();
      break;
      case keyRight:
      goRight();
      break;
      case keyRotateRight:
      rotateRight();
      break;
      case keyRotateLeft:
      rotateLeft();
      break;
      case keyRelease:
      updateMode(1);
      break;
      case keyHello:
      helloSound();
      break;
      case keyHowAreYou:
      //howAreYouSound();
      goodSound();
      break;
      case keyHelloHowAreYou:
      helloSound();
      delay(400);
      howAreYouSound();
      break;
      case keyMilkshake:
      tempSpeed = maxSpeed;
      maxSpeed = 255;
      rotateLeft();
      milkshakeSound();
      delay(100);
      rotateRight();
      milkshakeSound();
      delay(100);
      goIdle();
      maxSpeed = tempSpeed;
      break;
      case keyFaster:
      if (maxSpeed == 255) {
        dontKnowSound();
      }
      else if (maxSpeed == 96) {
        maxSpeed+=32;
      }
      else {
        maxSpeed+=64;
        if (maxSpeed > 255) {
          maxSpeed = 255;
        }
      }
      break;
      case keySlower:
      if (maxSpeed == 96) {
        dontKnowSound();
      }
      else {
        if (maxSpeed == 255)
          maxSpeed+=1;
        maxSpeed-=64;
        if (maxSpeed < 96) {
          maxSpeed = 96;
        }
      }
      break;
      case keySing:
      startUpSound();
      break;
      case keyName:
      robotNameSound();
      break;
      case keyDontKnow:
      dontKnowSound();
      break;
      default:
      goIdle();
      break;
    }
  }
}

void grabRemote() {
  if (getRemoteCommand()) {
    if (key == keyGrab) {
      updateMode(1);
    }
    else if (key == keyHello) {
      helloSound();
    }
    else if (key == keyHowAreYou) {
      goodSound();
    }
    else if (key == keyHelloHowAreYou) {
      helloSound();
      delay(400);
      howAreYouSound();
    }
    else if (key == keyMilkshake) {
      tempSpeed = maxSpeed;
      maxSpeed = 255;
      rotateLeft();
      milkshakeSound();
      delay(100);
      rotateRight();
      milkshakeSound();
      delay(100);
      goIdle();
      resetAlarms();
      alarm[1] = 10;
      maxSpeed = tempSpeed;
    }
    else if (key == keyFaster) {
      if (maxSpeed == 255) {
        dontKnowSound();
      }
      else if (maxSpeed == 96) {
        maxSpeed+=32;
      }
      else {
        maxSpeed+=64;
        if (maxSpeed > 255)
          maxSpeed = 255;
      }
    }
    else if (key == keySlower) {
      if (maxSpeed == 96) {
        dontKnowSound();
      }
      else {
        if (maxSpeed == 255)
          maxSpeed+=1;
        maxSpeed-=64;
        if (maxSpeed < 96) {
          maxSpeed = 96;
        }
      }
    }
    else if (key == keySing) {
      startUpSound();
    }
    else if (key == keyName) {
      robotNameSound();
    }
    else if (key == keyDontKnow) {
      dontKnowSound();
    }
  }
}

bool getRemoteCommand() {
  key = keyNone;
  if (BTSerial.available() <= 0) {
    return false;
  }
  ch = BTSerial.read();
  String voice = "";
  if (ch == '*') {
    voice = "*";
    while (ch != '#') {
      ch = BTSerial.read();
      if (ch != '#' && ch != (char)-1)
        voice+=ch;
    }
  }
  Serial.println(ch);
  if (voice.length() > 0) {
    //Serial.println(voice);
    if (voice == F("*go forward") || voice == F("*go straight")
    || voice == F("*go forwards") || voice == F("*come forward")
    || voice == F("*come here") || voice == F("*mustache power")
    || voice == F("*go") || voice == F("*go gears"))
      key = keyUp;
    else if (voice == F("*go back") || voice == F("*go backwards")
    || voice == F("*reverse") || voice == F("*come back"))
      key = keyDown;
    else if (voice == F("*turn left") || voice == F("*left")
    || voice == F("*go left"))
      key = keyLeft;
    else if (voice == F("*turn right") || voice == F("*right")
    || voice == F("*go right"))
      key = keyRight;
    else if (voice == F("*control") || voice == F("*grab")
    || voice == F("*listen") || voice == F("*hey listen")
    || voice == F("*listen to me") || voice == F("*hey listen gears")
    || voice == F("*listen gears") || voice == F("*are you ready")
    || voice == F("*are you ready gears") || voice == F("*start")
    || voice == F("*switch to auto") || voice == F("*switch to auto mode")
    || voice == F("*auto") || voice == F("*listen to my instructions")
    || voice == F("*listen to my commands") || voice == F("*follow what I say"))
      key = keyGrab;
    else if (voice == F("*finished controlling") || voice == F("*that's enough")
    || voice == F("*release") || voice == F("*carry on") || voice == F("*done")
    || voice == F("*do your own thing") || voice == F("*switch to manual")
    || voice == F("*switch to manual mode") || voice == F("*manual"))
      key = keyRelease;
    else if (voice == F("*rotate left") || voice == F("*rotate")
    || voice == F("*spin left") || voice == F("*turn around"))
      key = keyRotateLeft;
    else if (voice == F("*rotate right") || voice == F("*spin right"))
      key = keyRotateRight;
    else if (voice == F("*hello") || voice == F("*hello robot")
    || voice == F("*hello hello") || voice == F("*hi") || voice == F("*hey")
    || voice == F("*hi robot") || voice == F("*hey robot")
    || voice == F("*hello gears") || voice == F("*hi gears")
    || voice == F("*hey gears") || voice == F("*gears"))
      key = keyHello;
    else if (voice == F("*how are you") || voice == F("*what's up")
    || voice == F("*how are you doing") || voice == F("*how's it going")
    || voice == F("*how are things") || voice == F("*how's life")
    || voice == F("*how are you gears")
    || voice == F("*what's up gears") || voice == F("*how are you doing gears"))
      key = keyHowAreYou;
    else if (voice == F("*hello how are you") || voice == F("*hey how are you")
    || voice == F("*hello robot how are you") || voice == F("*hi how are you")
    || voice == F("*hi robot how are you") || voice == F("*are you ok")
    || voice == F("*hey robot how are you") || voice == F("*hey robot how are you doing")
    || voice == F("*hello gears how are you") || voice == F("*hello gears how are you doing")
    || voice == F("*hey gears how are you") || voice == F("*hi gears how are you")
    || voice == F("*hey gears what's up") || voice == F("*are you ok gears"))
      key = keyHelloHowAreYou;
    else if (voice == F("*milkshake") || voice == F("*milkshake milkshake")
    || voice == F("*do the milkshake") || voice == F("*can you dance")
    || voice == F("*dance") || voice == F("*hey robot can you dance")
    || voice == F("*hey gears can you dance") || voice == F("*do a dance"))
      key = keyMilkshake;
    else if (voice == F("*faster") || voice == F("*go faster")
    || voice == F("*speed up") || voice == F("*increase speed")
    || voice == F("*fast") || voice == F("*go fast") || voice == F("*hurry up"))
      key = keyFaster;
    else if (voice == F("*slower") || voice == F("*go slower")
    || voice == F("*slow down") || voice == F("*decrease speed")
    || voice == F("*slow") || voice == F("*go slow"))
      key = keySlower;
    else if (voice == F("*sing") || voice == F("*sing a song")
    || voice == F("*can you sing") || voice == F("*hey robot can you sing")
    || voice == F("*hey robot sing a song") || voice == F("*hey gears sing a song")
    || voice == F("*hey gears can you sing") || voice == F("*play music")
    || voice == F("*you make me laugh gears"))
      key = keySing;
    else if (voice == F("*what's your name") || voice == F("*what is your name")
    || voice == F("*hello what's your name") || voice == F("*hello robot what's your name")
    || voice == F("*hi what's your name"))
      key = keyName;
    else if (voice == F("*how old are you") || voice == F("*how's the weather")
    || voice == F("*why") || voice == F("*what's my name")
    || voice == F("*are you stupid") || voice == F("*what time is it"))
      key = keyDontKnow;
    else if (voice == F("*stop") || voice == F("*freeze")
    || voice == F("*rest") || voice == F("*stop moving")) {
      key = keyNone;
      if (mode == modeAuto)
        updateMode(1);
    }
    else
      key = lastKey;
    
    return true;
  }
  else { 
    switch (ch) {
      case '8':
      key = keyUp;
      break;
      case '2':
      key = keyDown;
      break;
      case '4':
      key = keyLeft;
      break;
      case '6':
      key = keyRight;
      break;
      case 'C':
      key = keyGrab;
      break;
      case 'D':
      key = keyRelease;
      break;
      case 'E':
      key = keyRotateLeft;
      break;
      case 'F':
      key = keyRotateRight;
      break;
      default:
      break;
    }
  }
  if (key != keyNone && key == lastKey && voice.length() <= 0) {
      return false;
  }
  lastKey = key;
  return true;
}

void updateMaxSpeed() {
  if (currentASpeed > maxSpeed)
    setMotorSpd(motorA, maxSpeed);
  if (currentBSpeed > maxSpeed)
    setMotorSpd(motorB, maxSpeed);
}

void checkBehind() {
  if (state == stateReversing) {
    if (alarm[4] <= 0) {
      alarm[4] = 500;
      backDistance = cm;
    }
  }
  else {
    alarm[4] = -1;
  }
}

void checkFront() {
  if (state == stateMoving) {
    if (alarm[5] <= 0) {
      alarm[5] = 800;
      frontDistance = cm;
    }
  }
  else {
    alarm[5] = -1;
  }
}

