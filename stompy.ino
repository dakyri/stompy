#include <MIDI.h>
#include "ADXL345mw.h"

#undef SERIAL_DEBUG
#define SERIAL_DEBUG_LEVEL 1

midi::MidiInterface<HardwareSerial> midiA((HardwareSerial&)Serial1);

const double kRadGrad = (180.0/M_PI);
const double kRad = (1.0/M_PI);

typedef byte btn_mode_t;

const byte kNote = 0;
const byte kCtrl = 1;
const byte kCtrlLatch = 2;
const byte kCtrlLatchOn = 3;

adxl345::I2cInterface accelerometer;

const int xlomReadDelay_ms = 100;
const int debounceDelay_ms = 50;
const int potChangeMinTime_ms = 50;
const int axisChangeMinTime_ms = 50;

byte buttonVelocity = 80;
byte outChannel = 1;


/*****************************************************
 * COLOR AND LAMP INTERFACE
 *****************************************************/
struct rgb_color {
  rgb_color(byte r, byte g, byte b)
    : red(r), green(g), blue(b) {}
  byte red;
  byte green;
  byte blue;  
};

class RGBLampa {
public:
  RGBLampa(byte r, byte g, byte b)
    : pinR(r), pinG(g), pinB(b) {}

  void setup() {
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);
  }

  void set(rgb_color c) {
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG_LEVEL > 2
    Serial.print(pinR);Serial.print(" ");Serial.print(c.red);Serial.print(" ");Serial.print(pinG);Serial.print(" ");Serial.print(c.green);Serial.print(" ");Serial.print(pinB);Serial.print(" ");Serial.println(c.blue);
#endif
    analogWrite(pinR, c.red);
    analogWrite(pinG, c.green);
    analogWrite(pinB, c.blue);
  }
  
protected: 
  byte pinR;
  byte pinG;
  byte pinB;
};

class StateLampa: public RGBLampa {
public:
  StateLampa(byte r, byte g, byte b, byte ns, rgb_color *sa)
    : RGBLampa(r, g, b), states(sa), blinkEndTime_ms(-1), nStates(ns), state(-1) {}

  void setState(int8_t sc) {
    if (sc < 0 || sc >= nStates) {
      set(rgb_color(0,0,0));
      state = -1;
    } else {
      set(states[sc]);
      state = sc;
    }
  }

  void check() {
    if (blinkEndTime_ms > 0 && millis() > blinkEndTime_ms) {
      if (state != 0) {
        setState(0);
      }
      blinkEndTime_ms = 0;
    } else if (state < 0) {
      if (accelerometer.isValid) {
        blink(5,1000);
      } else {
        setState(0);
      }
    }
  }

  void blink(int8_t sc=0, uint16_t dur_ms=0) {
    setState(sc);
    if (dur_ms > 0) {
      blinkEndTime_ms = millis() + dur_ms;
    } else {
      blinkEndTime_ms = 0;
    }
  }

protected:
  rgb_color *states;
  uint32_t blinkEndTime_ms;
  byte nStates;
  int8_t state;
};


struct rgb_color state_colors[] = {
    {10, 250, 10},
    {250, 10, 10},
    {80, 100, 10},
    {100, 10, 200},
    {150, 200, 10},
    {10, 10, 250}};
StateLampa lamp(5, 9, 6, sizeof(state_colors)/sizeof(rgb_color), state_colors);



/*****************************************************
 * BASIC BUTTONS
 *****************************************************/

struct stomp_button {
  stomp_button(byte _pin, btn_mode_t _mode, byte _target)
  : lastBounceTime_ms(0), mode(_mode), lastState(LOW), pressed(false), pin(_pin), target(_target) {}

  void setup() {
    pinMode(pin, INPUT);
  }
  
  long lastBounceTime_ms;
  btn_mode_t mode;
  byte lastState;
  bool pressed;
  byte pin;
  byte target;
};

/*****************************************************
 * BASIC CONTROL POTENTIOMETER
 *****************************************************/

struct ctrl_pot {
  ctrl_pot(byte _pin, byte _enablePin, byte _ctrl)
  : lastChangeTime_ms(0), pin(_pin), enablePin(_enablePin), ctrl(_ctrl), lastVal(255), lastVal2(255)  {}

  void setup() {
    pinMode(pin, INPUT);
    pinMode(enablePin, INPUT);
  }

  long lastChangeTime_ms;
  byte pin;
  byte enablePin;
  byte ctrl;
  byte lastVal;
  byte lastVal2;
};

/*****************************************************
 * XY CONTROL
 *****************************************************/

struct axis_ctl {
  axis_ctl(byte _ctrl, int _min, int _max, byte _ctrlMin, byte _ctrlMax):
      ctrl(_ctrl), minV(_min), maxV(_max), ctrlMin(_ctrlMin), ctrlMax(_ctrlMax), lastCV(255) {
    factor = (ctrlMax != ctrlMin)? float(ctrlMax - ctrlMin)/float(maxV - minV) : 0;
  }

  void checkSend(int angle) {
    if (angle < minV) angle = minV;
    else if (angle > maxV) angle = maxV;
    byte cv = ctrlMin + (angle - minV) * factor;
   if (cv != lastCV) {
#ifdef SERIAL_DEBUG
      Serial.print("axus control "); Serial.print(ctrl); Serial.print(" angle "); Serial.print(angle);Serial.print(" minV "); Serial.print(minV);Serial.print(" maxV "); Serial.print(maxV);Serial.print(" factor "); Serial.print(factor);Serial.print(" cv "); Serial.println(cv);
#endif 
      midiA.sendControlChange(ctrl, cv, outChannel);
      lamp.blink(2, 50);
      lastCV = cv;
    }
  }
  
  byte ctrl;
  int minV; // assuming we range -180..180, and probably a smaller ambit than even 180
  int maxV; // >  min
  byte ctrlMin; // ctrl value at min  0..127
  byte ctrlMax; // ctrl value at max . may be > or < than CtrlMin
  float factor;
  byte lastCV;
 
};

struct xl_joy {
  xl_joy(byte _activeButton,
        byte _pitchCtrl, int _pitchMin, int _pitchMax, byte _pitchCtrlMin, byte _pitchCtrlMax,
        byte _rollCtrl, int _rollMin, int _rollMax, byte _rollCtrlMin, byte _rollCtrlMax)
    : lastReading_ms(0), activeButton(_activeButton),
      pitch(_pitchCtrl, _pitchMin, _pitchMax, _pitchCtrlMin, _pitchCtrlMax),
      roll(_rollCtrl, _rollMin, _rollMax, _rollCtrlMin, _rollCtrlMax) {}
  
  long lastReading_ms;
  byte activeButton;

  struct axis_ctl pitch;
  struct axis_ctl roll;
};


/******************************************************
 * ROTARY ENCODER AND STUFF
 ******************************************************/

class RotaryEncoder {
public:
  RotaryEncoder(byte _pinA, byte _pinB, byte _pinPress)
    : pos(0), pinA(_pinA), pinB(_pinB), pinPress(_pinPress), pinALast(LOW), buttonLast(LOW), pressed(false) {}

  void setup() {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinPress, INPUT);
  }

  void check() {
    long theTime = millis();
    
    int pinAVal = digitalRead(pinA);
    if ((pinALast == LOW) && (pinAVal == HIGH)) {
      if (digitalRead(pinB) == LOW) {
        pos--;
      } else {
        pos++;
      }
//      Serial.print (pos);
//      Serial.print ("/");
    }
    pinALast = pinAVal;

    byte buttonVal = digitalRead(pinPress);
    if (buttonVal != buttonLast) {
      buttonLast = buttonVal;
      lastBounceTime_ms = theTime; //set the current time
    }
    if (theTime - lastBounceTime_ms > debounceDelay_ms) {
      if (buttonVal == HIGH && !pressed) {
        pressed = true;
        // transition high action
      } else if (buttonVal == LOW && pressed ) {
        pressed = false;
        // transition low action
      }
    }
  
  }

protected:
  long lastBounceTime_ms;
  int pos;
  byte pinA;
  byte pinB;
  byte pinPress;
  byte pinALast;
  byte buttonLast;
  bool pressed;
};

/******************************************************************************
 * DATA DEFINITIIONS
 ******************************************************************************/
struct stomp_button buttons[] = {
  { 19, kCtrl, 92 },  // KP3 touch
  { 18, kNote, 36 },  // KP3 sample1
  { 15, kNote, 37 },  // KP3 sample2
  { 14, kNote, 38 },  // KP3 sample3
  { 16, kNote, 39 },  // KP3 sample4
  { 10, kCtrlLatch, 95 }  // KP3 hold
};
const byte n_buttons = sizeof(buttons)/sizeof(stomp_button);

struct ctrl_pot pots[] = {
  { 20, 21, 94 }  // KP3 fx depth, analog pin 9, enable on pin 8
};
byte n_pots = sizeof(pots)/sizeof(ctrl_pot);

struct xl_joy xl{/* activate */ 0, /* pitch */  12, -45, 45, 0, 127, /* roll */ 13, -45, 45, 0, 127};

/**
 * check the digital pushbuttons
 * our buttons are high when pressed and grounded with a 4.7K resistor when switched off
 */
void checkStompButton(struct stomp_button & button) {

  byte buttonState = digitalRead(button.pin);
  long theTime = millis();
  if (buttonState != button.lastState) {
    button.lastState = buttonState;
    button.lastBounceTime_ms = theTime; //set the current time
  }
  if (theTime - button.lastBounceTime_ms > debounceDelay_ms) {
    if (buttonState == HIGH && !button.pressed) {
      button.pressed = true;
      if (button.mode == kCtrl) {
        midiA.sendControlChange(button.target, 127, outChannel);
        lamp.blink(1, 200);
      } else if (button.mode == kNote){
        midiA.sendNoteOn(button.target, buttonVelocity, outChannel);
        lamp.blink(2, 200);
     } else if (button.mode == kCtrlLatch){
        button.mode = kCtrlLatchOn;
        midiA.sendControlChange(button.target, 127, outChannel);
        lamp.blink(3, 200);
      } else if (button.mode == kCtrlLatchOn){
        button.mode = kCtrlLatch;
        midiA.sendControlChange(button.target, 0, outChannel);
        lamp.blink(4, 200);
      }
    } else if (buttonState == LOW && button.pressed ) {
//      Serial.println("low");
      button.pressed = false;
      if (button.mode == kCtrl) {
        midiA.sendControlChange(button.target, 0, outChannel);
        lamp.blink(1, 200);
      } else if (button.mode == kNote){
        midiA.sendNoteOff(button.target, buttonVelocity, outChannel); 
        lamp.blink(2, 200);
      }
    }
  }
}

/**
 * check the analog pots
 */
void checkCtrlPot(struct ctrl_pot & pot) {
  bool enabled = true;
  if (pot.enablePin != 255) {
    byte enableState = digitalRead(pot.enablePin);
    enabled = (enableState == HIGH);
  }
  if (enabled) {
    int potState = analogRead(pot.pin); /* 0..1023 */
    potState -= 512;
    if (potState < 0) potState = 0;
    byte ctrlVal = potState / 4; /* looks like it automatically converts to a shift. sanity */
    int8_t chg = ctrlVal - pot.lastVal;
    if (chg != 0) {
      long theTime = millis();
      int8_t chg2 = ctrlVal - pot.lastVal2;
      if (theTime - pot.lastChangeTime_ms > potChangeMinTime_ms) {
  //      Serial.print(chg); Serial.print(' '); Serial.print(chg2); Serial.print(' '); 
  //      Serial.print(pot.lastVal); Serial.print(' '); Serial.print(pot.lastVal2); Serial.print(' '); 
  //      Serial.println(ctrlVal);
        if (pot.lastVal2 == 255
                || (ctrlVal <= 64 && (chg < 0 || chg2 > 3))
                || (ctrlVal > 64 && (chg > 0 || chg2 < -3))) {
          pot.lastVal2 = pot.lastVal;
          pot.lastVal = ctrlVal;
          pot.lastChangeTime_ms = theTime;
          midiA.sendControlChange(pot.ctrl, ctrlVal, outChannel);
        }
      }
    }
  }
}

void checkAccelerometer(adxl345::I2cInterface & xlm8r, struct xl_joy &xl) {
  if (xlm8r.isValid && (xl.activeButton == 255 || buttons[xl.activeButton].pressed)) {
    long theTime = millis();
    if (theTime - xl.lastReading_ms > xlomReadDelay_ms) {
      int16_t x, y, z;
      xlm8r.readRaw(x, y, z);
      adxl345::Vector v(x,y,z);
/* filtered = filtered.lowPassFilter(v, 0.5); // Low Pass Filter to smooth out data. 0.1 - 0.9 */
// convert to degrees: these are probably going to be simpler to work with if we need to configure on the fly through front panel
      int pitch = -atan2(v.x, sqrt(v.y*v.y + v.z*v.z))*kRadGrad;
      int roll  = atan2(v.y, v.z)*kRadGrad;
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG_LEVEL > 2
      Serial.print("xyz ->");Serial.print(x); Serial.print(' '); Serial.print(y); Serial.print(' '); Serial.print(z);
          Serial.print(" :: "); Serial.print(pitch); Serial.print(' '); Serial.println(roll);
#endif
      xl.pitch.checkSend(pitch);
      xl.roll.checkSend(roll);
      xl.lastReading_ms = theTime;
    }
  }
}

/*************************************************************************
 * main arduino hooks
 *************************************************************************/

void setup() {

#ifdef SERIAL_DEBUG
  Serial.begin (9600);
#endif

  for (byte i=0; i<n_buttons; i++) {
    buttons[i].setup();
  }
  for (byte i=0; i<n_pots; i++) {
    pots[i].setup();
  }
  lamp.setup();
  
  midiA.turnThruOn();
  midiA.begin(MIDI_CHANNEL_OMNI);
  if (accelerometer.begin()) {
#ifdef SERIAL_DEBUG
    Serial.println("Initialized accelerometer");
#endif
    accelerometer.setRange(adxl345::kRange16G);
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("Failed to initialize accelerometer");
#endif   
  }
}


void loop() {
  if (midiA.read()) {
#ifdef SERIAL_DEBUG
    Serial.print(midiA.getType(), HEX);Serial.print(' ');
    Serial.print(midiA.getData1(), HEX);Serial.print(' ');
    Serial.print(midiA.getData2(), HEX);Serial.print(' ');
    Serial.println(midiA.getChannel(), HEX);
#endif   
  } else {
//    Serial.println("nope");
  }
  
  for (byte i=0; i<n_buttons; ++i) {
    checkStompButton(buttons[i]);
  }
  for (byte i=0; i<n_pots; ++i) {
    checkCtrlPot(pots[i]);
  }
  checkAccelerometer(accelerometer, xl);
  
  lamp.check();
}
