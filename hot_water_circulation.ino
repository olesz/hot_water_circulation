/* hot water circulation controller
   --------------------------------------------------------

   Turns on curculation of hot water in water circuit
   Trigger is a flow meter measuring a certan amount of water fow

   Created 30 March 2023
   Copyright 2023 Lajos Olah <lajos.olah.jr@gmail.com>

*/

#include <pt.h>

#define FLOW_SENSOR_PIN 2         // flow switch input - DIGITAL
#define WAIT_TIME_PIN 1           // potentiometer input - ANALOGUE
#define PUMP_RUNTIME_PIN 2        // potentiometer input - ANALOGUE

#define ZONE_VALVE_PIN 3          // zone valve output - DIGITAL
#define CIRCULATION_PUMP_PIN 4    // circulation pump output - DIGITAL

#define MILLISEC_1000 1000
#define MILLISEC_100 100
#define MILLISEC_60000 60000.0
#define MILLISEC_300000 300000.0
#define MILLISEC_600000 600000.0

#define ZONE_VALVE_DELAY 10000.0

#define STATE_ON 0                        // state on value
#define STATE_OFF 1                       // state off value
#define ANALOGUE_MAX_VALUE 1023.0         // maximum potentiometer value
#define MINIMAL_CIRCULATION_TIME 5000     // seconds while circulation is on
#define MINIMAL_FLOW_TRIGGER 5            // measured L/min to trigger circulation

unsigned long circulationTime;   // seconds while compressor is on or off
unsigned long waitTime;   // seconds while compressor is on or off
unsigned long waterFlowMeterSignalCounter = 0;

volatile int waterFlowTrigger;
int enableFlag = 1;

static struct pt ptReadAnalogueValue;
static struct pt ptDoCirculationOnOff;

/* This function reads analogut value from potentiometer */
static int readAnalogueValue(struct pt *pt) {
  static unsigned long timestamp = 0;
  static unsigned long localCirculationTime;
  static unsigned long localWaitTime;
  
  PT_BEGIN(pt);
  
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > MILLISEC_100);
    timestamp = millis(); // take a new timestamp
    localCirculationTime = calculateCirculationTime();
    localWaitTime = calculateWaitTime();

    if (localCirculationTime != circulationTime || localWaitTime != waitTime) {
      circulationTime = localCirculationTime;
      waitTime = localWaitTime;
      Serial.println("Circulation time: " + String(circulationTime / MILLISEC_1000) + "s, wait time: " + String(waitTime / MILLISEC_1000) + "s");
    }
  }
  
  PT_END(pt);
}

/* This function does circulation on-off */
static int doCirculationOnOff(struct pt *pt) {
  static unsigned long timestampForSleep = 0;
  static unsigned long timestampForOnOff = 0;
  static int circulationPumpState = STATE_OFF;
  static int zoneValveState = STATE_OFF;
  static int justEnabled = 0;
  static String spinningString = "|";
  static String timeString = "";
  
  PT_BEGIN(pt);
  
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestampForSleep > MILLISEC_1000);
    timestampForSleep = millis(); // take a new timestamp for sleep

    waterFlowTrigger = waterFlowMeterSignalCounter > MINIMAL_FLOW_TRIGGER;

    if (enableFlag == 1) {
      if (waterFlowTrigger) {
        Serial.println("waterflow triggered");
        zoneValveState = STATE_ON;
        timestampForOnOff = millis(); // take a new timestamp for on/off
        enableFlag = 0;
        justEnabled = 1;
      }
    }

    //circulation is delayed to let the valve open
    if (justEnabled && millis() - timestampForOnOff >= ZONE_VALVE_DELAY) {
      circulationPumpState = STATE_ON;
      justEnabled = 0;
    }

    if (millis() - timestampForOnOff >= circulationTime + ZONE_VALVE_DELAY) {
      circulationPumpState = STATE_OFF;
      zoneValveState = STATE_OFF;
    }

    if (millis() - timestampForOnOff >= waitTime + ZONE_VALVE_DELAY) {
      enableFlag = 1;
      timestampForOnOff = 0;
    }

    setCirculationPump(circulationPumpState);
    setZoneValve(zoneValveState);

    if (enableFlag == 0) {
      timeString = "Since last trigger - " + String((millis() - timestampForOnOff) / MILLISEC_1000) + "s";
    } else {
      timeString = "";
    }

    spinningString = getSpinningString(spinningString);

    Serial.println(spinningString + " " + timeString + " (pump: " + getStateString(circulationPumpState) +
        " - valve: " + getStateString(zoneValveState) + " - enable flag: " + getStateString(enableFlag != 0) + 
        " - measured water flow: " + String(waterFlowMeterSignalCounter) + " L/min)");

    waterFlowMeterSignalCounter = 0;
  }
  
  PT_END(pt);
}

void setup() {
  Serial.begin(9600);

  waterFlowTrigger = 0;

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), falling, FALLING);

  pinMode(CIRCULATION_PUMP_PIN, OUTPUT);
  pinMode(ZONE_VALVE_PIN, OUTPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

  PT_INIT(&ptReadAnalogueValue);
  PT_INIT(&ptDoCirculationOnOff);
}

void loop() {
  readAnalogueValue(&ptReadAnalogueValue);
  doCirculationOnOff(&ptDoCirculationOnOff);
}

unsigned long calculateCirculationTime() {
  int analogueValue = ANALOGUE_MAX_VALUE - analogRead(PUMP_RUNTIME_PIN);

  unsigned long result = MILLISEC_60000 + MILLISEC_300000 * (1 - (float)analogueValue / ANALOGUE_MAX_VALUE);

  return result;
}

unsigned long calculateWaitTime() {
  int analogueValue = ANALOGUE_MAX_VALUE - analogRead(WAIT_TIME_PIN);

  unsigned long result = MILLISEC_300000 + MILLISEC_600000 * (1 - (float)analogueValue / ANALOGUE_MAX_VALUE);

  return result;
}

void setCirculationPump(int state) {
  digitalWrite(CIRCULATION_PUMP_PIN, state);
}

void setZoneValve(int state) {
  digitalWrite(ZONE_VALVE_PIN, state);
}

String getStateString(int state) {
  return String(state == STATE_ON ? "on" : "off");
}

String getSpinningString(String previousString) {
  if (previousString == "/")
    return "-";
  if (previousString == "-")
    return "\\";
  if (previousString == "\\")
    return "|";
  if (previousString == "|")
    return "/";
}

void falling()
{
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), rising, RISING);
}

void rising()
{
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), falling, FALLING);
  waterFlowMeterSignalCounter++;
}
