#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#define ONE_WIRE_BUS 7 // pin connected to DS18B20 temperature sensor
#define READ_RPM_PIN 5 // pin connected to fan rpm wire (usually green)
#define PWM_PIN 3 // pin connected to fan control pin (usually blue)
#define POT_PIN A0 // pin to read potentiometer
#define TEMP_PERIOD 250 // period in milliseconds between temperature measurements
#define POT_PERIOD 100 // period in milliseconds between potentiometer measurements
#define TEMPERATURE_PRECISION 9 // accuracy of DS18B20 sensor,
// resolution 9 requires 94 ms delay between requestTemeratures() and getTempC(),
// 10 - 188 ms,
// 11 - 375 ms
// 12 - 750 ms, according to DallasTemperature.cpp
#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define SLAVE_ADDR 0x31 // Slave address, should be changed for other slaves

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddress;

#define SETPOINT_TEMP_MIN -5 // bottom bound for setpoint temperature
#define SETPOINT_TEMP_MAX 125 // top bound for setpoint temperature

#define TEMP_ERROR -127
#define RPM_ERROR -1
#define PERCENTAGE_ERROR 255

//TODO is volatile required?
int volatile tempInt = TEMP_ERROR;
int volatile rpm = RPM_ERROR;
int volatile setpointTemp = TEMP_ERROR;
byte volatile fanSpeedPercentage = PERCENTAGE_ERROR;

int rpmPinStateOld;
int rpmInit = false;
unsigned long rpmTimeStart;
unsigned long previousPotTime;
unsigned long previousTempTime;
#define RPM_READ_COUNT 4
unsigned long rpmTimeSum = 0;
int revolutionsSum = 0;
int revolutions;
int currRpmRead = 0;

int potValue;

double pidSetpoint;
double pidInput;
double pidOutput;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, 20, 1, 5, REVERSE);

void setup()
{
  //Serial.begin(9600);
  pinMode(READ_RPM_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Configure PWM 25Khz on pin 3
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
  OCR2A = 79;
  OCR2B = 0;

  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(slavesRespond);

  sensors.begin();
  sensors.setWaitForConversion(false);

  oneWire.reset_search();
  if (oneWire.search(sensorAddress)) {
    sensors.setResolution(sensorAddress, TEMPERATURE_PRECISION);
  }
  sensors.requestTemperatures();

  potValue = analogRead(POT_PIN);
  potToTemp();

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(TEMP_PERIOD);
  previousTempTime = previousPotTime = millis();
}

void loop()
{
  unsigned long currTime = millis();
  unsigned long timeDiff = currTime - previousTempTime;
  if (timeDiff >= TEMP_PERIOD) {
    previousTempTime = currTime;
    if (rpmInit) {
      rpmTimeSum += currTime - rpmTimeStart;
      revolutionsSum += revolutions;
      currRpmRead++;
      if (currRpmRead == RPM_READ_COUNT) {
        // rpm is revolutionsSum / 2 / 2 * 60000 / rpmTimeSum
        unsigned long res = revolutionsSum * 15000l / rpmTimeSum;
        rpm = res;

        currRpmRead = 0;
        rpmTimeSum = 0;
        revolutionsSum = 0;
      }
    }

    pidInput = sensors.getTempC(sensorAddress);
    tempInt = round(pidInput);
    sensors.requestTemperatures();

    myPID.Compute();

    // pid value is from 0 to 255, OCR2B value should be from 0 to 79
    int out = map(pidOutput, 0, 255, 0, 79);
    OCR2B = out;
    // fan speed in percent from 0% to 100%
    fanSpeedPercentage = map(out, 0, 79, 0, 100);

    restartRpm();
    rpmInit = true;
  } 
  else {
    if (rpmInit) {
      readRpm();
    }
  }

  currTime = millis();
  timeDiff = currTime - previousPotTime;
  if (timeDiff >= POT_PERIOD) {
    previousPotTime = currTime;
    readPot();
    potToTemp();
  }
}

void restartRpm() {
  revolutions = 0;
  rpmPinStateOld = digitalRead(READ_RPM_PIN);
  rpmTimeStart = millis();
}

void readRpm() {
  int state = digitalRead(READ_RPM_PIN);
  if (state != rpmPinStateOld) {
    revolutions++;
  }
  rpmPinStateOld = state;
}

void potToTemp() {
  int in = potValue;
  // treat values below 50 as 0 (min)
  if (in < 50) {
    in = 0;
  } 
  // treat values above 970 as 1023 (max)
  else if (in > 970) {
    in = 1023;
  }
  setpointTemp = map(in, 0, 1023, SETPOINT_TEMP_MIN, SETPOINT_TEMP_MAX);
  pidSetpoint = setpointTemp;
}

void readPot() {
  int in = analogRead(POT_PIN);
  // the value of analogRead is not stable
  if ((in < potValue - 3) || (in > potValue + 3)) {
    potValue = in;
  }
}

void slavesRespond() {
  byte buffer[7];
  buffer[0] = tempInt >> 8;
  buffer[1] = tempInt & 255;
  buffer[2] = rpm >> 8;
  buffer[3] = rpm & 255;
  buffer[4] = setpointTemp >> 8;
  buffer[5] = setpointTemp & 255;
  buffer[6] = fanSpeedPercentage;
  Wire.write(buffer, 7);
}









