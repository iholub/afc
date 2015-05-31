#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#define ONE_WIRE_BUS 7
#define READ_RPM_PIN 5
#define PWM_PIN 3
#define POT_PIN A0
#define TEMP_PERIOD 250
#define POT_PERIOD 100
#define TEMPERATURE_PRECISION 9
#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define SLAVE_ADDR 0x31 // Slave address, should be changed for other slaves

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddress;

int volatile tempInt = -127;
// read RPM
int stateOld;
int rpmInit = false;
int volatile rpm = -1;
unsigned long rpmTimeStart;
unsigned long previousPotTime;
unsigned long previousTempTime;
#define RPM_READ_COUNT 4
unsigned long rpmTimeSum = 0;
int revolutionsSum = 0;
int revolutions;
int currRpmRead = 0;

int potValue;
int volatile setpointTemp = -127;
byte volatile fanSpeedPercentage = 255;

//Setup PID
double Setpoint;
double Input;
double Output;
PID myPID(&Input, &Output, &Setpoint, 20, 1, 5, REVERSE);

void setup()
{
  Serial.begin(9600);
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
        // revolutions / 2 / 2 * 60000 / rpmTimeSum
        unsigned long res = revolutionsSum * 15000l / rpmTimeSum;
        //Serial.print(revolutionsSum);
        //Serial.print(" ");
        //Serial.println(rpmTimeSum);
        rpm = res;

        currRpmRead = 0;
        rpmTimeSum = 0;
        revolutionsSum = 0;
      }
    }

    Input = sensors.getTempC(sensorAddress);
    tempInt = round(Input);
    sensors.requestTemperatures();

    //Compute PID value
    myPID.Compute();

    int out = map(Output, 0, 255, 0, 79);
    OCR2B = out;
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
  revolutions = 0;// Restart the RPM counter
  stateOld = digitalRead(READ_RPM_PIN);
  rpmTimeStart = millis();
}

void readRpm() {
  int state = digitalRead(READ_RPM_PIN);
  if (state != stateOld) {
    revolutions++;
  }
  stateOld = state;
}

void potToTemp() {
  int in = potValue;
  if (in < 50) {
    in = 0;
  } 
  else if (in > 970) {
    in = 1023;
  }
  setpointTemp = map(in, 0, 1023, -1, 125);
  Setpoint = setpointTemp;
}

void readPot() {
  int in = analogRead(POT_PIN);
  if ((in < potValue - 3) || (in > potValue + 3)) {
    potValue = in;
  }
}

void slavesRespond(){
  byte buffer[7];                 // split int value into two bytes buffer
  buffer[0] = tempInt >> 8;
  buffer[1] = tempInt & 255;
  buffer[2] = rpm >> 8;
  buffer[3] = rpm & 255;
  buffer[4] = setpointTemp >> 8;
  buffer[5] = setpointTemp & 255;
  buffer[6] = fanSpeedPercentage;
  Wire.write(buffer, 7);          // return response to last command
}








