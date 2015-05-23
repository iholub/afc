/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
//temperature
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 7
#define READ_RPM_PIN 5
#define PWM_PIN 3 // Only works with Pin 3
#define POT_PIN A0 // Analog 0
#define TEMP_PERIOD 500
#define TEMP_PERIOD_SETUP 100
#define POT_PERIOD 100
#define SLAVE_ADDR 0x31
#define TEMPERATURE_PRECISION 9
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress sensorAddress;

int volatile tempInt = -127;

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

/*-----( Declare Variables )-----*/

// read RPM
int stateOld = -1;
int rpmInit = false;
int volatile rpm = -1;
unsigned long rpmTimeStart;
unsigned long potTimeNext = 0;
unsigned long tempTimeNext;
#define RPM_READ_COUNT 2
unsigned long rpmTimeSum = 0;
int revolutionsSum = 0;
int revolutions;
int currRpmRead;

int previousPotValue = -1;

void setup()
{
  //Serial.begin(9600);
  pinMode(READ_RPM_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  // Fast PWM Mode, Prescaler = /8
  // PWM on Pin 3, Pin 11 disabled
  // 16Mhz / 8 / (79 + 1) = 25Khz
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
  // Set TOP and initialize duty cycle to zero(0)
  OCR2A = 79;   // TOP - DO NOT CHANGE, SETS PWM PULSE RATE
  OCR2B = 0;    // duty cycle for Pin 3 (0-79) generates 1 500nS pulse even when 0

    Wire.begin(SLAVE_ADDR);
  Wire.onRequest(slavesRespond);  // Perform on master's request

  // Start up the library
  sensors.begin();
  sensors.setWaitForConversion(false);

  // Must be called before search()
  oneWire.reset_search();
  if (oneWire.search(sensorAddress)) {
    sensors.setResolution(sensorAddress, TEMPERATURE_PRECISION);
  }
  sensors.requestTemperatures();
  tempTimeNext = millis() + TEMP_PERIOD_SETUP;
}

void loop()
{
  unsigned long currTime = millis();
  if (currTime >= tempTimeNext) {
    if (rpmInit) {
      rpmTimeSum += currTime - rpmTimeStart;
      revolutionsSum += revolutions;
      currRpmRead++;
      if (currRpmRead == RPM_READ_COUNT) {
        int rpmRes = revolutionsSum * 30000 / rpmTimeSum; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.
        rpm = rpmRes;

        currRpmRead = 0;
        rpmTimeSum = 0;
        revolutionsSum = 0;
      }
    }

    readRequestTemp();

    tempTimeNext = millis() + TEMP_PERIOD;

    restartRpm();
    rpmInit = true;
  } 
  else {
    if (rpmInit) {
      readRpm();
    }
  }

  readPot();
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

void readRequestTemp() {
  float tempC = sensors.getTempC(sensorAddress);
  tempInt = round(tempC);
  sensors.requestTemperatures();
}

void readPot() {
  if (millis() >= potTimeNext) {
    potTimeNext = millis() + POT_PERIOD;
    int in = analogRead(POT_PIN);
    if (in != previousPotValue) {
      previousPotValue = in;
      //Serial.println("pot: ");
      //Serial.println(in);
      int out = map(in, 0, 1023, 0, 79);
      OCR2B = out;
    }
  }

}

void slavesRespond(){
  byte buffer[4];                 // split int value into two bytes buffer
  buffer[0] = tempInt >> 8;
  buffer[1] = tempInt & 255;
  buffer[2] = rpm >> 8;
  buffer[3] = rpm & 255;
  Wire.write(buffer, 4);          // return response to last command
}





