#include <LiquidCrystal_I2C.h>

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 7
#define READ_RPM_PIN 5
#define PWM_PIN 3
#define POT_PIN A0
#define RPM_PERIOD 250
#define TEMP_PERIOD 100
#define POT_PERIOD 100
#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddress;

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

int volatile tempInt = -127;

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

int stateOld = LOW;
int half_revolutions = 0;
int volatile rpm = -1;
unsigned long rpmTimeNext = 0;
unsigned long potTimeNext = 0;
unsigned long tempTimeNext;

int previousPotValue = -1;

int volatile setpointTemp = -127;
byte volatile fanSpeedPercentage = 255;

#define I2C_FAN2_ADDR 0x31
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
int volatile temp2 = -127;
int volatile rpm2 = -1;
int volatile setpointTemp2 = -127; 
byte volatile fanSpeedPercentage2 = 255;

void setup()
{
  Serial.begin(9600);
  pinMode(READ_RPM_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
  OCR2A = 79;
  OCR2B = 0;

  lcd.begin(16,2);

  sensors.begin();
  sensors.setWaitForConversion(false);

  oneWire.reset_search();

  if (oneWire.search(sensorAddress)) {
    sensors.setResolution(sensorAddress, TEMPERATURE_PRECISION);
  }

  sensors.requestTemperatures();
  tempTimeNext = millis() + TEMP_PERIOD;

  lcd.clear();
  displayFans();
}

void loop()
{
  readRpm();

  readTemp();

  readPot();

}

void readRpm() {
  int state = digitalRead(READ_RPM_PIN);
  if (stateOld == LOW && state == HIGH) {
    half_revolutions++;
  }
  stateOld = state;

  if (millis() >= rpmTimeNext){
    rpm = half_revolutions * 30;

    half_revolutions = 0;

    readFan2();

    displayFans();

    rpmTimeNext = millis() + RPM_PERIOD;
  }  
}

void displayFans() {
  displayFan(0, tempInt, rpm, setpointTemp, fanSpeedPercentage);
  displayFan(1, temp2, rpm2, setpointTemp2, fanSpeedPercentage2);
}

void displayFan(int row, int temp, int rpm, int setpoint, int percentage) {
  // 16.5 ms
  char buf[17];
  buf[16] = 0;
  if (temp == -127) {
    strncpy(buf, "ERR", 3);
  } 
  else {
    sprintf(buf, "%3d", temp);
  }

  if (rpm == -1) {
    strncpy(&buf[3], "  ERR", 5);
  } 
  else {
    sprintf(&buf[3], "%5d", rpm);
  }

  if (setpoint == -127) {
    strncpy(&buf[8], " ERR", 4);
  } 
  else {
    sprintf(&buf[8], "%4d", setpoint);
  }

  if (percentage == 255) {
    strncpy(&buf[12], " ERR", 4);
  } 
  else {
    sprintf(&buf[12], "%4d", percentage);
  }

  // 1 ms
  lcd.setCursor(0,row);
  // 15.3 ms
  lcd.print(buf);
}

void readTemp() {
  if (millis() >= tempTimeNext) {
    //12.85 ms
    float tempC = sensors.getTempC(sensorAddress);
    //2.14 ms
    sensors.requestTemperatures();
    tempTimeNext = millis() + TEMP_PERIOD;
    tempInt = round(tempC);
    unsigned long m4 = micros();
  }
}

void readPot() {
  if (millis() >= potTimeNext) {
    potTimeNext = millis() + POT_PERIOD;
    int in = analogRead(POT_PIN);
    if (in < 50) {
      in = 0;
    } 
    else
      if (in > 980) {
        in = 1023;
      }
    if (in != previousPotValue) {
      previousPotValue = in;
      int out = map(in, 0, 1023, 0, 79);
      OCR2B = out;
    }
  }

}

void readFan2() {
  int res = Wire.requestFrom(I2C_FAN2_ADDR, 7);
  if (res)
  {
    temp2 = Wire.read() << 8 | Wire.read();
    rpm2 = Wire.read() << 8 | Wire.read();
    setpointTemp2 = Wire.read() << 8 | Wire.read();
    fanSpeedPercentage2 = Wire.read();
  }
  else
  {
    temp2 = -127;
    rpm2 = -1;
    setpointTemp2 = -127;
    fanSpeedPercentage2 = 255;
  }
}






