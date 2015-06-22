#define MASTER

#ifdef MASTER
#include <LiquidCrystal_I2C.h>
#endif

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#define ONE_WIRE_BUS 7 // pin connected to DS18B20 temperature sensor
#define READ_RPM_PIN 2 // pin connected to fan rpm wire (usually green)
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

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddress;

#define SETPOINT_TEMP_MIN -5 // bottom bound for setpoint temperature
#define SETPOINT_TEMP_MAX 125 // top bound for setpoint temperature

#define TEMP_ERROR -127
#define RPM_ERROR -1
#define PERCENTAGE_ERROR 255

int volatile tempInt = TEMP_ERROR;
int volatile rpm = RPM_ERROR;
int volatile setpointTemp = TEMP_ERROR;
byte volatile fanSpeedPercentage = PERCENTAGE_ERROR;

unsigned long rpmTimeStart;
unsigned long previousPotTime;
unsigned long previousTempTime;
#define RPM_READ_COUNT 4
unsigned long rpmTimeSum = 0;
int revolutionsSum = 0;
int volatile revolutions;
int currRpmRead = 0;

int potValue;

double pidSetpoint;
double pidInput;
double pidOutput;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, 20, 1, 5, REVERSE);

#ifdef MASTER
#define SCR_POT_PIN A2 // pin of potentiometer to change screen
int scrPotValue;
#define I2C_FAN2_ADDR 0x31
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
int volatile temp2 = -127;
int volatile rpm2 = -1;
int volatile setpointTemp2 = -127; 
byte volatile fanSpeedPercentage2 = 255;
int volatile temp3 = -127;
int volatile rpm3 = -1;
int volatile setpointTemp3 = -127; 
byte volatile fanSpeedPercentage3 = 255;

//http://www.quinapalus.com/hd44780udg.html
byte flame[8] = {
4,6,14,29,25,25,10,4
};

#else
#define SLAVE_ADDR 0x31 // Slave address, should be changed for other slaves
#endif

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

#ifdef MASTER
  Wire.begin();
#else
  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(slavesRespond);
#endif

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

#ifdef MASTER
  lcd.begin(16,2);
  lcd.createChar(0, flame);
  lcd.clear();
  displayFans();
#endif

  currRpmRead = 0;
  rpmTimeSum = 0;
  revolutionsSum = 0;
  revolutions = 0;
  rpmTimeStart = millis();
  attachInterrupt(0, rpm_fan, FALLING);
}

void loop()
{
  unsigned long currTime = millis();
  unsigned long timeDiff = currTime - previousTempTime;
  if (timeDiff >= TEMP_PERIOD) {
    detachInterrupt(0);
    previousTempTime = currTime;
    rpmTimeSum += currTime - rpmTimeStart;
    revolutionsSum += revolutions;
    currRpmRead++;
    if (currRpmRead == RPM_READ_COUNT) {
      // rpm is revolutionsSum / 2 * 60000 / rpmTimeSum
      unsigned long res = revolutionsSum * 30000l / rpmTimeSum;
      rpm = res;

      currRpmRead = 0;
      rpmTimeSum = 0;
      revolutionsSum = 0;
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

#ifdef MASTER    
    readFan2();
    displayFans();
#endif

    revolutions = 0;
    rpmTimeStart = millis();
    attachInterrupt(0, rpm_fan, FALLING);
  } 

  currTime = millis();
  timeDiff = currTime - previousPotTime;
  if (timeDiff >= POT_PERIOD) {
    previousPotTime = currTime;
    readPot();
    potToTemp();
  }
}

void rpm_fan(){
  revolutions++;
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

#ifdef MASTER
void displayFans() {
  scrPotValue = analogRead(SCR_POT_PIN);
  if (scrPotValue < 512) {
    displayScreen0();
  } 
  else {
    displayFan(0, tempInt, rpm, setpointTemp, fanSpeedPercentage);
    displayFan(1, temp2, rpm2, setpointTemp2, fanSpeedPercentage2);
  }
}

void displayScreen0() {
  displayScreen0temps(tempInt, temp2, temp3);
  displayScreen0rpms(rpm, rpm2, rpm3);
}

void displayScreen0temps(int t1, int t2, int t3) {
  lcd.setCursor(0,0);
  lcd.print(char(0));
  
  char buf[16];
  buf[15] = 0;

  if (t1 == -127) {
    strncpy(buf, " ERR\337", 5);
  } 
  else {
    sprintf(buf, "%4d\337", t1);
  }

  if (t2 == -127) {
    strncpy(&buf[5], " ERR\337", 5);
  } 
  else {
    sprintf(&buf[5], "%4d\337", t2);
  }

  if (t3 == -127) {
    strncpy(&buf[10], " ERR\337", 5);
  } 
  else {
    sprintf(&buf[10], "%4d\337", t3);
  }
  
  lcd.print(buf);
}

void displayScreen0rpms(int r1, int r2, int r3) {
  lcd.setCursor(0,1);
  lcd.print(char(0));
  lcd.print(" ");
  
  char buf[15];
  buf[14] = 0;

  if (r1 == -1) {
    strncpy(buf, " ERR", 4);
  } 
  else {
    sprintf(buf, "%4d", r1);
  }

  if (r2 == -1) {
    strncpy(&buf[4], "  ERR", 5);
  } 
  else {
    sprintf(&buf[4], "%5d", r2);
  }

  if (r3 == -1) {
    strncpy(&buf[9], "  ERR", 5);
  } 
  else {
    sprintf(&buf[9], "%5d", r3);
  }

  lcd.print(buf);
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
#else
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
#endif










