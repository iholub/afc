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

byte flameIcon[8] = {
  B00000100,
  B00000110,
  B00001110,
  B00011101,
  B00011001,
  B00011001,
  B00001010,
  B00000100
};
byte fanIcon[8] = {
  B00000000,
  B00000000,
  B00011011,
  B00011111,
  B00001010,
  B00011111,
  B00011011,
  B00000000
};
byte setpointIcon[8] = {
  0,15,3,5,9,16,0,0
};
byte percentageIcon[8] = {
  31,17,10,4,10,17,31,0
};
#else
#define SLAVE_ADDR 0x31 // Slave address, should be changed for other slaves
#endif

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
  lcd.createChar(1, flameIcon);
  lcd.createChar(2, fanIcon);
  lcd.createChar(3, setpointIcon);
  lcd.createChar(4, percentageIcon);
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

#ifdef MASTER
int screenNumber = 0;

void displayFans() {
  scrPotValue = analogRead(SCR_POT_PIN);
  screenNumber = scrPotValue / 256; // 4 screens
  if (screenNumber == 0) {
    displayScreen0(tempInt, temp2, temp3, rpm, rpm2, rpm3);
  } 
  else if (screenNumber == 1) {
    displayFan(1, tempInt, rpm, setpointTemp, fanSpeedPercentage);
  }
  else if (screenNumber == 2) {
    displayFan(2, temp2, rpm2, setpointTemp2, fanSpeedPercentage2);
  }
  else if (screenNumber == 3) {
    displayFan(3, temp3, rpm3, setpointTemp3, fanSpeedPercentage3);
  }
}

void displayScreen0(int t1, int t2, int t3,
int r1, int r2, int r3) {
  char buf[17];
  buf[0] = 1; // flame icon
  buf[1] = ' ';
  writeTemp(buf, 2, t1);
  buf[6] = ' ';
  writeTemp(buf, 7, t2);
  buf[11] = ' ';
  writeTemp(buf, 12, t3);
  buf[16] = 0;

  lcd.setCursor(0,0);
  lcd.print(buf);

  buf[0] = 2; // fan icon
  buf[1] = ' ';
  writeRpm(buf, 2, r1);
  buf[6] = ' ';
  writeRpm(buf, 7, r2);
  buf[11] = ' ';
  writeRpm(buf, 12, r3);
  buf[16] = 0;

  lcd.setCursor(0,1);
  lcd.print(buf);
}

void displayFan(int num, int temp, int rpm, int setpoint, int percentage) {
  char buf[17];
  buf[0] = (num == 1) ? '1' : ((num == 2) ? '2' : '3');
  buf[1] = ':';
  buf[2] = 1; // flame icon
  buf[3] = ' ';
  writeTemp(buf, 4, temp);
  buf[8] = ' ';
  buf[9] = 3; // setpoint icon
  buf[10] = ' ';
  writeTemp(buf, 11, setpoint);
  buf[15] = ' ';
  buf[16] = 0;

  lcd.setCursor(0,0);
  lcd.print(buf);

  buf[0] = ' ';
  buf[1] = ' ';
  buf[2] = 2; // fan icon
  buf[3] = ' ';
  writeRpm(buf, 4, rpm);
  buf[8] = ' ';
  buf[9] = 4; // percentage icon
  buf[10] = ' ';
  writePercentage(buf, 11, percentage);
  buf[15] = ' ';
  buf[16] = 0;

  // 1 ms
  lcd.setCursor(0,1);
  // 15.3 ms
  lcd.print(buf);
}

void writeTemp(char * buf, int pos, int t) {
  if (t == -127) {
    strncpy(&buf[pos], "ERR\337", 4);
  } 
  else {
    sprintf(&buf[pos], "%3d\337", t);
  }
}

void writeRpm(char * buf, int pos, int r) {
  // TODO > 9999
  if (r == -1) {
    strncpy(&buf[pos], " ERR", 4);
  } 
  else {
    sprintf(&buf[pos], "%4d", r);
  }
}

void writePercentage(char * buf, int pos, int p) {
  if (p == 255) {
    strncpy(&buf[pos], "ERR%", 4);
  } 
  else {
    sprintf(&buf[pos], "%3d%%", p);
  }
}
#endif










