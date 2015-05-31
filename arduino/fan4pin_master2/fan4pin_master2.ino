/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include <LiquidCrystal_I2C.h>
//temperature
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 7
#define READ_RPM_PIN 5
#define PWM_PIN 3 // Only works with Pin 3
#define POT_PIN A0 // Analog 0
#define RPM_PERIOD 1000
#define TEMP_PERIOD 100
#define POT_PERIOD 100
#define TEMPERATURE_PRECISION 9
#define I2C_FAN2_ADDR 0x31
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress sensorAddress;

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

/*-----( Declare Variables )-----*/
//NONE

int volatile tempInt = -127;

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

/*-----( Declare Variables )-----*/

// read RPM
int stateOld = LOW;
int half_revolutions = 0;
int volatile rpm = 0;
unsigned long rpmTimeNext = 0;
unsigned long potTimeNext = 0;
unsigned long tempTimeNext;

int previousPotValue = -1;

int temp2 = -127;
int rpm2 = -1;
int setpointTemp2 = -1; 
byte fanSpeedPercentage2 = 255;

int volatile setpointTemp = -127;
byte volatile fanSpeedPercentage = 255;

void setup()
{
  Serial.begin(9600);
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

    lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

    // Start up the library
  sensors.begin();
  sensors.setWaitForConversion(false);

  // Must be called before search()
  oneWire.reset_search();

  if (oneWire.search(sensorAddress)) {
    sensors.setResolution(sensorAddress, TEMPERATURE_PRECISION);
  }

  sensors.requestTemperatures();
  tempTimeNext = millis() + TEMP_PERIOD;

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

  if (millis() >= rpmTimeNext){ //Uptade every one second, this will be equal to reading frecuency (Hz).
    rpm = half_revolutions * 30; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.

    half_revolutions = 0; // Restart the RPM counter
    
    readFan2();

    displayFan(0, tempInt, rpm, setpointTemp, fanSpeedPercentage);
displayFan(1, temp2, rpm2, setpointTemp2, fanSpeedPercentage2);
    
 //   lcd.setCursor(0,1);
    //tempStr = tempToStr(temp2);
   // int tempZ = temp2;
    //if (tempZ == -127) {
    //  tempZ = -99;
    //}
    //char buf[17];
    //sprintf(buf, "%3d%5d%4d%4d", tempZ, rpm2, setpointTemp2, fanSpeedPercentage2);
    //lcd.print(buf);
    
    rpmTimeNext = millis() + RPM_PERIOD;
  }  
}

void displayFan(int row, int temp, int rpm, int setpoint, int percentage) {
    // 33 ms
    unsigned long m1 = micros();

  char buf[17];
    //1 ms
  lcd.setCursor(0,row);
    //lcd.clear();
    unsigned long m2 = micros();
    //String tempStr = tempToStr(tempInt);
    if (temp == -127) {
      char tempBuf[4] = "ERR";
      strncpy(buf, tempBuf, 3);
    } else {
      sprintf(buf, "%3d", temp);
    }
    unsigned long m3 = micros();
    sprintf(&buf[3], "%5d", rpm);
    // 0.24 ms
    //sprintf(buf, "%3d%5d%4d%4d", );
    unsigned long m4 = micros();
    // 15.3 ms
    lcd.print(buf);
    unsigned long m5 = micros();
   Serial.print(m2 - m1);
    Serial.print(" ");
    Serial.print(m3 - m2);
    Serial.print(" ");
    Serial.print(m4 - m3);
    Serial.print(" ");
    Serial.print(m5 - m4);
    Serial.print(" ");
    Serial.println(micros() - m1);
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
    } else
    if (in > 980) {
      in = 1023;
    }
    if (in != previousPotValue) {
      previousPotValue = in;
      //Serial.println("pot: ");
      //Serial.println(in);
      int out = map(in, 0, 1023, 0, 79);
      OCR2B = out;
    }
  }

}

void readFan2() {
  byte buf [7];

  if (Wire.requestFrom(I2C_FAN2_ADDR, 7))  // if request succeeded
    {
      temp2 = Wire.read() << 8 | Wire.read();
      rpm2 = Wire.read() << 8 | Wire.read();
      setpointTemp2 = Wire.read() << 8 | Wire.read();
      fanSpeedPercentage2 = Wire.read();
    }
  else
    {
    // failure ... maybe slave wasn't ready or not connected
    }
}


