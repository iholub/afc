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
#define TEMPERATURE_PRECISION 9
#define I2C_FAN2_ADDR 0x31
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress sensorAddress;
int volatile tempInt = -127;

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

/*-----( Declare Variables )-----*/
//NONE

// read RPM
volatile int half_revolutions = 0;
int rpm = 0;
unsigned long lcdTimeNext = 0;

unsigned long nextPotRead = 0;

int previousPotValue = -1;

int stateOld = LOW;
boolean stateHigh = false;

int time = 0;
unsigned long timeNext = 0;

unsigned long mcrMax = 0;

int temp2 = -127;
int rpm2 = -1;

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

    lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

  // Start up the library
  sensors.begin();

  // Must be called before search()
  oneWire.reset_search();

  if (oneWire.search(sensorAddress)) {
    sensors.setResolution(sensorAddress, TEMPERATURE_PRECISION);
  }

}

void loop()
{
  unsigned long m1 = micros();
  unsigned long ct = millis();
  if (ct >= timeNext) {
    timeNext = ct + 300;
    time++;
    if (time > 99) {
      time = 0;
    }
  }  
  int state = digitalRead(READ_RPM_PIN);
  if (stateOld == LOW && state == HIGH) {
    half_revolutions++;
  }
  stateOld = state;
  ct = millis();
  boolean disp = false;
  if (ct >= lcdTimeNext){ //Uptade every one second, this will be equal to reading frecuency (Hz).
    sensors.requestTemperatures();
    //delay(500);
    disp = true;
    rpm = half_revolutions * 30; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.

    float tempC = sensors.getTempC(sensorAddress);
    tempInt = round(tempC);

    readFan2();
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Rpm: ");
    lcd.print(rpm);
    lcd.print(" m:");
    lcd.print(mcrMax);

    lcd.setCursor(0,1);
    lcd.print("Rpm: ");
    lcd.print(rpm2);
    lcd.print(" t:");
    lcd.print(temp2);

    //Serial.print("RPM =\t"); //print the word "RPM" and tab.
    //Serial.print(rpm); // print the rpm value.
    //Serial.print("\t Hz=\t"); //print the word "Hz".
    //Serial.println(half_revolutions); //print revolutions per second or Hz. And print new line or enter.
    half_revolutions = 0; // Restart the RPM counter
    mcrMax = 0;
    lcdTimeNext = millis() + 1000;
  }  

  int in, out;

  ct = millis();
  if (ct >= nextPotRead) {
    nextPotRead = ct + 50;
    in = analogRead(POT_PIN);
    if ((in < previousPotValue - 5) ||
      (in > previousPotValue + 5)) {
      previousPotValue = in;
      //Serial.println("pot: ");
      //Serial.println(in);
      out = map(in, 0, 1023, 0, 79);
      OCR2B = out;
    }
  }
  unsigned long mcr = micros() - m1;
  if (!disp && (mcr > mcrMax)) {
    mcrMax = mcr;
  }
}

void readFan2() {
  byte buf [4];

  if (Wire.requestFrom(I2C_FAN2_ADDR, 4))  // if request succeeded
    {
      temp2 = Wire.read() << 8 | Wire.read();
      rpm2 = Wire.read() << 8 | Wire.read();
    }
  else
    {
    // failure ... maybe slave wasn't ready or not connected
    }
}


