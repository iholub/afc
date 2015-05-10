/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
//temperature
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 7
#define READ_RPM_PIN 5
#define PWM_PIN 3 // Only works with Pin 3
#define POT_PIN A0 // Analog 0

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

    // Start up the library
  sensors.begin();

  // Must be called before search()
  oneWire.reset_search();

  if (oneWire.search(sensorAddress)) {
    sensors.setResolution(sensorAddress, TEMPERATURE_PRECISION);
  }

  Wire.begin(0x31);
  Wire.onRequest(slavesRespond);  // Perform on master's request
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
    disp = true;
    rpm = half_revolutions * 30; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.

    //Serial.print("Rpm: ");
    //Serial.print(rpm);
    //Serial.print(" m:");
    //Serial.print(mcrMax);

    float tempC = sensors.getTempC(sensorAddress);
    tempInt = round(tempC);
    //TODO -127
    //Todo fix
    rpm = 1000;

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

void slavesRespond(){
  byte buffer[4];                 // split int value into two bytes buffer
  buffer[0] = tempInt >> 8;
  buffer[1] = tempInt & 255;
  buffer[2] = rpm >> 8;
  buffer[3] = rpm & 255;
  Wire.write(buffer, 4);          // return response to last command
}

