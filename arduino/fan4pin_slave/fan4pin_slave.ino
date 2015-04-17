/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
//temperature
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 7
#define SENSORS_COUNT 2
#define TEMPERATURE_PRECISION 9
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress sensor[SENSORS_COUNT];
String addressString[SENSORS_COUNT];
int counter = 0;

int volatile tempInt = -127;

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

/*-----( Declare Variables )-----*/
//NONE

const int PWMPin = 3;  // Only works with Pin 3
const int PotPin = A0;  // Analog 0

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
  Serial.begin(9600);
  pinMode(5, INPUT);
  pinMode(PWMPin, OUTPUT);
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

  while ((counter < SENSORS_COUNT) && oneWire.search(sensor[counter])) {
    addressString[counter] = addressToHexString(sensor[counter]);
    Serial.print(addressString[counter]);
    Serial.print(" ");
    Serial.println(addressString[counter].substring(8));
    counter++;
  }

  // set the temperature resolution
  for (int i = 0; i < counter; i++) {
    sensors.setResolution(sensor[i], TEMPERATURE_PRECISION);
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
  int state = digitalRead(5);
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

    Serial.print("Rpm: ");
    Serial.print(rpm);
    Serial.print(" m:");
    Serial.print(mcrMax);
    for (int i = 0; i < counter; i++) {
      printTemperature(sensor[i], i);
    }
    Serial.println();

    half_revolutions = 0; // Restart the RPM counter
    mcrMax = 0;
    lcdTimeNext = millis() + 1000;
  }  

  int in, out;

  ct = millis();
  if (ct >= nextPotRead) {
    nextPotRead = ct + 50;
    in = analogRead(PotPin);
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
// this code will be executed every time the interrupt 0 (pin2) gets low.
void rpm_fan(){
  half_revolutions++;
}

// function to print a device address
String addressToHexString(DeviceAddress deviceAddress)
{
  char charArr[17]; //Note there needs to be 1 extra space for this to work as snprintf null terminates.
  char* myPtr = &charArr[0]; //or just myPtr=charArr; but the former described it better.
  for (uint8_t i = 0; i < 8; i++){
    snprintf(myPtr,3,"%02x",deviceAddress[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
    myPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
  }
  return String(charArr);
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress, int ind)
{
  float tempC = sensors.getTempC(deviceAddress);

  tempInt = round(tempC);
//TODO -127
  Serial.print(" Temp: ");
  Serial.print(tempC);
  Serial.print(" ");
  Serial.print(tempInt);


}


void slavesRespond(){
 
  int returnValue = tempInt;
 
  byte buffer[2];                 // split int value into two bytes buffer
  buffer[0] = returnValue >> 8;
  buffer[1] = returnValue & 255;
  Wire.write(buffer, 2);          // return response to last command
}
