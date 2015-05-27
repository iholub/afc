/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
//temperature
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#define ONE_WIRE_BUS 7
#define READ_RPM_PIN 5
#define PWM_PIN 3 // Only works with Pin 3
#define POT_PIN A0 // Analog 0
#define TEMP_PERIOD 250
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
int stateOld;
int rpmInit = false;
int volatile rpm = -1;
unsigned long rpmTimeStart;
unsigned long potTimeNext = 0;
unsigned long tempTimeNext;
#define RPM_READ_COUNT 4
unsigned long rpmTimeSum = 0;
int revolutionsSum = 0;
int revolutions;
int currRpmRead = 0;

int potValue;

//Setup PID
double Setpoint, Input, Output; //I/O for PID
double aggKp=40, aggKi=2, aggKd=10;//original: aggKp=4, aggKi=0.2, aggKd=1, Aggressive Turning,50,20,20
double consKp=20, consKi=1, consKd=5; //original consKp=1, consKi=0.05, consKd=0.25, Conservative Turning,20,10,10
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);  //Initialize PID

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

  potValue = analogRead(POT_PIN);
  potToTemp();

  //PID Setup
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(TEMP_PERIOD);
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
    double gap = Setpoint-Input; //distance away from setpoint
//    if(gap<1)
//    {  
      //Close to Setpoint, be conservative
//      myPID.SetTunings(consKp, consKi, consKd);
//    }
//    else
//    {
//      //Far from Setpoint, be aggresive
//      myPID.SetTunings(aggKp, aggKi, aggKd);
//    } 
    myPID.Compute();
    
    int out = map(Output, 0, 255, 0, 79);
    OCR2B = out;

    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(gap);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print(" ");
    Serial.print(out);
    Serial.println(" ");

    tempTimeNext = millis() + TEMP_PERIOD;

    restartRpm();
    rpmInit = true;
  } 
  else {
    if (rpmInit) {
      readRpm();
    }
  }

  if (millis() >= potTimeNext) {
    readPot();
    potToTemp();
    potTimeNext = millis() + POT_PERIOD;
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
  else
    if (in > 970) {
      in = 1023;
    }
  Setpoint = map(in, 0, 1023, -55, 125);
}

void readPot() {
  //unsigned long m1 = micros();
  int in = analogRead(POT_PIN);
  if ((in < potValue - 3) || (in > potValue + 3)) {
    potValue = in;
    //Serial.println("pot: ");
    //Serial.println(in);
  }
  //Serial.println(micros() - m1);
}

void slavesRespond(){
  byte buffer[4];                 // split int value into two bytes buffer
  buffer[0] = tempInt >> 8;
  buffer[1] = tempInt & 255;
  buffer[2] = rpm >> 8;
  buffer[3] = rpm & 255;
  Wire.write(buffer, 4);          // return response to last command
}






