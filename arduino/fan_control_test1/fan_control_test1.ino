#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
 
//Definitions
#define FAN 9           // Output pin for fan
#define ONE_WIRE_BUS 8  // Temperature Input is on Pin 2
#define click 3         //Rotary Encoder Click
#define encoder0PinA  2 //Rotary Encoder Pin A
#define encoder0PinB  4 //Rotary Encoder Pin B
#define CRITICAL 50.00  //Critical temperature to ignore PID and turn on fans
 
volatile unsigned int encoder0Pos = 0;  //Encoder value for ISR
 
LiquidCrystal lcd(12, 11, 13, 5,6,7);  //set up LCD
 
//Setup Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 
//Setup PID
double Setpoint, Input, Output;                                             //I/O for PID
double aggKp=40, aggKi=2, aggKd=10;                                         //original: aggKp=4, aggKi=0.2, aggKd=1, Aggressive Turning,50,20,20
double consKp=20, consKi=1, consKd=5;                                       //original consKp=1, consKi=0.05, consKd=0.25, Conservative Turning,20,10,10
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);  //Initialize PID
 
//interface
int timeCounter;
void setup()
{  
  // start serial port for temperature readings
  Serial.begin(9600);
  Serial.println("Start");
 
  //Temperature Setup
  sensors.begin();                    //Start Library
  sensors.requestTemperatures();      // Send the command to get temperatures
  Input = sensors.getTempCByIndex(0); //Set Input to Current Temperature
  Setpoint = 28;                      //Inintialize desired Temperature in Deg C
  encoder0Pos=28;
 
  //PID Setup
  myPID.SetMode(AUTOMATIC);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;  //adjust the PWM Frequency, note: this changes timing like delay()
 
  //Setup Pins
  pinMode(FAN, OUTPUT);                   // Output for fan speed, 0 to 255
  pinMode(click, INPUT);                  // Click button is an input
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // Turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // Turn on pullup resistor
 
  //Set up Interupts
  attachInterrupt(1, clicked, RISING);    // Click button on interrupt 1 - pin 3
  attachInterrupt(0, doEncoder, CHANGE);  // Encoder pin on interrupt 0 - pin 2
 
  //interface
  timeCounter=0;
 
  //Setup LCD 16x2 and display startup message
  lcd.begin(16, 2);
  lcd.print("  Smart   Fan");
  lcd.setCursor(0,1);
  lcd.print("  Starting Up");
  delay(1000);
  lcd.clear();
}
void loop()
{
   timeCounter++;
 
   //Get temperature and give it to the PID input
   sensors.requestTemperatures();
   Input=sensors.getTempCByIndex(0);
 
  //print out info to LCD
  lcd.setCursor(1,0);
  lcd.print("Temp:");
  lcd.print((int)Input);
  lcd.setCursor(9,0);
  lcd.print("RPM:");
  lcd.print((int)Output*4.7059);    //Scaling factor calculated with a regression, read yellow wire for real RPM
  lcd.setCursor(1,1);
  lcd.print("Set:");
  lcd.print((int)Setpoint);
 
  //Compute PID value
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<1)
  {  
    //Close to Setpoint, be conservative
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //Far from Setpoint, be aggresive
     myPID.SetTunings(aggKp, aggKi, aggKd);
  } 
  myPID.Compute();
  Serial.print(timeCounter);
  Serial.print(" ");
  Serial.print(Input);
  Serial.print(" ");
  Serial.println(Output);
  analogWrite(FAN,255);
  //Write PID output to fan if not critical
  if (Input<CRITICAL)
    analogWrite(FAN,Output);
  else
    analogWrite(FAN,255);
}
 
void doEncoder()
{
  //pinA and pinB are both high or both low, spinning forward, otherwise it's spinning backwards
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
  {
    encoder0Pos++;
  }
  else
  {
    encoder0Pos--;
  }
  Serial.println (encoder0Pos, DEC);  //Print out encoder value to Serial
  Setpoint=encoder0Pos;
}
void clicked()
{
  //For interface
  lcd.clear();
  lcd.print("clicked!");    //This is unused, but feel free to use the click for something using this interrupt
  delay(1000);
}
