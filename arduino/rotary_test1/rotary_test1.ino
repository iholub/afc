#define click 3         //Rotary Encoder Click
#define encoder0PinA  2 //Rotary Encoder Pin A
#define encoder0PinB  4 //Rotary Encoder Pin B
 
volatile unsigned int encoder0Pos = 0;  //Encoder value for ISR
 
void setup()
{  
  // start serial port for temperature readings
  Serial.begin(9600);
  Serial.println("Start");
 
  encoder0Pos=28;
 
  //Setup Pins
  pinMode(click, INPUT);                  // Click button is an input
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // Turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // Turn on pullup resistor
 
  //Set up Interupts
  attachInterrupt(1, clicked, RISING);    // Click button on interrupt 1 - pin 3
  attachInterrupt(0, doEncoder, CHANGE);  // Encoder pin on interrupt 0 - pin 2
 
}
void loop()
{
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
}
void clicked()
{
  Serial.println("clicked!");    //This is unused, but feel free to use the click for something using this interrupt
  delay(1000);
}
