const int PWMPin = 3;  // Only works with Pin 3
const int PotPin = A0;  // Analog 0

// read RPM
volatile int half_revolutions = 0;
int rpm = 0;
unsigned long lastmillis = 0;

unsigned long nextPotRead = 0;

int previousPotValue = -1;

void setup()
{
  Serial.begin(9600);
  pinMode(PWMPin, OUTPUT);
  // Fast PWM Mode, Prescaler = /8
  // PWM on Pin 3, Pin 11 disabled
  // 16Mhz / 8 / (79 + 1) = 25Khz
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
  // Set TOP and initialize duty cycle to zero(0)
  OCR2A = 79;   // TOP - DO NOT CHANGE, SETS PWM PULSE RATE
  OCR2B = 0;    // duty cycle for Pin 3 (0-79) generates 1 500nS pulse even when 0

    attachInterrupt(0, rpm_fan, FALLING);
}

void loop()
{
  if (millis() - lastmillis == 1000){ //Uptade every one second, this will be equal to reading frecuency (Hz).
    detachInterrupt(0);//Disable interrupt when calculating
    rpm = half_revolutions * 30; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.
    Serial.print("RPM =\t"); //print the word "RPM" and tab.
    Serial.print(rpm); // print the rpm value.
    Serial.print("\t Hz=\t"); //print the word "Hz".
    Serial.println(half_revolutions); //print revolutions per second or Hz. And print new line or enter.
    half_revolutions = 0; // Restart the RPM counter
    lastmillis = millis(); // Uptade lasmillis
    attachInterrupt(0, rpm_fan, FALLING); //enable interrupt
  }  
  
  int in, out;

  unsigned long currTime = millis();
  if (currTime >= nextPotRead) {
    nextPotRead = currTime + 50;
    in = analogRead(PotPin);
    if ((in < previousPotValue - 5) ||
    (in > previousPotValue + 5)) {
      previousPotValue = in;
      Serial.println("pot: ");
      Serial.println(in);
      out = map(in, 0, 1023, 0, 79);
      OCR2B = out;
    }
  }

}
// this code will be executed every time the interrupt 0 (pin2) gets low.
void rpm_fan(){
  half_revolutions++;
}


