#include <ClickEncoder.h>
#include <TimerOne.h>

const int PWMPin = 3;  // Only works with Pin 3
const int PotPin = 0;  // Analog 0

ClickEncoder *encoder;
int16_t last;
int16_t value = 80;

void timerIsr() {
  encoder->service();
}

void setup() {
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

  encoder = new ClickEncoder(5, 6, 7);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  last = -1;
}

void loop() {  
  value += encoder->getValue();
  
  if (value != last) {
    last = value;
    int regOut;
    if (value < 0) {
      regOut = 0;
    } else if (value >= 80) {
      regOut = 79;
    } else {
      regOut = map(value, 0, 80, 0, 79);
    }
    OCR2B = regOut;
    Serial.print("Encoder Value: ");
    Serial.println(value);
    Serial.print("Reg Value: ");
    Serial.println(regOut);
  }
  
  ClickEncoder::Button b = encoder->getButton();
    switch (b) {
      case ClickEncoder::Clicked:
          Serial.println("Button ClickEncoder::Clicked");
          break;
      case ClickEncoder::DoubleClicked:
          Serial.println("Button ClickEncoder::DoubleClicked");
        break;
    }
}

