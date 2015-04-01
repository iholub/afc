#include <ClickEncoder.h>
#include <TimerOne.h>

#define FAN 9           // Output pin for fan

ClickEncoder *encoder;
int16_t last;
int16_t value = 80;
int pwmOut = 255; //full speed

void timerIsr() {
  encoder->service();
}

void setup() {
  Serial.begin(9600);

  //Setup Pins
  pinMode(FAN, OUTPUT);                   // Output for fan speed, 0 to 255  
  analogWrite(FAN, pwmOut);  
  encoder = new ClickEncoder(5, 6, 7);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  last = -1;
}

void loop() {  
  value += encoder->getValue();
  
  if (value != last) {
    last = value;
    if (value < 0) {
      pwmOut = 0;
    } else if (value >= 80) {
      pwmOut = 255;
    } else {
      int z = value % 80;
      pwmOut = map(z, 0, 80, 0, 255);
    }
    analogWrite(FAN, pwmOut);
    Serial.print("Encoder Value: ");
    Serial.println(value);
    Serial.print("Pwm Value: ");
    Serial.println(pwmOut);
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

