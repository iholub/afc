// read RPM
int half_revolutions = 0;
int rpm = 0;
unsigned long lcdTimeNext = 0;

int stateOld = -1;
boolean initDone = false;
unsigned long hM = 0;
unsigned long lM = 0;
unsigned long hSum = 0;
unsigned long lSum = 0;
unsigned long hCount = 0;
unsigned long lCount = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(5, INPUT);
}

void loop()
{
  int state = digitalRead(5);
  unsigned long m = micros();
  if ((stateOld == LOW) && (state == HIGH)) {
    hCount++;
    if (initDone) {
      lSum += m - lM;
    } else {
      initDone = true;
    }
    hM = m;
    half_revolutions++;
  }
  else if ((stateOld == HIGH) && (state == LOW)) {
    lCount++;
    if (initDone) {
      hSum += m - hM;
    } else {
      initDone = true;
    }
    lM = m;
  }
  stateOld = state;
  unsigned long ct = millis();
  if (ct >= lcdTimeNext){ //Uptade every one second, this will be equal to reading frecuency (Hz).
    rpm = half_revolutions * 30; // Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use half_revolutions * 30.

    Serial.print("Rpm: ");
    Serial.print(rpm);
    Serial.print(" avg low: ");
    Serial.print(lSum/lCount);
    Serial.print(" avg high: ");
    Serial.print(hSum/hCount);
    Serial.print(" count low: ");
    Serial.print(lCount);
    Serial.print(" count high: ");
    Serial.println(hCount);

    half_revolutions = 0; // Restart the RPM counter
    stateOld = -1;
    hSum = 0;
    lSum = 0;
    hCount = 0;
    lCount = 0;
    initDone = false;
    lcdTimeNext = millis() + 1000;
  }  
}

