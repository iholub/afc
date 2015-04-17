volatile byte half_revolutions;
 unsigned int rpm;
 unsigned long timeold;
 void setup()
 {
   Serial.begin(9600);
   attachInterrupt(0, rpm_fun, RISING);
   half_revolutions = 0;
   rpm = 0;
   timeold = 0;
 }
 void loop()
 {
   if (half_revolutions >= 20) { 
     cli();
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     rpm = 30*1000/(millis() - timeold)*half_revolutions;
     half_revolutions = 0;
     Serial.print("rpm1: ");
     Serial.println(rpm,DEC);
     timeold = millis();
     sei();
   }
 }
 void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
 }
//-----------------------------------------------
