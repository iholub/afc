#Arduino Fan Control

Arduino application to control speed of 4-pin fan (cooler) depending on temperature. Fan speed is determined by PID algorithm. Information is displayed on 1602 screen.

Hardware required for 1 fan:
- Arduino Pro Mini 5v or similar (Arduino Uno, Arduino Nano)
- 12v 4-pin pc fan (cooler)
- lcd display 1602
- 10k potentiomenter (for changing screens on lcd)
- 10k potentiomenter (for changing setpoint temperature)
- temperature sensor DS18B20
- 4.7k resistor (for DS18B20 pull-up)
- 10k resistor (for rpm wire pull-up)
- 10k resistor (for I2C SDA wire pull-up)
- 10k resistor (for I2C SCL wire pull-up)

For each additional fan following hardware required:
- Arduino Pro Mini 5v or similar (Arduino Uno, Arduino Nano)
- 12v 4-pin pc fan (cooler)
- 10k potentiomenter (for changing setpoint temperature)
- temperature sensor DS18B20
- 4.7k resistor (for DS18B20 pull-up)
- 10k resistor (for rpm wire pull-up)

Current code is written for 2 fans.
The first arduino is master, it controls its own fan, reads informations from other arduinos by I2C protocol and displays all information on lcd.
Each other arduino is slave, it controls its own fan and send information to master arduino when requested. 

Wiring for each slave arduino:  
pin VCC: to 5v  
pin GND: to ground  
pin A0: to data wire of potentiometer to control setpoint temperature  
pin 2: to RPM wire of 4-pin fan, with 10k pull-up resistor  
pin 3: to PWM wire of 4-pin fan  
pin 7: to data wire of DS18B20, with 4.7k pull-up resistor  
pin A4: to A4 pin of master arduino  
pin A5: to A5 pin of master arduino  

Wiring for master arduino is the same as for slave, and additionally:  
pin A2: to data wire of potentiometer to change screens on lcd    
pin A4: to SDA pin of lcd, with 10k pull-up resistor  
pin A5: to SCL pin of lcd, with 10k pull-up resistor  

For Arduino Uno there are special pins SDA and SCL instead of A4 and A5.

Wiring for 4-pin fan:  
 - ground: to ground of power supply, to GND of arduino
 - 12v: to 12v
 - RPM: see Arduino wiring above
 - PWM: see Arduino wiring above

Wiring for 10k potentiometers:
- ground: to ground
- 5v: to 5v
- data wire (usually in the middle): see Arduino wiring above

Wiring for DS18B20:  
- ground: to ground
- 5v: to 5v
- data wire (usually in the middle): see Arduino wiring above

Wiring for lcd screen 1602:  
- ground: to ground
- 5v: to 5v
- SDA: see Arduino wiring above
- SCL: see Arduino wiring above
