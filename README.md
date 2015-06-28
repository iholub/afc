#Arduino Fan Control

Arduino application to control speed of 4-pin fan (cooler) depending on temperature. Fan speed is determined by PID algorithm. Information is displayed on 1602 screen.

![alt tag](https://cloud.githubusercontent.com/assets/9932463/8341615/4ffc93bc-1ac6-11e5-80be-25e4d682c45d.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8341614/4ff7ad16-1ac6-11e5-9c4c-c71db2fc96cd.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8341616/50032efc-1ac6-11e5-9edf-95fa8ba489ca.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8341840/128c2932-1ac8-11e5-82f2-75b205cd3236.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8341841/12abd14c-1ac8-11e5-88c0-140e438a0d5f.jpg)
Installing DS18B20 temperature sensor and 4-wire fan to power supply:
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397852/664c891c-1dda-11e5-8797-18bbc4211376.jpg)
Replace fan in power supply to 4-wire fan.
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397853/6cf33c34-1dda-11e5-9bda-53fb346738c9.jpg)
Prepare DS18B20 temperature sensor.
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397857/7b53c1d6-1dda-11e5-9349-af0bc07514c0.jpg)
Use termal glue.
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397854/712be756-1dda-11e5-89f9-9b812eef99ef.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397855/74793c7e-1dda-11e5-9956-6922dc5acb6e.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397858/7ffbaed8-1dda-11e5-8ff4-e37fda732b33.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397871/dbce0ee0-1dda-11e5-84cc-f730d265581f.jpg)
Fix sensor with hot glue gun.
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397859/85e433ba-1dda-11e5-9d51-50e2013daba8.jpg)
Installing DS18B20 temperature sensor to video card:
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397863/9e6ece4a-1dda-11e5-9ed7-18f2d58c87dc.jpg)
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397866/a50ea19e-1dda-11e5-9f8e-8160730c7e62.jpg)
Install 4-wire fan in the place where it can blow to video card.
![alt tag](https://cloud.githubusercontent.com/assets/9932463/8397867/aa3323b6-1dda-11e5-8856-efa8a70de12e.jpg)

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
