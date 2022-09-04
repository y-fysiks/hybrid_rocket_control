#include "encoder/MD_REncoder.h"
#include <Servo.h>
#include <HX711_ADC.h>
#include <EEPROM.h>

#define DOUTLC 6 // DOUT pin for load cell
#define CLKLC 5 // CLK pin for load cell
#define CLKENCODER 7 // CLK pin for encoder
#define DTENCODER 8 // DOUT pin for encoder
#define PRESSURE_SENSOR_PIN A0 // Pressure sensor pin
#define SERVO_PIN 9 // Servo pin


MD_REncoder encoder(CLKENCODER, DTENCODER);
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int offset = 6; // Offset for the servo

double calibration_factor = 221.7125;
HX711_ADC loadcell(DOUTLC, CLKLC); // create load cell object to control a load cell

unsigned long prevDataMillis = 0;
unsigned long prevServoMillis = 0;

//Data
int throttle = 0;
float loadcell_reading = 0;
bool ARMED = false;

void setup() {
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  throttle = EEPROM.read(0);
  if (throttle < 0 || throttle > 100 || throttle % 5 != 0) {
    throttle = 100;
  }
  myservo.write(100 - ((int) (0.94 * throttle)));
  prevServoMillis = millis();

  Serial.begin(115200);
  encoder.begin();

  loadcell.begin();
  loadcell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  loadcell.start(stabilizingtime, _tare);

  if (loadcell.getTareTimeoutFlag() || loadcell.getSignalTimeoutFlag()) {
  }
  else {
    loadcell.setCalFactor(calibration_factor); // user set calibration value (float), initial value 1.0 may be used for this sketch
  }
}

bool newdata = false;

unsigned long pressureCumulative = 0;
unsigned long pressureCount = 0;

void loop() {
  unsigned long currentMillis = millis();

  if (loadcell.update()) newdata = true;


  uint8_t x = encoder.read();
  if (x == DIR_CW && throttle < 100) 
  {
    throttle += 5;
    EEPROM.write(0, throttle);
    myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
    myservo.write(100 - ((int) (0.94 * throttle)));
    prevServoMillis = millis();
  }
  else if (x == DIR_CCW && throttle > 0) 
  {
    throttle -= 5;
    EEPROM.write(0, throttle);
    myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
    myservo.write(100 - ((int) (0.94 * throttle)));
    prevServoMillis = millis();
  }
  if (prevServoMillis + 200 < currentMillis && myservo.attached()) {
    myservo.detach();
  }

  int rawPres = analogRead(PRESSURE_SENSOR_PIN);
  pressureCumulative += rawPres;
  pressureCount++;
  
  //double pressure = 0;

  if (currentMillis - prevDataMillis > 40) {
    prevDataMillis = currentMillis;
    if (newdata) {
      loadcell_reading = loadcell.getData();
      newdata = false;
    }
    float avgPres = pressureCumulative / (pressureCount * 1.0);
    float pressure = (avgPres - 102.3) * 667 / (920.7-102.3);
    Serial.print(loadcell_reading);
    Serial.print(" ");
    Serial.print(pressure);
    Serial.print(" ");
    Serial.print(throttle);
    Serial.print(" ");
    Serial.print(currentMillis);
    Serial.print(" ");
    Serial.print('\r');

    pressureCumulative = 0;
    pressureCount = 0;
  }
}
