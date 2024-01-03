#include "encoder/MD_REncoder.h"
#include <Servo.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include "PWMread_RCfailsafe.hpp"

#define PREFIRE_THROT 35 // Throttle value to prefire at

#define DOUTLC 6 // DOUT pin for load cell
#define CLKLC 5 // CLK pin for load cell
#define CLKENCODER 7 // CLK pin for encoder
#define DTENCODER 8 // DOUT pin for encoder
#define PRESSURE_SENSOR_PIN A0 // Pressure sensor pin
#define SERVO_PIN 9 // Servo pin

#define LC_calibration_factor 221.7125 // Calibration factor for load cell

#define USE_RC true // Set to true if using PWM RC receiver, false if using rotary encoder

MD_REncoder encoder(CLKENCODER, DTENCODER);

Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards
int servoDir = -1; // 1 is normal, -1 is reversed
float servo_rate = 0.53;
float servo_subtrim = -0.4;

unsigned long now;

const int channels = 3;
unsigned long rc_update;
float RC_in[channels];

HX711_ADC loadcell(DOUTLC, CLKLC); // create load cell object to control a load cell

unsigned long prevDataMillis = 0;
unsigned long prevServoMillis = 0;

//Data
int throttle = 0; // throttle value normalized from 0-100
float loadcell_reading = 0;

int calcThrottle(int throttle) { // returns PWM value for throttle value. Input takes 0-100
    double pwm = throttle / 50.0 - 1.0;
    int cmd = 1500 + (pwm*servo_rate * servoDir + servo_subtrim)*1000;   // apply servo rates and sub trim, then convert to a   uS value
    if(cmd > 2500) cmd = 2500;                                      //   limit pulsewidth to the range 500 to 2500us
    else if(cmd < 500) cmd = 500;
    return cmd;
}

int calcThrottlePWM(double RCThrot) {
    RCThrot = constrain(RCThrot, -1.0, 1.0);
    int cmd = 1500 + (RCThrot*servo_rate * servoDir + servo_subtrim)*1000;   // apply servo rates and sub trim, then convert to a   uS value
    if(cmd > 2500) cmd = 2500;                                      //   limit pulsewidth to the range 500 to 2500us
    else if(cmd < 500) cmd = 500;
    return cmd;
}

void setup() {
    myservo.attach(SERVO_PIN, 500, 2500); // attaches the servo on pin 9 to the servo object


    if (USE_RC) {
        setup_pwmRead(); // Setup PWM read
    }
    else {
        throttle = EEPROM.read(0);
        if (throttle < 0 || throttle > 100 || throttle % 5 != 0) {
            throttle = 100;
        }
        myservo.write(calcThrottle(throttle));
        encoder.begin();
    }

    prevServoMillis = millis();

    Serial.begin(115200);

    loadcell.begin();
    loadcell.setReverseOutput(); //uncomment to turn a negative output value to positive
    unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    loadcell.start(stabilizingtime, _tare);

    if (loadcell.getTareTimeoutFlag() || loadcell.getSignalTimeoutFlag()) {
    }
    else {
        loadcell.setCalFactor(LC_calibration_factor); // user set calibration value (float), initial value 1.0 may be used for this sketch
    }

    // while (RC_decode(2) > 0.0) {
    //     delay(50);
    // }
}

bool newdata = false;

unsigned long pressureCumulative = 0;
unsigned long pressureCount = 0;

bool armed = false;
bool fired = false;

void loop() {
    now = millis();

    if (loadcell.update()) newdata = true;

    if (USE_RC) {
        if(RC_avail() || now - rc_update > 25){   // if RC data is available   or 25ms has passed since last update (adjust to > frame rate of receiver)
      
            rc_update = now;                           
            
            // print_RCpwm();                         // uncommment to print raw data from receiver to serial
                            
            for (int i = 0; i<channels; i++){       // run through each RC channel
                int CH = i+1;
                
                RC_in[i] = RC_decode(CH);             //   decode receiver channel and apply failsafe
                
                //print_decimal2percentage(RC_in[i]);    // uncomment to print calibrated receiver input (+-100%) to serial       
            }

            double throtCMD = RC_in[0];       // variables to store the pulse widths to be sent to the servo. -1-1 range
            throtCMD = constrain(throtCMD, -1.0, 1.0);

            throttle = round((throtCMD + 1.0) * 50.0);

            if (!armed) {
                armed = (RC_in[1] > 0.0 && throttle < 5.0);
            } else {
                armed = (RC_in[1] > 0.0);
            }

            bool prefire = RC_in[2] > 0.0;

            int servo1_uS = calcThrottlePWM(throtCMD);

            if (armed) {
                Serial.print(servo1_uS);
                Serial.print(" ");
                Serial.print(throttle);
                Serial.print("\n");
                if (armed) {
                    //set pre-fire throttle to ignite engine
                    if (throttle > PREFIRE_THROT || fired) {
                        myservo.writeMicroseconds(servo1_uS); // send pulsewidth to servo
                        fired = true;
                    } else {
                        myservo.writeMicroseconds(calcThrottlePWM(PREFIRE_THROT / 50.0 - 1.0));
                    }
                } else {
                  myservo.writeMicroseconds(calcThrottlePWM(-1.0));
                  fired = false;
                }
            }
            else {
              myservo.writeMicroseconds(calcThrottlePWM(-1.0));
              fired = false;
            }


       }
    } else {
        uint8_t x = encoder.read();
        if (x == DIR_CW && throttle < 100) 
        {
            throttle += 5;
            EEPROM.write(0, throttle);
            if (!myservo.attached()) myservo.attach(SERVO_PIN); // attaches the servo on pin 9 to the servo object
            myservo.write(calcThrottle(throttle));
            prevServoMillis = millis();
            digitalWrite(LED_BUILTIN, HIGH);
        }
        else if (x == DIR_CCW && throttle > 0) 
        {
            throttle -= 5;
            EEPROM.write(0, throttle);
            if (!myservo.attached()) myservo.attach(SERVO_PIN); // attaches the servo on pin 9 to the servo object
            myservo.write(calcThrottle(throttle));
            prevServoMillis = millis();
        }
        if (prevServoMillis + 400 < now && myservo.attached()) {
            myservo.detach();
        }
    }
    

    int rawPres = analogRead(PRESSURE_SENSOR_PIN);
    pressureCumulative += rawPres;
    pressureCount++;
    
    //double pressure = 0;

    if (now - prevDataMillis > 100) {
        prevDataMillis = now;
        if (newdata) {
            loadcell_reading = loadcell.getData();
            newdata = false;
        }
        float avgPres = pressureCumulative / (pressureCount * 1.0);
        float pressure = (avgPres - 102.3) * 652.5 / (920.7-102.3) + 14.5;
        // Serial.print(loadcell_reading);
        // Serial.print(" ");
        // Serial.print(pressure);
        // Serial.print(" ");
        // Serial.print(throttle);
        // Serial.print(" ");
        // Serial.print(now);
        // Serial.print(" ");
        // Serial.print('\r');

        pressureCumulative = 0;
        pressureCount = 0;
    }
}
