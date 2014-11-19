// Homework 3 - Calibrated Controlled Propeller
// Calibration of ESCs
// Joseph DiMarino
// Calibrates the attached ESC. Initializes the ESC, sends maxmium PWM
// to place in programmer mode, and then attached potentiometer
// is set to minimum value to finish calibration.
// Program also sends value of potentiometer converted to PWM to the
// attached ESC for testing after calibration.
#include <Servo.h>
#define MOTOR_PIN_1 9       // motor status connected to digital pin 9#define MOTOR_PIN_2 10      // motor status connected to digital pin 10Servo motor;                // attached esc
unsigned long prevTime = 0; // simulate timer

// Initialize ESC
void setup()
{
   pinMode(A0, INPUT);
   motor.attach(MOTOR_PIN_2);
   Serial.begin(9600);
} // end setup()

// Calibration of ESC
void loop()
{
   unsigned long curTime = millis();

   int adcVal = analogRead(0); // current value read from potentiometer
   int throttle = map(adcVal, 0, 1023, 1000, 2000); // map to PWM
   throttle = (throttle >= 1950) ? 2000 : throttle; // max for calibration
   throttle = (throttle <= 1050) ? 1000 : throttle; // min for calibration
   motor.writeMicroseconds(throttle); // write to ESC

   // Output PWM sent to ESC
   if (curTime - prevTime >= 1000)
   {
      prevTime = curTime;
      Serial.println(throttle);
   }

} // end loop()
