// Homework 3 - Calibrated Controlled Propeller
// Control and Determine Speed of Propellers
// Joseph DiMarino
// Calculates the Revolutions Per Minute (RPMs) of an attached
// propellers. RPM value determined through IR detector
// and emitter. When fan/propeller blade blocks IR from the
// detector and interrupt is run determining the duration
// since the last interrupt. This duration is used to
// calculate the RPM of the attached device.
// A simulation is run for discrete values representing up to 50%
// of max RPM. After which control can be sent through an attached
// potentiometer.
#include <Servo.h>   // needed for ESC manipulation
#include <FreeRTOS_AVR.h> // needed for RTOS

#define ESC_PIN_1 9   // ESC 1 status connected to digital pin 9
#define ESC_PIN_2 10   // ESC 1 status connected to digital pin 9
#define ESC_PIN_3 11   // ESC 1 status connected to digital pin 9
#define ESC_PIN_4 12   // ESC 1 status connected to digital pin 9

Servo motor1;            // motor to send voltage values to
Servo motor2;            // motor to send voltage values to
Servo motor3;            // motor to send voltage values to
Servo motor4;            // motor to send voltage values to

static int throttle;        // PWM of ESC
static int modifier;        // based on 5-seconds for simulation

const int MIN_VAL = 0;           // minimum accepted ESC voltage value
const int MAX_VAL = 180;         // maximum accepted ESC voltage value
const int SAMPLE_SIZE = 5;       // number of samples

const int FINAL_STEP = 20;
const int MIDDLE_STEP = 10;

// Thread 1 - PWM write to ESCs at 50hz
static void Thread1(void* arg)
{
   while (1)
   {
      if(Serial.available() > 0)
      {
         // read the value
         int newValue = Serial.read() == 'a' ? 1000 : 1100;
         throttle = newValue;  
         Serial.println("LOLLOL:LDSD");
      }
     Serial.println(throttle);
      motor1.writeMicroseconds(throttle);
      motor2.writeMicroseconds(throttle);
      motor3.writeMicroseconds(throttle);
      motor4.writeMicroseconds(throttle);
      
      vTaskDelay((50L * configTICK_RATE_HZ) / 1000L); // run at 50 hz
   }

} // end Thread1(void* arg)

//// Thread 3 - Print values
//static void Thread3(void* arg)
//{
//   while (1)
//   {
//      // Write it out to serial port in CSV form
//      Serial.print(millis() * .001, 1); // print current duration
//      Serial.print(",");
//      Serial.print(map(throttle, 1000, 2000, 0, 1023)); // map discrete value
//      Serial.print(",");
//      Serial.print(rpm1, DEC); // print rpm
//      Serial.print(",");
//      Serial.println(rpm2, DEC); // print rpm
//      vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L); // run every second
//   }
//} // end Thread3(void* arg)

// Setup run after Arduino startup
void setup()
{
   modifier = 0;
   portBASE_TYPE s1, s2, s3;
   motor1.attach(ESC_PIN_1); // connection of motor's ESC
   motor2.attach(ESC_PIN_2); // connection of motor's ESC
   motor3.attach(ESC_PIN_3); // connection of motor's ESC
   motor4.attach(ESC_PIN_4); // connection of motor's ESC

   // Required for I/O from Serial monitor
   Serial.begin(9600);

   throttle = 2000;

   s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE + 50, NULL, 1,
         NULL);
//   s3 = xTaskCreate(Thread3, NULL, configMINIMAL_STACK_SIZE + 75, NULL, 3,
//         NULL);

   // start scheduler
   vTaskStartScheduler();
   Serial.println(F("Insufficient RAM"));
   while (1)
      ;
} // end setup()

// Program execution after setup
void loop()
{
   // Not used
} // end loop()

