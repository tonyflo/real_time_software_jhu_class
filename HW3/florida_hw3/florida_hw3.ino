#include <Queue.h>
#include <Servo.h>

//JHU RTSW HW 3 - HW3 - Calibrated Controlled Propeller
//Tony Florida
//2014-10-06
//References:
// http://www.instructables.com/id/Arduino-Based-Optical-Tachometer/
// http://techvalleyprojects.blogspot.com/2012/06/arduino-control-escmotor-tutorial.html
// https://github.com/Zuph/AVRQueue

//CMD variables (range of 15 to 150)
int serial_cmd = 5; //serial commands start at 10
int MAX_SERIAL_CMD = 100; //max serial command
int SERIAL_CMD_INCREMENT = 5; //increment serial commands by 20

//Queue variables
Queue myQueue;

//ESC variables
// This is our motor.
Servo myMotor;
Servo myMotor2;
// This is the final output
// written to the motor.
String incomingString;

//IR Emitter Detector variables
int ledPin = 13;                // IR LED connected to digital pin 13
int statusPin = 12;             // LED connected to digital pin 12
volatile byte rpmcount;
volatile int status;
unsigned int rpm;
unsigned long timeold;

//IR Emitter Detector function
void rpm_fun()
{
   //Each rotation, this interrupt function is run twice, so take that into consideration for 
   //calculating RPM
   //Update count
      rpmcount++;
      
   //Toggle status LED   
   if (status == LOW) {
     status = HIGH;
   } else {
     status = LOW;
   }
   digitalWrite(statusPin, status);
}

int cmd()
{
  serial_cmd+=SERIAL_CMD_INCREMENT; //increment the serial command
  if(serial_cmd > MAX_SERIAL_CMD)
  {
    SERIAL_CMD_INCREMENT *= -1; //incrementally spin down the motors
  }
  
  if(serial_cmd <= 0)
  {
      // we are done!
      myQueue.scheduleRemoveFunction("ESC");
  }
  
  return serial_cmd;
}

void setup() {
    // Required for I/O from Serial monitor
    Serial.begin(9600);
  
    //ESC setup
    Serial.println("Initializing ESC");
    // Put the motors to Arduino pin 9 and 10
    myMotor.attach(9);
    myMotor2.attach(10);
    
    //IR Emitter Detector setup
    //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
    //Triggers on FALLING (change from HIGH to LOW)
    attachInterrupt(0, rpm_fun, FALLING);
    //Turn on IR LED
    pinMode(ledPin, OUTPUT); 
    digitalWrite(ledPin, HIGH);
    //Use statusPin to flash along with interrupts
    pinMode(statusPin, OUTPUT);
    rpmcount = 0;
    rpm = 0;
    timeold = 0;
    status = LOW;

    //Function queue scheduling setup
    Serial.println("Initializing function queue scheduling");
    myQueue.scheduleFunction(esc, "ESC", 0, 3000);

   //Print table header
   Serial.println("Time(ms),RPM,Command");
   
   //Wait until start command i.e. any input serial comms
   Serial.println("Plug battery into motors, then send any serial command");
   while(!Serial.available()) {}
   
    while(1) {
        myQueue.Run(millis());
        rpm_counter();
    }
}



//Receive ESC commands via serial
int esc(unsigned long now)
{
   int val = cmd(); //new rotation speed

   /*  We only want to write an integer between
    *  0 and 180 to the motor. 
    */
   if (val > -1 && val < 181)
   {
       // Print confirmation that the
       // value is between 0 and 180
       // Write to Servo
       myMotor.write(val);
       myMotor2.write(val);
   }
}

//Count RPMs
void rpm_counter()
 {
   //Update RPM every second
   delay(1000);
   //Don't process interrupts during calculations
   detachInterrupt(0);
   //Note that this would be 60*1000/(millis() - timeold)*rpmcount if the interrupt
   //happened once per revolution instead of twice. Other multiples could be used
   //for multi-bladed propellers or fans
   rpm = 30*1000/(millis() - timeold)*rpmcount;
   timeold = millis();
   rpmcount = 0;
   
   //Write it out to serial port
   Serial.print(millis());
   Serial.print(",");
   Serial.print(rpm,DEC);
   Serial.print(",");
   Serial.println(serial_cmd);
   
   //Restart the interrupt processing
   attachInterrupt(0, rpm_fun, FALLING);
}

//not using the loop in this program
void loop() {
}
