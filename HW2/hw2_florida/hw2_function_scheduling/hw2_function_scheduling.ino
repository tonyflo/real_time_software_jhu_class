#include <Queue.h>
#include <Servo.h>

//JHU RTSW HW 2 - Optical Tachometer
//Tony Florida
//2014-09-24
//References:
// http://www.instructables.com/id/Arduino-Based-Optical-Tachometer/
// http://techvalleyprojects.blogspot.com/2012/06/arduino-control-escmotor-tutorial.html
// https://github.com/Zuph/AVRQueue

//ESC variables
// This is our motor.
Servo myMotor;
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

void setup() {
    // Required for I/O from Serial monitor
    Serial.begin(9600);
  
    //ESC setup
    Serial.println("Initializing ESC");
    // Put the motor to Arduino pin #9
    myMotor.attach(9);
    
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
    Queue myQueue;
    myQueue.scheduleFunction(esc, "ESC", 5000, 1000);

    while(1) {
        myQueue.Run(millis());
        rpm_counter();
    }
}

//Receive ESC commands via serial
int esc(unsigned long now)
{
  // If there is incoming value
  if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();
  
    /*
    *  If ch isn't a newline
    *  (linefeed) character,
    *  we will add the character
    *  to the incomingString
    */
    if (ch != 10){
      // Print out the value received
      // so that we can see what is
      // happening
      //Serial.print("I have received: ");
      //Serial.print(ch, DEC);
      //Serial.print('\n');
    
      // Add the character to
      // the incomingString
      incomingString += ch;
    }
    // received a newline (linefeed) character
    // this means we are done making a string
    else
    {
      // print the incoming string
      //Serial.println("I am printing the entire string");
      //Serial.println(incomingString);
    
      // Convert the string to an integer
      int val = incomingString.toInt();
    
      // print the integer
      //Serial.println("Printing the value: ");
      //Serial.println(val);
    
      /*
      *  We only want to write an integer between
      *  0 and 180 to the motor. 
      */
      if (val > -1 && val < 181)
      {
       // Print confirmation that the
       // value is between 0 and 180
       //Serial.println("Value is between 0 and 180");
       // Write to Servo
       myMotor.write(val);
      }
      // The value is not between 0 and 180.
      // We do not want write this value to
      // the motor.
      else
      {
       //Serial.println("Value is NOT between 0 and 180");
      
       // IT'S a TRAP!
       //Serial.println("Error with the input");
      } //0 to 180
    
      // Reset the value of the incomingString
      incomingString = "";
    } //ch not 10
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
   Serial.print(" ");
   Serial.println(rpm,DEC);
   
   //Restart the interrupt processing
   attachInterrupt(0, rpm_fun, FALLING);
}

//not using the loop in this program
void loop() {
}
