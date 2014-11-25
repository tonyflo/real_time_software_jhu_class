// Recieve roll, pitch, yaw from BBB via USB serial

#include <Servo.h>

//constants
#define num_cmds 4 //4 commands, 1 for each motor

//variables
String command; // accumulating string read from serail
String command_arr[num_cmds]; //array of strings
int command_val[num_cmds]; //array of ints
int command_i; // counter to keep track of which value we're parsing

//objects
Servo motor06;
Servo motor09;
Servo motor10;
Servo motor11;

void variableReset()
{ 
  for(int i = 0; i < num_cmds; i++)
  {
    command_arr[i] = "";
  }
  command_i = 0;
  command = ""; 
}

void setup() {       
  // Serial init  
  Serial.begin(9600);
  
  // Variable init
  variableReset();
  
  // Associate the motors at the appropriate Arduino pins
  motor06.attach(6);
  motor09.attach(9);
  motor10.attach(10);
  motor11.attach(11);
}

// When a delimeter has been found, save the value (r, p, y) into an array
void save_value()
{
  // Save the current r or p or y to the array
  command_arr[command_i]=command;
  
  // Moving onto next index in sequence of r, p, y
  command="";
  command_i++; 
}

void update_command()
{
  boolean bad = false;
  
  // Check for an invalid command
  for(int i = 0; i < num_cmds; i++)
  {
    if( command_arr[i] == "")
    {
      Serial.print("bad @ ");
      Serial.println(i);
      // This command seems to be invalid, throw it away
      bad = true;
      break; 
    }
    else
    {
      Serial.print("good ");
      Serial.println( command_arr[i]);
    }
  }
  
  if(!bad)
  {
    // We have a valid commmand so update the motor values
    for(int i = 0; i < num_cmds; i++)
    {
      command_val[i]=command_arr[i].toInt();
    }
  }
}

void spin_motors()
{
  // Spin the motors
  for(int i = 0; i < num_cmds; i++)
  {
    Serial.print(command_val[i]);
    Serial.print('\t');
  }
  Serial.print('\n');  
  motor06.writeMicroseconds(command_val[0]);
  motor09.writeMicroseconds(command_val[1]);
  motor10.writeMicroseconds(command_val[2]);
  motor11.writeMicroseconds(command_val[3]);
}

void loop() {
  if(Serial.available() > 0)
  {
    // Read serial character
    char inChar = Serial.read();
     
    if(inChar == ',')
    {
      save_value();
    }
    // New line signals end command string
    else if(inChar == '\n')
    {
      save_value();
      
      //Update with a new command
      update_command();
      
      // Ready for next value
      variableReset();
    }
    else
    {
      // Accumulate the command string one character at a time
      command += inChar;
    }
  }
  
  spin_motors();
}
