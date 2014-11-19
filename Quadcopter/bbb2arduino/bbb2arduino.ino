// Recieve roll, pitch, yaw from BBB via USB serial
// http://electronics.stackexchange.com/questions/45543/how-do-i-receive-an-entire-string-as-opposed-to-1-character-at-a-time-on-the-ard

int led = 3;
int state = 1;
String rpy;

void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  digitalWrite(led, state);
}

void loop() {
  if(Serial.available() > 0)
  {
    // Read serial character
    char inChar = Serial.read();
    
    // Accumulate the roll, pitch, yaw string one character at a time
    rpy += inChar;
     
    // New line signals end of roll, pitch, yaw string
    if(inChar == '\n')
    {
      //Test BBB comms
      if(rpy == "1,2,3\n")
      {
        state = state ? 0 : 1;
        digitalWrite(led, state);
      }
      
      // Clear roll, pitch, yaw string
      rpy="";
    }
  }
}
