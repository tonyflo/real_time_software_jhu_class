#include <OneWire.h>
#include <DallasTemperature.h>

//JHU RTSW HW 1 - Calibrated Temperature
//Tony Florida
//2014-09-17
//References: http://www.hobbytronics.co.uk/ds18b20-arduino

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
 
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("JHU RTSW HW1");
  Serial.println("Tony Florida");
  Serial.println("2014-09-17");

  // Start up the library
  sensors.begin();
}

//remember the most recent temperatures
int ARRAY_LEN = 5;
int recent_temperatures[] = {0.0, 0.0, 0.0, 0.0, 0.0};
int index = 0; //index into temperature array
boolean temp_stabilized = false; //remember once the temp has stablilize

//function to check if the temperature has stabilized
boolean stabilized()
{
  //get the first 
  double temp = recent_temperatures[0]; 
  for(int i = 1; i < ARRAY_LEN; i++)
  {
    if(recent_temperatures[i] != temp)
    {
      return false; 
    }
  }
  temp_stabilized = true;
  return true;
}

//convert celsius to fahrenheit
double convert2fahrenheit(double celsius)
{
  return (celsius * 1.8) + 32; 
}

//main round robin loop
void loop(void)
{
  // call sensors.requestTemperatures() to issue a global temperature
  sensors.requestTemperatures();

  double temp = sensors.getTempCByIndex(0);
  
  //wait until the temperature stabilizes
  if(!temp_stabilized)
  {
    //keep index between 0 and 5
    index++;
    if(index > ARRAY_LEN)
    {
      index = 0; 
    }
    
    //save the temperature into the array
    recent_temperatures[index] = temp;
    
    //check stabilization
    stabilized();

    Serial.println("Waiting for temperature to stabilize...");  
  }
  else
  {
    Serial.print("Temperature is: ");
    // 0 refers to the first IC on the wire
    Serial.println(convert2fahrenheit(temp));
    
    delay(10000); //delay 10 seconds
  }
}
