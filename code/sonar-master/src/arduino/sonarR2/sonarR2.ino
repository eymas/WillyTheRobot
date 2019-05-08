/*  SonarR2
 *  This code reads out 8 sonars sensors, and sends the distance [cm] over serial to the Raspberry Pi
 *  Electrically it triggers al the sensors with one pin, but it reads them out individually
 * Last updated: 8-5-2019
 */

int trigPin = 13;    // Trigger

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);

  for(int i = 4; i < 12; i++)
  {
    pinMode(i, INPUT_PULLUP);
  }
}

int GetDistance(int pin)
{
  //delays on trigger pin as required for the HC-SR-04
  delay(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //pinMode(echoPin, INPUT);
  long Duration = pulseIn(pin, HIGH, 12000);

  int Distance = 0;
  
  if (Duration > 0)
  {
    //Calculation of distance in cm, using speed of sound (0.0343) and 2 for the fact that the sound travels double the distance.
    Distance = Duration * 0.0343 / 2;
  }
  else
  {
    //limit the distance
    Distance = 200;
  }

  if (Distance > 200) 
  {
    //limit the distance
    Distance = 200;
  }
  
  return Distance;
    
}

void loop() {

  //send data over serial
  for(int i = 4; i < 12; i++)
  {
    Serial.print(GetDistance(i));
    if (i < 11)
    {
      Serial.print("|");
    }
  }
  
  Serial.print("\r\n");
}
