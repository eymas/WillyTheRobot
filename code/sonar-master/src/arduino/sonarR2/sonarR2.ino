int trigPin = 12;    // Trigger

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

float GetDistance(int pin)
{
  delay(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //pinMode(echoPin, INPUT);
  long Duration = pulseIn(pin, HIGH, 12000);
  int Distance = Duration * 0.0343 / 2;
  return Distance;
}

void loop() {

  
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
