
// define pins
const int triggerLeft = 5;
const int echoLeft = 6;

const int triggerRight = 7;
const int echoRight = 8;

const int triggerCenter = 9;
const int echoCenter = 10;

//Back Right
const int triggerBackRight = 11;
const int echoBackRight = 12;

//Back Left
const int triggerBackLeft = 3;
const int echoBackLeft = 4;

// defines variables
long duration;
int distance;


void setup()
{
  pinMode(triggerLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(triggerRight, OUTPUT);
  pinMode(echoRight, INPUT);

  pinMode(triggerCenter, OUTPUT);
  pinMode(echoCenter, INPUT);

  pinMode(triggerBackRight, OUTPUT);
  pinMode(echoBackRight, INPUT);

  pinMode(triggerBackLeft, OUTPUT);
  pinMode(echoBackLeft, INPUT);

  Serial.begin(9600);
}

void loop()
{
  int distances[3];
  distances[0] = GetDistance(triggerLeft, echoLeft);
  distances[1] = GetDistance(triggerCenter, echoCenter);
  distances[2] = GetDistance(triggerRight, echoRight);
  

  Serial.print(GetMinNumber(distances));
  Serial.print('|');
  Serial.print(GetDistance(triggerBackRight,echoBackRight));
  Serial.print('|');
  Serial.print(GetDistance(triggerBackLeft,echoBackLeft));
  Serial.print("\r\n");

//  delay(500);
}



int GetDistance(int triggerPin, int echoPin)
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

  delay(50);

  return distance;
}

int GetMinNumber(int distances[])
{
  int result = distances[0];
  for (int i = 0; i < 3; i++)
  {
    if (distances[i] < result)
    {
      result = distances[i];
    }
  }
  return result;
}

