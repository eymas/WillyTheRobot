#include "Encoder.h"
#include "ControlLoop.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
ros::NodeHandle nh;
std_msgs::Bool emergency;

int analogValue;
float volatile irun, iturn;
unsigned char data[6];

void messageCb(const geometry_msgs::Twist& twistMsg)
{
  irun = twistMsg.linear.x * 6;
  iturn = twistMsg.angular.z * 6;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
ros::Publisher pub("emergency", &emergency);

Encoder LeftWheel(20, 21, false);
Encoder RightWheel(2, 3, true);

ControlLoop Willy(20, 5, 0, 20, 5, 0, 1);

const int LeftPulseInputA = 20;
const int LeftPulseInputB = 21;
const int RightPulseInputA = 2;
const int RightPulseInputB = 3;

int ViewTrig = 0;
int PIDTrig = 0;

float SetD = 0;

float SpeedLeft = 0;
float SpeedRight = 0;

int Udata[6];

int incomingByte = 0;
bool Received = true;
int UserIn = 0;
int pos = 0;

int PIDTurn = 0;
int PIDDrive = 0;

void LeftPulsAChange()
{
  LeftWheel.AChange();
}

void LeftPulsBChange()
{
  LeftWheel.BChange();
}

void RightPulsAChange()
{
  RightWheel.AChange();
}

void RightPulsBChange()
{
  RightWheel.BChange();
}

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  Serial1.begin(19200, SERIAL_8E1);
  pinMode(LeftPulseInputA, INPUT);
  pinMode(LeftPulseInputB, INPUT);
  pinMode(RightPulseInputA, INPUT);
  pinMode(RightPulseInputB, INPUT);

  attachInterrupt(digitalPinToInterrupt(LeftPulseInputA), LeftPulsAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftPulseInputB), LeftPulsBChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightPulseInputA), RightPulsAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightPulseInputB), RightPulsBChange, CHANGE);
}



void SendToMotor(int Setdrive, int Setturn)
{
  int drive = Setdrive;                                                       //Set value from ros to motorcontroller
  int turn = Setturn;                                                       //Set value from ros to motorcontroller

  if (drive > 100) {
    drive = 100;
  }

  if (drive < -100) {
    drive = -100;
  }

  if (turn > 100) {
    turn = 100;
  }
  if (turn < -100) {
    turn = -100;
  }

  data[0] = 0x6A;                                            //-Datagram always start with 0x6A
  data[1] = drive;                                           //-Drive +-100
  data[2] = turn;                                            //-Turn +-100
  data[3] = 0;                                               //-Driv mode
  data[4] = 0x0c;                                            //-Drive mode: 0x0c=fastest
  data[5] = 0xff - (data[0] + data[1] + data[2] + data[3] + data[4]); //-Checksum

  for (unsigned char i = 0; i < 6; i++)
  {
    Serial1.write(data[i]);                                  // Writing the data to the motorcontroller
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  //GetUserInput(); --> getROSinput();
  SendToMotor(PIDDrive, PIDTurn);
  //SendToMotor(UserIn,0);

  PIDTrig++;

  if (PIDTrig > 5)
  {
    SpeedLeft = LeftWheel.GetSpeed();
    SpeedRight = RightWheel.GetSpeed();
    SetD = UserIn;
    SetD = SetD / 100;
    Willy.SetInputRef(irun, iturn, SpeedLeft, SpeedRight);

    PIDTurn = (int)Willy.Turn_Output;
    PIDDrive = (int)Willy.Drive_Output;

    PIDTrig = 0;
  }

  pub.publish(&emergency);

  nh.spinOnce();

  delay(20);
}
