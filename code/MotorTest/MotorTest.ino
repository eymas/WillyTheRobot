#include <Ethernet.h>

#include <SpacebrewYun.h>

/*  This is the main file in the arduino Mega controlling the motorcontroller.
 *  
 *  Tasks:
 *  -Turn and drive speed inputs are read out from ros.
 *  -Speed is read out from both wheel encoders.
 *  -An speed error value is calculated
 *  -The error is converted into an control value (by means of PID) for the motorcontroller
 *  -The control value is send to the motorcontroller.
 *  -Repeat
 *  
 *  Revision 3
 *  6-3-2019
 */

#include "Encoder.h"
#include "ControlLoop.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;
std_msgs::Bool emergency;

//Data used for controlling the motorcontroller.
int analogValue;
float volatile irun, iturn;
unsigned long time_stamp;
unsigned char data[6];

/*  Turn and drive variables are read out from ros and stored in irun and idrive.
 */
void messageCb(const geometry_msgs::Twist& twistMsg)
{
  irun = twistMsg.linear.x;
  iturn = twistMsg.angular.z;
  time_stamp = millis();    // store time at which message was received so old instructions are not repeated.
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
ros::Publisher pub("emergency", &emergency);

//Encoders for both wheels with according Pin numbers attached.
Encoder LeftWheel(20, 21, false);
Encoder RightWheel(2, 3, true);

const int LeftPulseInputA = 20;
const int LeftPulseInputB = 21;
const int RightPulseInputA = 2;
const int RightPulseInputB = 3;

//Controlloop with 2 PID. One for turning and one for driving.
//Values represent: turn Kp, turn Ki, turn Kd, drive Kp, drive Ki, drive Kd, Diff (Diff is used to change the reference speed for turning if needed)
ControlLoop Willy(80, 10, 0, 150, 15, 0);


//used to lower the frequency of the PID loop.
int PIDTrig = 0;                

//Used to store values to be send to the motorcontroller
int PIDTurn = 0;
int PIDDrive = 0;

//4 functions triggered by interrupts will in turn trigger a function inside the encoder library.
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

/* Main setup
 */
void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  
  //Serial communication to motorcontroller
  Serial1.begin(19200, SERIAL_8E1);

  //Serial communication to external laptop for testing and tuning purposes
  Serial2.begin(9600);
  
  //Set up interrupts for encoders
  pinMode(LeftPulseInputA, INPUT);
  pinMode(LeftPulseInputB, INPUT);
  pinMode(RightPulseInputA, INPUT);
  pinMode(RightPulseInputB, INPUT);

  attachInterrupt(digitalPinToInterrupt(LeftPulseInputA), LeftPulsAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftPulseInputB), LeftPulsBChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightPulseInputA), RightPulsAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightPulseInputB), RightPulsBChange, CHANGE);
}

/*  Send values to motorcontroller of the mobility scooter by serial communication.
 *  The motorcontroller will automatically turn off the brakes and the motors will provide torque
 */
void SendToMotor(int Setdrive, int Setturn)
{
  int drive = Setdrive;                                                       //Set value from ros to motorcontroller
  int turn = Setturn;                                                       //Set value from ros to motorcontroller

  //limit values to max 100 and min -100.
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

  //Create data array
  data[0] = 0x6A;                                            //-Datagram always start with 0x6A
  data[1] = drive;                                           //-Drive +-100
  data[2] = turn;                                            //-Turn +-100
  data[3] = 0;                                               //-Driv mode
  data[4] = 0x0c;                                            //-Drive mode: 0x0c=fastest
  data[5] = 0xff - (data[0] + data[1] + data[2] + data[3] + data[4]); //-Checksum

  // Writing the data to the motorcontroller
  for (unsigned char i = 0; i < 6; i++)
  {
    Serial1.write(data[i]);                                 
  }
}

void writeString(String stringData) { // Used to serially push out a String with Serial2.write()

  for (int i = 0; i < stringData.length(); i++)
  {
    Serial2.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }

}// end writeString

/*  Activate PID
 *  Read out sensors
 *  Calculate turn and drive
 */
void ActivatePID()
{
    Willy.SetInputRef(iturn, irun, LeftWheel.GetSpeed(), RightWheel.GetSpeed());

    PIDTurn = (int)Willy.Turn_Output;
    PIDDrive = (int)Willy.Drive_Output;

    PIDTrig = 0;
}

void SendSerial()
{
    String TI = String(iturn,5);   //Turn input
    String DI = String(irun,5);    //Drive input

    String TS = String(Willy.Turn_Speed,5);  //Turn speed
    String DS = String(Willy.Drive_Speed,5);  //Drive speed

    String TO = String(Willy.Turn_Output,5);  //Turn output
    String DO = String(Willy.Drive_Output,5);  //Drive output
    
    Serial2.println(TI + ";" + DI + ";" + TS + ";" + DS + ";" + TO + ";" + DO);
}

/* Main loop 
 * Run at 50Hz.
 */
void loop() {
  
  if((millis() - time_stamp) > 750) {
    irun = 0;
    iturn = 0;
    SendToMotor(0, 0);
  }
  

  PIDTrig++;

    //Run only at 10Hz.
    if (PIDTrig > 5)
    {
     ActivatePID();
     SendSerial();
    }

    SendToMotor(PIDDrive, PIDTurn);

    pub.publish(&emergency);
  
    nh.spinOnce();
    //Serial.flush();
    delay(20);
  
}
