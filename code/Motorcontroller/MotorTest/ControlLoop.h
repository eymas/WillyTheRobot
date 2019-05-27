/*  This class calculates the error on both wheels and calls in the PIDCalculation to get a control value back.
 *  Revision 2
 *  5-3-2019
 */

#include "PIDCalculation.h"

class ControlLoop {

  private:
  
    //include PID's for both inputs/outputs.
    PIDCalculation DriveController;
    PIDCalculation TurnController;

  public: 

    //SPEED
    float Turn_Speed;   //unit:  [rad/s]
    float Drive_Speed;  //unit:  [m/s]
  
    //ERROR
    float Turn_Error = 0;  //unit:  [rad/s]
    float Drive_Error = 0; //unit:  [m/s]
    
    //OUTPUTS
    float Turn_Output = 0; 
    float Drive_Output = 0;  

    //CONSTRUCTOR
    ControlLoop::ControlLoop(float Kpt, float Kit, float Kdt, float Kpd, float Kid, float Kdd)
    {
      //initialize both controllers
      DriveController.Initialize(Kpd,Kid,Kdd);
      TurnController.Initialize(Kpt,Kit,Kdt);
    }

    //Function sets reference, calculates error, then activate the calculation of outputs by the turn and drive functions.
    SetInputRef(float Turn_Input, float Drive_Input, float Speed_Sensor_L, float Speed_Sensor_R)
    {
      if(Turn_Input == 0 && Drive_Input == 0)
      {
        DriveController.Reset();
        TurnController.Reset();
      }

      //calculate speed at which the robot is rotating, unit:  [rad/s]. 0.6 is the distance between the wheels in meters.
      Turn_Speed = (Speed_Sensor_R - Speed_Sensor_L)/0.6; 

      //calculate the speed at which the robot is moving, unit:  [m/s]. 0.5 is used to provide an average between both wheels.
      Drive_Speed = (Speed_Sensor_R + Speed_Sensor_L)*0.5; 

      //calculate errors.
      Turn_Error = Turn_Input - Turn_Speed;
      Drive_Error = Drive_Input - Drive_Speed;

      //errors are send to both controllers to get an output signal.
      Turn_Output = TurnController.Get(Turn_Error);
      Drive_Output = DriveController.Get(Drive_Error);
    }
};
