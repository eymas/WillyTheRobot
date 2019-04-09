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
    float Turn_Error = 0;
    float Drive_Error = 0;
    
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

      Turn_Speed = (Speed_Sensor_R - Speed_Sensor_L)/0.6;
      Drive_Speed = (Speed_Sensor_R + Speed_Sensor_L)*0.5;

      Turn_Error = Turn_Input - Turn_Speed;
      Drive_Error = Drive_Input - Drive_Speed;

      Turn_Output = TurnController.Get(Turn_Error);
      Drive_Output = DriveController.Get(Drive_Error);
    }
};
