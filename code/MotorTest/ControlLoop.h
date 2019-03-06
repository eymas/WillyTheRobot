/*  This class calculates the error on both wheels and calls in the PIDCalculation to get a control value back.
 *  Revision 2
 *  5-3-2019
 */

#include "PIDCalculation.h"

class ControlLoop {

  private:
  
    float Diff_Factor = 0;
    float Error_R = 0;
    float Error_L = 0;

    //include PID's for both inputs/outputs.
    PIDCalculation DriveController;
    PIDCalculation TurnController;

    //Calculate the drive error.
    Drive(float Error_Right, float Error_Left)
    {
      //Calculate error
      float Drive_Error;
      Drive_Error = 0.5 * (Error_Left + Error_Right);
      Drive_Output = DriveController.Get(Drive_Error);
    }

    //Calculate the turn error.
    Turn(float Error_Right, float Error_Left)
    {
      //Calculate error
      float Turn_Error;
      Turn_Error = 0.5 * (Error_Left - Error_Right);
      Turn_Output = TurnController.Get(Turn_Error);
    }
    
  public: 

    //OUTPUTS
    float Turn_Output = 0;  
    float Drive_Output = 0;  

    //CONSTRUCTOR
    ControlLoop::ControlLoop(float Kpt, float Kit, float Kdt, float Kpd, float Kid, float Kdd, float Diff)
    {
      //initialize both controllers
      DriveController.Initialize(Kpd,Kid,Kdd);
      TurnController.Initialize(Kpt,Kit,Kdt);
      
      Diff_Factor = Diff;
    }

    //Function sets reference, calculates error, then activate the calculation of outputs by the turn and drive functions.
    SetInputRef(float Turn_Input, float Drive_Input, float Speed_Sensor_L, float Speed_Sensor_R)
    {
      if(Turn_Input == 0 && Drive_Input == 0)
      {
        DriveController.Reset();
        TurnController.Reset();
      }
      
      float Error_Left = Turn_Input * Diff_Factor + Drive_Input - Speed_Sensor_L;
      float Error_Right = -Turn_Input * Diff_Factor + Drive_Input - Speed_Sensor_R;

      Drive(Error_Left,Error_Right);
      Turn(Error_Left,Error_Right);
    }
};
