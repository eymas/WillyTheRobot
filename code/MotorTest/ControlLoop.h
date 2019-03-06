#include "PIDCalculation.h"

class ControlLoop {

  private:
  
    //REST
    float Diff_Factor = 0;
    float Tot_Ref_L = 0;
    float Tot_Ref_R = 0;

    float Error_R = 0;
    float Error_L = 0;

    PIDCalculation DriveController;
    PIDCalculation TurnController;

    Drive(float Error_Right, float Error_Left)
    {
      //Calculate error
      float Drive_Error;
      Drive_Error = 0.5 * (Error_Left + Error_Right);
      Drive_Output = DriveController.Get(Drive_Error);
    }

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
      DriveController.Initialize(Kpd,Kid,Kdd);
      TurnController.Initialize(Kpt,Kit,Kdt);
      
      Diff_Factor = Diff;
    }

    //Activate pids
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
