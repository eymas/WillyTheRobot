/* This class converts an error into an control signal using PID variables.
 * Revision 3
 * 1-3-2019
 */

class PIDCalculation {

  private:
    float K_P;
    float K_I;
    float K_D;
    float Error_1 = 0;
    float Error_2 = 0;
    float Output_1 = 0;

  public:

    void Initialize(float P, float I, float D)
    {
      K_P = P;
      K_I = I;
      K_D = D;
    }

    float Get(float Error)
    {
      //Calculate Output with PID in velocity form.
      float Output = 0;
      Output = Output_1 + K_P * (Error - Error_1);
      Output += K_I * Error;
      Output += K_D * (Error - 2 * Error_1 + Error_2);

      //save parameters
      Output_1 = Output;
      Error_2 = Error_1;
      Error_1 = Error;

      //limit output to max 100 and min -100
      if (Output > 100)
      {
        Output = 100;
      }
      if (Output < -100)
      {
        Output = -100;
      }

      return Output;
    }

    void Reset()
    {
      Error_1 = 0;
      Error_2 = 0;
      Output_1 = 0;
    }
};
