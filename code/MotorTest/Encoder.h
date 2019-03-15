/* This class is intended to decode a quadruature encoder in order to get a speed value in rad/s
 * Revision 1
 * 1-3-2019
 */

class Encoder {
  private:

    const float Pi = 3.14159265359;               //Pi
    const float PulsesPerRotation = 1024;          //amount of pulses per rotation

    bool Inverse = false;

    bool CCW = false;
    bool SpeedRead = false;
    float counts = 0;

    long int PulsTrainStartTime = 0;    //Time stamp of start point from current puls train
    long int LastPulsTrainPeriod = 0;   //Period since Puls train has started
    long int LastPulsTime = 0;          //Time stamp of last measered puls

    int PulseInputA;
    int PulseInputB;

    float SpeedCalculation()
    {
      float Speed = 0;
      Speed = LastPulsTrainPeriod;                      //period in microseconds
      Speed = Speed / 1000000;                          //period in seconds
      Speed = 1 / Speed;                                //frequenty in hz
      Speed = Speed / (PulsesPerRotation / counts);     //Rotations per second
      Speed = Speed * 2 * Pi;                         //Radians per second
      return Speed;
    }

    void PulsChange() {

      if (SpeedRead)                        //Speed has been read so the puls train can be reset
      {
        PulsTrainStartTime = LastPulsTime;  //Save the last puls time as it being the start of this new puls train
        counts = 0;                         //Reset the counts to zero
        SpeedRead = false;                  //Reset the SpeedRead flag
      }

      counts++;                                             //update the amount of pulses
      LastPulsTrainPeriod = micros() - PulsTrainStartTime;  //calculate the puls train period = current time stamp - time stamp of the start of the puls train
      LastPulsTime = micros();                              //update the last puls time in case it is needed when this is the last puls of the train
    }

  public:
    //Constructor
    Encoder::Encoder(int A, int B, bool Invert)
    {
      Inverse = Invert;
      
      PulseInputA = A;
      PulseInputB = B;

      pinMode(PulseInputA, INPUT);
      pinMode(PulseInputB, INPUT);
    }

    float GetSpeed()
    {
      float Speed = 0;

      if (SpeedRead)                        //There is no new puls received, thus estimate the speed.
      {
        Speed = 0;
      }
      else                                  //There is at least one new puls received, thus the speed can be calculated.
      {
        Speed = SpeedCalculation();
        SpeedRead = true;                   //update this variable when the speed has been read so the puls train can be resetted.
      }

      if (CCW)
      {
        Speed = -Speed;                     //invert the speed result when rotating counter clock wise.
      }
      if (Inverse)                          //invert when set
      {
        Speed = -Speed;
      }

      if(isnan(Speed))
      {
        Speed = 0;
      }

      //if(CheckBadConnection())
      //{
      //  Speed = 5;
      //}

      return Speed;
    }

    //Determine the direction
    void AChange()
    {
      PulsChange();

      if (digitalRead(PulseInputA) + digitalRead(PulseInputB) == 1)
      {
        CCW = false;
      }
      else
      {
        CCW = true;
      }
    }

    //Determine the direction
    void BChange()
    {
      PulsChange();

      if (digitalRead(PulseInputA) + digitalRead(PulseInputB) == 1)
      {
        CCW = true;
      }
      else
      {
        CCW = false;
      }
    }

/*
    bool CheckBadConnection()
    {
        int A = analogRead(PulseInputA);

      
        bool A = digitalRead(PulseInputA); //1
        bool B = digitalRead(PulseInputB);
      
        digitalWrite(PulseInputA, !A); //a is 0 als kapot
        digitalWrite(PulseInputB, !B);

        //G 1     1
        //F 1     0
        if (A != digitalRead(PulseInputA));
        {
          return true;
        }
        if (B != digitalRead(PulseInputB));
        {
          return true;
        }
      
        return false;
    }
    */
};
