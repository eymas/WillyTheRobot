/* This class is intended to decode a quadruature encoder in order to get a speed value in rad/s
 * Revision 1
 * 1-3-2019
 */

class Encoder {
  private:

    const float Pi = 3.14159265359;               //Pi
    const float pulsesPerRotation = 1024;          //amount of pulses per rotation

    bool Inverse = false;

    bool CCW = false;
    bool SpeedRead = false;
    float counts = 0;

    long int pulseTrainStartTime = 0;    //Time stamp of start point from current pulse train
    long int LastpulseTrainPeriod = 0;   //Period since pulse train has started
    long int LastpulseTime = 0;          //Time stamp of last measered pulse

    int pulseInputA;
    int pulseInputB;

    float SpeedCalculation()
    {
      float Speed = 0;
      Speed = LastpulseTrainPeriod;                      //period in microseconds
      Speed = Speed / 1000000;                          //period in seconds
      Speed = 1 / Speed;                                //frequenty in hz
      Speed = Speed / (pulsesPerRotation / counts);     //Rotations per second
      Speed = Speed * 2 * Pi;                         //Radians per second
      return Speed;
    }

    void pulseChange() {

      if (SpeedRead)                        //Speed has been read so the pulse train can be reset
      {
        pulseTrainStartTime = LastpulseTime;  //Save the last pulse time as it being the start of this new pulse train
        counts = 0;                         //Reset the counts to zero
        SpeedRead = false;                  //Reset the SpeedRead flag
      }

      counts++;                                             //update the amount of pulses
      LastpulseTrainPeriod = micros() - pulseTrainStartTime;  //calculate the pulse train period = current time stamp - time stamp of the start of the pulse train
      LastpulseTime = micros();                              //update the last pulse time in case it is needed when this is the last pulse of the train
    }

  public:
    //Constructor
    Encoder::Encoder(int A, int B, bool Invert)
    {
      Inverse = Invert;
      
      pulseInputA = A;
      pulseInputB = B;

      pinMode(pulseInputA, INPUT);
      pinMode(pulseInputB, INPUT);
    }

    float GetSpeed()
    {
      float Speed = 0;

      if (SpeedRead)                        //There is no new pulse received, thus estimate the speed.
      {
        Speed = 0;
      }
      else                                  //There is at least one new pulse received, thus the speed can be calculated.
      {
        Speed = SpeedCalculation();
        SpeedRead = true;                   //update this variable when the speed has been read so the pulse train can be resetted.
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
      pulseChange();

      if (digitalRead(pulseInputA) + digitalRead(pulseInputB) == 1)
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
      pulseChange();

      if (digitalRead(pulseInputA) + digitalRead(pulseInputB) == 1)
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
        int A = analogRead(pulseInputA);

      
        bool A = digitalRead(pulseInputA); //1
        bool B = digitalRead(pulseInputB);
      
        digitalWrite(pulseInputA, !A); //a is 0 als kapot
        digitalWrite(pulseInputB, !B);

        //G 1     1
        //F 1     0
        if (A != digitalRead(pulseInputA));
        {
          return true;
        }
        if (B != digitalRead(pulseInputB));
        {
          return true;
        }
      
        return false;
    }
    */
};
