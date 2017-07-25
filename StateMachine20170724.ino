//DOME CALIBRATION- State Machine 

enum States
{
  IDLE_STATE,
  CALIBRATION_STATE,
  END_CALIBRATION_STATE,
  MOTOR_TURNING_STATE,
  END_MOTOR_TURNING_STATE 
};

typedef struct
{
  int ldrPin;
  int count; 
  bool lastState;
  char counterName;
}CounterInfo;

typedef struct
{
  float startingRelativePosition;
  float targetRelativePosition;
  int motorTurningDirection;
  int gearCountsToTarget;
}MotorMovementInfo;

typedef struct
{
  char bufferData[8];
  int index;
  bool hasData;
}CommandInfo;

typedef struct
{
  float realativePosition;
  float absolutePosition;
  float gearCountPerDomeDegree;
}DomeInfo;

CounterInfo counterGear;
CounterInfo counterDome;

bool calculateCurrentState(int ldrStatus);
void updateCounter(CounterInfo* counter);
States currentState;
int readCommand();

const int SENSITIVITY_THRESHOLD_MIN = 550; // The value that we'll consider to be high or low for the photoresistor
const int SENSITIVITY_THRESHOLD_MAX = 900;  //Dead zone is between threshold MIN and MAX
int gearPerDomeDegree;


float countsToTurn;
bool motorDirection;

void setup() 
{
  Serial.begin(9600);   //speed in bits per second, talking to USB cable
  counterGear.ldrPin = A0;
  counterGear.count = 0;
  counterGear.counterName = 'G'; //G for Gear
  counterDome.ldrPin = A1;
  counterDome.count = 0;
  counterDome.counterName = 'D';  //D for Dome
  
  pinMode(counterGear.ldrPin, INPUT); //initialize photoresistor on gear
  pinMode(counterDome.ldrPin, INPUT);  //initialize photoresistor on dome

  counterGear.lastState = calculateCurrentState(analogRead(counterGear.ldrPin));
  counterDome.lastState = calculateCurrentState(analogRead(counterDome.ldrPin));  

  currentState = IDLE_STATE;
  
}

void loop() 
{
 switch(currentState)
 {
  case IDLE_STATE:
  {
    
    break;
  }
  case CALIBRATION_STATE:
    {
       updateCounter(&counterDome);
       if(counterDome.count > 1)
       {
        currentState = END_CALIBRATION_STATE;
       }
       
       if(counterDome.count == 1)
       {
        updateCounter(&counterGear);
       }
      break;
    }
  case END_CALIBRATION_STATE:
  {
    gearPerDomeDegree = float(counterGear.count)/360.0f;  //number of 1/3 turns per one degree of dome turning 
    Serial.print("Gear 1/3 Rotations per One Degree of Dome Rotation: ");
    Serial.println(gearPerDomeDegree);
    currentState = IDLE_STATE;
    break; 
  }
  case MOTOR_TURNING_STATE:
  {
    break;
  }
 }
}

bool calculateCurrentState(int ldrStatus)
{
  return (ldrStatus < SENSITIVITY_THRESHOLD_MIN);
}

void updateCounter(CounterInfo* counter)
{
   int ldrStatus = analogRead(counter->ldrPin);   //reads status of LDR value
   Serial.print(counter->counterName);
   Serial.print(" LDR Status: ");
   Serial.println(ldrStatus);

    if(ldrStatus < SENSITIVITY_THRESHOLD_MIN || ldrStatus > SENSITIVITY_THRESHOLD_MAX)
    {

        // True if photoresister has been covered (according to the average reading during our sample size)
        bool currentState = calculateCurrentState(ldrStatus);
        Serial.print(counter->counterName);
        Serial.print(" CurrentState: ");
        Serial.println(currentState);
    
        if(currentState == true && counter->lastState == false)
        {
            Serial.print(counter->counterName);
            Serial.println(" currentState == true && lastState == false. Incrementing Counter");
            counter->lastState = true;
            counter->count++;
            Serial.print(counter->counterName);
            Serial.print(" Number of turns: ");
            Serial.println(counter->count); 
        }
        else if(currentState == false && counter->lastState == true)
        {
            Serial.print(counter->counterName);
            Serial.println(" currentState == false && lastState == true");      
            counter->lastState = false;  
        }
    }


}

int readCommand()
{
  if(Serial.available() >0)
  {
    if(Serial.peek() == 'c')
    {
      Serial.read();                  
    }
    while(Serial.available()>0)
    {
      Serial.read();
    }
  }
}

