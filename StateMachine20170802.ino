//DOME CALIBRATION- State Machine 
# include <math.h>
# define BUFFER_SIZE 8
# define MOTION_CW 0
# define MOTION_CCW 1

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
  int sensorPin;
  int count; 
  bool lastState;
  char counterName;
}SensorInfo;

typedef struct
{
  int turningDirection;
  int gearCountsToTarget;
}MotorMovementInfo;

typedef struct
{
  char bufferData[BUFFER_SIZE];
  int index;
  bool hasData;
}CommandInfo;

typedef struct
{
  float absolutePosition;    //between 0-359 degrees
  float gearCountPerDomeDegree;
}DomeInfo;

SensorInfo sensorGear;
SensorInfo sensorDome;
CommandInfo commandInfo;
DomeInfo domeInfo;
MotorMovementInfo motorMovementInfo;

bool calculateCurrentState(int sensorStatus);
void updateCounter(SensorInfo* counter);
States currentState;
void readCommands();
bool updateMovement();
bool updateDeceleration();
void clearCommands();
void handleCalibrateCommand();
void handleMovementCommand();
void handleGetPositionCommand();
void handleParkCommand();

const int SENSITIVITY_THRESHOLD_MIN = 100; // The value that we'll consider to be high or low for the photoresistor
const int SENSITIVITY_THRESHOLD_MAX = 850;  //Dead zone is between threshold MIN and MAX

float countsToTurn;
bool motorDirection;

void setup() 
{
  Serial.begin(9600);   //speed in bits per second, talking to USB cable
  sensorGear.sensorPin = A0;
  sensorGear.count = 0;
  sensorGear.counterName = 'G'; //G for Gear
  sensorDome.sensorPin = A1;
  sensorDome.count = 0;
  sensorDome.counterName = 'D';  //D for Dome
  
  pinMode(sensorGear.sensorPin, INPUT); //initialize sensor on gear
  pinMode(sensorDome.sensorPin, INPUT);  //initialize sensor on dome

  domeInfo.absolutePosition = 0.0f; 

  currentState = IDLE_STATE;
  
}

void loop() 
{
 switch(currentState)
 {
  case IDLE_STATE:
  {
    //Serial.println("I'm in Idle");
    //commands from serial read from Serial Event function immediately when sent
    break;
  }
  case CALIBRATION_STATE:
    {
       updateCounter(&sensorDome);
       if(sensorDome.count > 1)
       {
        currentState = END_CALIBRATION_STATE;
       }
       
       if(sensorDome.count == 1)
       {
        updateCounter(&sensorGear);
       }
      break;
    }
  case END_CALIBRATION_STATE:
  {
    domeInfo.gearCountPerDomeDegree = float(sensorGear.count)/3.0f;  //number of 1/3 turns per one degree of dome turning 
    Serial.print("Gear 1/3 Rotations per One Degree of Dome Rotation: ");
    Serial.println(domeInfo.gearCountPerDomeDegree);
    currentState = IDLE_STATE;
    break; 
  }
  case MOTOR_TURNING_STATE:
  {
    break;
  }
 }

}

bool calculateCurrentState(int sensorStatus)
{
  return (sensorStatus < SENSITIVITY_THRESHOLD_MIN);
}

void updateCounter(SensorInfo* counter)
{
   int sensorStatus = analogRead(counter->sensorPin);   //reads status of sensor value
   Serial.print(counter->counterName);
   Serial.print(" Sensor Status: ");
   Serial.println(sensorStatus);

    if(sensorStatus < SENSITIVITY_THRESHOLD_MIN || sensorStatus > SENSITIVITY_THRESHOLD_MAX)
    {

        // True if photoresister has been covered (according to the average reading during our sample size)
        bool currentState = calculateCurrentState(sensorStatus);
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

void serialEvent()  //reads data from the serial connection when data sent
{    
  while(Serial.available())
  { 
   char currentChar = (char)Serial.read();
   commandInfo.bufferData[commandInfo.index] = currentChar;
   if(currentChar == ';')
   {
    commandInfo.hasData = true;
    commandInfo.bufferData[commandInfo.index] = '\0';
   }
   commandInfo.index++;
  }

  if(commandInfo.hasData == true)
  {
    readCommands();
  
    for(int i=0; i<commandInfo.index ; i++)
    {
      Serial.print(commandInfo.bufferData[i]);
    }

    clearCommands();
    }
}

void readCommands()
{
  Serial.print("commandInfo.index: ");
  Serial.println(commandInfo.index);
  Serial.print("commandInfo.bufferData[0]: ");
  Serial.println(commandInfo.bufferData[0]);
  
  if(commandInfo.index >= 3 && commandInfo.bufferData[0] == '+')
  {
    Serial.print("commandInfo.bufferData[1]: ");    
    Serial.println(commandInfo.bufferData[1]);
    switch(commandInfo.bufferData[1])
    {
      case 'C':
      {
        handleCalibrateCommand();
        break;
      }
      case 'M':
      {
        handleMovementCommand(commandInfo.bufferData); 
        break;
      }
      case 'G':
      {
        handleGetPositionCommand();
        break;
      }
      case 'P':
      {
        handleParkCommand();
        break;
      }
    }
  }
}

void clearCommands()
{
  commandInfo.hasData = false;
  memset(commandInfo.bufferData,0,BUFFER_SIZE); //sets buffer array to zero
  commandInfo.index = 0;
  Serial.println();
}

bool updateMovement()
{
  return false;
}

bool updateDeceleration()
{
  return false;
}

void handleCalibrateCommand()
{
  domeInfo.absolutePosition = 0.0f;
  domeInfo.gearCountPerDomeDegree = 0.0f;
  sensorGear.count = 0;
  sensorDome.count = 0;
  sensorGear.lastState = calculateCurrentState(analogRead(sensorGear.sensorPin));
  sensorDome.lastState = calculateCurrentState(analogRead(sensorDome.sensorPin));
  currentState = CALIBRATION_STATE;
}

void handleMovementCommand(char* commandBuffer)
{
  bool isRelativeMovement = false;
  int moveDegrees; 
    
  switch(commandBuffer[2])
  {
    case 'A':
    {
     isRelativeMovement = false;
     Serial.println("I am in case A ");
     break;
    }
    case 'R':
    {
      isRelativeMovement = true;
      Serial.println("I am in case R");
      break;
    }
  }

  moveDegrees = atoi(&commandInfo.bufferData[3]);
  Serial.print("Degrees entered to move: ");
  Serial.println(moveDegrees);
 Serial.print("Current Absolute Dome Position: ");
  Serial.println(domeInfo.absolutePosition);

  if(isRelativeMovement == false) //converting to relative movement from absolute movement
  {
    int targetPosition = moveDegrees;
    float relativeMovement = fmod(targetPosition - domeInfo.absolutePosition + 180.0f, 360.0f) - 180;
    relativeMovement += (relativeMovement < -180) ? 360 : 0;
    moveDegrees = relativeMovement;
    Serial.print("Absolute Degrees entered to move converted to Relative: ");
    Serial.println(moveDegrees);
   //FOR DEBUG CODE ONLY, REMOVE BEFORE USE!!
    domeInfo.absolutePosition = targetPosition;
  }
 
 motorMovementInfo.gearCountsToTarget = abs(moveDegrees)*domeInfo.gearCountPerDomeDegree; //TODO: round function
 motorMovementInfo.turningDirection = moveDegrees > 0 ? MOTION_CW : MOTION_CCW;

 currentState = IDLE_STATE;
 sensorGear.count = 0;
}

void handleGetPositionCommand() //Retrieves Absolute position 
{
  
}
void handleParkCommand()
{
  
}

