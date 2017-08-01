//DOME CALIBRATION- State Machine 

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
  float relativePosition;   // how many degrees to move from current position
  float absolutePosition;    //between 0-359 degrees
  float gearCountPerDomeDegree;
}DomeInfo;

SensorInfo sensorGear;
SensorInfo sensorDome;
CommandInfo commandInfo;
DomeInfo domeInfo;
MotorMovementInfo motorMovementInfo;

bool calculateCurrentState(int ldrStatus);
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

const int SENSITIVITY_THRESHOLD_MIN = 550; // The value that we'll consider to be high or low for the photoresistor
const int SENSITIVITY_THRESHOLD_MAX = 800;  //Dead zone is between threshold MIN and MAX

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

void updateCounter(SensorInfo* counter)
{
   int ldrStatus = analogRead(counter->sensorPin);   //reads status of LDR value
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
    Serial.println("I made it here 2!");
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
  domeInfo.relativePosition = 0.0f;
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
  Serial.print("Degrees needed to move: ");
  Serial.println(moveDegrees);

  if(isRelativeMovement == false)
  {
    //need to do math to figure out how to convert absolute movement to relative
  }
 
 int motorMovementInfo.gearCountsToTarget = abs(moveDegrees)*domeInfo.gearCountPerDomeDegree; //TODO: round function
 motorMovementInfo.turningDirection = moveDegrees > 0 ? MOTION_CW : MOTION_CCW;

 currentState = MOTOR_TURNING_STATE;
 sensorGear.count = 0;
}

void handleGetPositionCommand() //Retrieves Absolute position 
{
  
}
void handleParkCommand()
{
  
}

