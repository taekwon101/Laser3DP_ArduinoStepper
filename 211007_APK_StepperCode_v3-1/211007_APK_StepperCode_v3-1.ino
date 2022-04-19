//This code is for controlling the stepper motors of the 3DP platform with arduino software and hardware inputs

//Define pin connections
const int dirPinX = 2;
const int stepPinX = 3;
const int dirPinY = 4;
const int stepPinY = 5;
const int buttonPin = 6;
const int relayRest = 7;
const int relayFan = 8;
const int joystickSwitchPin = 9;
const int blueLEDPin = 10;
const int whiteLEDPin = 11;
const int redLEDPin = 12;
const int echoPin = 53;
const int trigPin = 52;
const int joystickXPin = A0;
const int joystickYPin = A1;

//Define stage movement parameters (DIRECT USER CONTROLS)
const int stageSpeed = 100;       //stage speed (meant to simplify stepDelay by scaling it from 0-100; still need to find good range to map over it)
const int beamDiameter = 5;       //length of beam diameter in millimeters (used with sampleLength to calculate number of raster lines) (usually use 1-5)
const int sampleLength = 50;     //length (left-right) in millimeters of sample to raster over (used to calculate stepsPerRevolutionX) (usually use 300 for slide length)
const int sampleHeight = 500;     //height (forward-backward) in millimeters of sample to raster over (used to calculate stepsPerRevolutionY) (usually use 0 for sweep or 100)
const int scanIterations = 100;    //number of loops to scan in 2D stage movement(used in handheld procedure)
const int sweepIterations = 100;    //number of loops to sweep in 1D stage movement (used in wire and 670nm procedure)

//Define stage movement parameters (UNDER THE HOOD MAPPING TO DIRECT STEPPER CONTROLS)
const int stepDelay = 50000 / stageSpeed;                      //time between steps -- higher number = slower stage movement (avoid direct interaction when scaled to 100 with stageSpeed input)
const int stepDelayReturn = 50;                                //time between steps for the stageReturn loop (need to find fastest possible)
const int rasterLines = (sampleLength / beamDiameter / 5) + 1; //number of times to run up/down raster loop (int truncates, adding 1 rounds up, still needs scale)
const int stepsPerRevolutionX = beamDiameter * 50 *0;             //number of times to run step loop -- higher number = larger distance stage travels
const int stepsPerRevolutionY = sampleHeight * 20;             //number of times to run step loop -- higher number = larger distance stage travels
const int buttonPause = 1000;

//Select procedure (Controlled here but preferably with X joystick movement (only while not mode is active))
bool handheldScanMode = true;    //BLUE LED - will add conditionals for routing stepper commands, currently the mode booleans do nothing
bool diodeScanMode = false;      //WHITE LED - will add option to scan mirror back and forth
bool wireScanMode = false;       //RED LED - will add option to spin external motor for wire cure intstead of moving printer arm

//Initialize loop states
const int loopDelay = 100;
int buttonState = LOW;
int buttonPrintDelay;
int joystickSwitchState = LOW;
int fanState = HIGH;
int joystickXValue = 0;
int joystickYValue = 0;
int joystickWaitMillis = 500;
unsigned long joystickLastMillis = 0;
unsigned long joystickCurrentMillis = 0;
int y = 0;
int z = 0;
int xReturn = 0;
int stageDirection = 0;
long duration = 0;
int distance = 0;
unsigned long progressInitialMillis = 0;
unsigned long progressCurrentMillis = 0;

//broad strokes to-do:
//think on where to use millis instead of delays (definitely in progress bar)
//change raster pattern to interlace so adjacent lines add together less
//keep track of all movements in scan loop so returnOrigin can interrupt and also update progress to serial monitor (along with estimated time per scan and total time)
//set up fan holder pointing away from user (but not so low that it cools the surface (unless I wanna?)
//luxmeter - read LDR signal and write values appended to raster position into a CSVs

void setup()
{
  //Declare digital output pins(except the two temp button inputs)
  pinMode(dirPinY, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinX, OUTPUT);
  pinMode(relayRest, OUTPUT);
  pinMode(relayFan, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(joystickSwitchPin, INPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(whiteLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(blueLEDPin, HIGH);

  //Set serial monitor baud rate
  Serial.begin(9600);
}

//Define stage/stepper movement operations for clean code in main loop
void stageMoveAway()
{
  //Set stage direction to away in  Y
  digitalWrite(dirPinY, HIGH);

  //Move stage away in Y
  for (int x = 0; x < stepsPerRevolutionY; x++)
  {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelay);
  }
}

void stageMoveLeft()
{
  // Set stage direction to left in X
  digitalWrite(dirPinX, HIGH);

  // Move stage left in X
  for (int x = 0; x < stepsPerRevolutionX; x++)
  {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelay);
  }
}

void stageMoveCloser()
{
  // Set stage direction towards front in  Y
  digitalWrite(dirPinY, LOW);

  // Move stage closer in Y
  for (int x = 0; x < stepsPerRevolutionY; x++)
  {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelay);
  }
}

void scanReturnOrigin()
{
  // Set stage direction to left in X
  digitalWrite(dirPinX, LOW);

  // Move stage left in X (10x faster) than other movements
  for (int xReturn = 0; xReturn < stepsPerRevolutionX * rasterLines * 2; xReturn++)
  {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayReturn);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayReturn);
  }
}

void returnOriginClose()
{
  // Set stage direction to left in X
  digitalWrite(dirPinY, LOW);

  // Move stage closer in y (10x faster than other movements)
  for (int YReturn = 0; YReturn < stepsPerRevolutionY; YReturn++)
  {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(stepDelayReturn);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(stepDelayReturn);
  }
}

void serialMonitorMessages()
{
  //print raster line and scan loop progress into serial monitor
  Serial.print("loop ");
  Serial.print(z + 1);
  Serial.print(" of ");
  Serial.print(scanIterations);
  Serial.print("; raster line ");
  Serial.print(y + 1);
  Serial.print(" of ");
  Serial.println(rasterLines);
}

void wireSpinner()
{
  // Set wire spinner motor spin direction (arbitrarily LOW) and stage direction right
  digitalWrite(dirPinY, LOW);
  digitalWrite(dirPinX, HIGH);

  // Spin wire while stage moves right
  for (int x = 0; x < stepsPerRevolutionX; x++)
  {
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelay);
  }
}

void supersonic()
{
   //[easter egg]: how far away from the breadboard are you?
   // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  analogRead(joystickYPin);
}

void loop()
{
  //Read hardware input states
  buttonState = digitalRead(buttonPin);
  joystickSwitchState = digitalRead(joystickSwitchPin);
  joystickXValue = analogRead(joystickXPin);
  joystickYValue = analogRead(joystickYPin);

  //Updates current time to compare against delay for joystick X/Y read value operations
  joystickCurrentMillis = millis();
  

  //Read joystickY and close (or open) 8th relay circuit if it is pushed forward and not too recently
  if ((joystickYValue < 200) && ((joystickCurrentMillis - joystickLastMillis) > joystickWaitMillis))
  {
    if (fanState == HIGH)
    {
      fanState = LOW;
      digitalWrite(relayFan, HIGH);
    }
    else
    {
      fanState = HIGH;
      digitalWrite(relayFan, LOW);
    }
    joystickLastMillis = joystickCurrentMillis;
  }

  //Use joystickXValue to change procedure leftward if it is low, rightward if it is high
  if (((joystickXValue < 200) || (joystickXValue > 700)) && ((joystickCurrentMillis - joystickLastMillis) > joystickWaitMillis))
  {
    if (handheldScanMode == true)
    {
      handheldScanMode = false;
      digitalWrite(blueLEDPin, LOW);
      if (joystickXValue > 700) {
        diodeScanMode = true;
        digitalWrite(whiteLEDPin, HIGH);
      }
      else
      {
        wireScanMode = true;
        digitalWrite(redLEDPin, HIGH);
      }
    }
    else if (diodeScanMode == true)
    {
      diodeScanMode = false;
      digitalWrite(whiteLEDPin, LOW);
      if (joystickXValue > 700) {
        wireScanMode = true;
        digitalWrite(redLEDPin, HIGH);
      }
      else
      {
        handheldScanMode = true;
        digitalWrite(blueLEDPin, HIGH);
      }
    }
    else if (wireScanMode == true)
    {
      wireScanMode = false;
      digitalWrite(redLEDPin, LOW);
      if (joystickXValue > 700) {
        handheldScanMode = true;
        digitalWrite(blueLEDPin, HIGH);
      }
      else
      {
        diodeScanMode = true;
        digitalWrite(whiteLEDPin, HIGH);
      }
    }
    joystickLastMillis = joystickCurrentMillis;
  }
/*
  if ((joystickYValue > 900) && ((joystickCurrentMillis - joystickLastMillis) > joystickWaitMillis))
  {
    while (joystickYValue > 900) {supersonic();}
    joystickLastMillis = joystickCurrentMillis;
  }
*/

  //TROUBLESHOOTING: Regularly write the state of the push button value until pushed
  //Serial.println(buttonState);

  //Use button state AND the procedure booleans to define when and which procedure to use (selected procedure indicated by LEDs as well)
  if (buttonState == HIGH)
{
  progressInitialMillis = millis();
  if (handheldScanMode == true)
    {
      for (int z = 0; z < scanIterations; z++)
      {
        //enable driver by closing the relayRest-disabling circuit until loopCounter loop is over
        digitalWrite(relayRest, HIGH);
        {
          for (int y = 0; y < rasterLines; y++)
          {
            //print raster line and scan loop progress into serial monitor
            progressCurrentMillis = millis();
            Serial.print("loop ");
            Serial.print(z + 1);
            Serial.print(" of ");
            Serial.print(scanIterations);
            Serial.print("; raster line ");
            Serial.print(y + 1);
            Serial.print(" of ");
            Serial.println(rasterLines);
            Serial.print("seconds since loop 1 start: ");
            Serial.println((progressCurrentMillis-progressInitialMillis)/1000);

            //Perform 2D stage movements
            stageMoveAway();
            stageMoveLeft();
            stageMoveCloser();
            stageMoveLeft();
          }
          //Return to origin
          scanReturnOrigin();
        }
      }
    }
    else if (diodeScanMode == true)
  {
    for (int z = 0; z < scanIterations; z++)
      {
        //enable driver by closing the relayRest-disabling circuit until loopCounter loop is over
        digitalWrite(relayRest, HIGH);
        {
          for (int y = 0; y < stepsPerRevolutionY; y++)
          {
            //print progress into serial monitor (return to)

            //Perform 1D stage movements
            stageMoveAway();
          }
          //Return to origin
          returnOriginClose();
        }
      }
    }
    else
    {
    for (int z = 0; z < scanIterations; z++)
      {
        //enable driver by closing the relayRest-disabling circuit until loopCounter loop is over
        digitalWrite(relayRest, HIGH);
        {
          for (int y = 0; y < rasterLines; y++)
          {
           wireSpinner();
           wireSpinner();
           }
          //Return to origin
          scanReturnOrigin();
        }
      }
    }
    //disable driver by opening the relayRest-disabling circuit until button is pressed again
    digitalWrite(relayRest, LOW);
  }
}
