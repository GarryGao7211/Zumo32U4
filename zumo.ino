/* This example shows how you might use the Zumo 32U4 in a robot
sumo competition.

It uses the line sensors to detect the white border of the sumo
ring so it can avoid driving out of the ring (similar to the
BorderDetect example).  It also uses the Zumo 32U4's proximity
sensors to scan for nearby opponents and drive towards them.

For this code to work, jumpers on the front sensor array
must be installed in order to connect pin 4 to RGT and connect
pin 20 to LFT.

This code was tested on a Zumo 32U4 with 75:1 HP micro metal
gearmotors. */

#include <Wire.h>
#include <Zumo32U4.h>
#include <StopWatch.h>

Zumo32U4LCD lcd;//declare an object for LCD display
Zumo32U4ProximitySensors proxSensors;//declare an object for proximity sensors
Zumo32U4LineSensors lineSensors;//declare an object for line sensors
Zumo32U4Motors motor;//declare the motor 
Zumo32U4ButtonA buttonA;// declare button A
StopWatch scanningWatch; 
StopWatch goingWatch;
StopWatch borderTurningWatch;
StopWatch sensorTurningWatch;
StopWatch goingDetectWatch;
StopWatch scanningDetectWatch;
StopWatch attackDetectWatch;
StopWatch speedWatch;
LSM303 compass;// declare accelerometer for Zumo32U4
Zumo32U4Encoders encoder;//declare encoder 
L3G gyro; //declare gyro for Zumo32U4


const uint16_t forwardSpeedMax = 400;//max speed for driving forward

const uint16_t reverseSpeedMax = -400;//mas speed for driving backward

const uint16_t forwardVeerSpeed = 300;

const uint16_t reverseVeerSpeed = -300;

const uint16_t lineSensorLimit = 1000;

const uint16_t midSensorLimit = 500;

const uint16_t proxSensorLimit = 1;

unsigned int lineSensorValues[3]; // store the line sensor value 

const uint16_t searchingTime = 1500;// in milliseconds max time for searching around 

const uint16_t reverseTime = 200;//backing speed while detecting a border

const uint16_t goingTime = 1000;//max time for going to search if object is not seen during searching time 

const uint16_t turningTime = 500; //time for stopwatch1 when border detected.

uint16_t trash = 0;

bool scanningMode = true; //scanning state

bool initialCase = true;

bool borderMode = false;

bool attackMode = false;

bool searchingMode = true;

bool borderModeinit = false;

bool objectSeen = false;// determine whether an object is seen or not 

bool borderdetect = false; //determine whether border has been detected or not 

bool turnLeftDetect = false;  //the boarder is approching from right, turn left

bool turnRightDetect = false;  //the boarder is approching from left, turn right

bool turnAroundDetect = false;

bool backLeftDetect = false;

bool backRightDetect = false;

bool leftDetect = false;

bool rightDetect = false;

bool collisionDetect = false;//detect collision 


void setup() {
  Serial.begin(9600);
// put your setup code here, to run once:
proxSensors.initThreeSensors();
lineSensors.initThreeSensors();
//compass.init();
//compass.enableDefault();
lcd.setCursor(0,0);
lcd.print("Press A");
buttonA.waitForButton();//wait a button push to start
lcd.clear();
delay(5000);
}

void loop() {
    lineSensors.read(lineSensorValues);
    proxSensors.read();
    uint8_t sum =proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
    uint8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();
    uint8_t leftSensor = proxSensors.countsLeftWithLeftLeds(); //left Detect
    int8_t rightSensor = proxSensors.countsRightWithRightLeds();// right Detect 

    
  if (searchingMode == true && borderMode == false && attackMode == false && collisionDetect ==false)// peform searchingMode if both borderMode  and attackMode are off 
  {
    //initial case
    if(initialCase == true)
    {
      scanningWatch.start();
      scanningMode = true;
      initialCase = false;
      delay(10);
    }
    if(scanningWatch.value() == 0 && goingWatch.value() == 0)
    {      
      goingWatch.start();
      scanningMode = false;
    }
    if (lineSensorValues[0] < lineSensorLimit || lineSensorValues[2] < lineSensorLimit)
    {
      borderMode = true;
      scanningWatch.reset();
      goingWatch.reset();
      //borderModeinit = true;
      speedWatch.reset();
      goingDetectWatch.reset();
     }
     
     if(goingWatch.value()> goingTime )
     { 
        goingDetectWatch.reset();/////////////////////////////////////////
        goingWatch.stop();
        goingWatch.reset();
        scanningMode = true; 
        scanningWatch.start();
        //scanningDetectWatch.start();////////////////////////////////////////
        
      }
      else if(scanningWatch.value() > searchingTime )
        {
        scanningDetectWatch.reset();///////////////////////////////////////
        scanningWatch.stop();
        scanningWatch.reset();
        scanningMode = false;
        goingWatch.start();
        
        
        }

        
     if (scanningMode == true)
     {  
        motor.setSpeeds(reverseVeerSpeed,forwardVeerSpeed);
        //During turing at each case, perform scanning an object continuously     
        if (sum >= 2  && diff <=1 || leftSensor >=1 || rightSensor >= 1 )    
        {  
           searchingMode =false;    
           attackMode = true;  
           scanningWatch.reset(); 
           goingWatch.reset();
           speedWatch.reset();
           goingDetectWatch.reset();
           //goingDetectWatch.reset();
           if (leftSensor >=1)
           {
            leftDetect = true;
            Serial.print("scanning true left");
            Serial.println(leftSensor);
           }
           if (rightSensor >=1)
           {
            rightDetect =true;
            Serial.print("scanning true left");
            Serial.println(leftSensor);
           }
         }
     }           
            
     else if(scanningMode == false) // if no object seen perform going to search 
       {  
          motor.setSpeeds(forwardVeerSpeed,forwardVeerSpeed);//------------------------------------------------------------------------------------------------------------
          goingDetectWatch.start();
          if(goingDetectWatch.value() > 100)
          {
            speedWatch.start();
            if (speedWatch.value() <= 10)
              {
                trash = encoder.getCountsAndResetLeft();
                trash = encoder.getCountsAndResetRight();
              }
           if (speedWatch.value() >= 100 && encoder.getCountsAndResetLeft() < 320 && encoder.getCountsAndResetRight() < 320)
              {
                
                collisionDetect = true;
                attackMode = false;
                searchingMode = false;
//                goingWatch.reset();
                goingDetectWatch.reset();
                speedWatch.reset();
              }
              else if(speedWatch.value() >= 120)
              {
                
                speedWatch.reset();
                goingDetectWatch.reset();
              }
          }
     
          ///////////////////////////////////////////////////////////////////////////
          if (sum >= 2  && diff <=1 || leftSensor >=1 || rightSensor >= 1 )    
          { 
            searchingMode =false;                 
            attackMode = true;
            scanningWatch.reset();
            goingWatch.reset();
            speedWatch.reset();
            goingDetectWatch.reset();
            //goingDetectWatch.reset();
            if (leftSensor >=1)
           {
            leftDetect = true;
            Serial.print("left");
            Serial.println(leftSensor);
            
           }
            if (rightSensor >=1)
           {
            rightDetect =true;
            Serial.print("right");
            Serial.println(rightSensor);
           }
          }             
        }
     
  }

//==============================================================================================================================================================================================================
  if (attackMode == true && borderMode == false  && collisionDetect ==false)
  {
    
    if (sum == 0 && leftSensor == 0 && rightSensor ==0)
    {
      attackMode = false;
      searchingMode = true;
    }
    if (sum >= 2 && diff <=1 )    
    {                  
     motor.setSpeeds(400,400);
     goingDetectWatch.start();///////////////////////////////////////////
     if(goingDetectWatch.value() > 100)
     {
       speedWatch.start();
       if (speedWatch.value() <= 10)
       {
         trash = encoder.getCountsAndResetLeft();
         trash = encoder.getCountsAndResetRight();
        }
       if (speedWatch.value() >= 100 && encoder.getCountsAndResetLeft() < 430 && encoder.getCountsAndResetRight() < 430)
        {
          Serial.print("true");
          collisionDetect = true;
          attackMode = false;
          searchingMode = false;
          //goingWatch.reset();
          goingDetectWatch.reset();
          speedWatch.reset();
        }
       else if(speedWatch.value() >= 120)
       {
         speedWatch.reset();
         goingDetectWatch.reset();
         }
       }          
    }
         

    if (leftDetect == true)
 {           
     sensorTurningWatch.start();   
    if( sensorTurningWatch.value() < 500 )
    {
      motor.setSpeeds(-300,300);   
      if (sum >= 2 && diff <=1 && leftSensor == 0 ) 
      {
        leftDetect = false;
        sensorTurningWatch.reset();       
      }
    }
    else if (sensorTurningWatch.value() >= 500)
    {
      
      leftDetect = false;
      sensorTurningWatch.reset();
    }
 }
    else if (rightDetect ==true)
    {    
         sensorTurningWatch.start();
    
    if( sensorTurningWatch.value() < 500 )
    {
      motor.setSpeeds(300,-300);
      if (sum >= 2 && diff <=1 && rightSensor == 0 ) 
      {
        rightDetect = false;
        sensorTurningWatch.reset();       
      }
    }
    
    else if (sensorTurningWatch.value() >= 500)
    {
      rightDetect = false;
      sensorTurningWatch.reset();
    }
   }
     lineSensors.read(lineSensorValues);
     if (lineSensorValues[0] < lineSensorLimit || lineSensorValues[2] < lineSensorLimit)
     {
      borderMode = true;
      //borderModeinit = true;
      speedWatch.reset();
      goingDetectWatch.reset();
     }
  }
      


  


 //==============================================================================================================================================================================================================

  if (borderMode == true)
  {
    if(lineSensorValues[0] <lineSensorLimit && lineSensorValues[2] > lineSensorLimit && lineSensorValues[1] > midSensorLimit) // left side detects border
  {
    turnRightDetect = true;
    borderTurningWatch.reset();
    borderTurningWatch.start();
    
  }
  else if (lineSensorValues[0] >lineSensorLimit && lineSensorValues[2] < lineSensorLimit && lineSensorValues[1] > midSensorLimit) //right side detects border
   {
      turnLeftDetect = true;
      borderTurningWatch.reset();
      borderTurningWatch.start();
          
   }
   else if(lineSensorValues[0] <lineSensorLimit && lineSensorValues[2] > lineSensorLimit && lineSensorValues[1] < midSensorLimit)
   {
    backRightDetect = true;
    borderTurningWatch.reset();
    borderTurningWatch.start();
   }
      else if(lineSensorValues[0] > lineSensorLimit && lineSensorValues[2] < lineSensorLimit && lineSensorValues[1] < midSensorLimit)
   {
    backLeftDetect = true;
    borderTurningWatch.reset();
    borderTurningWatch.start();
   }
  else if(lineSensorValues[0] < lineSensorLimit && lineSensorValues[2] < lineSensorLimit) //both detects border
   {
      turnAroundDetect = true;
      borderTurningWatch.reset();
      borderTurningWatch.start();            
   }
  if (turnRightDetect = true)
  {
    if(borderTurningWatch.value() < 300)
    {
      motor.setSpeeds(300, -300);
    }
  }
  if (turnLeftDetect = true)
  {
    if(borderTurningWatch.value() < 300)
    {
      motor.setSpeeds(-300, 300);
    }
  }
  if (turnAroundDetect = true)
  {
    if(borderTurningWatch.value() < 300)
    {
      motor.setSpeeds(forwardVeerSpeed, reverseVeerSpeed);
    }
  }
  if(backLeftDetect = true)
  {
    if(borderTurningWatch.value() < 50)
    {
      motor.setSpeeds(-400, -400);
    }
    else if(borderTurningWatch.value() < 300 && borderTurningWatch.value()>= 50)
    {
      motor.setSpeeds(-300, 300);
    }
  }
    
      if(backRightDetect = true)
  {
    if(borderTurningWatch.value() < 50)
    {
      motor.setSpeeds(-300, -300);
    }
    else if(borderTurningWatch.value() < 300 && borderTurningWatch.value()>= 50)
    {
      motor.setSpeeds(300, -300);
    }
    
  }

  if(borderTurningWatch.value() >= 300)
  {
    turnRightDetect = false;
    turnLeftDetect = false;
    turnAroundDetect = false;
    backRightDetect = false;
    backLeftDetect = false;
    borderTurningWatch.reset();
    borderMode = false;
  }
  if (sum >= 2  && diff <=1 || leftSensor >=1 || rightSensor >= 1)    
   { 
   searchingMode = false;     
   attackMode = true;
    speedWatch.reset();
    goingDetectWatch.reset();

   } 
           
}
//===================================================================================================================================================================================================================================
if(collisionDetect == true && borderMode ==false )
{
  motor.setSpeeds(400,400);
  goingDetectWatch.start();
  ledRed(1);
  lineSensors.read(lineSensorValues);
  
   if (lineSensorValues[0] < lineSensorLimit || lineSensorValues[2] < lineSensorLimit)
    {
      borderMode = true;
      goingDetectWatch.reset();
      speedWatch.reset();
      borderModeinit = true;
      collisionDetect = false;
      ledRed(0);
     }

            if(goingDetectWatch.value() > 100)
          {
            speedWatch.start();
            if (speedWatch.value() <= 10)
              {
                trash = encoder.getCountsAndResetLeft();
                trash = encoder.getCountsAndResetRight();
              }
           if (speedWatch.value() >= 100 && encoder.getCountsAndResetLeft() > 470 && sum < 2 && encoder.getCountsAndResetRight() > 470)
              {
                collisionDetect = false;
                attackMode = false;
                searchingMode = true;
//                goingWatch.reset();
                goingDetectWatch.reset();
                speedWatch.reset();
                ledRed(0);
              }
              else if (speedWatch.value() >= 130)
              {
                speedWatch.reset();
                goingDetectWatch.reset();
              }
          }    
}



}




  
