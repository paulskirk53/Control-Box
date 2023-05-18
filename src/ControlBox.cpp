/*

Next steps: 
1 - put together a data packet for transmission to the monitor on receipt of a request
    perhaps assemble the packet at the end of loop(). The monitor program informs the data which is needed
    the target az -     targetazimuth
    movement direction  querydir   
    movement status     movementstate
    target status?      targetmessage
    Degrees to target   String(CDArray[CurrentAzimuth])
    Dome Azimuth        
    Camera power state

2 - look through the code to identify receipts and transmissions related to the two datastreams - ASCOM and MONITOR

Note Note Note Note Note Note

This project is the Code for the new (1-5-23) control box (stepper and encoder functions) on one AVR4809 chip


Note Note Note Note Note Note
*/

/*

AVR4809 pinout for the control box - see the google sheet 

https://docs.google.com/spreadsheets/d/1RLFg1F5WgP97Ck7IOUJbF8Lhts_1J4T0fl-OKxMCbDc/edit#gid=0

*/



//
// see this sheets URL for values related to deceleration used to inform values in this code
// https://docs.google.com/spreadsheets/d/1IBvHXLke9fBvjETHgyCWAYHGpPX9WbbuqvsiXYv9Ix0/edit#gid=0

// verion 6.0 - change the variable in setup too
// DECELVALUE AND NORMALACCELERATION LOOK GOOD
//  check the final moveto values as they may need empirical change on testing
//  This routine accepts these commands from the ASCOM Driver via USB Serial Cable:
//
// ES# - emergency stop
// SA999# - Slew to azimuth
// SL# - Slew status request
//
//  The routine drives the stepper motor to move the Dome
//  It acquires the current azimuth via hardware serial from the encoder
//  todo - the former spi transaction included a flag for the home position which was set in eastsync() - need to perhaps put this into westsync()
//  as there is no eastsync in the observatory
#include <Arduino.h>
#include <avr/cpufunc.h> /* Required header file for wdt resets*/
#include <AccelStepper.h>
#include "linkedList.h"


// Forward declarations

void Emergency_Stop(int azimuth, String mess);
String WhichDirection();
void WithinFiveDegrees();
int getCurrentAzimuth();
void check_If_SlewingTargetAchieved();
void createDataPacket();
void PowerOn();
void PowerOff();
void resetViaSWR();
void lightup();
bool checkForValidAzimuth();

uint16_t encoder();
bool PowerForCamera(bool State);
void interrupt();
void WestSync();

// end declarations
// defines for the encoder inclusion
#define power_pin 2   
#define A_PHASE 4 // USES PINS 4 AND 5 for encoder interrupt todo check that these pins will work as interrupts
#define B_PHASE 5
#define CameraPower 6  // power for the imaging camera
#define dirPin 10  // connection for motor direction signal
#define stepPin 11  // connection for motor step signal

#define EncoderledPin 14  // led flash for a function to be defined in control box
#define WestPin 28        // sync connection for dome
//
#define off false
#define on true

// meaningful names for the serial ports
#define Monitor Serial2
#define ASCOM Serial

#define ledpin 3  // no clash with encoder

// Define a stepper and the pins it will use

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, true);

String receivedData;
boolean DoTheDeceleration;
boolean Slewing; // controls whether the stepper is stepped in the main loop
boolean homing;
boolean homeSensor;
float StepsPerSecond; // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25

boolean TargetChanged = false;
boolean monitorSendFlag = false; // this only becomes true after the MCU is connected successfully and when true, the data stream to the monitor program is enabled
float normalAcceleration;        // was incorrectly set to data type int

int stepsToTarget = 0;
int DecelValue = 400; // set at this value of 800 after empirical test Oct 2020. Update April 22 with Pulsar dome this may need to be halved to 400 
int EncoderReplyCounter = 0;
int savedAzimuth = 0;
long monitorTimerInterval = 0.0l; // note l after 0.0 denotes long number - same type as millis()
long azimuthTimerInterval = 0.0l;

String TargetMessage = "";
String QueryDir = "No Direction";
String movementstate = "Not Moving";
String pkversion = "6.0";
String dataPacket = "";

volatile long A_Counter; // volatile because it's used in the interrupt routine
float Azimuth;           // The data type is important to avoid integer arithmetic in the encoder() routine
uint16_t integerAzimuth; // this is what is returned from the encoder routine
                         // and also because we really don't need fractional degrees for dome movement.
float ticksperDomeRev = 25880;  //was 10513 (changed 20/4/22) this was worked out empirically by counting the number of encoder wheel rotations for one dome rev. 11-9-21

bool cameraPowerState = off;


/*
  --------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------
*/

void setup()
{
//Pinmodes for the stepper code
  pinMode(power_pin, OUTPUT);
  digitalWrite(power_pin, LOW); // initialise the pin state so that the mosfet gate is Low and therefore power to the MA860H is off
  pinMode(9, INPUT_PULLUP);     // see the notes in github. this pulls up the serial Rx pin to 5v.
  pinMode(ledpin, OUTPUT);


// Pin modes for the encoder


  pinMode(WestPin, INPUT_PULLUP);


  pinMode(EncoderledPin, OUTPUT);
  pinMode(CameraPower, OUTPUT);

  //turn the camera power of at startup:
  digitalWrite (CameraPower, LOW);           //  LOW is camera power OFF

 // encoder:
  pinMode(A_PHASE, INPUT_PULLUP);
  pinMode(B_PHASE, INPUT_PULLUP);



  // pins 2,3,18,19,20,21 are the only pins available to use with interrupts on the mega2560 (no pin limit)restrictions on 4809)
  attachInterrupt(digitalPinToInterrupt(A_PHASE), interrupt, RISING); // interrupt for the encoder device

  // interupts for the azimuth syncs below
  
  attachInterrupt(digitalPinToInterrupt(WestPin), WestSync, RISING);

  A_Counter = ticksperDomeRev / (360.0 / 261.0); //  the position of due west - 261 for the dome when the scope is at 270.

  PowerForCamera(off); // camera power is off by default



  stepper.stop(); // set initial state as stopped

  // Change below to suit the stepper

  Slewing = false;
  StepsPerSecond = 300.0;              // changed following empirical testing Oct 2020
  normalAcceleration = 140.0;          // changed following empirical testing October 17th 2020 - changed from 40 to 20 for trial
  stepper.setMaxSpeed(StepsPerSecond); // steps per second see below -
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(normalAcceleration); // steps per second per second.
  
  // initialise

  CurrentAzimuth       = 0;
  DoTheDeceleration    = true; // used to set deceleration towards target azimuth
  monitorTimerInterval = millis();
  azimuthTimerInterval = millis();

  homeSensor = false;          // this later set in the getcurrentazimuth() spi transaction todo find out about how this works

  ASCOM.begin(19200);   // start serial ports ASCOM driver - usb with PC - rx0 tx0 and updi
  
  Monitor.begin(19200); // serial with the Monitor program

  lightup(); // 10 SECOND DELAY flash Led to indicate reset when the box lid is off for testing
             // ALLOWS setup time for serial comms

  // ASCOM.println(" before get azimuth");



  TargetAzimuth = getCurrentAzimuth(); // todo - check that a valid azimuth is returned

  initialiseCDArray();

  
} // end setup

/*
  \\\\\\\\\\\\\\\\/////////////////////////\\\\\\\\\\\\\\\\\\\\\///////////////
  ////////////////\\\\\\\\\\\\\\\\\\\\\\\\\/////////////////////\\\\\\\\\\\\\\\
  ////////////////\\\\\\\\\\\\\\\\\\\\\\\\\/////////////////////\\\\\\\\\\\\\\\
  \\\\\\\\\\\\\\\\/////////////////////////\\\\\\\\\\\\\\\\\\\\\///////////////
*/

void loop()
{

  // put your main code here, to run repeatedly, perhaps for eternity if the power holds up....

  if (Monitor.available() > 0)
  {
    String monitorReceipt = Monitor.readStringUntil('#');

    if (monitorReceipt.indexOf("dataRequest", 0) > -1)   //this will be received from the monitor program
    {
      Monitor.print(dataPacket);
    }

    
    if (monitorReceipt.indexOf("reset", 0) > -1)
    {
      Monitor.print("resetting");
      // ASCOM.print("get this");
      // TODO MAYBE REINSTATE THE LINE BELOW - done
      resetViaSWR();
    }
  } // endif Monitor.available

  if (ASCOM.available() > 0) // when serial data arrives from the driver on USB capture it into a string
  {

    receivedData = ASCOM.readStringUntil('#'); // read a string from PC serial port usb
    

    //*************************************************************************
    //******** code for MCU Identity process below ****************************
    //**** Used by the ASCOM driver to identify the COM port in use. **********
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("controlbox", 0) > -1)
    {
      ASCOM.print("controlbox#");
    }

    //*************************************************************************
    //******** code for emergency stop process below **************************
    //******** data sent by driver ES#               **************************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("ES", 0) > -1) // Emergency stop requested from C# driver
    {
      // lcd.clear();
      Emergency_Stop(CurrentAzimuth, "Received ES");
      receivedData = "";
    } // end Emergency Stop

    //*************************************************************************
    //******** code for SA process below **************************************
    //**** format of data sent by driver SA220.00#   **************************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("SA", 0) > -1) //
    {

      PowerOn(); // turn on the power supply for the stepper motor
      // strip off 1st 2 chars
      receivedData.remove(0, 2);

      TargetAzimuth = receivedData.toInt(); // store the target azimuth for comparison with current position
      // the way the code works is to treat a rrquest for az = 360 as az =0 , hence the if clause below
      if (TargetAzimuth == 360)
      {
        TargetAzimuth = 0;
      }
      bool AzOK = checkForValidAzimuth();
      if (AzOK)
      {
        TargetChanged = true;

        //  Serial.println();
        //  Serial.print("in slewto target received ");
        //  Serial.println(TargetAzimuth);

        if (Slewing == false) // only do this if not slewing
        {
          Slewing = true;
          stepper.setAcceleration(normalAcceleration); // set the acceleration
          stepper.setCurrentPosition(0);               // initialise the stepper position
          QueryDir = WhichDirection();                 // work out which direction of travel is optimum
          // todo remove 2 lines blow
          // ASCOM.print("So the direction is  ");
          // ASCOM.println(QueryDir);

          if (QueryDir == "clockwise")
          {
            stepper.moveTo(150000000); // positive number means clockwise in accelstepper library. This number must be sufficiently large
                                       // to provide enough steps to reach the target.
          }

          if (QueryDir == "anticlockwise")
          {

            stepper.moveTo(-150000000); // negative is anticlockwise in accelstepper library
          }

          DoTheDeceleration = true;

          // MOVED THE FOLLOWING FROM HERE TO NEXT LEVEL receivedData = "";
        }
        receivedData = "";

      } // end if azok
    } // end if SA

    //**********************************************************
    //******** code for SL process below ***********************
    //******** data sent by driver SL#   ***********************
    //**********************************************************
    //

    if (receivedData.indexOf("SL", 0) > -1) //
    {

      if (Slewing)
      {
        ASCOM.print("Moving#");
        // stepper.run();
      }
      else
      {
        ASCOM.print("Notmoving#"); // sent to ASCOM serial and picked up by the ASCOM driver
      }
      receivedData = "";

    } // end SL case

    //*************************************************************
    //******** code for FH process below **************************
    //******** data sent by driver FH#   **************************
    //*************************************************************
    //*************************************************************

    if (receivedData.indexOf("FH", 0) > -1)
    {
       StepsPerSecond = 300.0;                 // changed following empirical testing Oct 2020
       normalAcceleration = 140.0;             // changed following empirical testing October 17th 2020 - changed from 40 to 20 for trial
       stepper.setMaxSpeed(StepsPerSecond);    // steps per second see below -
       stepper.setCurrentPosition(0);          // wherever the motor is now is set to position 0
       stepper.setAcceleration(normalAcceleration/2.0); // half normal for homing

       stepper.moveTo(150000000);              // this number has to be large enough for the drive to be able to complete a full circle.

       PowerOn(); 
       homing = true;                          // used in loop() to control motor movement

       // send to monitor - started homing
      TargetMessage  = "Started Homing ";
      QueryDir       = "clockwise";                  //set the direction of movement - this is also sent to the monitor program
      movementstate  = "Homing...";

      receivedData = "";

    }

  } // end if ASCOM Available

  // so from here down is code to deliver SA function

  // start grouping for Slewing functions here
  if (Slewing) // if the slew status is true, run the stepper and update the data in the monitor program
  {
    WithinFiveDegrees();

    stepper.run();

    

    check_If_SlewingTargetAchieved();   //checks if slew is ended and updates monitor

  }  // endif Slewing

if (homing)
{
  if ((millis() - azimuthTimerInterval) > 200.0) // one FIFTH second checks for HOMESENSOR STATE as the dome moves
  {
    getCurrentAzimuth();                      // The spi transaction gets the homesensor state todo find out
    azimuthTimerInterval = millis();
    //ASCOM.print("VALUE OF HOMESENSOR IS true if activated ");
    //ASCOM.println(homeSensor);
  }

  if (homeSensor==true)                     // true indicates the sensor at the home position has been activated
  {
    movementstate = "Not Moving";
    QueryDir      = "None";
    TargetMessage = "Homing Complete";
    homing        = false;
    homeSensor    = false;      //homing is finished, so set the sensor to false. It may be set true again by calls to getcurrentazimuth()
    //load and try
    PowerOff();
  }


}

 // update the data packet for monitoring program
    //
    if ((millis() - monitorTimerInterval) > 1000.0) // one second checks for azimuth value as the dome moves
    {
      // TODO UNCOMMENT THE LINE BELOW
      createDataPacket();

      monitorTimerInterval = millis();
    }



  stepper.run();   // stepper run - works for slewing and for findHome

} // end void Loop //////////////////////////////////////////////////////////////////////////////////////////////////////

void Emergency_Stop(int azimuth, String mess)
{

  stepper.stop();
  Slewing = false;

  // turn off power to the stepper
  PowerOff();
}

String WhichDirection()
{
  // this routine decides the shortest direction to go based on the difference betwen current and target azimuth
  // optimises battery use by the motor.

  CurrentAzimuth = getCurrentAzimuth(); // this comes from the encoder
  // azimuth = CurrentAzimuth;          //save this to work out the distance to go
  int clockwiseSteps = calculateClockwiseSteps();
  int antiClockwiseSteps = 360 - clockwiseSteps;

  if (clockwiseSteps <= antiClockwiseSteps)
  {
    stepsToTarget = clockwiseSteps; // used to define the number of items in the countdown array
    countDown("clockwise");         // populate the cdarray with the smaller number of steps
    return "clockwise";
  }
  else
  {
    stepsToTarget = antiClockwiseSteps; // used to define the number of items in the countdown array
    countDown("anticlockwise");         // populate the cdarray with the smaller number of steps
    return "anticlockwise";
  }
}

void WithinFiveDegrees()
{

  if (DoTheDeceleration)
  {

    CurrentAzimuth = getCurrentAzimuth();

    if ((abs(CurrentAzimuth - TargetAzimuth) < 5) && (TargetChanged == true)) // within 5 degrees of target
    {

      DoTheDeceleration = false;
      if (QueryDir == "clockwise")
      {
        // set the moveto position to allow 100 steps more for deceleration  +ve for clockwise -ve for anticclock

        stepper.moveTo(stepper.currentPosition() + DecelValue); // FROM MA860H Datasheet @0.225 step angle, it requires 1600 steps per rotation
        // of the stepper drive wheel, so 1000 is 0.6 of a rotation
      }

      if (QueryDir == "anticlockwise")
      {
        //  stepper.setCurrentPosition(0);
        stepper.moveTo(stepper.currentPosition() - DecelValue); // check this by printing out current position is it negative?
      }
    }
  }
}

int getCurrentAzimuth()
{
   encoder();
  return integerAzimuth;

} // end getCurrentAzimuth()

void check_If_SlewingTargetAchieved()
{

    if (abs(stepper.distanceToGo()) < 20)
    {
      Slewing = false;              // used to stop the motor in main loop
      movementstate = "Stopped.  "; // for updating the lcdpanel

      // Serial.print("ABS STEPPER distance to go....");
      // Serial.println();
      // update the LCD
      TargetMessage = "Target achieved ";
      QueryDir = "None";

      createDataPacket();

      PowerOff(); // power off the stepper now that the target is reached.
    }
    else
    {
      movementstate = "Moving"; // for updating the lcdpanel
      TargetMessage = "Awaiting Target ";
      stepper.run();
    }


}

void createDataPacket()
{
  
  CurrentAzimuth = getCurrentAzimuth(); 
  String dataPacket ="";
    
  dataPacket = String(TargetAzimuth) + '#' + movementstate + '#' + QueryDir + '#' + TargetMessage + '#' + String(CDArray[CurrentAzimuth]) + '#';
  
    /*
    the line above can be removed and the commented section below reinstated. Done to try to improve speed
      Monitor.print("START#");
      Monitor.print(String(TargetAzimuth)        + '#');
      Monitor.print(movementstate                + '#');
      Monitor.print(QueryDir                     + '#');
      Monitor.print(TargetMessage                + '#');

      */
  
}

//---------------------------------------------------------------------------------------------------------------

void PowerOn() // set the power SSR gate high
{
  digitalWrite(power_pin, HIGH);

  delay(2000); // gives time for the MA860H unit to power on and stabilise
}

//---------------------------------------------------------------------------------------------------------------

void PowerOff() // set the power SSR gate low
{
  digitalWrite(power_pin, LOW);
}

void resetViaSWR()
{
  _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
}

void lightup()
{
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(ledpin, HIGH);
    delay(1000);
    digitalWrite(ledpin, LOW);
    delay(1000);
  }
}
bool checkForValidAzimuth()
{
  if ( (TargetAzimuth >= 0) && (TargetAzimuth <=359) )
  {
    return true;
  }
  else
  {
    return false;
  }
}


uint16_t encoder()
{
  // Encoder:

  if (A_Counter < 0)
  {
    A_Counter = A_Counter + ticksperDomeRev; // set the counter floor value
  }

  if (A_Counter > ticksperDomeRev) // set the counter ceiling value
  {
    A_Counter = A_Counter - ticksperDomeRev;
  }

  Azimuth = float(A_Counter) / (ticksperDomeRev / 360.0); // (ticks for one dome rev) / 360 (degrees) - about 29
  // i.e number of ticks per degree

  // some error checking
  if (Azimuth < 1)
  {
    Azimuth = 1.0;
  }

  if (Azimuth > 360.0)
  {
    Azimuth = 360.0;
  }

  integerAzimuth = Azimuth; // REMEMBER Azimuth needs to be float due to effects of integer arithmetic.
  return integerAzimuth;
} // end void encoder


bool PowerForCamera(bool State)
{
  if (State)
  {
    digitalWrite(CameraPower, HIGH);  //
    cameraPowerState = on;
  }
  else
  {
    digitalWrite(CameraPower, LOW); //NB as above
    cameraPowerState = off;
  }
}

void interrupt() // Interrupt function
{

  char i, j;
  i = digitalRead(B_PHASE);
  j = digitalRead(A_PHASE);
  if (i == j)
  {
    A_Counter -= 1;
  }

  else
  {

    A_Counter += 1; // increment the counter which represents increasing dome angle

  } // end else clause
} // end void interrupt


void WestSync()
{
  // this routine is called when the westsync interrupt fires
  A_Counter = ticksperDomeRev / 4.0;
  homeSensor==true;   //set this when the hall sesnor is detected. It indicates the dome is at the home (261) degrees position when the homing process runs
}
