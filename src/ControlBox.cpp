/*


NOte re encoder calibration:
see this spreadsheet for calcuations of ticks per dome revolution - google sheets 'encoder ticks per dome rev'

Next steps: 
1 - put together a data packet for transmission to the monitor on receipt of a request. The packet is updated once per sec in the monitortimerinterval() routine - done
    the packet is assembled using global vars which are uodated as any of the data items below are changed 
    The monitor program informs the data which is needed in the packet
    the target az -     targetazimuth
    movement direction  querydir   
    movement status     movementstate
    target status?      targetmessage
    Degrees to target   String(CDArray[CurrentAzimuth])
    Dome Azimuth        getcurrentazimuth()  ??
    Camera power state  cameraPowerState

2 - look through the code to identify receipts and transmissions related to the two datastreams - ASCOM and MONITOR

Note Note Note Note Note Note

This project is the Code for the new (1-5-23) control box (stepper and encoder functions) on one AVR4809 chip


Note Note Note Note Note Note
*/

/*

AVR4809 pinout for the control box - see the mailbox sheet 


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
//TODO SET THE STEPPER CURRENT POS TO ZERO BEFORE SETTING THE TARGET FOR QUERYDIR

#include <Arduino.h>
#include <avr/cpufunc.h> /* Required header file for wdt resets*/
#include <AccelStepper.h>
#include "linkedList.h"
#include <avr/eeprom.h>

// Forward declarations

void Emergency_Stop(int azimuth, String mess);
String WhichDirection();
void WithinFiveDegrees();
int getCurrentAzimuth();
void check_If_SlewingTargetAchieved();
void createDataPacket();
void domePowerOn();
void domePowerOff();
void resetViaSWR();
void lightup();
bool checkForValidAzimuth();

uint16_t encoder();
bool PowerForCamera(bool State);
void interrupt();
void WestSync();
void ledToggle();

// end declarations
// defines for the encoder inclusion
#define power_pin 2   
#define A_PHASE 4      // USES PINS 4 AND 5 for encoder interrupt - check that these pins will work as interrupts
#define B_PHASE 5
#define CameraPower 6  // power for the imaging camera
#define dirPin 10      // connection for motor direction signal
#define stepPin 11     // connection for motor step signal


#define WestPin 29        // sync connection for dome
//
#define off false
#define on true

// meaningful names for the serial ports
#define Monitor Serial2
#define ASCOM Serial

#define ledpin 23  

// Define a stepper and the pins it will use

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, true);

// EEPROM vars for storing the Azimuth and Toggle
uint16_t EEMEM NonVolatileAzimuth;
uint16_t EEMEM NonVolatileToggle;
// SRAM Vars paired to the above
uint16_t SRAMAzimuth;
uint16_t SRAMToggle;


String receivedData;
boolean DoTheDeceleration;
boolean Slewing;                 // controls whether the stepper is stepped in the main loop
boolean homing = false;
boolean homeSensor = false;
float StepsPerSecond;            // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25

boolean TargetChanged = false;
boolean monitorSendFlag = false; // this only becomes true after the MCU is connected successfully and when true, the data stream to the monitor program is enabled
float normalAcceleration;        // was incorrectly set to data type int

int stepsToTarget = 0;
int DecelValue = 400;            // set at this value of 800 after empirical test Oct 2020. Update April 22 with Pulsar dome this may need to be halved to 400 
int EncoderReplyCounter = 0;
int savedAzimuth = 0;
long monitorTimerInterval = 0.0l; // note l after 0.0 denotes long number - same type as millis()
long azimuthTimerInterval = 0.0l;
long LedTimerInterval     = 0.0l;

String TargetMessage = "No Target";
String QueryDir = "No Direction";
String movementstate = "Not Moving";
String pkversion = "6.0";
String dataPacket = "";

volatile long A_Counter;    // volatile because it's used in the interrupt routine
volatile int syncCount = 0; // counts the number of syncs and acts as an indicator on the monitor program
float Azimuth;              // The data type is important to avoid integer arithmetic in the encoder() routine
uint16_t integerAzimuth;    // this is what is returned from the encoder routine
                            // and also because we really don't need fractional degrees for dome movement.
float ticksperDomeRev = 15675;  // a dome rotation shows the wheel on the encoder turns through 26.125 turns. Use the spreadsheet 
                                // mentioned at top, to calculate the number of ticks (for this current encoder 26.125 * 600)
float ticksPerDegree  = ticksperDomeRev /360.0;   // do the calculation here just once
bool cameraPowerState = off;


/*
  --------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------
*/

void setup()
{
//Pinmodes for the stepper code
  pinMode(power_pin, OUTPUT);
  digitalWriteFast(power_pin, LOW); // initialise the pin state so that the mosfet gate is Low and therefore power to the MA860H is off
  pinMode(9, INPUT_PULLUP);     // see the notes in github. this pulls up the serial Rx pin to 5v.
  pinMode(ledpin, OUTPUT);


// Pin modes for the encoder


  pinMode(WestPin, INPUT_PULLUP);

  pinMode(CameraPower, OUTPUT);

  //turn the camera power of at startup:
  digitalWriteFast (CameraPower, LOW);           //  LOW is camera power OFF

 // encoder:
  pinMode(A_PHASE, INPUT_PULLUP);
  pinMode(B_PHASE, INPUT_PULLUP);



  // pins 2,3,18,19,20,21 are the only pins available to use with interrupts on the mega2560 (no pin limit)restrictions on 4809)
  attachInterrupt(digitalPinToInterrupt(A_PHASE), interrupt, RISING); // interrupt for the encoder device

  // interupts for the azimuth syncs below
  
  attachInterrupt(digitalPinToInterrupt(WestPin), WestSync, FALLING);    // the sync line is high until the magnet arrives when it falls,

// Read the EEPROM to check whether this is a power cycled startup (toggle != 1) or a software reset (via the monitor program, Toggle = 1)

SRAMAzimuth = eeprom_read_word(&NonVolatileAzimuth);
SRAMToggle  = eeprom_read_word(&NonVolatileToggle);


if (SRAMToggle == 1 )
  {
    // software resets execute the line below, which preserves the dome azimuth at point of reset.
    A_Counter = ticksperDomeRev / (360.0 / float(SRAMAzimuth) );  // set the azimuth to the value stored in EEPROM
  }
  else
  {
    // initial power cycled starts execute the line below
    A_Counter = ticksperDomeRev / (360.0 / 261.0); //  the position of due west - 261 (calculation checked) for the dome when the scope is at 270.
  }

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
  LedTimerInterval     = millis();

  homeSensor = false;          // 

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

    if (monitorReceipt.indexOf("dataRequest", 0) > -1)   // request for data packet 
    {
      Monitor.print(dataPacket);
    }
    

    //*************************************************************************
    //******** code for Monitor MCU Identity process below **********************
    //**** Used by the ASCOM driver to identify the COM port in use. **********
    //*************************************************************************
    //*************************************************************************

    if (monitorReceipt.indexOf("monitorcontrol", 0) > -1)   // MCU id request 
    {
      Monitor.print("monitorcontrol#");
    }
    
    //*************************************************************************
    //********      code for MCU reset process below          *****************
    //*************************************************************************
    //*************************************************************************

    if (monitorReceipt.indexOf("reset", 0) > -1)     // reset the control box MCU
    {
      Monitor.print("resetting");
      // preserve  the dome azimuth and set the reset Toggle in EPROM
      eeprom_update_word(&NonVolatileAzimuth, getCurrentAzimuth());  // write the current azimuth value to EEPROM
      eeprom_update_word(&NonVolatileToggle, 1 );  // write a 1 to te toggle in EEPROM to signify a software reset has taken place

      delay(1000);   //
      resetViaSWR();
    }

    //*************************************************************************
    //************** code for Camera power process below **********************
    //**** Used by the Monitor program to control the power state (on /off) ***
    //*************************************************************************
    //*************************************************************************

if (monitorReceipt.indexOf("CAMON", 0) > -1)     // turn imaging camera power on
    {
       PowerForCamera(on);
       //Monitor.print("rec'd camon");
    }

if (monitorReceipt.indexOf("CAMOFF", 0) > -1)     // turn imaging camera power off
    {
       PowerForCamera(off);
       //Monitor.print("rec'd camOFF");
    }

   // 
if (monitorReceipt.indexOf("eepromtoggle", 0) > -1)   // the toggle will be set to zero which is used to indicate a power down reset - no preservation of the dome azimuth
    {                                                     // at the time of reset i.e. this would be an end of observing session power down
      eeprom_update_word(&NonVolatileToggle, 0);
    }


  } // endif Monitor.available

    //*************************************************************************
    //************     code for ASCOM functions below    **********************
    //**** Used by the ASCOM driver to facilitate  dome functions    **********
    //*************************************************************************
    //*************************************************************************



  if (ASCOM.available() > 0) // when serial data arrives from the driver on USB capture it into a string
  {

    receivedData = ASCOM.readStringUntil('#'); // read a string from PC serial port usb
    
    //*************************************************************************
    //********      code for ASCOM Azimuth FUNCTION      **********************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("AZ", 0) > -1)
    {

      String x = (String) getCurrentAzimuth();
      x += "#";
      ASCOM.print(x);
    }

//TEST LINES X BELOW TODO REMOVE

/*
if (receivedData.indexOf("DI", 0) > -1)     // THIS IS PURELY FOR DEBUG and returns the distance to go to the serial monitor. The command is not used by ASCOm Driver
    {
      int x =  stepper.distanceToGo();
      ASCOM.println(String(x) + "#");
    }

*/

    //*************************************************************************
    //******** code for ASCOM MCU Identity process below **********************
    //**** Used by the ASCOM driver to identify the COM port in use. **********
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("controlbox", 0) > -1)
    {
      ASCOM.print("controlbox#");
    }

    //*************************************************************************
    //******** code for emergency stop process below **************************
    //********            data sent by ASCOM driver ES#         ***************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("ES", 0) > -1) // Emergency stop requested from C# driver
    {
      // lcd.clear();
      Emergency_Stop(CurrentAzimuth, "Received ES");
      receivedData = "";
    } // end Emergency Stop

    //*************************************************************************
    //********      code for SA process below            **********************
    //******** format of data sent by driver SA220.00#   **********************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("SA", 0) > -1) //
    {

      domePowerOn(); // turn on the power supply for the stepper motor
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
            stepper.setCurrentPosition(0);
            stepper.moveTo(150000000); // positive number means clockwise in accelstepper library. This number must be sufficiently large
                                       // to provide enough steps to reach the target.
          }

          if (QueryDir == "anticlockwise")
          {
            stepper.setCurrentPosition(0);
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

      if (Slewing | homing)
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

       domePowerOn(); 
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
    getCurrentAzimuth();                      
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
    domePowerOff();
  }


}

 // create / update the data packet for monitoring program
    //
    if ((millis() - monitorTimerInterval) > 1000.0) // one second checks for azimuth value as the dome moves and tick the heartbeat LED
    {
      
      createDataPacket();
      monitorTimerInterval = millis();

    }

    if ((millis() - LedTimerInterval) > 20000.0) // twenty second timer for a led flash - an indicator that code is running
    {
      
      ledToggle();
      LedTimerInterval = millis();
    }


  stepper.run();   // stepper run - works for slewing and for findHome

/*
remove the code between //start and //end below - it was inserted as a way of simulating the home position for testing



//start
if ( (Azimuth > 260.0) && (Azimuth <265) )
{
  homeSensor = true;
}

*/

//end



} // end void Loop //////////////////////////////////////////////////////////////////////////////////////////////////////

void Emergency_Stop(int azimuth, String mess)
{

  stepper.stop();
  Slewing = false;

  // turn off power to the stepper
  domePowerOff();
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
      //TODO REMOVE 3 TEST LINE BELOW
     // int x = abs(stepper.distanceToGo() );
     // Monitor.println(x);
      Slewing = false;              // used to stop the motor in main loop
      movementstate = "Stopped.  "; // for updating the lcdpanel

      // Serial.print("ABS STEPPER distance to go....");
      // Serial.println();
      // update the LCD
      TargetMessage = "Target achieved ";
      QueryDir = "None";

     //todo - this is probably not needed here - just the var updates which will be used in the monitortimerinterval() routine once per second to create the packet 
     //createDataPacket();

      domePowerOff(); // power off the stepper now that the target is reached.
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
      
  //eight items below
  // dataPacket = String(CurrentAzimuth) + '#' + String(TargetAzimuth) + '#' + movementstate + '#' + QueryDir + '#' + TargetMessage + '#' + String(CDArray[CurrentAzimuth]) + '#' + String(cameraPowerState) + '#' +String(syncCount) + '#' + '$';
  dataPacket = String(CurrentAzimuth) + '#' + String(TargetAzimuth) + '#' + movementstate + '#' + QueryDir + '#' + TargetMessage + '#' + String(CDArray[CurrentAzimuth]) + '#' + String(cameraPowerState) + '#' +String(syncCount) + '#' + '$';
  //                  dome azimuth,                  target azimuth,        movementstate,       querydir,         targetmessage,               cdarray[currentazimut] ,                cameraPowerState
  //note the string item delimiter is # 
  //note the string delimiter is $
  //todo remove the line below which was just for testing 
  //Monitor.println(dataPacket);
}

//---------------------------------------------------------------------------------------------------------------

void domePowerOn() // set the dome power SSR gate high
{
  digitalWriteFast(power_pin, HIGH);

  delay(2000); // gives time for the MA860H unit to power on and stabilise
}

//---------------------------------------------------------------------------------------------------------------

void domePowerOff() // set the dome power SSR gate low
{
  digitalWriteFast(power_pin, LOW);
}

void resetViaSWR()
{
  _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
}

void lightup()
{
  for (int i = 0; i < 10; i++)
  {
    digitalWriteFast(ledpin, HIGH);
    delay(500);
    digitalWriteFast(ledpin, LOW);
    delay(500);
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

  Azimuth = float(A_Counter) / (ticksPerDegree); // (ticks for one dome rev) / 360 (degrees) - about 29
                                                          // i.e number of ticks per degree

  // some error checking
  if (Azimuth < 1.0)
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
    digitalWriteFast(CameraPower, HIGH);  //
    cameraPowerState = on;
  }
  else
  {
    digitalWriteFast(CameraPower, LOW); //NB as above
    cameraPowerState = off;
  }
}

void interrupt() // Interrupt function
{

  char i, j;
  i = digitalReadFast(B_PHASE);
  j = digitalReadFast(A_PHASE);
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
  
  A_Counter = ticksperDomeRev / (360.0 / 261.0); // the position of due west - 261 (calculation checked)
                                                 // for the dome when the scope is at 270.
  
  homeSensor=true;                    // set this when the hall sesnor is detected. It indicates
                                      // the dome is at the home (261) degrees position when the homing process runs
  syncCount ++;
}
void ledToggle()
{
  
  if ( digitalReadFast(ledpin))
    {
      digitalWriteFast(ledpin, LOW);
    }
    else
    {
      digitalWriteFast(ledpin, HIGH);
    }
}