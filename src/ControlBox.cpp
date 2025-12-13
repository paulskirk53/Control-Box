/*

December 2025 some important TODOS added along with some notes - important changes are required :
e.g. 1 - to find home
     2 - to save the home position in EEPROM from a request sent by the monitor program

comment on review of this code 4-3-2025
It looks like this code is ready to test - need to do the calculation of steps per degree of dome movement as per todo below, then try it out.
Make a completely separate control box so that in the event it fails the working control box can be reinstated. They should be pin compatible.

Problem todo - the code sends the motor to the target position, but no check is made of the Azimuth value. Perhaps a check of azimuth, then move would be best.
this could be iterated.

Next steps: 
1 - (done) put together a data packet for transmission to the monitor on receipt of a request. The packet is updated once per sec in the monitortimerinterval() routine - done
    the packet is assembled using global vars which are uodated as any of the data items below are changed 
    the list below checked against the current master branch code December 2025 and accurately reflects the data message required by
    the monitor program as of 13-12-25

    The current Azimuth       - CurrentAzimuth
    The target Az.            - TargetAzimuth
    Moving to target?         - movementstate 
    which direction to rotate - QueryDir 
    Target status             - TargetMessage 
    degrees to target         - CDArray[CurrentAzimuth] 
    camera power              - cameraPowerState 
    a heartbeat counter       - syncCount

2 - look through the code to identify receipts and transmissions related to the two datastreams - ASCOM and MONITOR

Note ========== 8-9-23 ======= Note Note Note Note Note

This project is the Code for the new (1-5-23) control box (stepper and encoder functions) on one AVR4809 chip
and this branch looks to use degrees as the moveto method rather than steps 

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
//TODO SET THE STEPPER CURRENT POS TO ZERO BEFORE SETTING THE TARGET FOR QUERYDIR

#include <Arduino.h>
#include <avr/cpufunc.h> /* Required header file for wdt resets*/
#include <AccelStepper.h>
#include "linkedList.h"


// Forward declarations

void Emergency_Stop(int azimuth, String mess);
String WhichDirection();

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
void heartBeat();

// end declarations
// defines for the encoder inclusion
#define power_pin 2   
#define A_PHASE 4      // USES PINS 4 AND 5 for encoder interrupt - check that these pins will work as interrupts
#define B_PHASE 5
#define CameraPower 6  // power for the imaging camera
#define dirPin 10      // connection for motor direction signal
#define stepPin 11     // connection for motor step signal


#define WestPin 29     // sync connection for dome
//
#define off false
#define on true

// meaningful names for the serial ports
#define Monitor Serial2
#define ASCOM Serial

#define ledpin 23  

// Define a stepper and the pins it will use

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, true);

String receivedData;

boolean Slewing; // controls whether the stepper is stepped in the main loop
boolean homing = false;
boolean homeSensor = false;      // this is set true if Westsync interrupt fires - the home sensor
float StepsPerSecond; // used in stepper.setMaxSpeed - 50 the controller (MAH860) IS SET TO step size 0.25
                      //NB as of summer '23 the controller is the TB6600



float normalAcceleration;        // was incorrectly set to data type int

int stepsToTarget = 0;           // for monitor program


int savedAzimuth = 0;             //int is 16 bit 
long monitorTimerInterval = 0.0l; // note l after 0.0 denotes long number - same type as millis()
long azimuthTimerInterval = 0.0l;
long LedTimerInterval     = 0.0l;
long stepsPerDegree       =10;   //todo find the real value and update this statement
// e.g. process for defining the above:
/*
use arduino sermon for control and comms
start at say 270 degrees
set stepsPerDegree = some value x (try 400) and then upload the code.
SA90# from a known position say 270 and end up at position y - this will give a value of steps used to move the dome z = y-270 degrees
calculate stepsPerDegree as x/ z  and ASCOM.print it
Now set this value in the code (line 123 in this file) and try a slew from 270 to 90 (180 degrees) to see how accurate the figure is. It might need adjusting?
*/
String TargetMessage = "No Target";
String QueryDir = "No Direction";
String movementstate = "Not Moving";
String pkversion = "7.0";
String dataPacket = "";



volatile long A_Counter;        // volatile because it's used in the interrupt routine
volatile int syncCount = 0;     // counts the number of syncs and acts as an indicator on the monitor program
float Azimuth;                  // The data type is important to avoid integer arithmetic in the encoder() routine
uint16_t integerAzimuth;        // this is what is returned from the encoder routine
                                // and also because we really don't need fractional degrees for dome movement.
float ticksperDomeRev = 20700;  // 12-9-25 (added here 13-12-25): The encoder is now attached to the base of the stepper motor shaft. 
                                // The shaft rotates 34.5 turns for one dome rotation
                                // so the ticks per dome rev are 34.5 x 600 = 20,700

float ticksPerDegree  = ticksperDomeRev / 360.0;  // do the calculation once here
bool cameraPowerState = off;


/*
  --------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------
*/

void setup()
{
  //todo Dec' 2025 add in the code which writes to eeprom from the master branch

//Pinmodes for the stepper code
  pinMode(power_pin, OUTPUT);
  digitalWriteFast(power_pin, LOW); // initialise the pin state so that the mosfet gate is Low and therefore power to the MA860H is off
  pinMode(9, INPUT_PULLUP);     // see the notes in github. this pulls up the serial Rx pin to 5v.
  pinMode(ledpin, OUTPUT);


// Pin modes for the west sync and camera power
  pinMode(WestPin, INPUT_PULLUP);
  pinMode(CameraPower, OUTPUT);

  //turn the camera power of at startup:
  digitalWriteFast (CameraPower, LOW);           //  LOW is camera power OFF

 // Pin modes for the encoder:
  pinMode(A_PHASE, INPUT_PULLUP);
  pinMode(B_PHASE, INPUT_PULLUP);



  // pins 2,3,18,19,20,21 are the only pins available to use with interrupts on the mega2560 
  // no interrupt pin limit restrictions on 4809)
  attachInterrupt(digitalPinToInterrupt(A_PHASE), interrupt, RISING); // interrupt for the encoder device

  // interupts for the west azimuth sync below
  
  attachInterrupt(digitalPinToInterrupt(WestPin), WestSync, RISING);

  A_Counter = ticksperDomeRev / (360.0 / 261.0); // the position of due west - 261 (calculation checked)
                                                 // for the dome when the scope is at 270.

  PowerForCamera(off); // camera power is off by default



  stepper.stop(); // set initial stepper motor state as stopped

  // Change below to suit the stepper

  Slewing = false;
  StepsPerSecond = 300.0;              // changed following empirical testing Oct 2020
  normalAcceleration = 140.0;          // changed following empirical testing October 17th 2020 - changed from 40 to 20 for trial
  stepper.setMaxSpeed(StepsPerSecond); // steps per second see below -
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(normalAcceleration); // steps per second per second.
  
  // initialise

  CurrentAzimuth       = 0;
 
  monitorTimerInterval = millis();
  azimuthTimerInterval = millis();
  LedTimerInterval     = millis();

  homeSensor = false;          // this uses the west sync hardware device

  ASCOM.begin(19200);   // start serial ports ASCOM driver - usb with PC - rx0 tx0 and updi
  
  Monitor.begin(19200); // serial with the Monitor program

  lightup(); // 10 SECOND DELAY flash Led to indicate reset when the box lid is off for testing
             // ALLOWS setup time for serial comms

  // ASCOM.println(" before get azimuth");       // debug only



  TargetAzimuth = getCurrentAzimuth(); // set the target az equal to the current az, so that no mevement takes place
                                       // at power up

  initialiseCDArray();    // used by the monitor program to provide a countdown to target during slew operations

  
} // end setup

/*
  \\\\\\\\\\\\\\\\ ///////////////////////// End Setup() \\\\\\\\\ /////////////// \\\\\\\\\\\\\\\\\\\\\\\\\ /////////////////////
  //////////////// \\\\\\\\\\\\\\\\\\\\\\\\\             ///////// \\\\\\\\\\\\\\\ ///////////////////////// \\\\\\\\\\\\\\\\\\\\\
*/

void loop()
{

  // put your main code here, to run repeatedly, perhaps for eternity if the power holds up....How long is eternity?
  // also there are other considerations such as corrosion of connectors and joints over a period such as eternity...

  if (Monitor.available() > 0)    // monitor is the serial data stream used for comms to the monitor program
  {
    String monitorReceipt = Monitor.readStringUntil('#');

    if (monitorReceipt.indexOf("dataRequest", 0) > -1)   // request for data packet with all the monitoring data
    {
      
      Monitor.print(dataPacket);   // send the requested datapacket to the monitor program
     
    }

    //*************************************************************************
    //******** code for Monitor MCU Identity process below **********************
    //**** Used by the ASCOM driver to identify the COM port in use. **********
    //*************************************************************************
    //*************************************************************************

    if (monitorReceipt.indexOf("monitorcontrol", 0) > -1)   // MCU id request - the monitor program uses this exchange
    {                                                       // to identify which windows com port is used by this MCU
      Monitor.print("monitorcontrol#");
    }
    
    //*************************************************************************
    //********      code for MCU reset process below          *****************
    //*************************************************************************
    //*************************************************************************

    if (monitorReceipt.indexOf("reset", 0) > -1)     // reset the control box MCU
    {
      Monitor.print("resetting");
      // ASCOM.print("get this");
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

  } // endif Monitor.available

    //*************************************************************************
    //************     code for ASCOM functions below    **********************
    //**** Used by the ASCOM driver to facilitate  dome functions    **********
    //*************************************************************************
    //*************************************************************************



  if (ASCOM.available() > 0) // when serial data arrives from the ASCOM driver capture it into a string
  {

    receivedData = ASCOM.readStringUntil('#'); // read a string from PC serial port usb
    
    //*************************************************************************
    //********      code for ASCOM Azimuth FUNCTION      **********************
    //*************************************************************************
    //*************************************************************************

    if (receivedData.indexOf("AZ", 0) > -1)
    {

      String x = (String) getCurrentAzimuth();   // set x to the current dome azimuth
      x += "#";                                  // add a # to comply with our comms protocol
      ASCOM.print(x);                            // send the azimuth to the ASCOM dome driver
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

    if (receivedData.indexOf("controlbox", 0) > -1 || receivedData.indexOf("identify", 0) > -1)   //  used for the ASCOM connection
    {
      ASCOM.print("controlbox#");
    }

    //*************************************************************************
    //********       code for emergency stop process below      ***************
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
      // strip off 1st 2 chars - SA
      receivedData.remove(0, 2);

      TargetAzimuth = receivedData.toInt(); // store the target azimuth for comparison with current position
      // the way the code works is to treat a request for az = 360 as az =0 , hence the if clause below
      if (TargetAzimuth == 360)
      {
        TargetAzimuth = 0;
      }
      bool AzOK = checkForValidAzimuth();   // is the az between 0 and 360
      if (AzOK)
      {
        long targetPosition = TargetAzimuth * stepsPerDegree;  // this gives the number of steps the motor has to move
                                                               // in order to reach the target
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
            stepper.moveTo(targetPosition); // positive number means clockwise in accelstepper library. 
                                  
          }

          if (QueryDir == "anticlockwise")
          {
            stepper.setCurrentPosition(0);
            stepper.moveTo(-targetPosition); // negative is anticlockwise in accelstepper library
          }

        }
        receivedData = "";

      } // end if azok

      // set messages for the data packet to be sent to the monitor program      
      movementstate = "Moving"; // for updating the datapacket
      TargetMessage = "Awaiting Target ";

    } // end if Slew to Azimuth

    //**********************************************************
    //******** code for SL process below ***********************
    //******** data sent by driver SL#   ***********************
    //**********************************************************
    //

    if (receivedData.indexOf("SL", 0) > -1) //
    {

      if (Slewing | homing)
      {
        ASCOM.print("Moving#");   // sent to ASCOM serial and picked up by the ASCOM driver
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
       //todo we know the number of motor shaft steps per degree of dome movement, so calculate the number of steps for a 360 degree rotation from that
       //todo but if westsync fires & homesensor therefore becomes true, stop the rotation
       stepper.moveTo(150000000);              // todo need to rotate until homesenor becomes true - watch out thogh if it doesn't - eternal rotation.
                                               // 

       domePowerOn(); 
       homing = true;                          // used in loop() to control motor movement

       // send to monitor - started homing
      TargetMessage  = "Started Homing ";
      QueryDir       = "clockwise";                  //set the direction of movement - this is also sent to the monitor program
      movementstate  = "Homing...";

      receivedData = "";
      homeSensor=false;   // the homesensor is set every time the dome moves past the trigger point so it may be set outside of find home, by virtue
                          // of normal dome movement, so it must be set false here to be valid for find home to be the sole trigger.

    }

  } // end if ASCOM Available

  // so from here down is code to deliver SA function

  // start grouping for Slewing functions here
  if (Slewing) // if the slew status is true, run the stepper and update the data in the monitor program
  {
    
    stepper.run();

    check_If_SlewingTargetAchieved();   //checks if slew is ended and updates monitor program

  }  // endif Slewing

  if (homing)
  {
    if ((millis() - azimuthTimerInterval) > 500.0) // one half second checks for HOMESENSOR STATE as the dome moves
    {
      getCurrentAzimuth();                      
      azimuthTimerInterval = millis();
      //ASCOM.print("VALUE OF HOMESENSOR IS true if activated ");
      //ASCOM.println(homeSensor);
    }

    if (homeSensor == true)                     // true indicates the sensor at the home position has been activated
    {
      movementstate = "Not Moving";
      QueryDir      = "None";
      TargetMessage = "Homing Complete";
      homing        = false;
      homeSensor    = false;      // homing is finished, so set the sensor to false. 
      //todo  1 - the azimuth is now the home value (whatever that is) so set it here
      // todo 2 - we need a facility to write the home position to EEPROM - perhaps do this from the monitor program, so that it removes the hardcoding
      // e.g. 261 used for 'west'
      //todo  3 - probably need to set targetazimuth = getcurrent azimuth here so once home is found, the code won't start a move because of a difference between target and current
      domePowerOff();
     }


  }  // endif homing

    // create / update the data packet for monitoring program
    //
    if ((millis() - monitorTimerInterval) > 1000.0) // one second checks for azimuth value as the dome moves and tick the heartbeat LED
    {
      
      createDataPacket();
      monitorTimerInterval = millis();

    }

    if ((millis() - LedTimerInterval) > 20000.0) // twenty second timer for a led flash - an indicator that code is running
    {
      
      
      heartBeat();
      LedTimerInterval = millis();
    }


  stepper.run();   // stepper run - works for slewing and for findHome



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



int getCurrentAzimuth()
{
   encoder();
  return integerAzimuth;

} // end getCurrentAzimuth()

void check_If_SlewingTargetAchieved()
{

    if (stepper.distanceToGo() == 0 )  // todo probably need to check azimuth here e.g. add to conition && azimuth within a range of say three degrees of target
    // if the slew is out by x degrees, nudge in appropriate direction by the number of steps corresponding to x degrees.
    // however, if the numbers of steps per degree is accurate and the drive does not stall thereby missing steps, the target azimuth should be good. :)
    {
      
      Slewing       = false;          // used to stop the motor in main loop
      movementstate = "Stopped.  ";   // Set the variables for the data packet
      TargetMessage = "Target achieved ";
      QueryDir      = "None";

      domePowerOff(); // power off the stepper now that the target is reached.
    }
    else
    {
      
      stepper.run();
    }


}

void createDataPacket()
{
  
  CurrentAzimuth = getCurrentAzimuth(); 
      
  //eight items below
  
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
  if ( (TargetAzimuth >= 0) && (TargetAzimuth <=359) )  // todo suggest that 360 is a valid azimuth and may be requested by the ASCOM driver, so 359 needs change to 360
  // not sure of any wider impact though so haven't changed at time of this comment 4-3-2025
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
    digitalWrite(CameraPower, HIGH);  //
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

  homeSensor=true;                               // set this when the hall sensor is detected.
                                                 // It indicates the dome is at the home (261) degrees
                                                 // position when the homing process runs
  syncCount ++;                                  // used to indicate a sync event for the monitor program
}
void heartBeat()
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