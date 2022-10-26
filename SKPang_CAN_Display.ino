/*
  Board = Teensy 4.0
	Simple diagnostic display, taking data from the 3 port can bridge.
  
  Developed to integrate an Electric Drive Train into a 2021 Polaris General XP 200
  
  CAN_Vehicle Port connects to the Vehicle CAN Harness
  
	Requires the following libraries for Teensy 4.0:

	https://github.com/tonton81/FlexCAN_T4
  TFT_eSPI.h

  You need to configure both User_Setup.h and User_Setup_Select.h
  Uncomment the line in User_Setup_Select.h that includes User_Setup.h, then use it to config the display
  
*/

#include <FlexCAN_T4.h>
#include "canframe.h"
#include <EEPROM.h>
#include <SPI.h>

#include <TFT_eSPI.h> // Hardware-specific library

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library


FlexCAN_T4<CAN3, RX_SIZE_8, TX_SIZE_8> CAN_Vehicle;  //To the vehicle




const double PACK_CAPACITY = 10.9;  //Usable pack size in kWh
const uint32_t CAN_ID_TEENSY         = 0x18FE66FE; //Message from Teensy Bridge (Battery main switch PGN, repurposed)
const uint32_t CAN_ID_ABS_SPEED      = 0x18FEBF0B; //Byte [0] LSB, Byte [1] MSB - Speed from ABS Controller mph x 405
int nSpeed = 0; //Vehicle speed from ABS Controller in kph
int nSOC = 0; //SoC from Teensy 3 Port Bridge
int nSOH = 0; //SoH from Teensy 3 Port Bridge
byte nOrion_Flags = 0; //Orion BMS Flags
byte nPolaris_Flags = 0; //Polaris Stats BMS Flags
int nVoltage; //Pack Voltage
int nCurrent; //Pack Voltage
long lInterval = millis(); //Interval Timer  
int nScreenIndex = 0;  //Which screen are we looking at? 0,1,2
double dRange = 0;
long lMeanRange[200]; //Container for mean range values
int nMRPointer = 0;
int nLastSOC = 0; //SoC from Teensy 3 Port Bridge
int nLastSOH = 0; //SoH from Teensy 3 Port Bridge




//Useful for printing frame data
//Serial.print(" Interface: "); Serial.print(which_interface); Serial.print(" - ");
//Serial.print(" PID: "); Serial.print(frame.get_id(), HEX); Serial.print(" Data: "); 
//print_hex(frame.get_data()->bytes, 8);  








void vehicle_got_frame( const CAN_message_t &orig_frame) {
  CANFrame frame = orig_frame;


  uint32_t lPID = (frame.get_id() & 0x00FFFF00);
  //Serial.print("CAN ID="); Serial.println(frame.get_id(),HEX);
  if(lPID == (CAN_ID_TEENSY & 0x00FFFF00)) {
    nVoltage = (frame.buf[1]*256 + frame.buf[0]);  //Pack Voltage x 10
    //Serial.print(" | Voltage "); Serial.print(nVoltage);
    nCurrent = (frame.buf[3]*256 + frame.buf[2]); //Current x 10
    //Serial.print(" | Current "); Serial.println(nCurrent);
    nOrion_Flags = frame.buf[4]; //Orion Flags
    // [bit 0] Discharge Enable
    // [bit 1] Charge Enable
    // [bit 2] Charger Safety
    // [bit 3] BMS Errors Present
    // [bit 4] Multi-Purpose Input
    // [bit 5] AM Power Status
    // [bit 6] Ready Power Status
    // [bit 7] Charge Power Status
    nPolaris_Flags = frame.buf[5]; //Flags for current state of Teensy
    // [bit 0] Speed Limited
    // [bit 1] Torque Limited
    // [bit 2] Handbrake
    // [bit 3] Seatbelt
    // [bit 4] 
    // [bit 5] 
    // [bit 6] 
    // [bit 7] 
    nSOC = frame.buf[6]; //SoC%
    nSOH = frame.buf[7]; //Pack State of Health%
  }
  if(lPID == (CAN_ID_ABS_SPEED & 0x00FFFF00)) {
    nSpeed = (frame.buf[1]*256 + frame.buf[0]);      
  }
}




void setup()
{
	Serial.begin(115200);
	Serial.println("Polaris CAN Display");
  

	//Teensy FlexCAN_T4 setup
	CAN_Vehicle.begin();
	CAN_Vehicle.setBaudRate(250000);
	CAN_Vehicle.setMaxMB(32);
	CAN_Vehicle.enableFIFO();
	CAN_Vehicle.onReceive(vehicle_got_frame);


	CAN_Vehicle.enableFIFOInterrupt();
	CAN_Vehicle.mailboxStatus();
  
  for(int nCnt=0; nCnt<200; nCnt++) {
    lMeanRange[nCnt] = -1;
  }      


  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
}

void loop(){
//Update the display periodically
long lJoules = 0;
double dWHour = 0;
double dMPS = 0;
double dDistance = 0;
double dWHRemaining = 0;
double dRange = 0;
long lTime;
long lLastJoules = 0;

  if(millis() - lInterval > 500) {
    lTime = millis() - lInterval; //Time since last
    lInterval = millis(); //reset timer

    lJoules = (nVoltage * nCurrent * lTime) / 1000;
    if(lJoules != lLastJoules) {  //Don't calculate if we're in regen
      //Test Data
      //nVoltage = 106; nCurrent = 22; nSpeed = 10; nSOC = 90; nSOH = 100;
      //Calculate power used this interval in WS or Joules
      lLastJoules = lJoules;
      //A Watt hour is 3600J
      dWHour = double(lJoules) / 3600.0;
      if(dWHour == 0) dWHour = 1;
      dMPS = double(nSpeed) / 405.0 * 0.44704; 
      dDistance = dMPS * double(lTime) / 1000.0;
  
      dWHRemaining = PACK_CAPACITY * nSOC * nSOH /10; //Includes battery state of health
      dRange = dWHRemaining / dWHour * dDistance; //in Metres
      dRange = dRange / 1609.0; //Range in Miles
  
      double dMaxRange = 48 * (nSOC + 5) * nSOH /10000;
      if(dMPS < 3) {  //If vehicle isn't moving, range calcs as 0.  
        //Recalculate very simplistically based on 48m per full charge
        dRange = dMaxRange;
      }

      if((dRange>dMaxRange) && (dMaxRange>0)) dRange=dMaxRange;  //Range not likely to be more
      
      if((lMeanRange[1]==-1) && (dMaxRange!=0)) {
        //This is the first run, load the array with current value
        for(int nCnt=0; nCnt<200; nCnt++) {
          lMeanRange[nCnt] = long(dMaxRange);
        }      
      }
      if((dRange!=0) && (dRange!=dMaxRange)) {
        nMRPointer++;
        lMeanRange[nMRPointer] = long(dRange);
      }
/*    
      Serial.print("| Time "); Serial.print(lTime);
      Serial.print(" | lJoules "); Serial.print(lJoules);
      Serial.print(" | dWHour "); Serial.print(dWHour);
      Serial.print(" | Voltage "); Serial.print(nVoltage);
      Serial.print(" | Current "); Serial.print(nCurrent);
      Serial.print(" | dMPS "); Serial.print(dMPS);
      Serial.print(" | dDistance "); Serial.print(dDistance);
      Serial.print(" | dWHRemaining "); Serial.print(dWHRemaining);
      Serial.print(" | nSOC "); Serial.print(nSOC);
      Serial.print(" | dRange "); Serial.println(dRange);
*/
    }
    
    if(nMRPointer>199) nMRPointer=0;

    //Calculate average over the last 200 seconds
    long lAvRange = 0;
    for(int nCnt=0; nCnt<200; nCnt++) {
      lAvRange = lAvRange + lMeanRange[nCnt];
    }
    lAvRange = lAvRange / 200;
    
    DisplayResults(lAvRange); //Average
    //DisplayResults(lMeanRange[nMRPointer]); //Instantaneous
    
  }


}

void DisplayResults(double dDRange) {
//Show data on M5Stack Display
int nHOffset = 105;
int nHeight = (320-nHOffset) / 16;
static double dLastDRange = 0;

  //Range
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(30, 2, 2); //X,Y,Font  Font 7 is a Sever Segment type
  tft.println("Range (Miles)");

  if(dDRange!=-1) {  //Only display if we have some values
    if(dDRange!=dLastDRange) {
      tft.setTextColor(TFT_BLACK,TFT_BLACK);
      tft.setCursor(30, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type
      tft.print(int(dDRange));
      dLastDRange = dDRange;
      tft.setTextColor(TFT_GREEN,TFT_BLACK);
      tft.setCursor(30, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type
      tft.print(int(dDRange));
    }
  }
  //SOC
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(210, 2, 2); //X,Y,Font  Font 7 is a Sever Segment type
  tft.print("SoC %");
  if(nSOC!=nLastSOC) {
    tft.setTextColor(TFT_BLACK,TFT_BLACK);
    tft.setCursor(210, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type
    tft.print(nLastSOC);
    nLastSOC = nSOC;
    tft.setTextColor(TFT_YELLOW,TFT_BLACK);
    tft.setCursor(210, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type
    tft.print(nSOC);
  }
  //SOH
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(380, 2, 2); //X,Y,Font  Font 7 is a Sever Segment type
  tft.println("SoH %");

  if(nSOH!=nLastSOH) {
    tft.setTextColor(TFT_BLACK,TFT_BLACK);
    if(nLastSOH<100) {
      tft.setCursor(380, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type
    } else {
      tft.setCursor(360, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type    
    }
    tft.println(nLastSOH);
    nLastSOH = nSOH;
    tft.setTextColor(TFT_YELLOW,TFT_BLACK);
    if(nSOH<100) {
      tft.setCursor(380, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type
    } else {
      tft.setCursor(360, 30, 7); //X,Y,Font  Font 7 is a Sever Segment type    
    }
    tft.println(nSOH);
  }
  //ORION LEDs
    // [bit 0] Discharge Enable
    // [bit 1] Charge Enable
    // [bit 2] Charger Safety
    // [bit 3] BMS Errors Present
    // [bit 4] Multi-Purpose Input
    // [bit 5] AM Power Status
    // [bit 6] Ready Power Status
    // [bit 7] Charge Power Status
  //Show 8 LEDs equally spaced
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.setCursor(30, nHOffset+nHeight * 1, 2);tft.println("OK to Discharge");
  tft.setCursor(30, nHOffset+nHeight * 3, 2);tft.println("OK to Charge");
  tft.setCursor(30, nHOffset+nHeight * 5, 2);tft.println("230v Charger Enabled");
  tft.setCursor(30, nHOffset+nHeight * 7, 2);tft.println("BMS Errors Present");
  tft.setCursor(30, nHOffset+nHeight * 9, 2);tft.println("J1772 (Charger) Pilot");
  tft.setCursor(30, nHOffset+nHeight * 11, 2);tft.println("12V From Battery");
  tft.setCursor(30, nHOffset+nHeight * 13, 2);tft.println("12V Ignition Power");
  tft.setCursor(30, nHOffset+nHeight * 15, 2);tft.println("Charger Powered Up");
  
  DisplayLED(15, nHOffset+nHeight * 1,  ((nOrion_Flags & 0B00000001)==0B00000001));
  DisplayLED(15, nHOffset+nHeight * 3,  ((nOrion_Flags & 0B00000010)==0B00000010)); 
  DisplayLED(15, nHOffset+nHeight * 5,  ((nOrion_Flags & 0B00000100)==0B00000100));
  DisplayLED(15, nHOffset+nHeight * 7,  ((nOrion_Flags & 0B00001000)==0B00001000));
  DisplayLED(15, nHOffset+nHeight * 9,  ((nOrion_Flags & 0B00010000)==0B00010000));
  DisplayLED(15, nHOffset+nHeight * 11, ((nOrion_Flags & 0B00100000)==0B00100000));
  DisplayLED(15, nHOffset+nHeight * 13, ((nOrion_Flags & 0B01000000)==0B01000000));
  DisplayLED(15, nHOffset+nHeight * 15, ((nOrion_Flags & 0B10000000)==0B10000000));

  tft.setCursor(275, nHOffset+nHeight * 1, 2);tft.println("Speed limited to 10mph");
  tft.setCursor(275, nHOffset+nHeight * 5, 2);tft.println("Torque limited to 50%");
  tft.setCursor(275, nHOffset+nHeight * 9, 2);tft.println("Handbrake Applied");
  tft.setCursor(275, nHOffset+nHeight * 13, 2);tft.println("Seatbelt Secured");

  //Show 4 LEDs equally spaced
  DisplayLED(250, nHOffset+nHeight * 1, ((nPolaris_Flags & 0B10000000)==0B10000000));
  DisplayLED(250, nHOffset+nHeight * 5, ((nPolaris_Flags & 0B01000000)==0B01000000));
  DisplayLED(250, nHOffset+nHeight * 9, ((nPolaris_Flags & 0B00100000)==0B00100000));
  DisplayLED(250, nHOffset+nHeight * 13, ((nPolaris_Flags & 0B00010000)==0B00010000));
   
  
}


void DisplayLED(int x, int y,bool bON) {
//Display an LED centred on X,Y
int nYOffset = y + 8;
#define TFT_GREY 0x7BEF

  if(bON) {
    tft.fillCircle(x, nYOffset, 8, TFT_RED);
    tft.drawCircle(x, nYOffset, 8, TFT_GREY);
  } else {
    tft.fillCircle(x, nYOffset, 8, TFT_BLACK);
    tft.drawCircle(x, nYOffset, 8, TFT_RED);
  }  
  
  
}
