/*
  EPD_Driver_demo
  This is a demo sketch for the 2.71" small EPD from Pervasive Displays, Inc.
  The aim of this demo and the library used is to introduce CoG usage and global update functionality.
  
  Hardware Suggested:
  * Launchpad MSP432P401R or (Tiva-C) with TM4C123/Arduino M0 Pro/Raspberry Pi Pico
  * EPD Extension Kit (EXT2 or EXT3)
  * 2.71" EPD
  * 20-pin rainbow jumper cable
*/

#include <EPD_Driver.h>

// DEMO Image Set <Comment out when in User Mode>
// Screen Size: 581, 741
#define SCREEN 581
#include "globalupdate_src/demoImageData.h"
// User Mode Image Set
//#include "globalupdate_src/userImageData.h"
//------------------------------------------------------------

// Uncomment to DEMO Fast Update features
// #include "fastupdate_src/FuPu_LUT_581.h"
// #include "fastupdate_src/FuPu_Data_581.h"

void setup()
{
  EPD_Driver epdtest(eScreen_EPD_581, boardLaunchPad_EXT3);
  // EPD_Driver epdtest(eScreen_EPD_581, boardRaspberryPiPico_RP2040_EXT3);
  
  // Initialize CoG
  epdtest.COG_initial();

  // Global Update Call
  epdtest.globalUpdate(BW_monoBuffer, BW_0x00Buffer);

  // // Load LUT file before Fast/Partial Update
  // epdtest.updateLUT(&ltb_custom);  

  // // Fast Update Call with 2 iterations
  // delay(1000);
  // epdtest.fastUpdate(fastImageSet, fastImageSet_Size, 1);

  // // Partial Update Call with 3 iterations
  // delay(1000);
  // epdtest.partialUpdate(partialImageSet, partialImageSet_config, partialWindowSize, 1);

  // Turn off CoG
  epdtest.COG_powerOff();
}

void loop()
{
  delay(1000);
}
