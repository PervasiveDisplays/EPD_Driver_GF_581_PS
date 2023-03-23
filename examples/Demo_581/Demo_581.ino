/*
  EPD_Driver_demo
  This is a demo sketch for the 5.8" small EPD from Pervasive Displays, Inc.
  The aim of this demo and the library used is to introduce CoG usage and global update functionality.
  
  Hardware Suggested:
  * Launchpad MSP432P401R or (Tiva-C) with TM4C123/Arduino M0 Pro/Raspberry Pi Pico
  * EPD Extension Kit (EXT2 or EXT3)
  * 5.81" EPD
  * 10-pin rainbow jumper cable
*/

#include <EPD_Driver.h>

// DEMO Image Set <Comment out when in User Mode>
#define SCREEN 581
#include "globalupdate_src/demoImageData.h"
#include "fastupdate_src/FuPu_Data_581.h"
//------------------------------------------------------------

#define EPD_SIZE eScreen_EPD_581
#define BOARD_TYPE boardRaspberryPiPico_RP2040_EXT3

void setup()
{
	EPD_Driver epdtest(EPD_SIZE, BOARD_TYPE);

	// Initialize CoG
	epdtest.COG_initial();
	epdtest.globalUpdate(BW_monoBuffer, BW_0x00Buffer);
	
	delay(1000);

	// Initialize CoG
	epdtest.COG_initial();
	epdtest.fastUpdateSet(fastImageSet, fastImageSet_Size, 1);
}

void loop()
{
  delay(1000);
}
