/*
	EPD_Driver.cpp
	
	--COPYRIGHT--
  * Brief The drivers and functions of development board
  * Copyright (c) 2012-2021 Pervasive Displays Inc. All rights reserved.
  *  Authors: Pervasive Displays Inc.
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *  1. Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *  2. Redistributions in binary form must reproduce the above copyright
  *     notice, this list of conditions and the following disclaimer in
  *     the documentation and/or other materials provided with the
  *     distribution.
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#if defined(ENERGIA)
#include "Energia.h"
#else
#include "Arduino.h"
#endif

#include "EPD_Driver.h"

typedef struct
{
    uint8_t FORMAT_REPEAT;
    uint8_t PHLINI_BSTSWa;
    uint8_t PHHINI_BSTSWb;
    uint8_t PHLVAR_DELAYa;
    uint8_t PHHVAR_DELAYb;
    uint8_t BSTSWa_RESERVE;
    uint8_t BSTSWb_RESERVE;
    uint8_t DELAY_RESERVE;
} COG_SOFT_START_DEF;

typedef struct
{
    uint8_t LAYOUTREV;
    uint8_t COG_TYPE;
    uint8_t VENDOR;
    uint8_t WAVEFORMREV;
    uint8_t FPLLOT[6];
    uint8_t COLOR;

    uint8_t TCON;
    uint8_t DRFW[4];
    uint8_t DCTL;
    uint8_t VCOM;
    uint8_t OSC_2nd;
    uint8_t RAM_RW[3];
    uint8_t DUW[6];
    uint8_t STV_DIR;
    uint8_t MS_SYNC;
    uint8_t BVSS;
    COG_SOFT_START_DEF SS[4];
} COG_OTP_USER_DATA;

COG_OTP_USER_DATA COG_userData;

// ---------- PUBLIC FUNCTIONS -----------

EPD_Driver::EPD_Driver(eScreen_EPD_t eScreen_EPD, pins_t board)
{
	spi_basic = board;
	
	// Type
	pdi_cp = (uint16_t) eScreen_EPD;
	pdi_size = (uint16_t) (eScreen_EPD >> 8);

	for ( uint8_t i = 0; i < sizeof( EPD_idx )/sizeof(* EPD_idx ); i++ )
	{
		if (pdi_size == EPD_idx[i])
		{
			_size = i;
			break;
		}
	}

	// Config registers according to screen size
	memcpy(register_data, register_data_mid, sizeof(register_data_mid));
}

void EPD_Driver::COG_getUserData()
{
    // uint8_t i = sNG;

    const uint8_t COG_initialData581[128] =
    {
        0x10, 0x63, 0x01, 0x01, 0x56, 0x45, 0x55, 0x30, 0x35, 0x34, 0x00, 0x25, 0x00, 0x1f, 0x50, 0xc8,
        0x09, 0x00, 0x00, 0x50, 0x10, 0x00, 0x1f, 0x50, 0x00, 0x1f, 0x03, 0x00, 0x00, 0x00, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x84, 0x50, 0x00, 0x00, 0x01, 0x1f, 0x9f, 0x81, 0x8a, 0x0a, 0x00, 0x00, 0x01, 0x1f, 0x9f, 0x81,
        0x88, 0x0a, 0x02, 0x00, 0x01, 0x7f, 0xff, 0x83, 0x88, 0x0a, 0x0a, 0xff, 0x00, 0x7f, 0xff, 0x81,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    };

    const uint8_t * COG_initialData;

	COG_initialData = COG_initialData581;

    if (COG_initialData[0] == 0x10)
    {
        COG_userData.LAYOUTREV = COG_initialData[0x00];
        COG_userData.COG_TYPE = COG_initialData[0x01];
        COG_userData.VENDOR = COG_initialData[0x02];
        COG_userData.WAVEFORMREV = COG_initialData[0x03];
        for (uint8_t i = 0; i < 6; i++)
        {
            COG_userData.FPLLOT[i] = COG_initialData[0x04 + i];
        }
        COG_userData.COLOR = COG_initialData[0x0a];

        COG_userData.TCON = COG_initialData[0x0b];
        for (uint8_t i = 0; i < 4; i++)
        {
            COG_userData.DRFW[i] = COG_initialData[0x0c + i];
        }

        COG_userData.DCTL = COG_initialData[0x10];

        COG_userData.VCOM = COG_initialData[0x11];
        for (uint8_t i = 0; i < 3; i++)
        {
            COG_userData.RAM_RW[i] = COG_initialData[0x12 + i];
        }
        for (uint8_t i = 0; i < 6; i++)
        {
            COG_userData.DUW[i] = COG_initialData[0x15 + i];
        }
        COG_userData.STV_DIR = COG_initialData[0x1b];
        COG_userData.MS_SYNC = COG_initialData[0x1c];
        COG_userData.BVSS = COG_initialData[0x1d];
        for (uint8_t i = 0; i < 4; i++)
        {
            COG_userData.SS[i].FORMAT_REPEAT = COG_initialData[0x28 + (8 * i)];
            COG_userData.SS[i].PHLINI_BSTSWa = COG_initialData[0x28 + (8 * i) + 1];
            COG_userData.SS[i].PHHINI_BSTSWb = COG_initialData[0x28 + (8 * i) + 2];
            COG_userData.SS[i].PHLVAR_DELAYa = COG_initialData[0x28 + (8 * i) + 3];
            COG_userData.SS[i].PHHVAR_DELAYb = COG_initialData[0x28 + (8 * i) + 4];
            COG_userData.SS[i].BSTSWa_RESERVE = COG_initialData[0x28 + (8 * i) + 5];
            COG_userData.SS[i].BSTSWb_RESERVE = COG_initialData[0x28 + (8 * i) + 6];
            COG_userData.SS[i].DELAY_RESERVE = COG_initialData[0x28 + (8 * i) + 7];
        }
        // i = sOK;
    }
    // return i;
}

// CoG initialization function
//		Implements Tcon (COG) power-on and temperature input to COG
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::COG_initial()
{	
	pinMode( spi_basic.panelBusy, INPUT );     //All Pins 0
	
	pinMode( spi_basic.panelDC, OUTPUT );
	digitalWrite(spi_basic.panelDC, HIGH);
	pinMode( spi_basic.panelReset, OUTPUT );
	digitalWrite(spi_basic.panelReset, HIGH);
	pinMode( spi_basic.panelCS, OUTPUT );
	digitalWrite(spi_basic.panelCS, HIGH);

	SPI.begin();

	#if defined(ENERGIA)
	{
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(SPI_CLOCK_DIV32);
		// SPI.setClockDivider(16000000 / min(16000000, 4000000));
		SPI.setBitOrder(MSBFIRST);
	}
	#else
	{
		SPISettings _settingScreen;
		_settingScreen = {8000000, MSBFIRST, SPI_MODE0};
		SPI.begin();
		SPI.beginTransaction(_settingScreen);
	}
	#endif
	
	_reset(5, 2, 4, 20, 5);
}

// CoG shutdown function
//		Shuts down the CoG and DC/DC circuit after all update functions
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::COG_powerOff()
{
	uint8_t register_turnOff[] = {0x7f, 0x7d, 0x00};
	_sendIndexData(0x09, register_turnOff, 3);
	delay(200);

	while ( digitalRead( spi_basic.panelBusy ) != HIGH );
	
	digitalWrite( spi_basic.panelDC, LOW );
	digitalWrite( spi_basic.panelCS, LOW );
	digitalWrite( spi_basic.panelBusy, LOW );
	delay( 150 );
	digitalWrite( spi_basic.panelReset, LOW );
}

// Global Update function
//		Implements global update functionality on either small/mid EPD
//		- INPUT:
//			- two image data (either BW and 0x00 or BW and BWR types)
void EPD_Driver::globalUpdate(const uint8_t * data1s, const uint8_t * data2s)
{
	
    COG_getUserData(); // get user data from initial data

	uint8_t iDCTL[2];
    iDCTL[0] = COG_userData.DCTL;
    iDCTL[1] = 0x00;
    _sendIndexData(0x01, &iDCTL[0], 2); //DCTL 0x10 of MTP
	
	// Send image data
	
    _sendIndexData(0x13, &COG_userData.DUW[0], 6);
    _sendIndexData(0x90, &COG_userData.DRFW[0], 4);
    
	_sendIndexData(0x12, &COG_userData.RAM_RW[0], 3);
	// send first frame
    _sendIndexData(0x10, data1s, image_data_size[_size]); // First frame
	
    _sendIndexData(0x12, &COG_userData.RAM_RW[0], 3);
	// send second frame
    _sendIndexData(0x11, data2s, image_data_size[_size]); // Second frame
	
	Serial.println("=== File uploaded ===");

    uint8_t data4[] = {0x7d};
    _sendIndexData(0x05, data4, 1);
    delay(50);
    uint8_t data5[] = {0x00};
    _sendIndexData(0x05, data5, 1);
    delay(1);
    _sendIndexData(0xD8, &COG_userData.MS_SYNC, 1);
    _sendIndexData(0xD6, &COG_userData.BVSS, 1);
    uint8_t data9[] = {0x10};
    _sendIndexData(0xa7, data9 , 1);
    delay(2);
    _sendIndexData(0xa7, data5, 1);
    delay(10);

	// Temperature sequence
    _sendIndexData(0x44, data5, 1);
    uint8_t data11[] = {0x80};
    _sendIndexData(0x45, data11, 1);
    _sendIndexData(0xa7, data9, 1);
    delay(2);
    _sendIndexData(0xa7, data5, 1);
    delay(10);
    uint8_t data12[] = {0x06};
    _sendIndexData(0x44, data12, 1);
	
    uint8_t data13[] = {0x41};
    _sendIndexData(0x45, data13, 1); // Temperature 0x82@25C
	//>> IF Fast, (0x45, FU_IDX)
    
	_sendIndexData(0xa7, data9, 1);
    delay(2);
    _sendIndexData(0xa7, data5, 1);
    delay(10);
    _sendIndexData(0x60, &COG_userData.TCON, 1);
    _sendIndexData(0x61, &COG_userData.STV_DIR, 1);
    _sendIndexData(0x02, &COG_userData.VCOM, 1);


	_DCDC_softStart();

	_displayRefresh();
	
	_DCDC_softShutdown();
	
	// digitalWrite(spi_basic.panelCS, HIGH);
}

// Global Update function
//		Implements global update functionality on either small/mid EPD
//		- INPUT:
//			- two image data (either BW and 0x00 or BW and BWR types)
void EPD_Driver::fastUpdate(const uint8_t * data1s, const uint8_t * data2s)
{
	COG_getUserData(); // get user data from initial data

	uint8_t iDCTL[2];
    iDCTL[0] = COG_userData.DCTL;
    iDCTL[1] = 0x00;
    _sendIndexData(0x01, &iDCTL[0], 2); //DCTL 0x10 of MTP
	
	// Send image data
	
    _sendIndexData(0x13, &COG_userData.DUW[0], 6);
    _sendIndexData(0x90, &COG_userData.DRFW[0], 4);
    
	_sendIndexData(0x12, &COG_userData.RAM_RW[0], 3);
	// send first frame
    _sendIndexData(0x10, data1s, image_data_size[_size]); // First frame
	
    _sendIndexData(0x12, &COG_userData.RAM_RW[0], 3);
	// send second frame
    _sendIndexData(0x11, data2s, image_data_size[_size]); // Second frame

	Serial.println("=== File uploaded ===");

    uint8_t data4[] = {0x7d};
    _sendIndexData(0x05, data4, 1);
    delay(50);
    uint8_t data5[] = {0x00};
    _sendIndexData(0x05, data5, 1);
    delay(1);
    _sendIndexData(0xD8, &COG_userData.MS_SYNC, 1);
    _sendIndexData(0xD6, &COG_userData.BVSS, 1);
    uint8_t data9[] = {0x10};
    _sendIndexData(0xa7, data9 , 1);
    delay(2);
    _sendIndexData(0xa7, data5, 1);
    delay(10);

	// Temperature sequence
    _sendIndexData(0x44, data5, 1);
    uint8_t data11[] = {0x80};
    _sendIndexData(0x45, data11, 1);
    _sendIndexData(0xa7, data9, 1);
    delay(2);
    _sendIndexData(0xa7, data5, 1);
    delay(10);
    uint8_t data12[] = {0x06};
    _sendIndexData(0x44, data12, 1);
	
    uint8_t data13[] = {0xc1};
    _sendIndexData(0x45, data13, 1); //>> IF Fast, (0x45, FU_IDX)
    
	_sendIndexData(0xa7, data9, 1);
    delay(2);
    _sendIndexData(0xa7, data5, 1);
    delay(10);
    _sendIndexData(0x60, &COG_userData.TCON, 1);
    _sendIndexData(0x61, &COG_userData.STV_DIR, 1);
    _sendIndexData(0x02, &COG_userData.VCOM, 1);


	_DCDC_softStart();

	_displayRefresh();
	
	Serial.println("=== FU-upload done ===");

	// digitalWrite(spi_basic.panelCS, HIGH);
}

// Fast Update function
//		Implements fast update functionality
//		- INPUT:
//			- array of image data to iterate on
//			- size of array
void EPD_Driver::fastUpdateSet(const unsigned char* fastImgSet[], uint8_t fastImgSize, uint8_t numLoops)
{
	uint8_t ii = 0;
	while (ii < numLoops)
	{
		for (uint8_t j = 0; j < fastImgSize -1; j++)
		{
			fastUpdate(fastImgSet[j+1], fastImgSet[j]);
		}
		ii++;
	}
	
	_DCDC_softShutdown();
}


// ---------- PROTECTED FUNCTIONS -----------

// SPI transfer function
//		Implements SPI transfer of index and data (consult user manual for EPD SPI process)
//		- INPUT:
//			- register address
//			- pointer to data char array
//			- length/size of data
void EPD_Driver::_sendIndexData( uint8_t index, const uint8_t *data, uint32_t len )
{
	uint32_t tempp = 50;
		
	digitalWrite( spi_basic.panelDC, LOW );      //DC Low
	digitalWrite( spi_basic.panelCS, LOW );      //CS Low
	delayMicroseconds(tempp);
	SPI.transfer(index);
	delayMicroseconds(tempp);
	digitalWrite( spi_basic.panelCS, HIGH );     //CS High
	digitalWrite( spi_basic.panelDC, HIGH );     //DC High
	digitalWrite( spi_basic.panelCS, LOW );      //CS Low
	delayMicroseconds(tempp);
	for ( uint32_t i = 0; i < len; i++ )
	{
		SPI.transfer(data[ i ]);
	}
	delayMicroseconds(tempp);
	digitalWrite( spi_basic.panelCS, HIGH );     //CS High
}

// EPD Screen refresh function
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::_displayRefresh()
{
	while (digitalRead(spi_basic.panelBusy) != HIGH)
	{
		delay(100);
	}
	uint8_t data18[] = {0x3c};
	_sendIndexData(0x15, data18, 1); //Display Refresh
	delay(5);
}

// CoG driver power-on hard reset
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::_reset(uint32_t ms1, uint32_t ms2, uint32_t ms3, uint32_t ms4, uint32_t ms5)
{
	// note: group delays into one array
	delay(ms1);
    digitalWrite(spi_basic.panelReset, HIGH); // RES# = 1
    delay(ms2);
    digitalWrite(spi_basic.panelReset, LOW);
    delay(ms3);
    digitalWrite(spi_basic.panelReset, HIGH);
    delay(ms4);
    digitalWrite(spi_basic.panelCS, HIGH); // CS# = 1
    delay(ms5);
}

// DC-DC power-on command
//		Implemented after image data are uploaded to CoG
//		Specific to small-sized EPDs only
//		- INPUT:
//			- none but requires global variables on SPI pinout and config register data
void EPD_Driver::_DCDC_powerOn()
{
	_sendIndexData( 0x04, &register_data[0], 1 );  //Power on
	while( digitalRead( spi_basic.panelBusy ) != HIGH );
}

// DC-DC soft-start command
//		Implemented after image data are uploaded to CoG
//		Specific to mid-sized EPDs only
void EPD_Driver::_DCDC_softStart()
{
    uint8_t iPH[2], iREP;
    for (uint8_t value = 0; value < 4; value++)
    {
        iPH[0] = COG_userData.SS[value].PHLINI_BSTSWa;
        iPH[1] = COG_userData.SS[value].PHHINI_BSTSWb;
        if (COG_userData.SS[value].FORMAT_REPEAT & 0x80) // format1
        {
            for (iREP = 0; iREP < (COG_userData.SS[value].FORMAT_REPEAT & 0x7f); iREP++)
            {
                _sendIndexData(0x09, &COG_userData.SS[value].BSTSWa_RESERVE, 1);
                iPH[0] += COG_userData.SS[value].PHLVAR_DELAYa;
                iPH[1] += COG_userData.SS[value].PHHVAR_DELAYb;
                _sendIndexData(0x51, iPH, 2);
                _sendIndexData(0x09, &COG_userData.SS[value].BSTSWb_RESERVE, 1);
                if (COG_userData.SS[value].DELAY_RESERVE & 0x80)
                {
                    delay(COG_userData.SS[value].DELAY_RESERVE & 0x7f); // ms
                }
                // else;
                delayMicroseconds(10 * COG_userData.SS[value].DELAY_RESERVE & 0x7f); // 10us
            }
        }
        else // format2
        {
            for (iREP = 0; iREP < (COG_userData.SS[value].FORMAT_REPEAT & 0x7f); iREP++)
            {
                _sendIndexData(0x09, &COG_userData.SS[value].PHLINI_BSTSWa, 1);
                if (COG_userData.SS[value].PHLVAR_DELAYa & 0x80)
                {
                    delay(COG_userData.SS[value].PHLVAR_DELAYa & 0x7f); // ms
                }
                // else;
                delayMicroseconds(10 * COG_userData.SS[value].PHLVAR_DELAYa & 0x7f); // 10us delaySysTick_10us

                _sendIndexData(0x09, &COG_userData.SS[value].PHHINI_BSTSWb, 1);
                if (COG_userData.SS[value].PHHVAR_DELAYb & 0x80)
                {
                    delay(COG_userData.SS[value].PHHVAR_DELAYb & 0x7f); // ms
                }
                // else;
                delayMicroseconds(10 * COG_userData.SS[value].PHHVAR_DELAYb & 0x7f); // 10us delaySysTick_10us
            }
        }
    }
}

// DC-DC soft-shutdown command
//		Implemented after image data are uploaded to CoG
//		Specific to mid-sized EPDs only
void EPD_Driver::_DCDC_softShutdown()
{
	// DC-DC off
    while (digitalRead(spi_basic.panelBusy) != HIGH)
    {
        delay(100);
    }
    uint8_t data19[] = {0x7b};
    _sendIndexData(0x09, data19, 1);
    uint8_t data20[] = {0x5d};
    _sendIndexData(0x05, data20, 1);
	uint8_t data555[] = {0x7a};
    _sendIndexData(0x09, data555, 1);
	delay(15);
	uint8_t data55[] = {0x00};
    _sendIndexData(0x09, data55, 1);
    
    while (digitalRead(spi_basic.panelBusy) != HIGH)
    {
        delay(100);
    }
    digitalWrite(spi_basic.panelDC, LOW);
    digitalWrite(spi_basic.panelCS, LOW);
    digitalWrite(spi_basic.panelReset, LOW);
    // digitalWrite(panelON_PIN, LOW); // PANEL_OFF# = 0

    digitalWrite(spi_basic.panelCS, HIGH); // CS# = 1
}
