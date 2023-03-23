# Introduction
 *  This library code implements the driving code of [Aurora(V231) EPD with iTC driver and embedded OTP LUT](https://www.pervasivedisplays.com/products/) made by [Pervasive Displays Inc (PDi)](https://www.pervasivedisplays.com/). This is compatible with [EPD Extension Kit Gen 3 (EXT3)](https://www.pervasivedisplays.com/product/epd-extension-kit-gen-3-ext3/).
 *  It implements the global (GU) and fast update modes (FU).
 *  The library supports the following EPD sizes: 5.81"
 *  This library was tested with Launchpad (Tiva-C) with TM4C123, MSP432P401R, Arduino M0 Pro, and Raspberry Pi Pico.

# Installation
  Download the ZIP file of this library and install as instructed in [Arduino](https://www.arduino.cc/en/guide/libraries) or Energia.

# Hardware Setup
 *  If using Launchpad (Tiva-C) with TM4C123 or MSP432P401R, the SPI pins are by default:
``` c++
// Using EXT3
    SCK = 7     // BROWN
    BUSY = 11   // RED
    D/C = 12    // ORANGE
    RST = 13    // YELLOW
    MOSI = 15   // BLUE
    ECSM = 19   // GRAY
    
 // Using EXT2
    SCK = 7    
    BUSY = 8  
    D/C = 9
    RST = 10
    MOSI = 15
    ECSM = 19
    PNLON = 11
    BS = 17
 ```
 *  If using Arduino M0 Pro,
  ```
    SCK = 13 
    BUSY = 4  
    D/C = 5
    RST = 6
    MOSI = 12
    ECSM = 8
 ```
  *  If using Raspberry Pi Pico,
  ```
    SCK = 18 
    BUSY = 13  
    D/C = 12
    RST = 11
    MOSI = 19
    ECSM = 17
 ```
For further wiring guidance, check out the User Manual [EXT3](https://docs.pervasivedisplays.com/epd-usage/development-kits/ext3-1).

# EPD_Driver Demo
 *  The sample code implements both global and fast update functionality for a [5.81" Monochrome+Fast EPD (E2581PS0B1)](https://www.pervasivedisplays.com/product/5-8-e-ink-display-aurora-mb-v231/).  
 
 First, a BW(black and white) image is flashed.  
  Then, after a two-second delay, a sequence of BW fast images are flashed.  
  
 *  The demo currently includes sample images for all the supported EPD sizes mentioned above. To implement the demo on these other sizes, update the following lines
``` c++
//------------------------------------------------------------
// DEMO Image Set <Comment out when in User Mode>
// Screen Size: 581
#define SCREEN 581
#include "globalupdate_src/demoImageData.h"
#include "fastupdate_src/FuPu_Data_581.h"
```
 *  Then select the appropriate EPD size, development board and extension kit:
``` c++
#define EPD_SIZE eScreen_EPD_581
#define BOARD_TYPE boardRaspberryPiPico_RP2040_EXT3
```

#  Tech Support
 *  For more information, please visit [Pervasive Displays, Inc. (龍亭新技股份有限公司)](https://www.pervasivedisplays.com/)
 *  [PDI Wiki](https://docs.pervasivedisplays.com/)
 *  [PDi Product Line up](https://www.pervasivedisplays.com/products/)
 *  [EPD Extension Kit Gen 2 (EXT2)](https://www.pervasivedisplays.com/product/epd-extension-kit-gen-2-ext2/)
 *  [EPD Extension Kit Gen 3 (EXT3)](https://www.pervasivedisplays.com/product/epd-extension-kit-gen-3-ext3/)
 *  [24/7 Support Service](https://www.pervasivedisplays.com/technical-support/)

