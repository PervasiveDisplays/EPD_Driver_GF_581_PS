/*
	FuPu_Data.h
  User set of Image Data
*/

// 5.81"

#include "fp_data\images581.h"

const unsigned char* fastImageSet[] = {
	BW_monoBuffer,
	(uint8_t *) & image_581_720x256_BW_product,
	BW_monoBuffer,
	(uint8_t *) & image_581_720x256_BW_product,
	(uint8_t *) & image_581_720x256_BW_0xff, BW_0x00Buffer
};

uint8_t fastImageSet_Size = sizeof(fastImageSet)/sizeof(*fastImageSet);