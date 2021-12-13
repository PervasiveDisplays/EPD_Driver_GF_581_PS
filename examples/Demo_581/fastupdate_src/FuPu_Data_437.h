/*
	FuPu_Data.h
  User set of Image Data
*/

// 4.37"

#include "fp_data\437_1.c"
#include "fp_data\objects\0.c"
#include "fp_data\objects\1.c"
#include "fp_data\objects\2.c"
#include "fp_data\objects\3.c"
#include "fp_data\objects\4.c"
#include "fp_data\objects\5.c"
#include "fp_data\objects\6.c"
#include "fp_data\objects\7.c"
#include "fp_data\objects\8.c"
#include "fp_data\objects\9.c"
#include "fp_data\objects\w.c"

const unsigned char* fastImageSet[] = {
	(uint8_t *) & image_437_480x176_BW_0x00, (uint8_t *) & image_437_480x176_BW_mono,
	(uint8_t *) & image_437_480x176_BW_mono, (uint8_t *) & Img_437,
	(uint8_t *) & Img_437, (uint8_t *) & image_437_480x176_BW_mono,
	(uint8_t *) & image_437_480x176_BW_mono, (uint8_t *) & Img_437,
	(uint8_t *) & Img_437, (uint8_t *) & image_437_480x176_BW_0x00};

const unsigned char* partialImageSet[] = {
	(uint8_t *) & Img_w, (uint8_t *) & Img_0,
	(uint8_t *) & Img_0, (uint8_t *) & Img_1,
	(uint8_t *) & Img_1, (uint8_t *) & Img_2,
	(uint8_t *) & Img_2, (uint8_t *) & Img_3,
	(uint8_t *) & Img_3, (uint8_t *) & Img_4,
	(uint8_t *) & Img_4, (uint8_t *) & Img_5,
	(uint8_t *) & Img_5, (uint8_t *) & Img_6,
	(uint8_t *) & Img_6, (uint8_t *) & Img_7,
	(uint8_t *) & Img_7, (uint8_t *) & Img_8,
	(uint8_t *) & Img_8, (uint8_t *) & Img_9,
	(uint8_t *) & Img_9, (uint8_t *) & Img_w};

uint8_t fastImageSet_Size = sizeof(fastImageSet)/sizeof(*fastImageSet);
long partialWindowSize = 280;
uint8_t partialImageSet_config[5] = {sizeof(partialImageSet)/sizeof(*partialImageSet), 0x07, 0x0b, 0x32 ,0x6a};
