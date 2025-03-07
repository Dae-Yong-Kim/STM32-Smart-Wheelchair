#ifndef __LCD_SHOWBMP_H
#define __LCD_SHOWBMP_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"
#include "fatfs.h"

#define MAX_BMP_FILES  20
	
	
void SD_Init(void);
void display_images(void);
	
#ifdef __cplusplus
}
#endif

#endif /* __LCD_SHOWBMP_H */



