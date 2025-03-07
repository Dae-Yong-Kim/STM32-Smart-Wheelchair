
#ifndef __FONTS_H
#define __FONTS_H
   
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

//ASCII
typedef struct tFont
{    
  const uint8_t *table;
  uint16_t fsize;
  uint16_t w; //width
  uint16_t h; //height
  
}FONT;

extern FONT Font24;
extern FONT Font16;

#ifdef __cplusplus
}
#endif

#endif 
 
