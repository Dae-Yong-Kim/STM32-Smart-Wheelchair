#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usart.h"
#include "lcd_driver.h"
#include "lcd_showbmp.h"
#include "fatfs_storage.h"

char* pDirectoryFiles[MAX_BMP_FILES];
FATFS SDFatFs;
uint8_t str[20];

void SD_Init(void)
{
	uint8_t file_cnt = 0;
  if(f_mount(&SDFatFs, (TCHAR const*)"/", 0) != FR_OK)
	{
		  printf("fatfs not mounted\r\n");
  }
	else
  {
			/* Initialize the Directory Files pointers (heap) */
		  printf("fatfs mount success!\r\n");
			for (file_cnt = 0; file_cnt < MAX_BMP_FILES; file_cnt++)
			{
				  pDirectoryFiles[file_cnt] = malloc(13); 
			}
  }
}

void display_images(void)
{
	  uint32_t bmplen = 0x00;
    uint32_t checkstatus = 0x00;
    uint32_t filesnumbers = 0x00;
    uint32_t bmpcounter = 0x00;
    DIR directory;
    FRESULT res;
	/* Open directory */
    res= f_opendir(&directory, "/");
    if((res != FR_OK))
    {
      if(res == FR_NO_FILESYSTEM)
      {
        /* Display message: SD card not FAT formated */
          printf("SD Card not formatted\r\n");				
          
      }
      else
      {
        /* Display message: Fail to open directory */
          printf("SD Card open fail\r\n");				
      }
    }
    
    /* Get number of bitmap files */
    filesnumbers = Storage_GetDirectoryBitmapFiles ("/", pDirectoryFiles);    
		printf("filesnumbers %d\r\n",filesnumbers);
    /* Set bitmap counter to display first image */
    bmpcounter = 1; 
    
    while (1)
    {     
				sprintf((char*)str, "%-11.11s", pDirectoryFiles[bmpcounter -1]);
        
        checkstatus = Storage_CheckBitmapFile((const char*)str, &bmplen);
        
        if(checkstatus == 0)
        {
           /* Format the string */
           Storage_OpenReadFile(0, 0, (const char*)str); 
        }
        else if (checkstatus == 1)
        {
          /* Display message: SD card does not exist */
					  printf("SD Card not found\r\n");
        }
        else
        {
          /* Display message: File not supported */
					  printf("SD Card file not supported\r\n");
        }
        
        bmpcounter ++;
        if(bmpcounter > filesnumbers)
        {
          bmpcounter = 1;
					break;
        }
				HAL_Delay(1000);
    }
}




