
/*
  ******************************************************************************
  * @file           : lcd1602.c
  * @brief          : 1602 LCD device control
  ******************************************************************************
*/
#include "main.h"

I2C_HandleTypeDef *hi2c = NULL;
#define I2C_ADDR 0x4E	// 0x27 << 1


int i2c_init(I2C_HandleTypeDef *p) {
	hi2c = p;
}

int i2c_scan() {
	if(hi2c == NULL) return;
	for(int addr = 0; addr < 256; addr++) {
		if(HAL_I2C_IsDeviceReady(hi2c, addr, 1, 10) == HAL_OK) {
			printf("  %02x", addr);
		} else {
			printf("  . ");
		}
		if((addr + 1) % 16 == 0) {
			printf("\r\n");
		}
	}
}

void lcd_command(char cmd) { // cmd_bit : abcd_efgh | RS(Register Selector) = 0
	char n1, n2, n3, n4, dd[4];
	n1 = cmd & 0xf0;					// n1 : abcd_0000	|	upper nibble
	n2 = cmd << 4;						// n2 : efgh_0000	|	lower nibble to upper
	n3 = (1 << 3) | (1 << 2) | 0 | 0;	// RW | EN_1 | NC | RS;	0x0c
	n4 = (1 << 3) | 0 | 0 | 0;			// RW | EN_0 | NC | RS;	0x08
	dd[0] = n1 | n3;
	dd[1] = n1 | n4;
	dd[2] = n2 | n3;
	dd[3] = n2 | n4;
	HAL_I2C_Master_Transmit(hi2c, I2C_ADDR, dd, 4, 10);
}

void lcd_data(char ch) { // data_bit : abcd_efgh | RS(Register Selector) = 1
	char n1, n2, n3, n4, dd[4];
	n1 = ch & 0xf0;					// n1 : abcd_0000	|	upper nibble
	n2 = ch << 4;						// n2 : efgh_0000	|	lower nibble to upper
	n3 = (1 << 3) | (1 << 2) | 0 | (1 << 0);	// RW | EN_1 | NC | RS;	0x0d
	n4 = (1 << 3) | 0 | 0 | (1 << 0);			// RW | EN_0 | NC | RS;	0x09
	dd[0] = n1 | n3;
	dd[1] = n1 | n4;
	dd[2] = n2 | n3;
	dd[3] = n2 | n4;
	HAL_I2C_Master_Transmit(hi2c, I2C_ADDR, dd, 4, 10);
}

void lcd_init() { // Transmit Initialization Sequence
	lcd_command(0x01); // Screen Clear
	lcd_command(0x02); // Cursor Home
	lcd_command(0x06); // Write Direction = Right
	lcd_command(0x0f); //
	//HAL_Delay(10);
}

void lcd_print(char* str) {
	int i = 0;
	while(*str) {
		lcd_data(*str++);
	}
}

void lcd_printEx(char* str, int ln) {
	if(ln == 0) {
		lcd_command(0x80);
	}
	if(ln == 1) {
		lcd_command(0xc0);
	}
	lcd_print(str);
}

void lcd_print_joystick(unsigned short val, int ln) {
	if(ln == 0) {
		lcd_command(0x80);
	}
	if(ln == 1) {
		lcd_command(0xc0);
	}

	char str[5];
	/*str[0] = (val / 1000) + 48;
	str[1] = ((val / 100) % 10) + 48;
	str[2] = ((val / 10) % 10) + 48;
	str[3] = (val % 10) + 48;
	str[4] = NULL;*/
	//sprintf(str, "%04d", val);
	if(ln) {
		sprintf(str, "Y : %-4d", val);
	} else {
		sprintf(str, "X : %-4d", val);
	}

	/*char* str;
	*str = (val / 1000) + 48;
	*(str + 1) = ((val / 100) % 10) + 48;
	*(str + 2) = ((val / 10) % 10) + 48;
	*(str + 3) = (val % 10) + 48;
	*(str + 4) = NULL;*/ // Not Working

	lcd_printEx(str, ln);
}

void lcd_print_joystick_float(double val, int ln) {
	if(ln == 0) {
		lcd_command(0x80);
	}
	if(ln == 1) {
		lcd_command(0xc0);
	}

	char str[5];
	if(ln) {
		sprintf(str, "Y : %-6.1f", val);
	} else {
		sprintf(str, "X : %-6.1f", val);
	}

	lcd_printEx(str, ln);
}

void lcd_print_helloWorld(char* str, int ln) {
	if(ln == 0) {
		lcd_command(0x80);
	}
	if(ln == 1) {
		lcd_command(0xc0);
	}

	char string[10];
	if(ln) {
		sprintf(string, str);
	} else {
		sprintf(string, str);
	}

	lcd_printEx(string, ln);
}
