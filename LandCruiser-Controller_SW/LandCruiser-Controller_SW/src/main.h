/*
 * main.h
 *
 * Created: 25.08.2017 20:55:00
 *  Author: DF4IAH
 */


#ifndef MAIN_H_
#define MAIN_H_

//#include <gfx_mono/gfx_mono.h>

//#include "twi.h"


/* ATmega 328P - fuses ext:0xFD hi:0xB7 lo:0xE2 */


/* VERSION: YYM, MDD */
#define VERSION_HIGH												170
#define VERSION_LOW													826

/* I2C-Version V1.1 */
#define I2C_VERSION													0x11


/* TC1 timer runs @ 8 MHz / overflows every 1 ms */
#define C_TC1_TOP_VAL												8000


enum C_SMART_LCD_MODE__ENUM {
	C_SMART_LCD_MODE_UNIQUE											= 0x00,
	C_SMART_LCD_MODE_SMARTLCD										= 0x10,
};

enum C_EEPROM_ADDR__ENUM {
	C_EEPROM_ADDR_VERSION											= 0x00,
	C_EEPROM_ADDR_LCD_PM											= 0x10,
};

enum C_EEPROM_NVM_SETTING__ENUM {
	C_EEPROM_NVM_SETTING_LCD_CONTRAST								= 0x01,
	C_EEPROM_NVM_SETTING_VERSION									= 0x80,
	C_EEPROM_NVM_SETTING_ALL										= 0xFF
};


#ifndef	TWI_SMART_LCD_SLAVE_BUF_LEN
# define TWI_SMART_LCD_SLAVE_BUF_LEN								16
#endif
typedef struct showData {
	uint16_t	newClkState											: 1;
	uint16_t	newDate												: 1;
	uint16_t	newTime												: 1;
	uint16_t	newSatUse											: 1;
	uint16_t	newSatDop											: 1;
	uint16_t	newPosState											: 1;
	uint16_t	newPosLat											: 1;
	uint16_t	newPosLon											: 1;
	uint16_t	newPosHeight										: 1;
	uint16_t	newRsvrd											: 7;

	uint8_t				cmd;
	uint8_t				data[TWI_SMART_LCD_SLAVE_BUF_LEN];

	#if 0
	gfx_mono_color_t	pixelType;
	gfx_coord_t			pencil_x;
	gfx_coord_t			pencil_y;
	#endif

	uint8_t		clkState_clk_state;
	uint8_t		date_month;
	uint8_t		date_day;
	uint8_t		time_hour;
	uint8_t		time_minute;
	uint8_t		time_second;
	uint8_t		satUse_west;
	uint8_t		satUse_east;
	uint8_t		satUse_used;
	uint8_t		posState_fi;
	uint8_t		posState_m2;
	uint8_t		posLat_sgn;
	uint8_t		posLat_deg;
	uint8_t		posLat_min_int;
	uint8_t		posLon_sgn;
	uint8_t		posLon_deg;
	uint8_t		posLon_min_int;
	uint8_t		pos_height_frac10;

} showData_t;


/* UTILITIES section */
uint64_t get_abs_time_ms(void);
void mem_set(uint8_t* buf, uint8_t count, uint8_t val);
void eeprom_nvm_settings_write(uint8_t flags);
void eeprom_nvm_settings_read(uint8_t flags);


/* TASK section */
void task(float timestamp);
void enter_sleep(uint8_t sleep_mode);
void halt(void);


/* MAIN section */
int main(void);


#endif /* MAIN_H_ */
