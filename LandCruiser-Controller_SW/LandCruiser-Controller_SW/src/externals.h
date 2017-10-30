/*
 * externals.h
 *
 * Created: 28.10.2017 16:43:00
 *  Author: DF4IAH
 */


#ifndef EXTERNALS_H_
#define EXTERNALS_H_

#include "serial.h"


extern uint_fast64_t		g_timer_abs_msb;
extern bool					g_led;
extern uint8_t				g_led_digits[C_BC_DIGITS];
extern led_bc_q_entry_t		g_led_blink_code_table[C_BC_T_LEN];
extern uint8_t				g_led_blink_code_idx;
extern uint64_t				g_led_blink_code_next_ts;
extern uint8_t				g_adc_state;
extern float				g_adc_12v;
extern uint16_t				g_adc_12v_1000;
extern bool					g_adc_12v_under;
extern float				g_adc_temp;
extern int32_t				g_adc_temp_100;
extern bool					g_speed_over;
extern showData_t			g_showData;
extern uint8_t				g_SmartLCD_mode;
//extern uint8_t			g_lcd_contrast_pm;
extern char					g_serial_rx_buf[C_SERIAL_RX_BUF_SIZE];
extern uint16_t				g_serial_rx_idx;
extern bool					g_serial_rx_eol;
extern char					g_serial_tx_buf[C_SERIAL_TX_BUF_SIZE];
extern uint16_t				g_serial_tx_len;
extern char					g_strbuf[C_SERIAL_TX_BUF_SIZE];


#endif /* EXTERNALS_H_ */
