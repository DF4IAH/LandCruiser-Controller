/**
 * \file
 *
 * \brief Interrupt Service Routines (ISR)
 *
 */

/**
 * \mainpage ISR
 *
 * \par Interrupt Service Routines (ISR)
 *
 * The Interrupt Service Routines are global functions which are called
 * when enabled interrupts are happening. Each routine is connected to its
 * sub-module where the interrupt is originated.
 *
 * The ISR routine has always a part of code which needs to block any other
 * interrupts to happen until this critical section is done. Often the ISR
 * contains a second (bottom) part, that is non-critical to any new interruption
 * and the global interrupt enable is activated again before entering this
 * bottom part.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Static and global helper functions for the ISR entry functions
 * -# Interrupt Service Routine vectors
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

#include "main.h"
//#include "twi.h"
//#include "lcd.h"

#include "isr.h"


/* External vars */

#include "externals.h"


/* Forward declarations */

static void s_bad_interrupt(void);


/* Helper functions */

void asm_break(void)
{
	__asm__ __volatile__ ("break" ::: "memory");
	nop();
}


/* ISR routines */

static void s_bad_interrupt(void)
{
	asm_break();
}


ISR(__vector_1, ISR_BLOCK)  // variants: ISR_BLOCK, ISR_NOBLOCK, ISR_NAKED
{	/* INT0 */
	s_bad_interrupt();
}

ISR(__vector_2, ISR_BLOCK)
{	/* INT1 */
	s_bad_interrupt();
}

ISR(__vector_3, ISR_BLOCK)
{	/* PCINT0 */
	s_bad_interrupt();
}

ISR(__vector_4, ISR_BLOCK)
{	/* PCINT1 */
	static uint64_t s_last = 0ULL;

	/* Pin PCINT10 has triggered - check for rising signal */
	if (ioport_get_pin_level(TACHO_GPIO)) {
		uint16_t i_now  = g_timer_abs_msb;

		if (i_now) {
			uint64_t diff = i_now - s_last;
			g_speed_over = diff <= C_TICKS_MAXSPEED ?  true : false;
			s_last = i_now;
		}
	}
}

ISR(__vector_5, ISR_BLOCK)
{	/* PCINT2 */
	s_bad_interrupt();
}

ISR(__vector_6, ISR_BLOCK)
{	/* WDT - Watchdog Timeout */
	s_bad_interrupt();
}

ISR(__vector_7, ISR_BLOCK)
{	/* TIMER 2 COMP-A */
	s_bad_interrupt();
}

ISR(__vector_8, ISR_BLOCK)
{	/* TIMER 2 COMP-B */
	s_bad_interrupt();
}

ISR(__vector_9, ISR_BLOCK)
{	/* TIMER 2 OVF - Overflow */
	#if 0
	if (g_status.isAnimationStopped && (g_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC)) {
		static uint8_t state_old = 0;
		static uint8_t second_old = 0;
		static uint8_t button_ctr = 0;
		uint8_t cur = PORTB & 0x3f;

		/* signaling the grade of deviation */
		g_audio_out_loudness = 0;
		if (g_showData.clkState_clk_state < 0xf) {
			cur |= _BV(PORTB6);  // LED = red
			if (state_old != 0x02) {
				g_audio_out_length = 122;  // 1 sec
			}
			state_old = 0x02;

		} else if ((g_showData.clkState_clk_state == 0xf) && (-4 < g_showData.ppb_int) && (g_showData.ppb_int < 4)) {
			cur |= _BV(PORTB7);  // LED = green
			state_old = 0x00;

			/* Acoustic phase tracker */
			if (g_showData.time_second != second_old) {
				second_old = g_showData.time_second;
				g_audio_out_length = 6;
			}

		} else {
			if (!state_old) {
				g_audio_out_length = 30;  // 1/4 sec
			}
			state_old = 0x01;
		}

		PORTB = cur;

		/* sampling I/Q and push buttons */
		if (button_ctr) {
			--button_ctr;
		} else {
			button_ctr = 12;
			uint8_t sw = (PINC & 0x06) >> 1;
			if (!(sw & 0x01)) {									// SW-I: decrement contrast voltage
				if (g_lcd_contrast_pm) {
					--g_lcd_contrast_pm;
					lcd_contrast_update();
				}
			} else if (!(sw & 0x02)) {							// SW-Q: increment contrast voltage
				if (g_lcd_contrast_pm < 0x3f) {
					++g_lcd_contrast_pm;
					lcd_contrast_update();
				}
			} else if (!(PINB & _BV(PINB2))) {					// Pushbutton: store value in EEPROM
				eeprom_nvm_settings_write(C_EEPROM_NVM_SETTING_LCD_CONTRAST);
			}
		}
	}

	/* Beep length enables audio output */
	if (g_audio_out_length) {
		--g_audio_out_length;
		g_audio_out_loudness = 9;  // max 9
	} else {
		g_audio_out_loudness = 0;
	}
	#else
	s_bad_interrupt();
	#endif
}

ISR(__vector_10, ISR_BLOCK)
{	/* TIMER 1 CAPT */
	s_bad_interrupt();
}

ISR(__vector_11, ISR_BLOCK)
{	/* TIMER 1 COMP-A */
	s_bad_interrupt();
}

ISR(__vector_12, ISR_BLOCK)
{	/* TIMER 1 COMP-B */
	s_bad_interrupt();
}

ISR(__vector_13, ISR_BLOCK)
{	/* TIMER 1 OVF - Overflow */
	++g_timer_abs_msb;

	/* Set-up blink queue */
	if (!g_led_blink_code_idx) {
		led_blink_code_enqueue();
	}

	/* Check for LED operations */
	{
		uint64_t now = get_abs_time_ms();

		if (g_led_blink_code_next_ts <= now) {
			g_led_blink_code_next_ts = now + g_led_blink_code_table[g_led_blink_code_idx].delta_ms;

			switch (g_led_blink_code_table[g_led_blink_code_idx++].op) {
			case LED_ON:
				g_led = true;
				led_set(true, g_led);
				break;

			case LED_OFF:
				g_led = false;
				led_set(true, g_led);
				break;

			case LED_LIST_END:
				g_led_blink_code_idx = 0;
				break;
			}
		}

		/* List not terminated - reset to known state */
		if (g_led_blink_code_idx >= C_BC_T_LEN) {
			g_led_blink_code_idx = 0;
			g_led = false;
			led_set(true, g_led);
		}
	}

	/* Start next ADC convertion */
	my_adc_init(g_adc_mux);
	while ((ADCSRA & (1 << ADSC))) {
		/* wait for conversion complete */
	}
	process_adc();
	my_adc_disable();


	#if 0
	/* Start A/D convertion by entering ADC sleep-mode */
	while ((ADCSRA & (1 << ADSC))) {
		/* wait for conversion complete */
	}
	ADCSRA |= _BV(ADIF);						// clear interrupt status bit by setting it to clear
	adc_enable_interrupt();						// enable the ADC interrupt
	cpu_irq_enable();
	adc_start_conversion();
	enter_sleep(SLEEP_MODE_ADC);
	adc_disable_interrupt();
	#endif
}

ISR(__vector_14, ISR_BLOCK)
{	/* TIMER 0 COMP-A */
	s_bad_interrupt();
}

ISR(__vector_15, ISR_BLOCK)
{	/* TIMER 0 COMP-B */
	s_bad_interrupt();
}

ISR(__vector_16, ISR_BLOCK)
{	/* TIMER 0 OVF - Overflow */
	#if 0
	/* Start A/D convertion by entering ADC sleep-mode */
	while ((ADCSRA & (1 << ADSC))) {
		/* wait for conversion complete */
	}
	ADCSRA |= _BV(ADIF);						// clear interrupt status bit by setting it to clear
	adc_enable_interrupt();						// enable the ADC interrupt
	cpu_irq_enable();

	enter_sleep(SLEEP_MODE_ADC);
	adc_disable_interrupt();
	#else
	s_bad_interrupt();
	#endif
}

ISR(__vector_17, ISR_BLOCK)
{	/* SPI, STC - Serial Transfer Complete */
	s_bad_interrupt();
}

ISR(__vector_18, ISR_BLOCK)
{	/* USART, RX - Complete */

	/* Get received character */
	uint8_t c = UDR0;
	g_serial_rx_buf[g_serial_rx_idx++] = c;
	g_serial_rx_buf[g_serial_rx_idx]   = 0;

	/* Line ending */
	if (c == '\n') {
		g_serial_rx_eol = true;
	}

	/* Buffer size clamping */
	if (g_serial_rx_idx >= C_SERIAL_RX_BUF_SIZE) {
		g_serial_rx_idx = C_SERIAL_RX_BUF_SIZE - 1;
	}
}

ISR(__vector_19, ISR_BLOCK)
{	/* USART, UDRE - Data Register Empty */

	if (g_serial_tx_len > 0) {
		/* Send next character */
		UDR0 = g_serial_tx_buf[0];

		/* Move string one position ahead */
		for (uint8_t idx = 0; idx < g_serial_tx_len; idx++) {
			g_serial_tx_buf[idx] = g_serial_tx_buf[idx + 1];
		}

		/* Resize length to be sent */
		--g_serial_tx_len;
	}
}

ISR(__vector_20, ISR_BLOCK)
{	/* USART, TX - Complete */

	g_serial_tx_ongoing = false;
}

ISR(__vector_21, ISR_BLOCK)
{	/* ADC */

	s_bad_interrupt();
}

void process_adc(void) {
	static uint8_t s_initCtr = 32;
	/* Get data from the ADC - read low byte first (@see 8271G�AVR�02/2013 page 243) */
	uint8_t adcl = ADCL;
	barrier();
	uint8_t adch = ADCH;

	volatile uint16_t adc_val = (uint16_t)adch << 8 | (uint16_t)adcl;

	/* Remove ADC offset */
	if (adc_val > 60) {
		adc_val -= 60;
	} else {
		adc_val = 0;
	}

	C_ADC_STATE__ENUM_t l_adc_state = g_adc_state;
	switch (l_adc_state) {
		case C_ADC_STATE_VLD_12V:
		{
			/* Low pass filtering and enhancing the data depth */
			float l_adc_12v			= 0.95f * g_adc_12v + 0.05f * adc_val;
			int32_t l_adc_12v_1000	= -32300 + (uint16_t) (((uint32_t)l_adc_12v * 122820UL) / 1024UL);  // Uref = 5.0 V: -32300 + x*122820UL

			g_adc_12v		= l_adc_12v;
			g_adc_12v_1000	= (uint16_t)l_adc_12v_1000;
			g_adc_12v_under = l_adc_12v_1000 < 10500U ?  true : false;

			g_adc_mux		= (ADC_MUX_TEMPSENSE | ADC_VREF_AVCC | ADC_ADJUSTMENT_RIGHT);
			g_adc_state		= C_ADC_STATE_VLD_TEMP;

			/* Drop first samples */
			if (s_initCtr) {
				--s_initCtr;
				g_adc_12v		= adc_val;
				g_adc_12v_1000	= 12000U;
				g_adc_12v_under = false;
			}
		}
		break;

		case C_ADC_STATE_VLD_TEMP:
		{
			const float C_temp_coef_k			= 1.0595703f * (5.f / 1.1f);
			const float C_temp_coef_ofs_atmel	= 1024 * 0.314f / 5.0f;
			const float C_temp_coef_ofs			= 52.00f;

			/* Low pass filtering and enhancing the data depth */
			float l_adc_temp		= 0.998f * g_adc_temp  + 0.002f * adc_val;
			float l_adc_temp_100	= 100.f * (C_temp_coef_ofs + 25.f + (l_adc_temp - C_temp_coef_ofs_atmel) * C_temp_coef_k);

			g_adc_temp		= l_adc_temp;
			g_adc_temp_100	= (int32_t)l_adc_temp_100;

			g_adc_mux		= (ADC_MUX_ADC0 | ADC_VREF_AVCC | ADC_ADJUSTMENT_RIGHT);
			g_adc_state		= C_ADC_STATE_VLD_12V;

			/* Drop first samples */
			if (s_initCtr) {
				g_adc_temp = adc_val;
			}
		}
		break;

		default:
			g_adc_state = C_ADC_STATE_VLD_12V;
	}

	/* Reset Timer1 overflow status bit (when no ISR for TOV1 is activated!) */
	//TIFR1 |= _BV(TOV1);
}

ISR(__vector_22, ISR_BLOCK)
{	/* EEREADY */
	s_bad_interrupt();
}

ISR(__vector_23, ISR_BLOCK)
{	/* ANALOG COMP */
	s_bad_interrupt();
}

ISR(__vector_24, ISR_BLOCK)
{	/* TWI */
	#if 0
	uint8_t tws = TWSR & (0b11111 << TWS3);
	uint8_t twd = TWDR;
	uint8_t twcr_cur = TWCR;

	uint8_t twcr_new = __vector_24__bottom(tws, twd, twcr_cur);
	TWCR = twcr_new | _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
	#else
	s_bad_interrupt();
	#endif
}

ISR(__vector_25, ISR_BLOCK)
{	/* SPM READY - Store Program Memory Ready */
	s_bad_interrupt();
}
