/**
 * \file
 *
 * \brief The main-loop, init and shutdown of the application
 *
 */

/**
 * \mainpage Main module
 *
 * \par The main-loop, init and shutdown of the application
 *
 * The Main module includes the main loop of the application. Any
 * needed initialization and shutdown code is located here, also.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
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
#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>

#include "isr.h"
#include "serial.h"
//#include "twi.h"
//#include "lcd.h"
//#include "gfx_mono/sysfont.h"

#include "main.h"


/* FUSES
 *
 * Please set fuses to these values:
 * Ext: 0xFE
 * Hi : 0xB7
 * Lo : 0xE2
 *
 */


/* GLOBAL section */

uint_fast64_t		g_timer_abs_msb							= 0ULL;
bool				g_led									= false;
uint8_t				g_led_digits[C_BC_DIGITS]				= { 0 };
led_bc_q_entry_t	g_led_blink_code_table[C_BC_T_LEN]		= { 0 };
uint8_t				g_led_blink_code_idx					= 0;
uint64_t			g_led_blink_code_next_ts				= 0;
C_ADC_STATE__ENUM_t	g_adc_state								= C_ADC_STATE_VLD_12V;
uint8_t				g_adc_mux								= (ADC_MUX_TEMPSENSE | ADC_VREF_AVCC | ADC_ADJUSTMENT_RIGHT);
float				g_adc_12v								= 0.f;
uint16_t			g_adc_12v_1000							= 0;
bool				g_adc_12v_under							= false;
float				g_adc_temp								= 0.f;
int32_t				g_adc_temp_100							= 0;
bool				g_speed_over							= false;
bool				g_o_kl									= false;
//bool				g_lcdbl_auto							= true;
//uint8_t			g_lcdbl_dimmer							= 0;
//uint8_t			g_lcd_contrast_pm						= 0;
showData_t			g_showData								= { 0 };
uint8_t				g_SmartLCD_mode							= C_SMART_LCD_MODE_UNIQUE;
//gfx_mono_color_t	g_lcd_pixel_type						= GFX_PIXEL_CLR;
//gfx_coord_t		g_lcd_pencil_x							= 0;
//gfx_coord_t		g_lcd_pencil_y							= 0;
uint8_t				g_resetCause							= 0;
char				g_serial_rx_buf[C_SERIAL_RX_BUF_SIZE]	= { 0 };
uint16_t			g_serial_rx_idx							= 0;
bool				g_serial_rx_eol							= false;
char				g_serial_tx_buf[C_SERIAL_TX_BUF_SIZE]	= { 0 };
uint16_t			g_serial_tx_len							= 0;
bool				g_serial_tx_ongoing						= false;
char				g_strbuf[C_SERIAL_TX_BUF_SIZE]			= { 0 };


/* MAIN STATIC section */

static uint8_t		runmode									= 0;



/* HELPERS */

static void s_reset_global_vars(void)
{
	irqflags_t flags = cpu_irq_save();

	runmode								= 0;

	g_timer_abs_msb						= 0ULL;
	g_led								= false;
	g_adc_state							= C_ADC_STATE_VLD_12V;
	g_adc_mux							= (ADC_MUX_TEMPSENSE | ADC_VREF_AVCC | ADC_ADJUSTMENT_RIGHT);
	g_adc_12v							= 0.f;
	g_adc_12v_1000						= 0;
	g_adc_12v_under						= false;
	g_adc_temp							= 0.f;
	g_adc_temp_100						= 0;
	g_speed_over						= false;
	g_o_kl								= false;

	#if 0
	g_lcdbl_auto						= true;
	g_lcdbl_dimmer						= 64;
	g_lcd_contrast_pm					= 0;
	#endif

	memset(&g_showData, 0, sizeof(g_showData));

	g_SmartLCD_mode						= C_SMART_LCD_MODE_UNIQUE;

	#if 0
	g_lcd_pixel_type					= GFX_PIXEL_CLR;
	g_lcd_pencil_x						= 0;
	g_lcd_pencil_y						= 0;
	#endif

	memset(&g_serial_rx_buf, 0, sizeof(g_serial_rx_buf));
	g_serial_rx_idx						= 0;
	g_serial_rx_eol						= false;

	memset(&g_serial_tx_buf, 0, sizeof(g_serial_tx_buf));
	g_serial_tx_len						= 0;

	g_resetCause						= 0;

	cpu_irq_restore(flags);
}


/* INIT section */

static void s_io_preinit(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts,
	 * most critical pins are handled first.
	 */

	DDRB  = 0b00111100;											// Out: PB5: MP_PV_G, PB4: MP_PV_O, PB3: MP_M1, PB2: MP_M2
	PORTB = 0b11000011;											// In:  PB7: MP_SK_O, PB6: MP_SK_G, PB1: NC, PB0: SH_G

	DDRC  = 0b00101000;											// Out: PC5: SCL, PC3: MP_KL
	PORTC = 0b00110000;											// In:  PC4: SDA, PC2: MP_TACHO (PCINT10), PC1: MP_ADC1, PC0: MP_ADC0

	DDRD  = 0b00001010;											// Out: PD3: MP_LED, PD1: MP_TXD
	PORTD = 0b11110111;											// In:  PD7: MP_SA_O, PD6: MP_SA_G, PD5: NC, PD4: MP_FB, PD2: MP_INT, PD0: MP_RXD

	/* Connect PCINT10 = Pin PC2 to PCI1 */
	PCMSK1	= _BV(PCINT10);
	PCIFR	= _BV(PCIF1);
	PCICR	= _BV(PCIE1);
}

void led_set(bool doOutput, bool setHigh)
{
	if (doOutput) {
		ioport_set_pin_level(LED_GPIO, setHigh);
		ioport_set_pin_dir(LED_GPIO, IOPORT_DIR_OUTPUT);

	} else {
		ioport_set_pin_dir(LED_GPIO, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(LED_GPIO, IOPORT_MODE_PULLDOWN);
	}
}

void led_blink_code_set(uint32_t code)
{
	irqflags_t	flags	= cpu_irq_save();

	for (int idx = 0; idx < C_LED_DIGITS; idx++) {
		uint8_t digit = 0;

		if (code) {
			digit = (code >> ((C_LED_DIGITS - idx - 1) << 2)) & 0x07;
			if (!digit || (digit > 5)) {
				digit = 5;
			}
		}
		g_led_digits[idx] = digit;
	}

	cpu_irq_restore(flags);
}

uint8_t led_blink_code_enqueue(void)
{
	uint8_t idx_old, idx_new;

	/* IRQ disabled section */
	{
		irqflags_t flags = cpu_irq_save();

		idx_new = idx_old = g_led_blink_code_idx;

		/* Do update only when list is empty */
		if (!idx_old) {
			for (int idx = 0; idx < C_LED_DIGITS; idx++) {
				uint8_t digit = g_led_digits[idx];

				if (digit) {
					for (int dot = 0; dot < digit; dot++) {
						g_led_blink_code_table[g_led_blink_code_idx]  .op		= LED_ON;
						g_led_blink_code_table[g_led_blink_code_idx++].delta_ms = C_LED_DOT_ON_TIME_MS;

						g_led_blink_code_table[g_led_blink_code_idx]  .op		= LED_OFF;
						g_led_blink_code_table[g_led_blink_code_idx++].delta_ms = C_LED_DOT_OFF_TIME_MS;
					}

					/* Additional time between each digit */
					g_led_blink_code_table[g_led_blink_code_idx]  .op		= LED_OFF;
					g_led_blink_code_table[g_led_blink_code_idx++].delta_ms = (C_LED_INTERDOT_TIME_MS - C_LED_DOT_OFF_TIME_MS);
				}
			}

			/* Additional time between each code */
			g_led_blink_code_table[g_led_blink_code_idx]  .op		= LED_OFF;
			g_led_blink_code_table[g_led_blink_code_idx++].delta_ms = (C_LED_CODEEND_TIME_MS - (C_LED_INTERDOT_TIME_MS + C_LED_DOT_OFF_TIME_MS));

			/* End of message */
			g_led_blink_code_table[g_led_blink_code_idx]  .op		= LED_LIST_END;
			g_led_blink_code_table[g_led_blink_code_idx].delta_ms	= 0;

			idx_new = g_led_blink_code_idx;

			/* Point to the start of table */
			g_led_blink_code_idx = 0;
		}

		cpu_irq_restore(flags);
	}

	return idx_new - idx_old;
}


static void s_tc_init(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts. */

	#if 0
	/* Timer Synchronous Mode - prepare for  s_tc_start(void) */
	GTCCR = _BV(TSM)											// Timer Synchronous Mode active
		  | _BV(PSRASY)											// Timer 2   prescaler is synced
		  | _BV(PSRSYNC);										// Timer 0/1 prescaler is synced
	#endif

	#if 0
	/* TC0 - OC0x: N/A */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM0_bm);

		TCCR0A  = (0b00  << COM0A0)								// Normal port operations
				| (0b00  << WGM00);								// Counter mode

		TCCR0B  = ( 0b0  << WGM02)
				| (0b100 << CS00);								// CLKio DIV 256 = 31250 Hz --> / 2**8 = 122 Hz looping rate

		TCNT0   = 0;											// Clear current value

		OCR0A   = 0x00;											// Compare value not used

		TIFR0   = 0b00000111;									// Clear all flags
		TIMSK0  = _BV(TOIE0);									// TOIE0: overflow interrupt
	}
	#endif

	/* TC1 global clock - OC1A: NC */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM1_bm);

		TCCR1A  = (0b00  << COM1A0)		 						// OC1A: disconnected - normal port function
				| (0b01  << WGM10);								// WGM: 0b0100 = PWM, Phase and Frequency Correct / Update on BOTTOM, TOP @ OCR1A

		TCCR1B  = ( 0b10 << WGM12)
				| (0b010 << CS10);								// CLKio DIV=8 --> 1 MHz

		TCNT1H  = 0b00000000           ;						// Clear current value for synchronous start (when restarting without reset)
		barrier();
		TCNT1L	=            0b00000000;

		OCR1AH  = (uint8_t) (C_TC1_TOP_VAL >> 8);				// 1 kHz overflow frequency (1 ms)
		barrier();
		OCR1AL  = (uint8_t) (C_TC1_TOP_VAL & 0xff);

		TIFR1   = 0b00100111;									// Clear all flags (when restarting without reset)
		TIMSK1  = _BV(TOIE1);									// TOIE1 interrupt
	}

	#if 0
	/* TC2 - OC2A: N/A */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM2_bm);

		ASSR    = 0;											// No async. TOSC1 mode

		TCCR2A  = (0b10  << COM2A0)								// HI --> LO when compare value is reached - non-inverted PWM mode
				| (0b11  << WGM20);								// WGM: 0b011 = Fast PWM mode 8 bit

		TCCR2B  = ( 0b0  << WGM22)
				| (0b101 << CS20);								// CLKio DIV 128 = 62500 Hz --> / 2**8 = 244 Hz looping rate

		TCNT2   = 0;											// Clear current value for synchronous start (when restarting without reset)

		OCR2A   = 0x00;											// LCD backlight dimmed down

		TIFR2   = 0b00000111;									// Clear all flags
		TIMSK2  = _BV(TOIE2);									// TOIE2: overflow interrupt
	}
	#endif
}

static void s_tc_start(void)
{
	#if 0
	/* TC0: N/A */
	/* TC1: Audio output @ 16-bit counter PWM, used: 10-bit resolution */
	/* TC2: N/A */
	{
		/* Timer Synchronous Mode - trigger */
		GTCCR = 0;												// Trigger the sync for all counters
	}
	#endif
}

static void s_tc_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	#if 0
	/* TC0 - Overflows with about 122 Hz for the ADC convertion */
	{
		TIMSK0  = 0;											// No interrupts

		TCCR0A  = 0;
		TCCR0B  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM0_bm);
	}
	#endif

	/* TC1 - OC1A: Audio output @ 16-bit counter PWM, used: 10-bit resolution - overflows with 15625 Hz */
	{
		// bring pin to high Z mode to reduce audible plop noise
		//ioport_set_pin_dir(AUDIO_PWM, IOPORT_DIR_INPUT);
		//ioport_set_pin_mode(AUDIO_PWM, IOPORT_MODE_PULLDOWN);

		TIMSK1  = 0;											// No interrupts

		TCCR1A  = 0;											// Release alternate port function
		TCCR1B  = 0;
		TCCR1C  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM1_bm);
	}

	#if 0
	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		//ioport_set_pin_dir(LCDBL_PWM, IOPORT_DIR_OUTPUT);
		//ioport_set_pin_level(LCDBL_PWM, false);				// Turn backlight off

		TIMSK2  = 0;											// No interrupts

		ASSR    = 0;											// No async TOSC1 mode

		TCCR2A  = 0;											// Release alternate port function
		TCCR2B  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM2_bm);
	}
	#endif

	sysclk_set_prescalers(SYSCLK_PSDIV_256);

	cpu_irq_restore(flags);
}


void my_adc_init(uint8_t mux)
{
	sysclk_enable_module(POWER_RED_REG0, PRADC_bm);				// Enable ADC sub-module
	adc_disable_interrupt();

	adc_disable_digital_inputs(_BV(ADC1D) | _BV(ADC0D));		// MP_ADC0, MP_ADC1: disable the digital input on the ADC0 and ADC1 port

	adc_init(ADC_PRESCALER_DIV16);
	adc_set_admux(mux);

	ADCSRA |= _BV(ADIF);										// Clear interrupt status bit by setting it to clear
	adc_enable_interrupt();										// Enable the ADC interrupt

	//adc_set_autotrigger_source(ADC_AUTOTRIGGER_SOURCE_FREERUNNING);
	//adc_enable_autotrigger();
	adc_start_conversion();										// Initial convertion doing an ADC set-up
}

void my_adc_disable(void)
{
	adc_disable_autotrigger();
	adc_disable_interrupt();									// Disable the ADC interrupt
	sysclk_disable_module(POWER_RED_REG0, PRADC_bm);			// Disable ADC sub-module
}


#if 0
static void s_twi_init(uint8_t twi_addr, uint8_t twi_addr_bm)
{
	sysclk_enable_module(POWER_RED_REG0, PRTWI_bm);

	irqflags_t flags = cpu_irq_save();

	TWSR = (0b00 << TWPS0);										// Prescaler value = 1
	TWBR = 2;													// TWI bit-rate = 400 kBit/sec @ 8 MHz when master mode active

	TWAR  = (twi_addr    << 1) /* | (TWI_SLAVE_ADDR_GCE << TWGCE)*/ ;
	TWAMR = (twi_addr_bm << 1);

	TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);					// Enable Acknowledge, Enable TWI port, Interrupt Enable, no START or STOP bit

	cpu_irq_restore(flags);
}

static void s_twi_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	TWCR = _BV(TWEN);											// Disable the interrupt source

	ioport_set_pin_dir(SDA_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SDA_GPIO, IOPORT_MODE_PULLUP);

	ioport_set_pin_dir(SCL_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SCL_GPIO, IOPORT_MODE_PULLUP);

	TWCR = 0;													// Disable the TWI port

	cpu_irq_restore(flags);

	sysclk_disable_module(POWER_RED_REG0, PRTWI_bm);
}
#endif


/* UTILITIES section */

uint64_t get_abs_time_ms(void)
{
	uint_fast64_t l_tmr_msb;

	irqflags_t flags = cpu_irq_save();
	l_tmr_msb = g_timer_abs_msb;
	cpu_irq_restore(flags);

	return l_tmr_msb;
}

void mem_set(uint8_t* buf, uint8_t count, uint8_t val)
{
	for (int i = count; i; --i) {
		*(buf++) = val;
	}
}

void eeprom_nvm_settings_write(uint8_t flags)
{
	/* I2C_VERSION */
	if (flags & C_EEPROM_NVM_SETTING_VERSION) {
		eeprom_write_byte((uint8_t *) C_EEPROM_ADDR_VERSION,
		I2C_VERSION);
	}

	#if 0
	/* LCD_PM */
	if (flags & C_EEPROM_NVM_SETTING_LCD_CONTRAST) {
		eeprom_write_byte((uint8_t *) C_EEPROM_ADDR_LCD_PM,
		g_lcd_contrast_pm & 0x3F);
	}
	#endif
}

void eeprom_nvm_settings_read(uint8_t flags)
{
	/* I2C_VERSION */
	if (flags & C_EEPROM_NVM_SETTING_VERSION) {
		uint8_t ver = eeprom_read_byte((const uint8_t *) C_EEPROM_ADDR_VERSION);
		if (ver != I2C_VERSION) {
			eeprom_nvm_settings_write(C_EEPROM_NVM_SETTING_VERSION);
		}
	}

	#if 0
	/* LCD_PM */
	//g_lcd_contrast_pm = C_LCD_PM;								// Preset value
	if (flags & C_EEPROM_NVM_SETTING_LCD_CONTRAST) {
		uint8_t val = eeprom_read_byte((const uint8_t *) C_EEPROM_ADDR_LCD_PM);
		if (val <= 0x3F) {										// Value from NVM is marked as being valid
			g_lcd_contrast_pm = val;
		} else {
			eeprom_nvm_settings_write(C_EEPROM_NVM_SETTING_LCD_CONTRAST);
		}
	}
	#endif
}


/* TASK section */

void task(uint64_t now)
{
	/* TASK when woken up */
	static uint64_t s_timer_task_next	= 0ULL;
	static uint64_t s_timer_fb			= 0ULL;
	static uint64_t s_timer_pv			= 0ULL;

	static uint8_t s_fsm_state			= 0x11;
	static uint8_t s_fsm_state_dbg		= 0;
	static bool s_change_dir			= false;

	static bool s_i_fb					= false;				// FB
	static bool s_i_fb_t0				= false;
	static bool s_i_fb_t1				= false;
	static bool s_i_fb_t2				= false;
	static bool s_i_fb_t3				= false;
	static bool s_i_uv					= false;				// ADC0 12V under-voltage
	static bool s_i_uv_t0				= false;
	static bool s_i_uv_t1				= false;
	static bool s_i_uv_t2				= false;
	static bool s_i_os					= false;				// Over-speed
	static bool s_i_os_t0				= false;
	static bool s_i_os_t1				= false;
	static bool s_i_os_t2				= false;
	static bool s_i_sk_g				= false;				// SK_G
	static bool s_i_sk_g_t0				= false;
	static bool s_i_sk_g_t1				= false;
	static bool s_i_sk_g_t2				= false;
	static bool s_i_sk_o				= false;				// SK_O
	static bool s_i_sk_o_t0				= false;
	static bool s_i_sk_o_t1				= false;
	static bool s_i_sk_o_t2				= false;
	static bool s_i_sa_g				= false;				// SA_G
	static bool s_i_sa_g_t0				= false;
	static bool s_i_sa_g_t1				= false;
	static bool s_i_sa_g_t2				= false;
	static bool s_i_sa_o				= false;				// SA_O
	static bool s_i_sa_o_t0				= false;
	static bool s_i_sa_o_t1				= false;
	static bool s_i_sa_o_t2				= false;
	static bool s_i_sh_g				= false;				// SH_G
	static bool s_i_sh_g_t0				= false;
	static bool s_i_sh_g_t1				= false;
	static bool s_i_sh_g_t2				= false;

	static bool s_o_kl					= false;				// KL
	static bool s_o_kl_t0				= false;
	static bool s_o_pv_g				= false;				// PV_G
	static bool s_o_pv_g_t0				= false;
	static bool s_o_pv_o				= false;				// PV_O
	static bool s_o_pv_o_t0				= false;
	static bool s_o_m1					= false;				// M1
	static bool s_o_m1_t0				= false;
	static bool s_o_m2					= false;				// M2
	static bool s_o_m2_t0				= false;

	/* Running every C_TASK_TIMESPAN milliseconds */
	if (s_timer_task_next <= now) {
		s_timer_task_next = now + C_TASK_TIMESPAN;
	} else {
		return;
	}

	/* Read in the current input vector */
	irqflags_t flags = cpu_irq_save();
	uint8_t l_pin_b = PINB;
	// uint8_t l_pin_c = PINC;
	uint8_t l_pin_d = PIND;
	cpu_irq_restore(flags);

	/* De-noising shifter */
	{
		s_i_fb_t3	= s_i_fb_t2;
		s_i_fb_t2	= s_i_fb_t1;
		s_i_fb_t1	= s_i_fb_t0;

		s_i_os_t2	= s_i_os_t1;
		s_i_os_t1	= s_i_os_t0;

		s_i_uv_t2	= s_i_uv_t1;
		s_i_uv_t1	= s_i_uv_t0;

		s_i_sk_g_t2	= s_i_sk_g_t1;
		s_i_sk_g_t1	= s_i_sk_g_t0;

		s_i_sk_o_t2	= s_i_sk_o_t1;
		s_i_sk_o_t1	= s_i_sk_o_t0;

		s_i_sa_g_t2	= s_i_sa_g_t1;
		s_i_sa_g_t1	= s_i_sa_g_t0;

		s_i_sa_o_t2	= s_i_sa_o_t1;
		s_i_sa_o_t1	= s_i_sa_o_t0;

		s_i_sh_g_t2	= s_i_sh_g_t1;
		s_i_sh_g_t1	= s_i_sh_g_t0;
	}

	/* Break up into single input signals */
	{  // @see conf_board.h
		s_i_fb_t0	= (l_pin_b & _BV(0)) ?  false : true;
		s_i_uv_t0	= g_adc_12v_under;
		s_i_os_t0	= g_speed_over;
		s_i_sk_g_t0	= (l_pin_b & _BV(6)) ?  false : true;
		s_i_sk_o_t0	= (l_pin_b & _BV(7)) ?  false : true;
		s_i_sa_g_t0	= (l_pin_d & _BV(6)) ?  false : true;
		s_i_sa_o_t0	= (l_pin_d & _BV(7)) ?  false : true;
		s_i_sh_g_t0	= (l_pin_d & _BV(4)) ?  false : true;
	}

	/* De-noising logics */
	{
		if (s_i_fb_t2 && s_i_fb_t1 && s_i_fb_t0) {
			s_i_fb = true;
		} else if (!s_i_fb_t2 || !s_i_fb_t1 || !s_i_fb_t0) {
			s_i_fb = false;
		}

		if (s_i_uv_t2 || s_i_uv_t1 || s_i_uv_t0) {
			s_i_uv = true;
		} else if (!s_i_uv_t2 && !s_i_uv_t1 && !s_i_uv_t0) {
			s_i_uv = false;
		}

		if (s_i_os_t2 || s_i_os_t1 || s_i_os_t0) {
			s_i_os = true;
		} else if (!s_i_os_t2 && !s_i_os_t1 && !s_i_os_t0) {
			s_i_os = false;
		}

		if (s_i_sk_g_t2 || s_i_sk_g_t1 || s_i_sk_g_t0) {
			s_i_sk_g = true;
			} else if (!s_i_sk_g_t2 && !s_i_sk_g_t1 && !s_i_sk_g_t0) {
			s_i_sk_g = false;
		}

		if (s_i_sk_o_t2 || s_i_sk_o_t1 || s_i_sk_o_t0) {
			s_i_sk_o = true;
		} else if (!s_i_sk_o_t2 && !s_i_sk_o_t1 && !s_i_sk_o_t0) {
			s_i_sk_o = false;
		}

		if (s_i_sa_g_t2 || s_i_sa_g_t1 || s_i_sa_g_t0) {
			s_i_sa_g = true;
		} else if (!s_i_sa_g_t2 && !s_i_sa_g_t1 && !s_i_sa_g_t0) {
			s_i_sa_g = false;
		}

		if (s_i_sa_o_t2 || s_i_sa_o_t1 || s_i_sa_o_t0) {
			s_i_sa_o = true;
		} else if (!s_i_sa_o_t2 && !s_i_sa_o_t1 && !s_i_sa_o_t0) {
			s_i_sa_o = false;
		}

		if (s_i_sh_g_t2 || s_i_sh_g_t1 || s_i_sh_g_t0) {
			s_i_sh_g = true;
		} else if (!s_i_sh_g_t2 && !s_i_sh_g_t1 && !s_i_sh_g_t0) {
			s_i_sh_g = false;
		}
	}

	/* FSM (Finite State Machine) */
	{
		/* Show current state of FSM */
		led_blink_code_set(s_fsm_state);

		/* Modem info */
		if (s_fsm_state_dbg != s_fsm_state) {
			serial_printFsmState(s_fsm_state_dbg, s_fsm_state);
			s_fsm_state_dbg = s_fsm_state;
		}
		if (s_i_fb_t0	!= s_i_fb_t1	||
			s_i_uv_t0	!= s_i_uv_t1	||
			s_i_os_t0	!= s_i_os_t1	||
			s_i_sk_g_t0	!= s_i_sk_g_t1	||
			s_i_sk_o_t0	!= s_i_sk_o_t1	||
			s_i_sa_g_t0	!= s_i_sa_g_t1	||
			s_i_sa_o_t0	!= s_i_sa_o_t1	||
			s_i_sh_g_t0	!= s_i_sh_g_t1) {
			serial_printFsmInput(s_i_fb_t0, s_i_uv_t0, s_i_os_t0, s_i_sk_g_t0, s_i_sk_o_t0, s_i_sa_g_t0, s_i_sa_o_t0, s_i_sh_g_t0);
		}
		if (s_o_kl		!= s_o_kl_t0	||
			s_o_pv_g	!= s_o_pv_g_t0	||
			s_o_pv_o	!= s_o_pv_o_t0	||
			s_o_m1		!= s_o_m1_t0	||
			s_o_m2		!= s_o_m2_t0) {
			serial_printFsmOutput(s_o_kl, s_o_pv_g, s_o_pv_o, s_o_m1, s_o_m2);

			/* Output modification check */
			{
				s_o_kl_t0	= s_o_kl;
				s_o_pv_g_t0	= s_o_pv_g;
				s_o_pv_o_t0	= s_o_pv_o;
				s_o_m1_t0	= s_o_m1;
				s_o_m2_t0	= s_o_m2;
			}
		}

		/* FSM_1 - Logic */
		switch (s_fsm_state) {
			case 0x11:
			{	/* INIT: Init and Error handling: check for correct starting vector */
				s_o_kl		= false;
				s_o_pv_g	= false;
				s_o_pv_o	= false;
				s_o_m1		= false;
				s_o_m2		= false;

				if (!s_i_uv && !s_i_os) {
					s_timer_fb	= 0ULL;
					s_fsm_state = 0x00;
				}
			}
			break;

			case 0x00:
			{
				/* STANDBY CLOSED: Tailgate closed, normal LandCruiser driving state, KL dark */

				/* Under-voltage or over-speed detected */
				if (s_i_uv || s_i_os) {
					s_fsm_state = 0x11;
					break;
				}

				/* Dropping the button allowance */
				if (!s_i_fb) {
					s_timer_fb = 0ULL;
				} else if ((s_i_fb && !s_i_uv && !s_i_os) && (!s_i_fb_t3 || s_i_uv_t2 || s_i_os_t2)) {
					/* Button just pressed - after repressing the button, wait until low-pass filtering time is over */
					s_timer_fb = now + C_FB_PRESS_SHORT_TIME;  // Short time is enough for the end points
				}

				/* Remote control button pressed and no under-voltage or over-speed detected */
				if (s_i_fb && !s_i_uv && !s_i_os && s_i_sa_g && !s_i_sa_o && s_i_sk_g && !s_i_sk_o && s_i_sh_g) {
					s_o_kl = true;

					/* Button pressed long enough, open valve for unlocking */
					if (s_timer_fb && (s_timer_fb <= now)) {
						s_o_pv_o	= true;
						s_timer_pv	= now + C_PV_ACTION_TIME;
						s_fsm_state = 0x21;
					}
				}
			}
			break;

			case 0x21:
			{
				/* GATE-OPENING UNLOCKING: valve for unlocking is opened for 0.5 sec */

				/* Release pressure of opening valve in case we block in this state */
				if (s_timer_pv && (s_timer_pv <= now)) {
					s_o_pv_o	= false;
				}

				/* Dropping the button allowance */
				if (!s_i_fb) {
					s_timer_fb = 0ULL;
				} else if (!s_i_fb_t3) {
					/* Button just pressed - after repressing the button, wait until low-pass filtering time is over */
					s_timer_fb = now + C_FB_PRESS_LONG_TIME;
				}

				/* Sensor reports being unlocked, start actor to open wings */
				if (s_i_sk_o && !s_i_sk_g && s_i_sh_g && s_i_fb && s_timer_fb && (s_timer_fb <= now)) {
					/* Combination for actor to OPEN wings */
					s_o_pv_o		= false;
					s_o_m1			= true;
					s_o_m2			= false;
					s_change_dir	= false;
					s_fsm_state		= 0x22;
				}
			}
			break;

			case 0x22:
			{
				/* GATE-OPENING MOTOR running: valve for unlocking is opened for 1 sec */

				/* Release pressure of opening valve */
				if (s_timer_pv && (s_timer_pv <= now)) {
					s_o_pv_o	= false;
				}

				/* Stop motor */
				if (s_i_sa_o) {
					s_o_m1		= false;
					s_o_m2		= false;
					s_fsm_state	= 0x23;

				} else {
					/* Button check */
					{
						/* Dropping the button allowance */
						if (!s_i_fb) {
							s_timer_fb = 0ULL;

							/* Stop motor at once and keep it off */
							s_o_m1			= false;
							s_o_m2			= false;

							if (s_i_fb_t3) {
								/* Button just released */
								s_change_dir = !s_change_dir;
							}

						} else if (!s_i_fb_t3) {
							/* Button just pressed - after repressing the button, wait until low-pass filtering time is over */
							s_timer_fb = now + C_FB_PRESS_LONG_TIME;
						}
					}

					/* Restart motor after qualified button press */
					if (s_i_fb && s_timer_fb && (s_timer_fb <= now)) {
						if (!s_change_dir) {
							/* Motor on, again */
							s_o_m1	= true;
							s_o_m2	= false;

						} else {
							/* Motor on, in opposite direction */
							s_o_m1			= false;
							s_o_m2			= true;
							s_change_dir	= false;
							s_fsm_state		= 0x41;
						}
					}
				}
			}
			break;

			case 0x23:
			{
				/* OPENING - WAIT FOR RELEASE: after motor stops, wait for button release */

				if (!s_i_fb) {
					s_fsm_state = 0x31;
				}
			}
			break;

			case 0x31:
			{
				/* STANDBY OPENED: wait for close command */

				/* Reset timer when button released or under-voltage detected within low-pass filtering time */
				if (!s_i_fb || s_i_uv) {
					s_timer_fb = 0ULL;
				} else {
					if ((s_i_fb && !s_i_uv) && (!s_i_fb_t3 || s_i_uv_t2)) {
						/* Button just pressed - after repressing the button, wait until low-pass filtering time is over */
						s_timer_fb = now + C_FB_PRESS_SHORT_TIME;
					}
				}

				/* When S-K.O has opened due to pressure release, re-open PV_O until contact is made again */
				if (s_i_fb && !s_i_uv && !s_i_sk_o) {
					s_o_pv_o = true;
				}

				/* Remote control button pressed and no under-voltage detected */
				if (s_i_fb && !s_i_uv && !s_i_sa_g && s_i_sa_o && !s_i_sk_g && s_i_sk_o && s_i_sh_g) {
					/* Button pressed long enough, move wings back to lock position */
					if (s_timer_fb && (s_timer_fb <= now)) {
						s_o_m1		= false;
						s_o_m2		= true;
						s_o_pv_o	= false;
						s_fsm_state = 0x41;
					}
				}
			}
			break;

			case 0x41:
			{
				/* MOVING WINGS BACK: motors running */

				/* Stop motor due to reaching end position and open valve for securing */
				if (s_i_sa_g) {
					s_o_m1		= false;
					s_o_m2		= false;
					s_timer_pv	= now + C_PV_ACTION_TIME;
					s_o_pv_g	= true;
					s_fsm_state	= 0x42;

				} else {
					/* Stop motor due to release of button */
					if (!s_i_fb) {
						s_timer_fb		= 0ULL;

						/* Stop motor at once and keep it off */
						s_o_m1			= false;
						s_o_m2			= false;

						if (s_i_fb_t3) {
							/* Button just released */
							s_change_dir = !s_change_dir;
						}

					} else {
						if (!s_i_fb_t3) {
							/* Button just pressed - after repressing the button, wait until low-pass filtering time is over */
							s_timer_fb = now + C_FB_PRESS_LONG_TIME;
						}

						/* When pressure releases during swing of arms, keep the valve open as long contact is opened */
						if (!s_i_sk_o) {
							s_o_pv_o	= true;
						} else {
							s_o_pv_o	= false;
						}
					}

					/* Restart motor after qualified button press */
					if (s_i_fb && s_timer_fb && (s_timer_fb <= now)) {
						if (!s_change_dir) {
							if (!s_i_sk_o) {
								/* Pressure has released, open valve to enable arms to be received */
								s_o_pv_o = true;

							} else {
								/* Motor on, again */
								s_o_m1		= false;
								s_o_m2		= true;
								s_o_pv_o	= false;
							}

						} else {
							/* Motor on, in opposite direction */
							s_o_m1			= true;
							s_o_m2			= false;
							s_change_dir	= false;
							s_fsm_state		= 0x22;
						}
					}
				}
			}
			break;

			case 0x42:
			{
				/* SECURING WINGS: valve for locking is opened for 1 sec */

				/* Release pressure of opening valve */
				if (s_timer_pv <= now) {
					s_o_pv_g	= false;
				}

				/* Locks at their position and timer done */
				if (s_i_sk_g) {
					s_o_pv_g	= false;
					s_o_kl		= false;
					s_fsm_state = 0x43;
				}
			}
			break;

			case 0x43:
			{
				/* PREPARING FOR STANDBY: wait for the button to be released */

				/* Button released, power off signal lamp and fall back to STANDBY */
				if (!s_i_fb) {
					s_fsm_state = 0x11;
				}
			}
			break;

			default:
				/* Jump to Init / error handling */
				s_fsm_state = 0x11;
		}
	}

	/* Assemble output signals */
	uint8_t l_port_b = 0b11000011 | (s_o_pv_g ?  _BV(5) : 0) | (s_o_pv_o ?  _BV(4) : 0) | (s_o_m1 ?  _BV(3) : 0) | (s_o_m2 ?  _BV(2) : 0);
	uint8_t l_port_c = 0b11110111 |                                                       (s_o_kl ?  _BV(3) : 0);
	uint8_t l_port_d = 0b11111111;

	/* Write out the new output vector */
	flags = cpu_irq_save();
	PORTB = l_port_b;
	PORTC = l_port_c;
	PORTD = l_port_d;
	g_o_kl = s_o_kl;
	cpu_irq_restore(flags);


	#if 0
	float l_adc_temp, l_adc_light;
	uint8_t l_portB, l_portC;
	irqflags_t flags;
	uint8_t l_SmartLCD_mode;
	uint8_t more;

	flags = cpu_irq_save();
	//l_adc_temp = g_adc_temp;
	//l_adc_light = g_adc_light;
	l_portB = PINB;
	l_portC = PINC;
	cpu_irq_restore(flags);

	/* Loops as long as more data is ready to be presented */
	do {
		more = 0;

		flags = cpu_irq_save();
		l_SmartLCD_mode = g_SmartLCD_mode;
		cpu_irq_restore(flags);

		/* Show received data from I2C bus */
		#if 0
		if (l_SmartLCD_mode == C_SMART_LCD_MODE_SMARTLCD) {
			more = lcd_show_new_smartlcd_data();

		} else if (l_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC) {
			more = lcd_show_new_refosc_data();
		}
		#endif

		#if 0
		lcd_cls();

		if (l_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC) {
			gfx_mono_generic_draw_rect(0, 0, 240, 128, GFX_PIXEL_SET);
			const char buf[] = "<==== LandCruiser-Controller ====>";
			gfx_mono_draw_string(buf, 3, 2, lcd_get_sysfont());
			lcd_show_template();
		}
		#endif
	} while (more);
	#endif
}

void enter_sleep(uint8_t sleep_mode)
{
	SMCR  = (sleep_mode << SM0)
		  | _BV(SE);											// Enable sleep command

	__asm__ __volatile__ ("sleep" ::: "memory");

	SMCR &= ~(_BV(SE));											// Disable sleep command
}


/* MAIN section */

void halt(void)
{
	/* MAIN Loop Shutdown */
	runmode = 0;
}

int main (void)
{
	uint8_t retcode = 0;

	/* Rapid I/O settings */
	s_io_preinit();

	/* Init of sub-modules */
	sysclk_init();
	sysclk_enable_module(POWER_RED_REG0, PRSPI_bm);				// For debugging this module has to be powered on, again

	ioport_init();
	s_tc_init();

	/* I/O pins go active here */
	board_init();

	/* Get boot reason code */
	reset_cause_t rc = reset_cause_get_causes();
	g_resetCause = rc;

	/* Reset all global vars */
	s_reset_global_vars();

	/* Read non-volatile settings */
	eeprom_nvm_settings_read(C_EEPROM_NVM_SETTING_ALL);			// Load all entries from NVM

	/* Serial terminal connection */
	serial_init(C_SERIAL_BAUD);

	/* I2C interface - 10 MHz-Ref-Osc. second display */
//	s_twi_init(TWI_SLAVE_ADDR_SMARTLCD, TWI_SLAVE_ADDR_BM);

	/* All interrupt sources prepared here - IRQ activation */
	cpu_irq_enable();

	/* Start of sub-modules */
	s_tc_start();												// All clocks and PWM timers start here

	/* Initialize external components */
//	lcd_init();
//	lcd_test(0b11111101);										// Debugging purposes

	/* Version information */
	serial_printLines(2);
	serial_printVersion();
	serial_printLines(1);

	/* main loop */
	runmode = 1;
    while (runmode) {
	    task(get_abs_time_ms());
	    enter_sleep(SLEEP_MODE_IDLE);
		led_set(true, g_led);
    }

	/* Shutdown external components */
//	lcd_shutdown();

	cpu_irq_disable();

	/* disable sub-modules */
	ACSR |= _BV(ACD);											// Disable AnalogCompare sub-module

//	sysclk_disable_module(POWER_RED_REG0, PRSPI_bm);
//	sysclk_disable_module(POWER_RED_REG0, PRUSART0_bm);

//	s_twi_disable();
	serial_disable();
	led_set(false, false);
	my_adc_disable();
	s_tc_disable();

    enter_sleep(SLEEP_MODE_PWR_DOWN);

    return retcode;												// Should never be reached
}
