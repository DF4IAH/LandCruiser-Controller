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
#include <avr/eeprom.h>

#include "isr.h"
//#include "twi.h"
//#include "lcd.h"
//#include "gfx_mono/sysfont.h"

#include "main.h"


/* GLOBAL section */

uint_fast64_t		g_timer_abs_msb						= 0;
//uint8_t			g_adc_state							= 0;
//bool				g_lcdbl_auto						= true;
//uint8_t			g_lcdbl_dimmer						= 0;
//uint8_t			g_lcd_contrast_pm					= 0;
showData_t			g_showData							= { 0 };
uint8_t				g_SmartLCD_mode						= C_SMART_LCD_MODE_UNIQUE;
//gfx_mono_color_t	g_lcd_pixel_type					= GFX_PIXEL_CLR;
//gfx_coord_t		g_lcd_pencil_x						= 0;
//gfx_coord_t		g_lcd_pencil_y						= 0;
uint8_t				g_resetCause						= 0;
//char				g_strbuf[48]						= { 0 };


/* MAIN STATIC section */

static uint8_t		runmode								= 0;			// static runmode of main.c



/* HELPERS */

static void s_reset_global_vars(void)
{
	irqflags_t flags	= cpu_irq_save();
	cpu_irq_disable();

	g_timer_abs_msb		= 0UL;
//	g_adc_state			= ADC_STATE_PRE_LDR;
//	g_lcdbl_dimmer		= 64;

	cpu_irq_restore(flags);
}


/* INIT section */

static void s_io_preinit(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts,
	 * most critical pins are handled first.
	 */

	PORTB = 0b00000000;								// in:  PB0: NC, PB1: NC, PB6: MP_SK_G, PB7: MP_SK_O
	DDRB  = 0b00111100;								// out: PB2: MP_M2, PB3: MP_M1, PB4: MP_PV_O, PB5: MP_PV_G

	PORTC = 0b00100000;								// in:  PC0: NC, PC1: NC, PC2: NC, PC4: SDA, PC6: NC, PC7: NC
	DDRC  = 0b00101000;								// out: PC3: MP_KL, PC5: SCL

	PORTD = 0b00000010;								// in:  PD0: MP_RXD, PD2: MP_INT, PD3: NC, PD4: MP_FB, PD5: NC, PD6: MP_SA_G, PD7: MP_SA_O
	DDRD  = 0b00000010;								// out: PD1: MP_TXD

	// Analog input: Digital Disable Register
	DIDR0 = 0b00000111;								// PC0: NC_A, PC1: NC_A, PC2: NC_A
}

static void s_tc_init(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts. */

	#if 0
	/* Timer Synchronous Mode - prepare for  s_tc_start(void) */
	GTCCR = _BV(TSM)								// Timer Synchronous Mode active
		  | _BV(PSRASY)								// Timer 2   prescaler is synced
		  | _BV(PSRSYNC);							// Timer 0/1 prescaler is synced
	#endif

	#if 0
	/* TC0 - Overflows with about 122 Hz for the ADC convertion */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM0_bm);

		TCCR0A  = (0b00  << COM0A0)					// Normal port operations
				| (0b00  << WGM00);					// Counter mode

		TCCR0B  = ( 0b0  << WGM02)
				| (0b100 << CS00);					// CLKio DIV 256 = 31250 Hz --> / 2**8 = 122 Hz looping rate

		TCNT0   = 0;								// Clear current value

		OCR0A   = 0x00;								// Compare value not used

		TIFR0   = 0b00000111;						// Clear all flags
		TIMSK0  = _BV(TOIE0);						// TOIE0: overflow interrupt
	}
	#endif

	/* TC1 global clock - OC1A: NC */
	{
		const uint16_t c_TC1_TOP_VAL = 8000U;

		sysclk_enable_module(POWER_RED_REG0, PRTIM1_bm);

		TCCR1A  = (0b00  << COM1A0)		 			// OC1A: disconnected - normal port function
				| (0b01  << WGM10);					// WGM: 0b1001 = PWM, Phase and Frequency Correct / Update on BOTTOM / TOP @ OCR1A

		TCCR1B  = ( 0b10 << WGM12)
				| (0b001 << CS10);					// CLKio DIV1, no prescaling --> 8 MHz

		TCNT1H  = 0b00000000           ;			// Clear current value for synchronous start (when restarting without reset)
		barrier();
		TCNT1L	=            0b00000000;

		OCR1AH  = (uint8_t) (c_TC1_TOP_VAL >> 8);	// 1 kHz overflow frequency (1 ms)
		barrier();
		OCR1AL  = (uint8_t) (c_TC1_TOP_VAL & 0xff);

		TIFR1   = 0b00100111;						// Clear all flags (when restarting without reset)
		TIMSK1  = _BV(TOIE1);						// TOIE1 interrupt
	}

	#if 0
	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM2_bm);

		ASSR    = 0;								// No async. TOSC1 mode

		TCCR2A  = (0b10  << COM2A0)					// HI --> LO when compare value is reached - non-inverted PWM mode
				| (0b11  << WGM20);					// WGM: 0b011 = Fast PWM mode 8 bit

		TCCR2B  = ( 0b0  << WGM22)
				| (0b101 << CS20);					// CLKio DIV 128 = 62500 Hz --> / 2**8 = 244 Hz looping rate

		TCNT2   = 0;								// Clear current value for synchronous start (when restarting without reset)

		OCR2A   = 0x00;								// LCD backlight dimmed down

		TIFR2   = 0b00000111;						// Clear all flags
		TIMSK2  = _BV(TOIE2);						// TOIE2: overflow interrupt
	}
	#endif
}

static void s_tc_start(void)
{
	#if 0
	/* TC0: Overflows with about 30 Hz for the ADC convertion */
	/* TC1: Audio output @ 16-bit counter PWM, used: 10-bit resolution */
	/* TC2: LCD backlight w/ 8-bit resolution */
	{
		/* Timer Synchronous Mode - trigger */
		GTCCR = 0;								// trigger the sync for all counters
	}
	#endif
}

static void s_tc_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	#if 0
	/* TC0 - Overflows with about 122 Hz for the ADC convertion */
	{
		TIMSK0  = 0;							// no interrupts

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

		TIMSK1  = 0;							// no interrupts

		TCCR1A  = 0;							// release alternate port function
		TCCR1B  = 0;
		TCCR1C  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM1_bm);
	}

	#if 0
	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		//ioport_set_pin_dir(LCDBL_PWM, IOPORT_DIR_OUTPUT);
		//ioport_set_pin_level(LCDBL_PWM, false);	// turn backlight off

		TIMSK2  = 0;							// no interrupts

		ASSR    = 0;							// no async TOSC1 mode

		TCCR2A  = 0;							// release alternate port function
		TCCR2B  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM2_bm);
	}
	#endif

	sysclk_set_prescalers(SYSCLK_PSDIV_256);

	cpu_irq_restore(flags);
}


#if 0
static void s_adc_init(void)
{
	sysclk_enable_module(POWER_RED_REG0, PRADC_bm);	// enable ADC sub-module

	adc_disable_digital_inputs(_BV(ADC0D));		// disable the digital input on the ADC0 port

	adc_init(ADC_PRESCALER_DIV64);
	adc_set_admux(ADC_MUX_ADC0 | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);

	#if 1
	/* ADC is started by TC0 timer directly - disadvantage: lower amplitude precision */
	adc_set_autotrigger_source(ADC_AUTOTRIGGER_SOURCE_TC0_OVERFLOW);
	adc_enable_autotrigger();
	#else
	adc_disable_autotrigger();
	#endif

	ADCSRA |= _BV(ADIF);						// clear interrupt status bit by setting it to clear
	adc_enable_interrupt();						// enable the ADC interrupt

	adc_start_conversion();						// initial convertion doing an ADC set-up
}

static void s_adc_disable(void)
{
	adc_disable_interrupt();					// disable the ADC interrupt
	adc_disable_autotrigger();
	adc_set_autotrigger_source(0);
	adc_set_admux(0);
	//adc_disable_digital_inputs(0);

	sysclk_disable_module(POWER_RED_REG0, PRADC_bm);	// disable ADC sub-module
}
#endif

#if 0
static void s_twi_init(uint8_t twi_addr, uint8_t twi_addr_bm)
{
	sysclk_enable_module(POWER_RED_REG0, PRTWI_bm);

	irqflags_t flags = cpu_irq_save();

	TWSR = (0b00 << TWPS0);						// Prescaler value = 1
	TWBR = 2;									// TWI bit-rate = 400 kBit/sec @ 8 MHz when master mode active

	TWAR  = (twi_addr    << 1) /* | (TWI_SLAVE_ADDR_GCE << TWGCE)*/ ;
	TWAMR = (twi_addr_bm << 1);

	TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// Enable Acknowledge, Enable TWI port, Interrupt Enable, no START or STOP bit

	cpu_irq_restore(flags);
}

static void s_twi_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	TWCR = _BV(TWEN);							// disable the interrupt source

	ioport_set_pin_dir(SDA_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SDA_GPIO, IOPORT_MODE_PULLUP);

	ioport_set_pin_dir(SCL_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SCL_GPIO, IOPORT_MODE_PULLUP);

	TWCR = 0;									// disable the TWI port

	cpu_irq_restore(flags);

	sysclk_disable_module(POWER_RED_REG0, PRTWI_bm);
}
#endif


/* UTILITIES section */

float get_abs_time(void)
{
	const float ticks_per_sec = 8e6f;  // TODO: recalc
	float now;						// TODO: no float
	uint8_t l_tmr_l;
	uint8_t l_tmr_h;
	uint_fast32_t l_tmr_msb;

	irqflags_t flags = cpu_irq_save();
	l_tmr_l = TCNT1L;
	l_tmr_h = TCNT1H;
	l_tmr_msb = g_timer_abs_msb;
	cpu_irq_restore(flags);

	/* calculate ticks to sec */
	now  = ((l_tmr_h << 8) | l_tmr_l  ) / ticks_per_sec;
	now += (        512.f  * l_tmr_msb) / ticks_per_sec;
	return now;
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
	//g_lcd_contrast_pm = C_LCD_PM;				// preset value
	if (flags & C_EEPROM_NVM_SETTING_LCD_CONTRAST) {
		uint8_t val = eeprom_read_byte((const uint8_t *) C_EEPROM_ADDR_LCD_PM);
		if (val <= 0x3F) {						// value from NVM is marked as being valid
			g_lcd_contrast_pm = val;
		} else {
			eeprom_nvm_settings_write(C_EEPROM_NVM_SETTING_LCD_CONTRAST);
		}
	}
	#endif
}


/* TASK section */

void task(float timestamp)
{
	/* TASK when woken up */
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
			/* Come up with the data presenter for the 10 MHz-Ref.-Osc. */
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
		  | _BV(SE);							// enable sleep command

	__asm__ __volatile__ ("sleep" ::: "memory");

	SMCR &= ~(_BV(SE));							// disable sleep command
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
	sysclk_init();	PRR = 0b11101011;			// For debugging this module has to be powered on, again
	ioport_init();
//	s_adc_init();
	s_tc_init();

	/* I/O pins go active here */
	board_init();

	reset_cause_t rc = reset_cause_get_causes();
	g_resetCause = rc;
	if (rc & CHIP_RESET_CAUSE_EXTRST	||
		rc & CHIP_RESET_CAUSE_BOD_CPU	||
		rc & CHIP_RESET_CAUSE_POR		||
		!rc) {
		s_reset_global_vars();
	} else {
		/* DEBUG */
		asm_break();
	}

	/* Read non-volatile settings */
	eeprom_nvm_settings_read(C_EEPROM_NVM_SETTING_ALL);			// load all entries from NVM

	/* I2C interface - 10 MHz-Ref-Osc. second display */
//	s_twi_init(TWI_SLAVE_ADDR_SMARTLCD, TWI_SLAVE_ADDR_BM);

	/* All interrupt sources prepared here - IRQ activation */
	cpu_irq_enable();

	/* Start of sub-modules */
	s_tc_start();								// All clocks and PWM timers start here

	/* Initialize external components */
//	lcd_init();
//	lcd_test(0b11111101);						// Debugging purposes


	/* main loop */
	runmode = 1;
    while (runmode) {
	    task(get_abs_time());
	    enter_sleep(SLEEP_MODE_IDLE);
    }


	/* Shutdown external components */
//	lcd_shutdown();

	cpu_irq_disable();

	/* disable sub-modules */
	ACSR |= _BV(ACD);							// disable AnalogCompare sub-module

//	sysclk_disable_module(POWER_RED_REG0, PRSPI_bm);
//	sysclk_disable_module(POWER_RED_REG0, PRUSART0_bm);

//	s_twi_disable();
//	s_adc_disable();
	s_tc_disable();

    enter_sleep(SLEEP_MODE_PWR_DOWN);

    return retcode;								// should never be reached
}
