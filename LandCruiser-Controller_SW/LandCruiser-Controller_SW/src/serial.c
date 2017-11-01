/**
 * \file
 *
 * \brief Serial Terminal Routines
 *
 */

/**
 * \mainpage serial
 *
 * \par Serial Terminal Routines
 *
 * This module contains functions for communication with the serial interface.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Functions for handling the serial communication
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

#include <string.h>
#include <stdio.h>

#include "main.h"

#include "serial.h"


/* External vars */

#include "externals.h"


const char								PM_IP_OUT_NewLine[]										= "\r\n";
PROGMEM_DECLARE(const char, PM_IP_OUT_NewLine[]);
const char								PM_IP_OUT_CmdLine[]										= "\r\n>";
PROGMEM_DECLARE(const char, PM_IP_OUT_CmdLine[]);
const char								PM_IP_OUT_help_1[]										= "help\t\tThis information page ";
PROGMEM_DECLARE(const char, PM_IP_OUT_help_1[]);
const char								PM_IP_OUT_help_2[]										=	"about all available commands\r\n";
PROGMEM_DECLARE(const char, PM_IP_OUT_help_2[]);


const char								PM_IP_CMD_help[]										= "help";
PROGMEM_DECLARE(const char, PM_IP_CMD_help[]);


const char								PM_DBG_Version[]										= "\r\nLandCruiser-Control version 20%03d%03d\r\n=====================================\r\n\r\n";
PROGMEM_DECLARE(const char, PM_DBG_Version[]);
const char								PM_DBG_FSM_State[]										= "# FSM- State: 0x%02X --> 0x%02X\r\n";
PROGMEM_DECLARE(const char, PM_DBG_FSM_State[]);
const char								PM_DBG_FSM_Input[]										= "# FSM- Input: Ifb=%d, Iuv=%d, Ios=%d, Iskg=%d, Isko=%d, Isag=%d, Isao=%d, Ishg=%d\r\n";
PROGMEM_DECLARE(const char, PM_DBG_FSM_Input[]);
const char								PM_DBG_FSM_Output[]										= "# FSM-Output: Okl=%d, Opvg=%d, Opvo=%d, Om1=%d, Om2=%d\r\n";
PROGMEM_DECLARE(const char, PM_DBG_FSM_Output[]);


/* Forward declarations */

//static void s_not_yet_defined(void);


/* Static functions */
inline
uint16_t s_baud_to_ubrr(uint32_t baud)
{
	const uint32_t fosc = 8000000UL;  // 8 MHz
	return (fosc / (baud << 4)) - 1;
}


/* Serial Terminal Functions */

void serial_init(uint32_t baud)
{
	/* Port definitions TX line */
	ioport_set_pin_dir(TXD_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(TXD_GPIO, IOPORT_PIN_LEVEL_HIGH);

	/* Port definitions RX line */
	ioport_set_pin_dir(RXD_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(RXD_GPIO, IOPORT_MODE_PULLUP);

	/* Enable USART0 sub-module */
	sysclk_enable_module(POWER_RED_REG0, PRUSART0_bm);

	/* Baud generator */
	uint16_t ubrr_val = s_baud_to_ubrr(baud);
	UBRR0H = (uint8_t)((ubrr_val >> 8) & 0x0f);
	UBRR0L = (uint8_t)  ubrr_val;

	/* Frame format: UART 8N1 */
	UCSR0C = (0 << UMSEL00) | (0 << UPM00) | (0 << USBS0) | (3 << UCSZ00);

	/* Status clear */
	UCSR0A = _BV(TXC0);

	/* Enabling Interrupts and TX and RX operations */
	UCSR0B = _BV(RXCIE0) | _BV(TXCIE0) | _BV(RXEN0) | _BV(TXEN0);
}

void serial_disable(void)
{
	/* Disabling Interrupts and TX and RX operations */
	UCSR0B = 0;
}


void serial_send(const char* buf, uint16_t len)
{
	irqflags_t flags = cpu_irq_save();

	/* Clip message when oversized */
	if (len >= (C_SERIAL_TX_BUF_SIZE - 1)) {
		len = C_SERIAL_TX_BUF_SIZE - 1;
	}

	while ((g_serial_tx_len + len + 1) >= C_SERIAL_TX_BUF_SIZE) {
		cpu_irq_restore(flags);

		/* Turn MCU in sleep mode until waken up by an interrupt */
		enter_sleep(SLEEP_MODE_IDLE);

		flags = cpu_irq_save();
	}

	/* Copy tail to buffer */
	memcpy(g_serial_tx_buf + g_serial_tx_len, buf, len);
	g_serial_tx_len += len;
	g_serial_tx_buf[g_serial_tx_len] = 0;

	/* Start transmission, when not running */
	if (UCSR0A & _BV(UDRE0)) {
		/* Start transmission */
		UDR0 = *buf;
	}

	cpu_irq_restore(flags);
}

void serial_receive_and_parse_line(void)
{
	uint8_t	cmdLine_buf[C_SERIAL_RX_BUF_SIZE];

	irqflags_t flags = cpu_irq_save();
	bool eol = g_serial_rx_eol;
	if (eol) {
		memcpy(cmdLine_buf, g_serial_rx_buf, g_serial_rx_idx);
		g_serial_rx_buf[0]	= 0;
		g_serial_rx_idx		= 0;
		g_serial_rx_eol		= false;
	}
	cpu_irq_restore(flags);

	/* Process when a complete line is in buffer */
	if (eol) {
		if (!strncasecmp_P((char*)cmdLine_buf, PM_IP_CMD_help, sizeof(PM_IP_CMD_help) - 1)) {
			serial_printHelp();
		} else {
			serial_printCmd();
		}
	}
}

void serial_printCmd(void)
{
	int len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_IP_OUT_CmdLine);
	serial_send(g_strbuf, len);
}

void serial_printHelp(void)
{
	int len;

	len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_IP_OUT_help_1);
	serial_send(g_strbuf, len);
	len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_IP_OUT_help_2);
	serial_send(g_strbuf, len);

	len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_IP_OUT_NewLine);
	serial_send(g_strbuf, len);
	len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_IP_OUT_CmdLine);
	serial_send(g_strbuf, len);
}

void serial_printVersion(void)
{
	int len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_DBG_Version, VERSION_HIGH, VERSION_LOW);
	serial_send(g_strbuf, len);
}

void serial_printFsmState(uint8_t stateOld, uint8_t stateNew)
{
	int len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_DBG_FSM_State, stateOld, stateNew);
	serial_send(g_strbuf, len);
}

void serial_printFsmInput(bool Ifb, bool Iuv, bool Ios, bool Iskg, bool Isko, bool Isag, bool Isao, bool Ishg)
{
	int len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_DBG_FSM_Input, Ifb, Iuv, Ios, Iskg, Isko, Isag, Isao, Ishg);
	serial_send(g_strbuf, len);
}

void serial_printFsmOutput(bool Okl, bool Opvg, bool Opvo, bool Om1, bool Om2)
{
	int len = snprintf_P(g_strbuf, sizeof(g_strbuf), PM_DBG_FSM_Output, Okl, Opvg, Opvo, Om1, Om2);
	serial_send(g_strbuf, len);
}
