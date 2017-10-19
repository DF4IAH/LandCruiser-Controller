/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define SH_G_GPIO						IOPORT_CREATE_PIN(PORTB, 0)
#define XXXB1_GPIO						IOPORT_CREATE_PIN(PORTB, 1)
#define M2_GPIO							IOPORT_CREATE_PIN(PORTB, 2)
#define M1_GPIO							IOPORT_CREATE_PIN(PORTB, 3)
#define PV_O_GPIO						IOPORT_CREATE_PIN(PORTB, 4)
#define PV_G_GPIO						IOPORT_CREATE_PIN(PORTB, 5)
#define SK_G_GPIO						IOPORT_CREATE_PIN(PORTB, 6)
#define SK_O_GPIO						IOPORT_CREATE_PIN(PORTB, 7)

#define ADC_12V_GPIO					IOPORT_CREATE_PIN(PORTC, 0)
#define ADC1_GPIO						IOPORT_CREATE_PIN(PORTC, 1)
#define TACHO_GPIO						IOPORT_CREATE_PIN(PORTC, 2)
#define KL_GPIO							IOPORT_CREATE_PIN(PORTC, 3)
#define SDA_GPIO						IOPORT_CREATE_PIN(PORTC, 4)
#define SCL_GPIO						IOPORT_CREATE_PIN(PORTC, 5)

#define RXD_GPIO						IOPORT_CREATE_PIN(PORTD, 0)
#define TXD_GPIO						IOPORT_CREATE_PIN(PORTD, 1)
#define INT_GPIO						IOPORT_CREATE_PIN(PORTD, 2)
#define LED_GPIO						IOPORT_CREATE_PIN(PORTD, 3)
#define FB_GPIO							IOPORT_CREATE_PIN(PORTD, 4)
#define XXXD5_GPIO						IOPORT_CREATE_PIN(PORTD, 5)
#define SA_G_GPIO						IOPORT_CREATE_PIN(PORTD, 6)
#define SA_O_GPIO						IOPORT_CREATE_PIN(PORTD, 7)

#endif // CONF_BOARD_H
