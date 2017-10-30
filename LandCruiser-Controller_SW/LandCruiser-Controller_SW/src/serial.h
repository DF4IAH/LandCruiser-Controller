/*
 * serial.h
 *
 * Created: 28.10.2017 15:38:00
 *  Author: DF4IAH
 */


#ifndef SERIAL_H_
#define SERIAL_H_


#define min(a,b)  ((a)<(b) ?  (a):(b))

#define C_SERIAL_BAUD				38400
#define C_SERIAL_RX_BUF_SIZE		32
#define C_SERIAL_TX_BUF_SIZE		256


void serial_init(uint32_t baud);
void serial_disable(void);

void serial_send(const char* buf, uint16_t len);
void serial_receive_and_parse_line(void);

void serial_printCmd(void);
void serial_printHelp(void);
void serial_printVersion(void);
void serial_printFsmState(uint8_t stateOld, uint8_t stateNew);
void serial_printFsmInput(bool Ifb, bool Iuv, bool Ios, bool Iskg, bool Isko, bool Isag, bool Isao, bool Ishg);


#endif /* SERIAL_H_ */
