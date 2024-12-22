#ifndef __uart_rtu_H
#define __uart_rtu_H

int send_data(uint8_t *buf, int len);
int receive_data(uint8_t *buf, int bufsz, int timeout, int bytes_timeout);
int uart_init(void);

#endif