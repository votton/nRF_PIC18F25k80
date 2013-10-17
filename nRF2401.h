#include "nRF2401_config.h"

#define MAX_PAYLOAD 32

unsigned char nrf_getStatus(void);
unsigned char nrf_send(unsigned char * tx_buf, unsigned char * rx_buf);
unsigned char nrf_recieve(unsigned char * rx_buf);

void nrf_init(void);
void nrf_txmode(void);
void nrf_rxmode(void);
void nrf_powerdown(void);

void nrf_setTxAddr(char addr);
void nrf_setRxAddr(char pipe, char addr);
