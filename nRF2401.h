#include "nRF2401_config.h"

//****************************************************/
// SPI(nRF24L01) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define ACTIVATE		0x50  // follow with 0x73 to activate feature register
#define R_RX_PL_WID     0x60  // Define RX payload length
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define WR_TX_PLOAD_NOACK     0xB0  // Define TX payload with no ACK 
#define W_ACK_PAYLOAD   0xA8  // Define ACK payload (default = Pipe0)
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NOP             0xFF  // Define No Operation, might be used to read status register
//***************************************************/
#define RX_DR    0x40
#define TX_DS    0x20
#define MAX_RT   0x10
#define PIPE_0   0x01
//*********	******************************************/
// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS_REG      0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
#define DYNPD    		0x1C  // per pipe DPL control
#define FEATURE    		0x1D  // 'Feature' register address



//nRF defines
//***************************************************/
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 4 unsigned chars TX payload


unsigned char SPI_RW(unsigned char);
unsigned char SPI_RW_Reg(unsigned char, unsigned char);
unsigned char SPI_Read(unsigned char);

unsigned char SPI_Read_Buf(unsigned char, unsigned char *, unsigned char);
unsigned char SPI_Write_Buf(unsigned char, unsigned char *, unsigned char);

#define NO_ACK			0x00
#define YES_ACK			0x01
#define NO_DATA			0x00
#define YES_DATA		0x01

void nrf_init(void);
unsigned char nrf_Send(unsigned char * tx_buf, unsigned char * rx_buf);
//unsigned char nrf_Send(void);
unsigned char nrf_Recieve(unsigned char * rx_buf);

void initTX(void);
void initRX(void);
unsigned char getStatus(void);
unsigned char write(unsigned char * /* Data */);
unsigned char read(unsigned char * /* Buffer */);






