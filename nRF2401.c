#include <xc.h>
#include "nRF2401.h"
#include "constants.h"

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
#define NRF_NOP             0xFF  // Define No Operation, might be used to read status register
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

#define nrf_SET 1
#define nrf_CLEAR 0
#define nrf_INPUT 1
#define nrf_OUTPUT 0

#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 4 unsigned chars TX payload
#define ACK_PAYLOAD 2

#define NO_ACK			0x00
#define YES_ACK			0x01
#define NO_DATA			0x00
#define YES_DATA		0x01

unsigned char TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x00}; // Define a static TX address

unsigned char nrf_SPI_RW(unsigned char);
unsigned char nrf_SPI_RW_Reg(unsigned char, unsigned char);
unsigned char nrf_SPI_Read(unsigned char);
unsigned char nrf_SPI_Read_Buf(unsigned char, unsigned char *, unsigned char);
unsigned char nrf_SPI_Write_Buf(unsigned char, unsigned char *, unsigned char);

unsigned char nrf_getStatus(void) {
    unsigned char status;
    CSN = nrf_CLEAR;
    SSPBUF = 0xFF;
    while(!SSPSTATbits.BF);
    status = SSPBUF;
    CSN = nrf_SET;
    return status;
}

unsigned char nrf_send(unsigned char * tx_buf, unsigned char * rx_buf) {
    char status;
    int i;

    nrf_SPI_RW_Reg(FLUSH_TX,0);
    
    status = nrf_getStatus();
    nrf_SPI_RW_Reg(WRITE_REG + STATUS_REG, status);	//nrf_CLEAR max RT bit
    nrf_SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH); //load the data into the NRF

    //wait for response
    CE = nrf_SET;
    for(i=0; i<2000; i++);
    CE = nrf_CLEAR;

    status = nrf_getStatus();
    if(status & RX_DR) {
        nrf_SPI_RW_Reg(WRITE_REG + STATUS_REG, RX_DR);		
        nrf_SPI_Read_Buf(RD_RX_PLOAD,rx_buf,ACK_PAYLOAD);
        nrf_SPI_RW_Reg(FLUSH_RX,0);
        return YES_ACK;
    } else {
        return NO_ACK;
    }
}

unsigned char nrf_recieve(unsigned char * tx_buf, unsigned char * rx_buf) {
    char status;
    char ffstat;
    char config;
    char ffstatcount;

    //------ load ACK payload data -------------

    nrf_SPI_RW_Reg(FLUSH_TX,0);
    nrf_SPI_Write_Buf(W_ACK_PAYLOAD,tx_buf,ACK_PAYLOAD);

    //config = nrf_SPI_Read(CONFIG);

    status = nrf_getStatus();
    ffstat = nrf_SPI_Read(FIFO_STATUS);

    if(((status & RX_DR))||(!(ffstat & 0x01))) {
        ffstatcount = 0;
        while((ffstatcount++ < 4) && ((ffstat & 0x01) == 0)) {
            //read entire buffer---------
            nrf_SPI_Read_Buf(RD_RX_PLOAD,rx_buf,32);
            ffstat = nrf_SPI_Read(FIFO_STATUS);
        }
        nrf_SPI_RW_Reg(WRITE_REG + STATUS_REG, 0x70);	//nrf_CLEAR all flags
        return YES_DATA;
    } else {
        return NO_DATA;
    }
}

void nrf_init(void) {
    char status;
    //Configure SPI for NRF
    // TODO set this up to work well with multiple devices on this SPI buffer
    SPI_STATUS = 0b00000000;	            //SPI, clock on idle to active clk trans
    SPI_CLK_EDGE = 1; 	                    //clock on active to idle clk trans
    SPI_CONFIG_1 = SPI_CONFIG_1_VALUE;	    //SPI nrf_SETup. clk at 1/16; idle low.
    SPI_CLK_POL = 0;	                    //clock polarity, idle low
    SPI_ENABLE = nrf_SET;	                //enable SPI module
    CE = nrf_SET;                           //default to Standby II, nrf_CLEAR to default to Standby I (which is low power mode; no TX/RX functions)
    CSN = nrf_SET;	


    CE = nrf_CLEAR;

    nrf_SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    nrf_SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device

    nrf_SPI_RW_Reg(ACTIVATE,0x73);					//activate feature register
    nrf_SPI_RW_Reg(WRITE_REG + FEATURE, 0x06);		//nrf_SET features for DPL
    nrf_SPI_RW_Reg(WRITE_REG + DYNPD, PIPE_0);		//enable DPL on pipe 0
    nrf_SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
    nrf_SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
	nrf_SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x12); // 500us + 86us, 2 retrans...
    nrf_SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
    nrf_SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
    nrf_SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR

    Delay10TCYx(3);

    nrf_SPI_RW_Reg(FLUSH_RX,0);
    nrf_SPI_RW_Reg(FLUSH_TX,0);

    status=nrf_SPI_Read(STATUS);
    nrf_SPI_RW_Reg(WRITE_REG + STATUS, status);

    CE = nrf_SET;
}

//Wait for 5ms after calling this before calling a send/receive
void nrf_rxmode(void) {
    CE = nrf_CLEAR;

    nrf_SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. RX_DR enabled..

    CE = nrf_SET;
}

//Wait for 5ms after calling this before calling a send/receive
void nrf_txmode(void) {
    CE = nrf_CLEAR;

    nrf_SPI_RW_Reg(WRITE_REG + CONFIG, 0x0E);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..

    CE = nrf_SET;
}

void nrf_powerdown(void) {
    CE = nrf_CLEAR;

    nrf_SPI_RW_Reg(WRITE_REG + CONFIG, 0x0C);     // Clear PWR_UP bit

    CE = nrf_SET;
}

void nrf_setTxAddr(char addr) {
    CE = nrf_CLEAR;
    Delay10TCYx(3);
    TX_ADDRESS[1] = addr;
    nrf_SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
    Delay10TCYx(3);
    CE = nrf_SET;
}

void nrf_setRxAddr(char pipe, char addr) {
    CE = nrf_CLEAR;
    Delay10TCYx(3);
    TX_ADDRESS[1] = addr;
    nrf_SPI_Write_Buf(WRITE_REG + RX_ADDR_P0 + pipe, TX_ADDRESS, TX_ADR_WIDTH);
    Delay10TCYx(3);
    CE = nrf_SET;
}







/////////////////////////////////////////////////
//  Internal Functions
/////////////////////////////////////////////////

/**************************************************
 * Function: nrf_SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char nrf_SPI_RW(unsigned char data) {
    SPI_BUFFER = data;
    while(!SPI_BUFFER_FULL_STAT);
    data = SPI_BUFFER;
    return(data);
}
/**************************************************/

/**************************************************
 * Function: nrf_SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
 * must be used along with the WRITE mask
**************************************************/
unsigned char nrf_SPI_RW_Reg(unsigned char reg, unsigned char value) {
    unsigned char status;

    CSN = nrf_CLEAR;                   // CSN low, init SPI transaction
    status = nrf_SPI_RW(reg);             // select register
    nrf_SPI_RW(value);                    // ..and write value to it..
    CSN = nrf_SET;                    // CSN high again

    return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: nrf_SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
**************************************************/
unsigned char nrf_SPI_Read(unsigned char reg) {
    unsigned char reg_val;

    CSN = nrf_CLEAR;                // CSN low, initialize SPI communication...
    nrf_SPI_RW(reg);                   // Select register to read from..
    reg_val = nrf_SPI_RW(0);           // ..then read register value
    CSN = nrf_SET;                  // CSN high, terminate SPI communication

    return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: nrf_SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
**************************************************/
unsigned char nrf_SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
    unsigned char status,i;

    CSN = nrf_CLEAR;                   // Set CSN low, init SPI tranaction
    status = nrf_SPI_RW(reg);       	    // Select register to write to and read status unsigned char

    for(i=0;i<bytes;i++) {
        pBuf[i] = nrf_SPI_RW(0xFF);    // Perform nrf_SPI_RW to read unsigned char from nRF24L01
    }

    CSN = nrf_SET;                   // Set CSN high again

    return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: nrf_SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
**************************************************/
unsigned char nrf_SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes) {
    unsigned char status,i;

    CSN = nrf_CLEAR;                   // Set CSN low, init SPI tranaction
    status = nrf_SPI_RW(reg);             // Select register to write to and read status unsigned char
    for(i=0;i<bytes; i++) {             // then write all unsigned char in buffer(*pBuf)
        nrf_SPI_RW(*pBuf);
        *pBuf++;
    }
    CSN = nrf_SET;                   // Set CSN high again
    return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/
