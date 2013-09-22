#include <p18f25k80.h>
#include "nRF2401.h"
#include "constants.h"
#include <delays.h>

unsigned char TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01}; // Define a static TX address


unsigned char SPI_RW(unsigned char);
unsigned char SPI_RW_Reg(unsigned char, unsigned char);
unsigned char SPI_Read(unsigned char);

//============ Status_nRF ===================================================
unsigned char getStatus(void) {
	unsigned char status;
	CSN = clear;
	SSPBUF = 0xFF;
	while(~SSPSTATbits.BF);
	status = SSPBUF;
	CSN = set;
	return status;
}

/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char SPI_RW(unsigned char data)
{
	SPI_BUFFER = data;
	while(!SPI_BUFFER_FULL_STAT);
	data = SPI_BUFFER;
	return(data);
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
 * must be used along with the WRITE mask
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  CSN = clear;                   // CSN low, init SPI transaction
  status = SPI_RW(reg);             // select register
  SPI_RW(value);                    // ..and write value to it..
  CSN = set;                    // CSN high again
	
  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  CSN = clear;                // CSN low, initialize SPI communication...
  SPI_RW(reg);                   // Select register to read from..
  reg_val = SPI_RW(0);           // ..then read register value
  CSN = set;                  // CSN high, terminate SPI communication

  return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  CSN = clear;                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0xFF);    // Perform SPI_RW to read unsigned char from nRF24L01
	//pBuf[0] = 0x77;
	//pBuf[1] = 0x88;
  }

  CSN = set;                   // Set CSN high again

  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  CSN = clear;                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf);
	*pBuf++;
  }
  CSN = set;                   // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/
/**************************************************
 * Function: nrf_init();
 * 
/**************************************************/
void nrf_init(void) {
	//===configure SPI for nordic RF module
	SPI_STATUS = 0b00000000;	//SPI, clock on idle to active clk trans
	SPI_CLK_EDGE = 1; 	//clock on active to idle clk trans
	SPI_CONFIG_1 = 0b00100001;	//SPI setup. clk at 1/16; idle low.
	SPI_CLK_POL = 0;	//clock polarity, idle low
	SPI_ENABLE = set;	//enable SPI module
	CE = set;  //default to Standby II, clear to default to Standby I (which is low power mode; no TX/RX functions)
	CSN = set;	
}

/**************************************************
 * Function: nrf_Send();
 * 
/**************************************************/
unsigned char nrf_Send(unsigned char * tx_buf, unsigned char * rx_buf) {
//unsigned char nrf_Send(void) {
	char status;
	//unsigned char tx_buf[TX_PLOAD_WIDTH];
	//volatile unsigned char rx_buf[32];

	//SPI_RW_Reg(FLUSH_TX,0);

	SPI_RW_Reg(WRITE_REG + STATUS_REG, MAX_RT);	//clear max RT bit
	SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH); //load the data into the NRF

	//wait for response
	CE = set;
	Delay1KTCYx(60);	
	CE = clear;

	status = getStatus();
	if(status & RX_DR) {
		SPI_RW_Reg(WRITE_REG + STATUS_REG, RX_DR);		
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,2);
		SPI_RW_Reg(FLUSH_RX,0);
		return YES_ACK;
	} else {
		return NO_ACK;
	}
}
/**************************************************/

/**************************************************
 * Function: nrf_Recieve();
 * 
/**************************************************/
unsigned char nrf_Recieve(unsigned char * rx_buf) {
	char status;
	char ffstat;
	unsigned char ACK_buf[2] = {0x12,0x34};
	unsigned char temp_buf[32];

	//------ load ACK payload data -------------
	//SPI_RW_Reg(FLUSH_TX,0); --DISABLED FOR SNOOPING--
	//SPI_Write_Buf(W_ACK_PAYLOAD,ACK_buf,2); --DISABLED FOR SNOOPING--

	// ----- get status for IRQ service ---------
	status = getStatus();
	ffstat = SPI_Read(FIFO_STATUS);

	if(((status & RX_DR))||(!(ffstat & 0x01))) {
		while((ffstat & 0x01) == 0) {
			//read entire buffer---------
			SPI_Read_Buf(RD_RX_PLOAD,rx_buf,32);
			ffstat = SPI_Read(FIFO_STATUS);
		}
		SPI_RW_Reg(WRITE_REG + STATUS_REG, RX_DR);	//clear RX flag
		return YES_DATA;
	} else {
		return NO_DATA;
	}
}

/**************************************************/


/**************************************************
 * Function: initRX();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * RX Mode, set RX address, writes RX payload width,
 * select RF channel, datarate & LNA HCURR.
 * After init, CE is toggled high, which means that
 * this device is now ready to receive a datapacket.
/**************************************************/
void initRX(void) {
	unsigned char status;

	CE = clear;

	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device

	SPI_RW_Reg(ACTIVATE,0x73);					//activate feature register
	SPI_RW_Reg(WRITE_REG + FEATURE, 0x06);		//set features for DPL
	SPI_RW_Reg(WRITE_REG + DYNPD, PIPE_0);		//enable DPL on pipe 0

	//SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0 --DISABLED FOR SNOOPING--
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
	
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. RX_DR enabled..
	Delay10TCYx(3);
	
	CE = set;
	
	//  This device is now ready to receive one packet of 32 unsigned chars payload from a TX device sending to address
	//  '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 1Mbps.

	SPI_RW_Reg(FLUSH_RX,0);
	status=SPI_Read(STATUS);
	SPI_RW_Reg(WRITE_REG + STATUS, status);

}
/**************************************************/

/**************************************************
 * Function: initTX();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 * 
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
void initTX(void)
{
	unsigned char tx_buf[TX_PLOAD_WIDTH];
	unsigned char status=0;
	unsigned char key=0;
	unsigned char config_reg;
	
	CE = clear;

	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack	
	
	SPI_RW_Reg(ACTIVATE,0x73);					//activate feature register
	SPI_RW_Reg(WRITE_REG + FEATURE, 0x06);		//set features for DPL
	SPI_RW_Reg(WRITE_REG + DYNPD, PIPE_0);		//enable DPL on pipe 0	
	
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x3F); // 1000us + 86us, 15 retrans...
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR

	SPI_RW_Reg(FLUSH_TX,0);
	status=SPI_Read(STATUS);
	SPI_RW_Reg(WRITE_REG + STATUS, status);
	SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);

	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0E);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
	Delay10TCYx(3);

	CE = set;

	//LCD_cmd(LINE_1);
	//LCD_putch('T'); LCD_putch('X'); LCD_putch('.'); LCD_putch('.'); 

	//--- fill buffer with dummy data ------
    for(key=0;key<32;key++) {
		tx_buf[key] = 0xA0+key;
	}

}
