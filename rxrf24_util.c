/* RX nRF24L01+ utility functions
 * Primary use is SPI I/O, and SPI port initialization.
 */

#include "iodefine.h"
#include "rxrf24_config.h"
#include <stdint.h>

/* USER-CONFIGURABLE FUNCTIONS
 * Tweak these as needed to set the correct port parameters.
 * e.g. if you choose to use the RSPI[0|1] A or B pins...
 */
void rxrf24_rspi_ports_enable()
{
	// RSPI1 pin choice "B"
	IOPORT.PFHSPI.BIT.RSPIS = 1;

	IOPORT.PFHSPI.BYTE |= 0x0E;  // MISO, MOSI, RSPCK enabled & connected to physical pins
	PORTE.DDR.BIT.B7 = 0;  // MISO = input
	PORTE.ICR.BIT.B7 = 1;

	PORTE.DDR.BIT.B6 = 1;  // MOSI output
	PORTE.DDR.BIT.B5 = 1;  // RSPCK output
}

void rxrf24_rspi_ports_disable()
{
	// Disconnects RSPI peripheral function only
	IOPORT.PFHSPI.BYTE &= ~0x0E;  // Disconnect MISO, MOSI, RSPCK
}

// NON-USER CONFIGURABLE (unless bitrate info needs tweaking)
void rxrf24_rspi_init()
{
	RXRF24_SPI.SPCR.BYTE = 0;
	RXRF24_SPI.SPPCR.BYTE = 0;
	RXRF24_SPI.SPBR = 2;  // 6MHz SPI, assuming PCLK=48MHz
	RXRF24_SPI.SPDCR.BYTE = 0;
	RXRF24_SPI.SPCKD.BYTE = 0;
	RXRF24_SPI.SSLND.BYTE = 0;
	RXRF24_SPI.SPND.BYTE = 0;
	RXRF24_SPI.SPCR2.BYTE = 0;
	RXRF24_SPI.SPCMD0.WORD = 0;
	RXRF24_SPI.SPCR.BIT.MSTR = 1;
	RXRF24_SPI.SPCR.BIT.SPE = 1;
	RXRF24_SPI.SPCR.BIT.SPMS = 1;  // Clock-synchronous operation, we control CS pin; not RSPI.

	rxrf24_rspi_ports_enable();
}

void rxrf24_rspi_delay(uint16_t us)
{
	uint32_t i, j, k;
	uint16_t spcmd0_save;
	uint8_t spdcr_save;

	rxrf24_rspi_ports_disable();  // Don't show our little strawman xfers over the wire
	spcmd0_save = RXRF24_SPI.SPCMD0.WORD;
	spdcr_save = RXRF24_SPI.SPDCR.BYTE;
	RXRF24_SPI.SPCMD0.WORD = 0;
	RXRF24_SPI.SPCMD0.BIT.SPB = 15;

	j = (us * 2) / 3;  // Assuming 18 12MHz pulses per 16-bit transfer
	for (i=0; i < j; i++) {
		RXRF24_SPI.SPDR.WORD.H = 0;
		while (!RXRF24_SPI.SPSR.BIT.SPRF)
			;
		k = RXRF24_SPI.SPDR.WORD.H;
		k++;   /* do something with 'k' so GCC quits bitching about setting
			* a variable but not doing anything with it
			*/
	}

	// Restore RSPI config
	RXRF24_SPI.SPDCR.BYTE = spdcr_save;
	RXRF24_SPI.SPCMD0.WORD = spcmd0_save;
	rxrf24_rspi_ports_enable();
}

uint8_t rxrf24_rspi_transfer(uint8_t data)
{
	RXRF24_SPI.SPCMD0.BIT.SPB = 7;
	RXRF24_SPI.SPDR.WORD.H = data;
	while (!RXRF24_SPI.SPSR.BIT.SPRF)
		;
	return (RXRF24_SPI.SPDR.WORD.H & 0xFF);
}

uint16_t rxrf24_rspi_transfer16(uint16_t data)
{
	RXRF24_SPI.SPCMD0.BIT.SPB = 15;
	RXRF24_SPI.SPDR.WORD.H = data;
	while (!RXRF24_SPI.SPSR.BIT.SPRF)
		;
	return (RXRF24_SPI.SPDR.WORD.H & 0xFFFF);
}

uint32_t rxrf24_rspi_transfer24(uint32_t data)
{
	RXRF24_SPI.SPCMD0.BIT.SPB = 1;
	RXRF24_SPI.SPDR.WORD.H = data;
	while (!RXRF24_SPI.SPSR.BIT.SPRF)
		;
	return (RXRF24_SPI.SPDR.WORD.H & 0xFFFFFF);
}

uint32_t rxrf24_rspi_transfer32(uint32_t data)
{
	RXRF24_SPI.SPCMD0.BIT.SPB = 2;
	RXRF24_SPI.SPDR.WORD.H = data;
	while (!RXRF24_SPI.SPSR.BIT.SPRF)
		;
	return RXRF24_SPI.SPDR.WORD.H;
}
