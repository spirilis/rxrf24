/* RX nRF24L01+ library
 */

#include "iodefine.h"
#include "rxrf24_config.h"
#include "rxrf24.h"
#include <stdint.h>
#include <stdlib.h>

rxrf24_t nrf24;  // Global transceiver configuration

/* Port I/O function setting--required before rxrf24_init() can be called */
void rxrf24_register_portcallback(portIOfunc csninit, portIOfunc csn,
				  portIOfunc ceinit, portIOfunc ce)
{
	nrf24.chipselect_init = csninit;
	nrf24.chipselect = csn;
	nrf24.chipenable_init = ceinit;
	nrf24.chipenable = ce;
}

/* Initialize rxrf24 library and load initial config state into transceiver. */
const uint8_t txaddr_default[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t rxaddr1_default[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
const uint8_t rxaddr2_default[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC3};
const uint8_t rxaddr3_default[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC4};
const uint8_t rxaddr4_default[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC5};
const uint8_t rxaddr5_default[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC6};

uint8_t rxrf24_init()
{
	// Do we have valid functions for chipselect and chipenable?
	if (nrf24.chipselect_init == NULL || nrf24.chipselect == NULL)
		return 0;
	if (nrf24.chipenable_init == NULL || nrf24.chipenable == NULL)
		return 0;

	nrf24.chipselect_init(1);
	nrf24.chipselect(1);  // CSN = HIGH (inactive)
	nrf24.chipenable_init(1);
	nrf24.chipenable(0);  // CE = LOW (inactive)

	rxrf24_flush_tx();
	rxrf24_flush_rx();
	rxrf24_set_option(RF24_EN_DPL);
	rxrf24_set_crc(nrf24.crc.BIT.en, nrf24.crc.BIT.crc16);
	rxrf24_set_channel(nrf24.channel);
	nrf24.rfsetup.BYTE = 0x00;
	rxrf24_set_txpower(0);
	if (nrf24.address_width < 3 || nrf24.address_width > 5)
		rxrf24_set_address_width(5);
	else
		rxrf24_set_address_width(nrf24.address_width);
	rxrf24_set_retrans_params(2000, 15);
	if (nrf24.speed != 250000 && nrf24.speed != 1000000 && nrf24.speed != 2000000)
		rxrf24_set_speed(1000000);
	else
		rxrf24_set_speed(nrf24.speed);
	rxrf24_close_pipe_all();
	rxrf24_set_txaddr((uint8_t*)txaddr_default);
	rxrf24_set_rxaddr(0, (uint8_t*)txaddr_default);
	rxrf24_set_rxaddr(1, (uint8_t*)rxaddr1_default);
	rxrf24_set_rxaddr(2, (uint8_t*)rxaddr2_default);
	rxrf24_set_rxaddr(3, (uint8_t*)rxaddr3_default);
	rxrf24_set_rxaddr(4, (uint8_t*)rxaddr4_default);
	rxrf24_set_rxaddr(5, (uint8_t*)rxaddr5_default);

	// All ready to go.
	return 1;
}

uint8_t rxrf24_read_reg(uint8_t reg)
{
	uint8_t val;

	nrf24.chipselect(0);
	val = rxrf24_rspi_transfer16(RF24_16_R_REGISTER | (reg << 8) | RF24_NOP) & 0x00FF;
	nrf24.chipselect(1);
	return val;
}

void rxrf24_write_reg(uint8_t reg, uint8_t val)
{
	nrf24.chipselect(0);
	rxrf24_rspi_transfer16(RF24_16_W_REGISTER | (reg << 8) | val);
	nrf24.chipselect(1);
}

void rxrf24_set_channel(uint8_t channel)
{
	if (channel > 125)
		channel = 125;
	nrf24.channel = channel;
	rxrf24_write_reg(RF24_RF_CH, channel);
}

void rxrf24_set_speed(uint32_t speed)
{
	switch (speed) {
		case 250000:
			nrf24.rfsetup.BYTE = (nrf24.rfsetup.BYTE & 0xD7) | 0x20;
			nrf24.speed = 250000;
			break;

		case 2000000:
			nrf24.rfsetup.BYTE = (nrf24.rfsetup.BYTE & 0xD7) | 0x08;
			nrf24.speed = 2000000;
			break;

		default:		// Default to 1Mbps if the value isn't a correct specifier.
			nrf24.rfsetup.BYTE = nrf24.rfsetup.BYTE & 0xD7;
			nrf24.speed = 1000000;
			break;
	}
	rxrf24_write_reg(RF24_RF_SETUP, nrf24.rfsetup.BYTE);
}

void rxrf24_set_txpower(int8_t dbm)
{
	switch (dbm) {
		case 0:
			nrf24.rfsetup.BIT.rfpwr = 0x03;
		case -6:
			nrf24.rfsetup.BIT.rfpwr = 0x02;
		case -12:
			nrf24.rfsetup.BIT.rfpwr = 0x01;
		default:		// Default to -18dBm if the value isn't a correct specifier.
			nrf24.rfsetup.BIT.rfpwr = 0x00;
	}
	rxrf24_write_reg(RF24_RF_SETUP, nrf24.rfsetup.BYTE);
}

void rxrf24_set_retrans_params(uint16_t delayUS, uint8_t retries)
{
	uint8_t reg;

	if (delayUS > 4000)
		delayUS = 4000;
	reg = retries & 0x0F;
	reg |= (((delayUS - 250) / 250) & 0x0F) << 4;
	rxrf24_write_reg(RF24_SETUP_RETR, nrf24.rfsetup.BYTE);
}

void rxrf24_set_address_width(uint8_t aw)
{
	if (aw < 3)
		aw = 3;
	if (aw > 5)
		aw = 5;
	nrf24.address_width = aw;
	rxrf24_write_reg(RF24_SETUP_AW, (aw-2));
}

void rxrf24_set_crc(uint8_t onoff, uint8_t crc16bit)
{
	uint8_t cfg, i;

	nrf24.crc.BIT.en = onoff & 0x01;
	nrf24.crc.BIT.crc16 = crc16bit & 0x01;
	cfg = rxrf24_read_reg(RF24_CONFIG);
	cfg &= 0xF3;
	cfg |= nrf24.crc.BYTE;
	rxrf24_write_reg(RF24_CONFIG, cfg);
}

void rxrf24_set_option(uint8_t feature)
{
	rxrf24_write_reg(RF24_FEATURE, feature);
}

/* Address management */
// Note that all addresses are loaded LSByte first (we assume the buf array is stored with MSByte first at position 0)
void rxrf24_set_txaddr(void *buf)
{
	// Uses address_width to determine size of buf
	uint8_t i;

	nrf24.chipselect(0);
	rxrf24_rspi_transfer16(RF24_16_W_REGISTER | RF24_16_TX_ADDR | ((uint8_t *)buf)[nrf24.address_width-1]);
	for (i=1; i<nrf24.address_width; i++)
		rxrf24_rspi_transfer( ((uint8_t *)buf)[nrf24.address_width-1-i] );
	nrf24.chipselect(1);
}

void rxrf24_set_rxaddr(uint8_t pipeid, void *buf)
{
	uint8_t i;

	if (pipeid > 5)
		return;

	nrf24.chipselect(0);
	if (pipeid < 2) {  // With pipes 2-5, only load the LSByte.
		rxrf24_rspi_transfer16( (RF24_16_W_REGISTER | RF24_16_RX_ADDR_P0 | ((uint8_t *)buf)[nrf24.address_width-1]) + (pipeid << 8) );
		for (i=1; i<nrf24.address_width; i++)
			rxrf24_rspi_transfer( ((uint8_t *)buf)[nrf24.address_width-1-i] );
	} else {
		rxrf24_rspi_transfer16( (RF24_16_W_REGISTER | RF24_16_RX_ADDR_P0 | ((uint8_t *)buf)[nrf24.address_width-1]) + (pipeid << 8) );
	}
	nrf24.chipselect(1);
}

/* Pipe management */
void rxrf24_open_pipe(uint8_t pipeid, uint8_t autoack)
{
	uint8_t rxen, enaa;

	if (pipeid > 5)
		return;

	rxen = rxrf24_read_reg(RF24_EN_RXADDR);
	enaa = rxrf24_read_reg(RF24_EN_AA);

	if (autoack)
		enaa |= 1 << pipeid;
	else
		enaa &= ~(1 << pipeid);
	rxen |= 1 << pipeid;

	rxrf24_write_reg(RF24_EN_RXADDR, rxen);
	rxrf24_write_reg(RF24_EN_AA, enaa);
}

void rxrf24_close_pipe(uint8_t pipeid)
{
	uint8_t rxen, enaa;

	if (pipeid > 5)
		return;

	rxen = rxrf24_read_reg(RF24_EN_RXADDR);
	enaa = rxrf24_read_reg(RF24_EN_AA);

	rxen &= ~(1 << pipeid);
	enaa &= ~(1 << pipeid);

	rxrf24_write_reg(RF24_EN_RXADDR, rxen);
	rxrf24_write_reg(RF24_EN_AA, enaa);
}

void rxrf24_close_pipe_all()
{
	rxrf24_write_reg(RF24_EN_RXADDR, 0x00);
	rxrf24_write_reg(RF24_EN_AA, 0x00);
	rxrf24_write_reg(RF24_EN_DPL, 0x00);
}

uint8_t rxrf24_pipe_isopen(uint8_t pipeid)
{
	uint8_t rxen;

	if (pipeid > 5)
		return;

	rxen = rxrf24_read_reg(RF24_EN_RXADDR);
	if (rxen & (1 << pipeid))
		return 1;
	return 0;
}

void rxrf24_set_pipe_packetsize(uint8_t pipeid, uint8_t size)
{
	uint8_t dynpdcfg, feature;

	if (pipeid > 5)
		return;

	dynpdcfg = rxrf24_read_reg(RF24_DYNPD);
	if (size < 1) {
		feature = rxrf24_read_reg(RF24_FEATURE);
		if ( !(feature & RF24_EN_DPL) )  // Cannot set dynamic payload if EN_DPL is disabled
			return;
		if (!( (1<<pipeid) & dynpdcfg )) {
			// DYNPD not enabled for this pipe; enable it
			dynpdcfg |= 1 << pipeid;
		}
	} else {
		dynpdcfg &= ~(1 << pipeid);
		if (size > 32)
		size = 32;
		rxrf24_write_reg(RF24_RX_PW_P0+pipeid, size);
	}
	rxrf24_write_reg(RF24_DYNPD, dynpdcfg);
}

/* Payload management */
void rxrf24_flush_tx()
{
	nrf24.chipselect(0);
	rxrf24_rspi_transfer(RF24_FLUSH_TX);
	nrf24.chipselect(1);
}

void rxrf24_flush_rx()
{
	nrf24.chipselect(0);
	rxrf24_rspi_transfer(RF24_FLUSH_RX);
	nrf24.chipselect(1);
}

uint8_t rxrf24_rx_pipe()
{
	uint8_t st;
	st = rxrf24_read_reg(RF24_STATUS);
	st = (st & 0x0E) >> 1;
	return st;
}

uint8_t rxrf24_rx_size()
{
	uint8_t sz;

	nrf24.chipselect(0);
	sz = rxrf24_rspi_transfer16(RF24_16_R_RX_PL_WID | RF24_NOP) & 0x00FF;
	return sz;
}

uint8_t rxrf24_payload_read(void *buf, size_t maxlen)
{
	uint8_t *cbuf = (uint8_t *)buf;
	uint8_t rxsz;
	uint16_t j;

}
