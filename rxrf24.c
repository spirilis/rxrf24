/* RX nRF24L01+ library
 */

#include "iodefine.h"
#include "rxrf24_config.h"
#include "rxrf24.h"
#include <stdint.h>

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
const uint8_t[] txaddr_default = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
const uint8_t[] rxaddr1_default = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
const uint8_t[] rxaddr2_default = {0xC2, 0xC2, 0xC2, 0xC2, 0xC3};
const uint8_t[] rxaddr3_default = {0xC2, 0xC2, 0xC2, 0xC2, 0xC4};
const uint8_t[] rxaddr4_default = {0xC2, 0xC2, 0xC2, 0xC2, 0xC5};
const uint8_t[] rxaddr5_default = {0xC2, 0xC2, 0xC2, 0xC2, 0xC6};

void rxrf24_init()
{
	// Do we have valid functions for chipselect and chipenable?
	if (nrf24.chipselect_init == 0 || nrf24.chipselect == 0)
		return;
	if (nrf24.chipenable_init == 0 || nrf24.chipenable == 0)
		return;

	nrf24.chipselect_init(1);
	nrf24.chipselect(1);  // CSN = HIGH (inactive)
	nrf24.chipenable_init(1);
	nrf24.chipenable(0);  // CE = LOW (inactive)

	rxrf24_flush_tx();
	rxrf24_flush_rx();
	rxrf24_set_option(RF24_EN_DPL);
	rxrf24_set_crc(nrf24.crc.BIT.en, nrf24.crc.BIT.crc16);
	rxrf24_set_channel(nrf24.channel);
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
}

void rxrf24_set_channel(uint8_t channel)
{
	if (channel > 125)
		channel = 125;
	nrf24.channel = channel;
	nrf24.chipselect(0);
	rxrf24_rspi_transfer16(RF24_16_W_REGISTER | RF24_16_RF_CH | channel);
	nrf24.chipselect(1);
}
