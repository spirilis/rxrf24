/* Renesas RX nRF24L01+ RF transceiver library
 * Designed for the RX62N with RSPI
 * Free and Open Source Software by Eric Brundick <spirilis@linux.com>
 *
    Copyright (c) 2013 Eric Brundick <spirilis [at] linux dot com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
 */

#ifndef RXRF24_H
#define RXRF24_H

#include "iodefine.h"
#include "nRF24L01.h"
#include <stdint.h>
#include <sys/types.h>


/* nRF24L01+ supports <=32 byte packets */
#define RXRF24_PACKET_SIZE_MAX 32

/* Pointer to simple I/O function provided by the user */
typedef void(*portIOfunc)(uint8_t onoff);

/* Transceiver Configuration datatype */
typedef struct {
	union {
		volatile uint8_t BYTE;
		struct {
			uint8_t :4;
			uint8_t txfailed:1;
			uint8_t tx:1;
			uint8_t rx:1;
			uint8_t flag:1;
		} volatile BIT;
	} irq;
	union {
		uint8_t BYTE;
		struct {
			uint8_t txfull:1;
			uint8_t rxpipe:3;
			uint8_t irq_txfailed:1;
			uint8_t irq_tx:1;
			uint8_t irq_rx:1;
			uint8_t :1;
		} BIT;
	} status;
	uint8_t address_width;
	uint8_t channel;
	uint32_t speed;
	union {
		uint8_t BYTE;
		struct {
			uint8_t :1;
			uint8_t rfpwr:2;
			uint8_t rfdrhigh:1;
			uint8_t plllock:1;
			uint8_t rfdrlow:1;
			uint8_t :1;
			uint8_t contwave:1;
		} BIT;
	} rfsetup;
	union {
		uint8_t BYTE;
		struct {
			uint8_t :2;
			uint8_t crc16:1;
			uint8_t en:1;
			uint8_t :4;
		} BIT;
	} crc;
	uint8_t rxpipe;

	portIOfunc chipselect_init;
	portIOfunc chipselect;

	portIOfunc chipenable_init;
	portIOfunc chipenable;
} rxrf24_t;

// There is only 1 transceiver supported by this library.
extern rxrf24_t nrf24;


/* Transceiver State flags */
#define RXRF24_STATE_NOTPRESENT  0x00
#define RXRF24_STATE_POWERDOWN   0x01
#define RXRF24_STATE_STANDBY     0x02
#define RXRF24_STATE_PTX         0x03
#define RXRF24_STATE_PRX         0x04
#define RXRF24_STATE_TEST        0x05


/* IRQ flags */
#define RXRF24_IRQ_TXFAILED      0x10
#define RXRF24_IRQ_TX            0x20
#define RXRF24_IRQ_RX            0x40
#define RXRF24_IRQ_MASK          0x70




// Register callback functions to enable/disable the CSN and CE pins
void rxrf24_register_portcallback(portIOfunc csninit, portIOfunc csn, portIOfunc ceinit, portIOfunc ce);

// SPI initialization and I/O
void rxrf24_rspi_init();
void rxrf24_rspi_ports_enable();  // User-tweaked function to flip on RSPI functionality on pins
void rxrf24_rspi_ports_disable(); // User-tweaked function to suspend RSPI functionality on pins
void rxrf24_rspi_delay(uint16_t us);  // Performs a strawman SPI transfer for X microseconds
uint8_t rxrf24_rspi_transfer(uint8_t);     // SPI transfers
uint16_t rxrf24_rspi_transfer16(uint16_t);
uint32_t rxrf24_rspi_transfer24(uint32_t);
uint32_t rxrf24_rspi_transfer32(uint32_t);

// Payload management
uint8_t rxrf24_read_reg(uint8_t reg);
void rxrf24_write_reg(uint8_t reg, uint8_t val);
void rxrf24_payload_write(size_t len, void *buf);  // Write TX payload
uint8_t rxrf24_payload_read(size_t maxlen, void *buf);  // Read RX payload, returns length
uint8_t rxrf24_rx_pipe();  // Reports pipe# of current RX payload
uint8_t rxrf24_rx_size();  // Reports size of current RX payload
uint8_t rxrf24_queue_state();
void rxrf24_flush_tx();
void rxrf24_flush_rx();

// Transceiver initialization & configuration
uint8_t rxrf24_init();
void rxrf24_set_channel(uint8_t channel);
void rxrf24_set_speed(uint32_t speed);  // Supports 250000, 1000000, 2000000
void rxrf24_set_txpower(int8_t dbm);  // Supports 0, -6, -12, -18
void rxrf24_set_retrans_params(uint16_t delayUS, uint8_t retries);  // delayUS=250 to 4000, retries=1 to 15
void rxrf24_set_address_width(uint8_t aw);  // 3, 4, 5 supported
void rxrf24_set_crc(uint8_t onoff, uint8_t crc16bit);
void rxrf24_set_option(uint8_t feature);  // Set FEATURE register

// RX pipe configuration
void rxrf24_open_pipe(uint8_t pipeid, uint8_t autoack);  // Open an RX pipe with or without AutoACK
void rxrf24_close_pipe(uint8_t pipeid);
void rxrf24_close_pipe_all();
void rxrf24_set_pipe_packetsize(uint8_t pipeid, uint8_t size);  // Set static payload size for pipe; 0 = Dynamic Payload

// RF address configuration
void rxrf24_set_txaddr(void *buf);
void rxrf24_set_rxaddr(uint8_t pipeid, void *buf);

// Power and Mode configuration
uint8_t rxrf24_current_state();
void rxrf24_powerdown();
void rxrf24_standby();
void rxrf24_activate_tx();
void rxrf24_activate_rx();
uint8_t rxrf24_is_alive();
void rxrf24_test_mode(uint8_t onoff);
uint8_t rxrf24_rpd();  // Dump RPD register to indicate if we have any live noise/signal

// IRQ and event management
uint8_t rxrf24_get_irq_reason();
void rxrf24_irq_clear(uint8_t irqflag);
void rxrf24_irq_handler() __attribute__((interrupt));

#endif
