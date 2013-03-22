/* Receive LED on/off request over Nordic Semiconductor nRF24L01+
 *
 * Demo program for YRDKRX62N with pure GCC+Newlib environment (no Renesas e2studio or HEW)
 * Register Nordic transceiver at address 0xDEADBEDD01 and wait for a simple packet with 1 byte
 * indicating on or off for an onboard LED.
 *
 * PD6 ("LED15" on the LED ring) used as the output.
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

#include "iodefine.h"
#include "rxrf24.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Relocatable Interrupt Vector Table--including it here */
typedef void (*Interrupt_Handler)(void);
extern Interrupt_Handler _vectors[256];

/* LED configuration */
void led_init()
{
	PORTD.DDR.BIT.B6 = 1;  // PD6 = output
	PORTD.DR.BIT.B6 = 1;   // LED = off
}

void led_set(uint8_t onoff)
{
	PORTD.DR.BIT.B6 = !onoff;  // Output of 0 (GND) = LED on, 1 (Vcc) = LED off.
}

/* CSN, CE pin configuration for Nordic transceiver */
void csn_init(uint8_t onoff)
{
	PORTC.DDR.BIT.B1 = onoff;
	PORTC.DR.BIT.B1 = 0;
}

void csn_set(uint8_t onoff)
{
	PORTC.DR.BIT.B1 = onoff;
}

void ce_init(uint8_t onoff)
{
	PORTE.DDR.BIT.B4 = onoff;
	PORTE.DR.BIT.B4 = 0;
}

void ce_set(uint8_t onoff)
{
	PORTE.DR.BIT.B4 = onoff;
}

const uint8_t demo_rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xDD, 0x01 };

int main()
{
	uint8_t sz, inbuf[32];

	SYSTEM.SCKCR.BIT.ICK = 0x00;  // ICLK = 96MHz
        SYSTEM.SCKCR.BIT.PCK = 0x02;  // PCLK = 24MHz
        SYSTEM.SCKCR.BIT.PSTOP1 = 1;
        SYSTEM.SCKCR.BIT.PSTOP0 = 1;
	MSTP(RSPI1) = 0;

        PORT2.DDR.BIT.B1 = 1;
	PORT2.DR.BIT.B1 = 0;   // Shut off that distracting LCD backlight
	PORTD.DDR.BIT.B2 = 1;  // Debug LED
	PORTD.DR.BIT.B2 = 1;

	// nRF24L01+ settings
	nrf24.address_width = 5;
	nrf24.channel = 10;
	nrf24.speed = 250000;
	nrf24.crc.BIT.en = 1;
	nrf24.crc.BIT.crc16 = 0;
	nrf24.chipselect_init = csn_init;
	nrf24.chipselect = csn_set;
	nrf24.chipenable_init = ce_init;
	nrf24.chipenable = ce_set;

	rxrf24_init();

	// RXaddr
	rxrf24_set_rxaddr(1, (uint8_t*)demo_rxaddr);
	rxrf24_open_pipe(1, 0);  // Open pipe#1 for reading with autoack=0
	rxrf24_set_pipe_packetsize(1, 0);  // Pipe#1 packet size = 0 (DYNAMIC)

	// Initialize IRQ13 to respond to incoming nRF24 IRQ requests
	_vectors[VECT_ICU_IRQ13] = rxrf24_irq_handler;
	PORT0.DDR.BIT.B5 = 0;  // P05 = input
	PORT0.ICR.BIT.B5 = 1;  // P05 input schmitt-trigger enabled
	ICU.IRQCR[13].BIT.IRQMD = 1;  // Configure GPIO IRQ for Falling Edge
				      /* FYI- values include 0 (level-triggered LOW)
				       *                     1 (falling-edge)
				       *                     2 (rising edge)
				       *                     3 (rising and falling edges)
				       */
	IR(ICU,IRQ13) = 0;   // Clear IRQ in case it's set
	IPR(ICU,IRQ13) = 1;  // Priority = 1
	IEN(ICU,IRQ13) = 1;  // Enable
	__builtin_rx_setpsw(8);  // Enable Interrupts

	rxrf24_activate_rx();
	led_init();

	while (1) {
		while (!nrf24.irq.BIT.flag)
			;

		rxrf24_get_irq_reason();
		if (nrf24.irq.BIT.rx) {
			sz = rxrf24_payload_read(inbuf, 32);
			if (sz) {
				if (inbuf[0] == '1')  // ASCII char "1" expected for 1
					led_set(1);
				if (inbuf[0] == '0')  // ASCII char "0" expected for 0
					led_set(0);
			}
			rxrf24_irq_clear(nrf24.irq.BYTE);
		}

		if (nrf24.irq.BIT.tx || nrf24.irq.BIT.txfailed) {
			rxrf24_irq_clear(nrf24.irq.BYTE);
		}
	}
	return 0;
}
