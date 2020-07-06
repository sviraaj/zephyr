/*
@file    EVE_target.h
@brief   target specific includes, definitions and functions
@version 4.0
@date    2019-11-17
@author  Rudolph Riedel

@section LICENSE

MIT License

Copyright (c) 2016-2019 Rudolph Riedel

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

@section History

4.0
- still 4.0 for EVE itself, switched to hardware-SPI on SAMC21
- minor maintenance
- added DMA to SAMC21 branch
- started testing things with a BT816
- added a block for the SAME51J18A
- started to add support for Imagecraft AVR
- moved all target specific lines from EVE_config.h to EVE_target.h
- cleaned up history
- added support for MSP432 - it compiles with Code Composer Studio but is for the most part untested...
- wrote a couple lines of explanation on how DMA is to be used
- replaced the dummy read of the SPI data register with a var for ATSAMC21 and ATSAME51 with "(void) REG_SERCOM0_SPI_DATA;"

*/

#ifndef EVE_TARGET_H_
#define EVE_TARGET_H_

/* While the following lines make things a lot easier like automatically compiling the code for the target you are compiling for, */
/* a few things are expected to be taken care of beforehand. */
/* - setting the Chip-Select and Power-Down pins to Output, Chip-Select = 1 and Power-Down = 0 */
/* - setting up the SPI which may or not include things like
       - setting the pins for the SPI to output or some alternate I/O function or mapping that functionality to that pin
	   - if that is an option with the controller your are using you probably should set the drive-strength for the SPI pins to high
	   - setting the SS pin on AVRs to output in case it is not used for Chip-Select or Power-Down
	   - setting SPI to mode 0
	   - setting SPI to 8 bit with MSB first
	   - setting SPI clock to no more than 11 MHz for the init - if the display-module works as high

  For the SPI transfers single 8-Bit transfers are used with busy-wait for completion.
  While this is okay for AVRs that run at 16MHz with the SPI at 8 MHz and therefore do one transfer in 16 clock-cycles,
  this is wasteful for any 32 bit controller even at higher SPI speeds.
  
  Check out the section for SAMC21E18A as it has code to transparently add DMA.
  
  If the define "EVE_DMA" is set the spi_transmit_async() is changed at compile time to write in a buffer instead directly to SPI.
  EVE_init() calls EVE_init_dma() which sets up the DMA channel and enables an IRQ for end of DMA.
  EVE_start_cmd_burst() resets the DMA buffer instead of transferring the first bytes by SPI.
  EVE_end_cmd_burst() just calls EVE_start_dma_transfer() which triggers the transfer of the SPI buffer by DMA.
  EVE_cmd_start() just instantly returns if there is an active DMA transfer.
  EVE_busy() does nothing but to report that EVE is busy if there is an active DMA transfer.
  At the end of the DMA transfer an IRQ is executed which clears the DMA active state, calls EVE_cs_clear() and EVE_cmd_start().

*/

#if !defined(ARDUINO)
#if defined(__IMAGECRAFT__)
#if defined(_AVR)
#include <iccioavr.h>

static inline void DELAY_MS(uint16_t val)
{
	uint16_t counter;

	while (val > 0) {
		for (counter = 0; counter < 2000;
		     counter++) // maybe ~1ms at 16MHz clock
		{
			__asm__ volatile("nop");
		}
		val--;
	}
}

#define EVE_CS_PORT PORTB
#define EVE_CS (1 << PB5)
#define EVE_PDN_PORT PORTB
#define EVE_PDN (1 << PB4)

static inline void EVE_pdn_set(void)
{
	EVE_PDN_PORT &= ~EVE_PDN; /* Power-Down low */
}

static inline void EVE_pdn_clear(void)
{
	EVE_PDN_PORT |= EVE_PDN; /* Power-Down high */
}

static inline void EVE_cs_set(void)
{
	EVE_CS_PORT &= ~EVE_CS; /* cs low */
}

static inline void EVE_cs_clear(void)
{
	EVE_CS_PORT |= EVE_CS; /* cs high */
}

static inline void spi_transmit_async(uint8_t data)
{
	SPDR = data; /* start transmission */
	while (!(SPSR & (1 << SPIF)))
		; /* wait for transmission to complete - 1us @ 8MHz SPI-Clock */
}

static inline void spi_transmit(uint8_t data)
{
	SPDR = data; /* start transmission */
	while (!(SPSR & (1 << SPIF)))
		; /* wait for transmission to complete - 1us @ 8MHz SPI-Clock */
}

static inline uint8_t spi_receive(uint8_t data)
{
	SPDR = data; /* start transmission */
	while (!(SPSR & (1 << SPIF)))
		; /* wait for transmission to complete - 1us @ 8MHz SPI-CLock */
	return SPDR;
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
	return *data;
}

#endif
#endif

#if defined(__GNUC__)
#if defined(__AVR__)

#include <avr/io.h>
#include <avr/pgmspace.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define DELAY_MS(ms) _delay_ms(ms)

#define EVE_CS_PORT PORTB
#define EVE_CS (1 << PB5)
#define EVE_PDN_PORT PORTB
#define EVE_PDN (1 << PB4)

static inline void EVE_pdn_set(void)
{
	EVE_PDN_PORT &= ~EVE_PDN; /* Power-Down low */
}

static inline void EVE_pdn_clear(void)
{
	EVE_PDN_PORT |= EVE_PDN; /* Power-Down high */
}

static inline void EVE_cs_set(void)
{
	EVE_CS_PORT &= ~EVE_CS; /* cs low */
}

static inline void EVE_cs_clear(void)
{
	EVE_CS_PORT |= EVE_CS; /* cs high */
}

static inline void spi_transmit_async(uint8_t data)
{
#if 1
	SPDR = data; /* start transmission */
	while (!(SPSR & (1 << SPIF)))
		; /* wait for transmission to complete - 1us @ 8MHz SPI-Clock */
#endif

#if 0
				uint8_t spiIndex  = 0x80;
				uint8_t k;

				for(k = 0; k <8; k++) {         // Output each bit of spiOutByte
					if(data & spiIndex) {   // Output MOSI Bit
						PORTC |= (1<<PORTC1);
					}
					else {
						PORTC &= ~(1<<PORTC1);
					}

					PORTA |= (1<<PORTA1); // toggle SCK
					PORTA &= ~(1<<PORTA1);

					spiIndex >>= 1;
				}
#endif
}

static inline void spi_transmit(uint8_t data)
{
#if 1
	SPDR = data; /* start transmission */
	while (!(SPSR & (1 << SPIF)))
		; /* wait for transmission to complete - 1us @ 8MHz SPI-Clock */
#endif

#if 0
				uint8_t spiIndex  = 0x80;
				uint8_t k;

				for(k = 0; k <8; k++) // Output each bit of spiOutByte
				{
					if(data & spiIndex) // Output MOSI Bit
					{
						PORTC |= (1<<PORTC1);
					}
					else
					{
						PORTC &= ~(1<<PORTC1);
					}

					PORTA |= (1<<PORTA1); // toggle SCK
					PORTA &= ~(1<<PORTA1);

					spiIndex >>= 1;
				}
#endif
}

static inline uint8_t spi_receive(uint8_t data)
{
#if 1
	SPDR = data; /* start transmission */
	while (!(SPSR & (1 << SPIF)))
		; /* wait for transmission to complete - 1us @ 8MHz SPI-CLock */
	return SPDR;
#endif

#if 0
				uint8_t spiIndex  = 0x80;
				uint8_t spiInByte = 0;
				uint8_t k;

				for(k = 0; k <8; k++) // Output each bit of spiOutByte
				{
					if(data & spiIndex) // Output MOSI Bit
					{
						PORTC |= (1<<PORTC1);
					}
					else
					{
						PORTC &= ~(1<<PORTC1);
					}

					PORTA |= (1<<PORTA1); // toggle SCK
					PORTA &= ~(1<<PORTA1);

					if(PINC & (1<<PORTC0))
					{
						spiInByte |= spiIndex;
					}

					spiIndex >>= 1;
				}
				return spiInByte;
#endif
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
#if defined(                                                                   \
	__AVR_HAVE_ELPM__) /* we have an AVR with more than 64kB FLASH memory */
	return (pgm_read_byte_far(data));
#else
	return (pgm_read_byte_near(data));
#endif
}

#endif /* AVR */

#if defined(__v851__)

#include <stdint.h>
#include "rh850_regs.h"
#include "os.h"

#define DELAY_MS(ms) OS_Wait(ms * 1000)

static inline void EVE_pdn_set(void)
{
	P0 &= ~(1u << 6);
}

static inline void EVE_pdn_clear(void)
{
	P0 |= (1u << 6);
}

static inline void EVE_cs_set(void)
{
	P8 &= ~(1u << 2); /* manually set chip-select to low */
}

static inline void EVE_cs_clear(void)
{
	P8 |= (1u << 2); /* manually set chip-select to high */
}

static inline void spi_transmit_async(uint8_t data)
{
	CSIH0CTL0 =
		0xC1; /* CSIH2PWR = 1;  CSIH2TXE=1; CSIH2RXE = 0; direct access mode  */
	CSIH0TX0H = data; /* start transmission */
	while (CSIH0STR0 & 0x00080)
		; /* wait for transmission to complete - 800ns @ 10MHz SPI-Clock */
}

static inline void spi_transmit(uint8_t data)
{
	CSIH0CTL0 =
		0xC1; /* CSIH2PWR = 1;  CSIH2TXE=1; CSIH2RXE = 0; direct access mode  */
	CSIH0TX0H = data; /* start transmission */
	while (CSIH0STR0 & 0x00080)
		; /* wait for transmission to complete - 800ns @ 10MHz SPI-Clock */
}

static inline uint8_t spi_receive(uint8_t data)
{
	CSIH0CTL0 =
		0xE1; /* CSIH2PWR = 1;  CSIH2TXE=1; CSIH2RXE = 1; direct access mode  */
	CSIH0TX0H = data; /* start transmission */
	while (CSIH0STR0 & 0x00080)
		; /* wait for transmission to complete - 800ns @ 10MHz SPI-Clock */
	return (uint8_t)CSIH0RX0H;
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
	return *data;
}

#endif /* RH850 */

#if defined(__TRICORE__)

#include "types.h"
#include "os.h"
#include "dio.h"
#include "spi.h"

#define DELAY_MS(ms) OS_Wait(ms * 1000)

static inline void EVE_pdn_set(void)
{
	HW_DIO_SetSync(IO_DIO_DIGOUT_PD_TFT, 0); /* Power-Down low */
}

static inline void EVE_pdn_clear(void)
{
	HW_DIO_SetSync(IO_DIO_DIGOUT_PD_TFT, 1); /* Power-Down high */
}

static inline void EVE_cs_set(void)
{
	HW_DIO_SetSync(IO_DIO_DIGOUT_CS_TFT,
		       0); /* manually set chip-select to low */
}

static inline void EVE_cs_clear(void)
{
	HW_DIO_SetSync(IO_DIO_DIGOUT_CS_TFT,
		       1); /* manually set chip-select to high */
}

static inline void spi_transmit_async(uint8_t data)
{
	SPI_ReceiveByte(data);
}

static inline void spi_transmit(uint8_t data)
{
	SPI_ReceiveByte(data);
}

static inline uint8_t spi_receive(uint8_t data)
{
	return SPI_ReceiveByte(data);
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
	return *data;
}

#endif /* __TRICORE__ */

#if defined(__SAMC21E18A__)

#include "sam.h"

#define EVE_DMA

#if defined(EVE_DMA)
extern uint8_t EVE_dma_buffer[4100];
extern volatile uint16_t EVE_dma_buffer_index;
extern volatile uint8_t EVE_dma_busy;

void EVE_init_dma(void);
void EVE_start_dma_transfer(void);
#endif

static inline void DELAY_MS(uint16_t val)
{
	uint16_t counter;

	while (val > 0) {
		for (counter = 0; counter < 8000;
		     counter++) // ~1ms at 48MHz Core-Clock
		{
			__asm__ volatile("nop");
		}
		val--;
	}
}

static inline void EVE_pdn_set(void)
{
	REG_PORT_OUTCLR0 = PORT_PA03;
}

static inline void EVE_pdn_clear(void)
{
	REG_PORT_OUTSET0 = PORT_PA03;
}

static inline void EVE_cs_set(void)
{
	REG_PORT_OUTCLR0 = PORT_PA05;
}

static inline void EVE_cs_clear(void)
{
	REG_PORT_OUTSET0 = PORT_PA05;
}

static inline void spi_transmit_async(uint8_t data)
{
#if defined(EVE_DMA)
	EVE_dma_buffer[EVE_dma_buffer_index++] = data;
#else
	REG_SERCOM0_SPI_DATA = data;
	while ((REG_SERCOM0_SPI_INTFLAG & SERCOM_SPI_INTFLAG_TXC) == 0)
		;
	(void)REG_SERCOM0_SPI_DATA; /* dummy read-access to clear SERCOM_SPI_INTFLAG_RXC */
#endif
}

static inline void spi_transmit(uint8_t data)
{
	REG_SERCOM0_SPI_DATA = data;
	while ((REG_SERCOM0_SPI_INTFLAG & SERCOM_SPI_INTFLAG_TXC) == 0)
		;
	(void)REG_SERCOM0_SPI_DATA; /* dummy read-access to clear SERCOM_SPI_INTFLAG_RXC */
}

static inline uint8_t spi_receive(uint8_t data)
{
	REG_SERCOM0_SPI_DATA = data;
	while ((REG_SERCOM0_SPI_INTFLAG & SERCOM_SPI_INTFLAG_TXC) == 0)
		;
	return REG_SERCOM0_SPI_DATA;
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
	return *data;
}

#endif /* __SAMC21J18A__ */

#if defined(__SAME51J19A__)

#include "sam.h"

//		#define EVE_DMA

#if defined(EVE_DMA)

#endif

static inline void DELAY_MS(uint16_t val)
{
	uint16_t counter;

	while (val > 0) {
		for (counter = 0; counter < 8800;
		     counter++) /* ~1ms at 120MHz Core-Clock, according to my Logic-Analyzer */
		{
			__asm__ volatile("nop");
		}
		val--;
	}
}

static inline void EVE_pdn_set(void)
{
	REG_PORT_OUTCLR1 = PORT_PB31;
}

static inline void EVE_pdn_clear(void)
{
	REG_PORT_OUTSET1 = PORT_PB31;
}

static inline void EVE_cs_set(void)
{
	REG_PORT_OUTCLR1 = PORT_PB01;
}

static inline void EVE_cs_clear(void)
{
	REG_PORT_OUTSET1 = PORT_PB01;
}

static inline void spi_transmit_async(uint8_t data)
{
#if defined(EVE_DMA)
	EVE_dma_buffer[EVE_dma_buffer_index++] = data;
#else
	REG_SERCOM5_SPI_DATA = data;
	while ((REG_SERCOM5_SPI_INTFLAG & SERCOM_SPI_INTFLAG_TXC) == 0)
		;
	(void)REG_SERCOM5_SPI_DATA; /* dummy read-access to clear SERCOM_SPI_INTFLAG_RXC */
#endif
}

static inline void spi_transmit(uint8_t data)
{
	REG_SERCOM5_SPI_DATA = data;
	while ((REG_SERCOM5_SPI_INTFLAG & SERCOM_SPI_INTFLAG_TXC) == 0)
		;
	(void)REG_SERCOM5_SPI_DATA; /* dummy read-access to clear SERCOM_SPI_INTFLAG_RXC */
}

static inline uint8_t spi_receive(uint8_t data)
{
	REG_SERCOM5_SPI_DATA = data;
	while ((REG_SERCOM5_SPI_INTFLAG & SERCOM_SPI_INTFLAG_TXC) == 0)
		;
	return REG_SERCOM5_SPI_DATA;
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
	return *data;
}

#endif /* __SAME51J19A__ */

#endif /* __GNUC__ */

#if defined(__TI_ARM__)

#if defined(__MSP432P401R__)

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>

#define RIVERDI_PORT GPIO_PORT_P1
#define RIVERDI_SIMO BIT6 // P1.6
#define RIVERDI_SOMI BIT7 // P1.7
#define RIVERDI_CLK BIT5 // P1.5
#define EVE_CS_PORT GPIO_PORT_P5
#define EVE_CS GPIO_PIN0 //P5.0
#define EVE_PDN_PORT GPIO_PORT_P5
#define EVE_PDN GPIO_PIN1 //P5.1

void EVE_SPI_Init(void);

static inline void DELAY_MS(uint16_t val)
{
	uint16_t counter;

	while (val > 0) {
		for (counter = 0; counter < 8000;
		     counter++) // ~1ms at 48MHz Core-Clock
		{
			__nop();
		}
		val--;
	}
}

static inline void EVE_pdn_set(void)
{
	//            GPIO_setOutputLowOnPin(EVE_PDN_PORT,EVE_PDN);   /* Power-Down low */
	P5OUT &= ~EVE_PDN; /* Power-Down low */
}

static inline void EVE_pdn_clear(void)
{
	//            GPIO_setOutputHighOnPin(EVE_PDN_PORT,EVE_PDN);   /* Power-Down high */
	P5OUT |= EVE_PDN; /* Power-Down high */
}

static inline void EVE_cs_set(void)
{
	//            GPIO_setOutputLowOnPin(EVE_CS_PORT,EVE_CS);   /* CS low */
	P5OUT &= ~EVE_CS; /* CS low */
}

static inline void EVE_cs_clear(void)
{
	//            GPIO_setOutputHighOnPin(EVE_CS_PORT,EVE_CS);    /* CS high */
	P5OUT |= EVE_CS; /* CS high */
}

static inline void spi_transmit_async(uint8_t data)
{
#if defined(EVE_DMA)

#else
	//            SPI_transmitData(EUSCI_B0_BASE, data);
	//            while (!(SPI_getInterruptStatus(EUSCI_B0_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

	UCB0TXBUF_SPI = data;
	while (!(UCB0IFG_SPI & UCTXIFG))
		; /* wait for transmission to complete */
#endif
}

static inline void spi_transmit(uint8_t data)
{
	//            SPI_transmitData(EUSCI_B0_BASE, data);
	//            while (!(SPI_getInterruptStatus(EUSCI_B0_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

	UCB0TXBUF_SPI = data;
	while (!(UCB0IFG_SPI & UCTXIFG))
		; /* wait for transmission to complete */
}

static inline uint8_t spi_receive(uint8_t data)
{
	//            SPI_transmitData(EUSCI_B0_BASE, data);
	//            while (!(SPI_getInterruptStatus(EUSCI_B0_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
	//            return EUSCI_B_CMSIS(EUSCI_B0_BASE)->RXBUF;

	UCB0TXBUF_SPI = data;
	while (!(UCB0IFG_SPI & UCTXIFG))
		; /* wait for transmission to complete */
	return UCB0RXBUF_SPI;
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
	return *data;
}

#endif /* __MSP432P401R__ */

#endif /* __TI_ARM */
#endif

#if defined(ARDUINO)
#include <stdio.h>
#include <SPI.h>

#define EVE_CS 9
#define EVE_PDN 8

#define DELAY_MS(ms) delay(ms)

#if defined(ESP8266)

#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>
#endif

static inline void EVE_pdn_set(void)
{
	digitalWrite(EVE_PDN, LOW); /* Power-Down low */
}

static inline void EVE_pdn_clear(void)
{
	digitalWrite(EVE_PDN, HIGH); /* Power-Down high */
}

static inline void EVE_cs_set(void)
{
	SPI.setDataMode(SPI_MODE0);
	digitalWrite(EVE_CS, LOW);
}

static inline void EVE_cs_clear(void)
{
	digitalWrite(EVE_CS, HIGH);
}

#if defined(ESP8266)
static inline void spi_transmit_async(uint8_t data)
{
	SPI.write(data);
}

static inline void spi_transmit(uint8_t data)
{
	SPI.write(data);
}
#else
static inline void spi_transmit_async(uint8_t data)
{
	SPI.transfer(data);
}

static inline void spi_transmit(uint8_t data)
{
	SPI.transfer(data);
}
#endif

static inline uint8_t spi_receive(uint8_t data)
{
	return SPI.transfer(data);
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
#if defined(__AVR__)
#if defined(RAMPZ)
	return (pgm_read_byte_far(data));
#else
	return (pgm_read_byte_near(data));
#endif
#else /* this may fail on your Arduino system that is not AVR and that I am not aware of */
	return *data;
#endif
}

#endif /* Arduino */

#define NRF_ZEPHYR
#if defined(NRF_ZEPHYR)

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <stdint.h>

#define DELAY_MS(ms) k_sleep(ms)

//#define EVE_PDN_PIN
#define EVE_CS_PIN 13

extern struct device *gpio_lcd_dev;
extern struct device *spi_lcd_dev;
extern struct spi_config spi_lcd_config;

static inline void EVE_pdn_set(void)
{
	//gpio_pin_write(&gpio_lcd_dev, EVE_PDN_PIN, 0);	/* Power-Down low */
}

static inline void EVE_pdn_clear(void)
{
	//gpio_pin_write(&gpio_lcd_dev, EVE_PDN_PIN, 1);	/* Power-Down low */
}

static inline void EVE_cs_set(void)
{
	gpio_pin_write(gpio_lcd_dev, EVE_CS_PIN, 0);
}

static inline void EVE_cs_clear(void)
{
	gpio_pin_write(gpio_lcd_dev, EVE_CS_PIN, 1);
}

static inline void spi_transmit_async(uint8_t data)
{
	struct spi_buf_set tx_bufs;
	struct spi_buf tx_buffers;
	tx_buffers.buf = &data;
	tx_buffers.len = 1;
	tx_bufs.count = 1;
	tx_bufs.buffers = &tx_buffers;

	spi_write(spi_lcd_dev, &spi_lcd_config, &tx_bufs);
}

static inline void spi_transmit(uint8_t data)
{
	struct spi_buf_set tx_bufs;
	struct spi_buf tx_buffers;
	tx_buffers.buf = &data;
	tx_buffers.len = 1;
	tx_bufs.count = 1;
	tx_bufs.buffers = &tx_buffers;

	spi_write(spi_lcd_dev, &spi_lcd_config, &tx_bufs);
}

static inline uint8_t spi_receive(uint8_t data)
{
	uint8_t rx_data;
	struct spi_buf_set rx_bufs;
	struct spi_buf_set tx_bufs;
	struct spi_buf buffers;
	struct spi_buf tx_buffers;
	tx_buffers.buf = &data;
	tx_buffers.len = 1;

	buffers.buf = &rx_data;
	buffers.len = 1;

	rx_bufs.count = 1;
	tx_bufs.count = 1;
	rx_bufs.buffers = &buffers;
	tx_bufs.buffers = &tx_buffers;

	if (spi_transceive(spi_lcd_dev, &spi_lcd_config, &tx_bufs, &rx_bufs) !=
	    0) {
		return 0;
	} else {
		return rx_data;
	}
}

static inline uint8_t fetch_flash_byte(const uint8_t *data)
{
	/* Not implemented yet */
	return 0;
}

static inline void SPI_Enable(void)
{
    EVE_cs_set();
}

static inline void SPI_Disable(void)
{
    EVE_cs_clear();
}

static inline void SPI_Write(uint8_t data)
{
    spi_transmit(data);
}

static inline void SPI_WriteByte(uint8_t data)
{
    spi_transmit(data);
}

static inline void SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length)
{
    struct spi_buf_set tx_bufs;
	struct spi_buf tx_buffers;
	tx_buffers.buf = Buffer;
	tx_buffers.len = Length;
	tx_bufs.count = 1;
	tx_bufs.buffers = &tx_buffers;

	spi_write(spi_lcd_dev, &spi_lcd_config, &tx_bufs);
}

static inline void SPI_ReadBuffer(uint8_t *Buffer, uint32_t Length)
{
    uint8_t data[4] = {0};

    struct spi_buf_set tx_bufs;
	struct spi_buf tx_buffers;
	tx_buffers.buf = data;
	tx_buffers.len = Length;
	tx_bufs.count = 1;
	tx_bufs.buffers = &tx_buffers;

    struct spi_buf_set rx_bufs;
	struct spi_buf rx_buffers;
	rx_buffers.buf = Buffer;
	rx_buffers.len = Length;
	rx_bufs.count = 1;
	rx_bufs.buffers = &rx_buffers;

    spi_transceive(spi_lcd_dev, &spi_lcd_config, &tx_bufs, &rx_bufs);
}

static inline void MyDelay(uint32_t DLY)
{
    k_sleep(DLY);
}


#endif

#endif /* EVE_TARGET_H_ */
