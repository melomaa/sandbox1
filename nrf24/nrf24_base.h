
#ifndef __LINUX_NRF24_H
#define __LINUX_NRF24_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/types.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/spi/spi.h>

#include "nRF24L01.h"

#define MAX_PAYLOAD_SIZE	32

#define LOW		0
#define HIGH	1

enum {
	TIO_NRF24_ACKPAYLOAD = 0xF240,
	TIO_NRF24_PAYLOADSIZE,
	TIO_NRF24_GETCONFIG,
	TIO_NRF24_SETCHANNEL,
};

enum {
	NRF24_NO_COMMAND,
	NRF24_POWERDOWN,
	NRF24_SETCHANNEL,
};

#define _BV(x) (1<<(x))
#define	STATUSCLEAR	( W_REGISTER | ( REGISTER_MASK & STATUS))

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

struct nrf24_platform_data {
	int	ce_pin;
	u8	active_pipes;
	u8	combine_pipes;
	unsigned int	uartclk;
	unsigned	uart_base;
};

struct nrf24_config {
	uint8_t rxAddr0[5];
	uint8_t rxAddr1[5];
	uint8_t	rxAddrN[4];
	uint8_t txAddr[5];
	uint8_t	rxWidth[6];
	uint8_t	autoAck;
	uint8_t rxEnable;
	uint8_t	channel;
	uint8_t setup;
	uint8_t config;
	uint8_t dynpd;
	uint8_t feature;
	uint8_t retransmit;
	uint8_t addrWidth;
};

struct nrf24_ioctl {
	int pipe;
	char ackPayload[32];
	int apLen;
};

struct nrf24_chip {
	struct spi_device *spi;
	struct uart_port	uart;
	dev_t                   devt;
	//	struct nrf24_channel channel[6];
	spinlock_t              lock;
	u8		pending;	/* Async transfer active (only one at a time)*/
	u8		state;		/* Which state (async spi) is being handled */
	u8 		queue;		/* Queued interrupt */
	u8		ctrl_cmd;	/* Device control command */
	struct spi_message      message; /* Message struct for async transfer */
	struct spi_transfer     *transfers; /* Transfer structs for the async message */
	u8 		*spiBuf;	/* Buffer for message data */
	u8		*txbuf;
	struct mutex txlock;
	bool		p_variant;
	uint64_t	pipe0_reading_address;
	struct nrf24_ioctl nioctl;
	bool	ackPayload;
	struct workqueue_struct *workqueue;
	struct work_struct	work;
	uint8_t payload_size; /**< Fixed size of payloads */
	bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
	bool wide_band; /* 2Mbs data rate in use? */
	struct nrf24_config	radioConfig;
};

void ce(int level);
int nrf24_write_from_buf(struct nrf24_chip *ts, u8 *buf, u8 len);
uint8_t write_register(struct nrf24_chip *ts, uint8_t reg, uint8_t val);
uint8_t write_buffer_to_register(struct nrf24_chip *ts, uint8_t reg, const uint8_t* data, uint8_t len);
uint8_t read_register(struct nrf24_chip *ts, uint8_t reg);
uint8_t read_buffer_from_register(struct nrf24_chip *ts, uint8_t reg, uint8_t* buf, uint8_t len);
int getConfiguration(struct nrf24_chip *ts);
int assign_command(struct nrf24_chip *ts, int command);

#endif
