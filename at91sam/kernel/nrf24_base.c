
#include "nrf24_base.h"
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/tty_flip.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "nrf24_funcs.h"

#define DRIVER_NAME		"nrf24_spi"
#define TYPE_NAME		"2.4GHz_radio"

#define FIFO_SIZE		32
#define USLEEP_DELTA		500
#define SPI_MAX_TRANSFERS	8
#define BUFFER_SIZE		(FIFO_SIZE*4)

void printDetails(void);

uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */

//  bool ack_payload_available; /**< Whether there is an ack payload waiting */

//  uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
//  uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */
static struct nrf24_chip *p_ts;
static unsigned int mcr = 0;

/* ******************************** SPI ********************************* */


/*
 * nrf24_write - Write a new register content (sync)
 *  <at> reg: Register offset
 */
#if 0
static int nrf24_write(struct nrf24_chip *ts, u8 len)
{
	return spi_write(ts->spi, ts->txbuf, len);
}
#endif

int nrf24_write_from_buf(struct nrf24_chip *ts, u8 *buf, u8 len)
{
	int ret;

	ret = mutex_lock_interruptible(&ts->txlock);
	if (ret == 0) {
		ret = spi_write(ts->spi, buf, len);
		mutex_unlock(&ts->txlock);
	}
	return ret;
}

/**
 * nrf24_read - Read back register content
 *  <at> reg: Register offset
 *
 * Returns positive 8 bit value from the device if successful or a
 * negative value on error
 */
static int nrf24_read(struct nrf24_chip *ts, unsigned reg)
{
	return spi_w8r8(ts->spi, R_REGISTER | ( REGISTER_MASK & reg ));
}

void ce(int level)
{
  gpio_set_value(ce_pin,level);
}

uint8_t read_buffer_from_register(struct nrf24_chip *ts, uint8_t reg, uint8_t* buf, uint8_t len)
{
  unsigned char localBuf[MAX_PAYLOAD_SIZE];

  localBuf[0] = (R_REGISTER | ( REGISTER_MASK & reg ));
  spi_write_then_read(ts->spi, localBuf, 1, buf, len);

  return len;
}

uint8_t read_register(struct nrf24_chip *ts, uint8_t reg)
{
  int result = nrf24_read(ts,reg);
  if (result < 0)
	printk(KERN_ERR "SPI failed (%d)!\n",result);
  return (uint8_t)result;
}

uint8_t write_buffer_to_register(struct nrf24_chip *ts, uint8_t reg, const uint8_t* data, uint8_t len)
{
  uint8_t status;
  u8 buf[MAX_PAYLOAD_SIZE];

  buf[0] = (W_REGISTER | ( REGISTER_MASK & reg ));
  memcpy(&buf[1], data, len);
  status = nrf24_write_from_buf(ts, buf, len+1);

  return status;
}

uint8_t write_register(struct nrf24_chip *ts, uint8_t reg, uint8_t val)
{
  uint8_t status;
  u8 buf[2];

  buf[0] = (W_REGISTER | ( REGISTER_MASK & reg ));
  buf[1] = val;
  status = nrf24_write_from_buf(ts, buf, 2);

  return status;
}

uint8_t write_payload(struct nrf24_chip *ts, const void* payload, uint8_t len)
{
  uint8_t status;
  u8 buf[MAX_PAYLOAD_SIZE];
  uint8_t data_len = min(len,ts->payload_size);
  uint8_t blank_len = ts->dynamic_payloads_enabled ? 0 : ts->payload_size - data_len;

  buf[0] = ( W_TX_PAYLOAD );
  memcpy(&buf[1], payload, data_len);
  memset(&buf[data_len + 1], 0, blank_len);
  status = nrf24_write_from_buf(ts, buf, 1 + data_len + blank_len);

  return status;
}

/* ******************************** CONFIG ********************************* */

/* ******************************** IRQ ********************************* */

int nrf24_write_read(struct nrf24_chip *ts, u8 *txbuf, unsigned n_tx, u8 *spibuf, unsigned n_rx);

static uint8_t request_comm(struct nrf24_chip *ts, int timeout_ms)
{
	int cnt = (int)(timeout_ms/50);
	while(cnt && ts->pending) {
		msleep(50);
		cnt--;
	}

	if (cnt > 0)
	{
		ts->pending = 1;
		return 0;
	}
	else {
		dev_err(&ts->spi->dev, "%s: Timeout waiting for comm.\n", __func__);
	}
	return 1;
}

static void release_comm(struct nrf24_chip *ts) {
	unsigned char localBuf[2];
	if (ts->queue) {
		ts->queue = 0;
		localBuf[0] = ( R_REGISTER | ( REGISTER_MASK & STATUS) );
		nrf24_write_read(ts,localBuf,1,ts->spiBuf,1);
	}
	else {
		ts->pending = 0;
	}
}


static void nrf24_set_ackpayload(struct nrf24_chip *ts)
{
	int status;
	uint8_t data_len;
	 u8 buf[MAX_PAYLOAD_SIZE];

	if (ts->ackPayload) {
//		writeAckPayload(ts, ts->nioctl.pipe, (const void*)ts->nioctl.ackPayload, ts->nioctl.apLen);

		  data_len = min((uint8_t)ts->nioctl.apLen, (uint8_t)MAX_PAYLOAD_SIZE);

		//  printk("ts: %x, txb: %x, buf: %x, len: %d\n",(unsigned int)ts, (unsigned int)&ts->txbuf[0],(unsigned int)buf, data_len);
		//  printk("nio: %x, app: %x, pl: %x\n",(unsigned int)&ts->nioctl, (unsigned int)&ts->nioctl.ackPayload, *(unsigned int*)ts->nioctl.ackPayload);

		  buf[0] = ( W_ACK_PAYLOAD | ( ts->nioctl.pipe & 0b111 ) );
		  memcpy(&buf[1], ts->nioctl.ackPayload, data_len);
		  status = nrf24_write_from_buf(ts, buf, data_len+1);
		ts->ackPayload = false;
	}
}

static void nrf24_device_control(struct nrf24_chip *ts)
{
	if (ts->ctrl_cmd != NRF24_NO_COMMAND) {
//		if (!request_comm(ts, 1000)) {
			switch(ts->ctrl_cmd) {
			case NRF24_POWERDOWN:
				ce(LOW);
				//powerDown(ts);
				break;
			case NRF24_SETCHANNEL:
				ce(LOW);
				powerDown(ts);
				write_register(ts, STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
				setChannel(ts, ts->radioConfig.channel);
				// Flush buffers
				flush_rx(ts);
				flush_tx(ts);
				powerUp(ts);
				// Go!
				ce(HIGH);
				break;
			}
//			release_comm(ts);
//		}
		ts->ctrl_cmd = NRF24_NO_COMMAND;
	}
}

static void nrf24_handle_tx(struct nrf24_chip *ts)
{
	struct uart_port *uart = &ts->uart;
	struct circ_buf *xmit = &uart->state->xmit;
	u8 buf[MAX_PAYLOAD_SIZE];
	unsigned long flags;
	unsigned len;
	int status, i;
	int cnt = 10;
	static u8 txOn = 0;

	if (uart_circ_empty(xmit) || uart_tx_stopped(uart)) {
		/* No data to send or TX is stopped */
		if (txOn) {
			startListening(ts);
			msleep(1);
			txOn = 0;
		}
		return;
	}

	status = read_register(ts, FIFO_STATUS);
	while ((status & (1<<TX_EMPTY)) == 0 && cnt > 0) {
		msleep(100);
		cnt--;
	}

	if (cnt == 0) {
		flush_tx(ts);
		status = read_register(ts, FIFO_STATUS);
		if ((status & (1<<TX_EMPTY)) == 0) {
			dev_info(&ts->spi->dev, "%s: FIFO not empty. Cannot TX.\n", __func__);
			return;
		}
	}
	if (!txOn) {
		txOn = 1;
		stopListening(ts);
	}

	/* number of bytes to transfer to the fifo */
	len = (int)uart_circ_chars_pending(xmit);
	dev_dbg(&ts->spi->dev, "xmit chars %d\n", len);
	if (len > MAX_PAYLOAD_SIZE) {
	    len = MAX_PAYLOAD_SIZE;
	}
	else if (ts->payload_size && (len > ts->payload_size)) {
	    len = ts->payload_size;
	}
	spin_lock_irqsave(&uart->lock, flags);
	for (i = 1; i <= len ; i++) {
		buf[i] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	}

	uart->icount.tx += len;
	uart_circ_clear(xmit);
	spin_unlock_irqrestore(&uart->lock, flags);
	buf[0] = W_TX_PAYLOAD;
	/* Debug print */
	dev_dbg(&ts->spi->dev, "TX(%d): ", len);
	for (i=0; i<=len; i++) {
	    dev_dbg(&ts->spi->dev, "%x ",buf[i]);
	}
	dev_dbg(&ts->spi->dev, "\n");
	/* Transfer to SPI */
	status = nrf24_write_from_buf(ts, buf, len+1);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
	    uart_write_wakeup(uart);
}

static void nrf24_work_routine(struct work_struct *w)
{
	struct nrf24_chip *ts = container_of(w, struct nrf24_chip, work);

	if (!request_comm(ts, 1000)) {
		nrf24_set_ackpayload(ts);
		nrf24_handle_tx(ts);
		nrf24_device_control(ts);
		release_comm(ts);
	}
	else {
		dev_err(&ts->spi->dev,"Device control failed.\n");
	}
}

/* Trigger work thread*/
static void nrf24_dowork(struct nrf24_chip *ts)
{
	if (!freezing(current))
		queue_work(ts->workqueue, &ts->work);
}

static void async_handle_rx(struct nrf24_chip *ts, int rxlvl)
{
	struct uart_port *uart = &ts->uart;
#ifdef KERNEL_PRE_3v9
	struct tty_struct *tty = uart->state->port.tty;
#else
	struct tty_port *tty = &uart->state->port;
#endif
	unsigned long flags;

	/* Check that transfer was successful */
	if (ts->message.status != 0) {
		printk(KERN_ERR "Message returned %d\n",ts->message.status);
		return;
	}

	/* Check the amount of received data */
	if (rxlvl <= 0) {
		return;
	} else if (rxlvl > FIFO_SIZE) {
		/* Ensure sanity of RX level */
		rxlvl = FIFO_SIZE;
	}

	ts->spiBuf[rxlvl + 1] = '\0';

	spin_lock_irqsave(&uart->lock, flags);
	/* Insert received data */
	tty_insert_flip_string(tty, &ts->spiBuf[1], rxlvl);
	/* Update RX counter */
	uart->icount.rx += rxlvl;

	spin_unlock_irqrestore(&uart->lock, flags);

	/* Push the received data to receivers */
	if (rxlvl)
		tty_flip_buffer_push(tty);

}

void nrf24_callback(void *data)
{
  struct nrf24_chip *ts = (struct nrf24_chip*)data;
  int cmd = ts->spiBuf[0];
  static unsigned char dataLen=0;
  unsigned long flags;
  static uint8_t chipStatus=0;
  unsigned char localBuf[MAX_PAYLOAD_SIZE];

  spin_lock_irqsave(&ts->lock, flags);

  switch (cmd)
  {
  case STATUS:
	  chipStatus = ts->spiBuf[1];
	  if ((chipStatus & 0x0E) != 0x0E) {
		  dev_dbg(&ts->spi->dev, "S: %x\n", chipStatus);
		  localBuf[0] = R_RX_PL_WID;
		  nrf24_write_read(ts,localBuf,1,ts->spiBuf,1);
	  }
	  else if (chipStatus & (1<<TX_DS)) {
		  dev_dbg(&ts->spi->dev, "TX data send\n");
		//  nrf24_handle_tx(ts);
		  nrf24_dowork(ts);
		  localBuf[0] = ( W_REGISTER | ( REGISTER_MASK & STATUS) );
		  localBuf[1] = 0x70;
		  nrf24_write_read(ts,localBuf,2,ts->spiBuf,1);
	  }
	  else if (chipStatus & 0x70) {
		  localBuf[0] = ( W_REGISTER | ( REGISTER_MASK & STATUS) );
		  localBuf[1] = 0x70;
		  nrf24_write_read(ts,localBuf,2,ts->spiBuf,1);
	  }
	  else {
		  if (ts->queue) {
			  ts->queue = 0;
			  localBuf[0] = ( R_REGISTER | ( REGISTER_MASK & STATUS) );
			  nrf24_write_read(ts,localBuf,1,ts->spiBuf,1);
		  }
		  else {
			  ts->pending = 0;
		  }
	  }
	  break;
  case R_RX_PL_WID:
	  dataLen = min(ts->spiBuf[1],ts->payload_size);
	  localBuf[0] = R_RX_PAYLOAD;
	  nrf24_write_read(ts, localBuf, 1, ts->spiBuf, dataLen);
	  break;
  case R_RX_PAYLOAD:
	  async_handle_rx(ts,dataLen);
	  localBuf[0] = ( W_REGISTER | ( REGISTER_MASK & STATUS) );
	  localBuf[1] = 0x70;
	  nrf24_write_read(ts,localBuf,2,ts->spiBuf,1);
	  break;
  case STATUSCLEAR:
	  localBuf[0] = ( R_REGISTER | ( REGISTER_MASK & STATUS) );
	  nrf24_write_read(ts,localBuf,1,ts->spiBuf,1);
	  break;
  default:
	  if (ts->queue) {
		  ts->queue = 0;
		  localBuf[0] = ( R_REGISTER | ( REGISTER_MASK & STATUS) );
		  nrf24_write_read(ts,localBuf,1,ts->spiBuf,1);
	  }
	  else {
		  ts->pending = 0;
	  }
	  break;
  }

  spin_unlock_irqrestore(&ts->lock, flags);

  return;
}

int nrf24_write_read(struct nrf24_chip *ts,
                u8 *txbuf, unsigned n_tx,
                u8 *spibuf, unsigned n_rx)
{
        int                     status;
        struct spi_message      *message;
        struct spi_transfer     *x;
	struct spi_device *spi = ts->spi;

	/* Sanity check */
        if ((n_tx + n_rx) > BUFFER_SIZE)
                return -EINVAL;

	if (spibuf == NULL)
		return -EINVAL;

	/* Init message */
	message = &ts->message;
	x = ts->transfers;
        spi_message_init(message);
        memset(ts->transfers, 0, sizeof ts->transfers);
        if (n_tx) {
                x[0].len = n_tx;
                spi_message_add_tail(&x[0], message);
        }
        if (n_rx) {
                x[1].len = n_rx;
                spi_message_add_tail(&x[1], message);
        }

	/* Copy TX data and assign buffers */
        memcpy(spibuf, txbuf, n_tx);
        x[0].tx_buf = spibuf;
        x[1].rx_buf = spibuf + n_tx;

	/* Setup callback */
        message->complete = nrf24_callback;
        message->context = ts;

        status = spi_async(spi, message);

        return status;
}

uint8_t get_status(struct nrf24_chip *ts)
{
  return read_register(ts, STATUS);
}

int get_register_async(struct nrf24_chip *ts, uint8_t reg)
{
	unsigned char localBuf[MAX_PAYLOAD_SIZE];
	localBuf[0] = ( R_REGISTER | ( REGISTER_MASK & reg) );
	nrf24_write_read(ts,localBuf,1,ts->spiBuf,1);
	return 0;
}

int get_status_async(struct nrf24_chip *ts)
{
	unsigned char localBuf[MAX_PAYLOAD_SIZE];
	localBuf[0] = NOP;
	nrf24_write_read(ts,localBuf,1,ts->spiBuf,1);
	return 0;
}

static irqreturn_t nrf24_irq(int irq, void *data)
{
	struct nrf24_chip *ts = data;
	unsigned long flags;

	spin_lock_irqsave(&ts->lock, flags);
	if (ts->pending != 0) {
		ts->queue = 1;
	}
	else {
		ts->pending = 1;
		get_register_async(ts,STATUS);
	}
	spin_unlock_irqrestore(&ts->lock, flags);

  return IRQ_HANDLED;
}

/* ******************************** FOPS ********************************* */

#define to_nrf24_struct(port) \
		container_of(port, struct nrf24_chip, uart)

static int nrf24_open(struct uart_port *port)
{
	struct nrf24_chip *ts = to_nrf24_struct(port);

	printk("%s\n", __func__);
	if (ts == NULL)
		return -ECHILD;
	powerUp(ts); //Power up by default when open() is called

	ts->ackPayload = false;
	if (ts->workqueue) {
		/* Flush and destroy work queue */
		flush_workqueue(ts->workqueue);
		destroy_workqueue(ts->workqueue);
		ts->workqueue = NULL;
	}
//	if (ts->workqueue == NULL) {
		/* Initialize work queue */
		ts->workqueue = create_freezable_workqueue("nrf24");
		if (!ts->workqueue) {
			dev_err(&ts->spi->dev, "Workqueue creation failed\n");
			return -EBUSY;
		}
//	}
	INIT_WORK(&ts->work, nrf24_work_routine);

  	msleep(2);

 	startListening(ts);

  	msleep(1);
  	printDetails();

  	return 0;
}

static int nrf24dev_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct nrf24_chip *ts;
	struct spi_device	*spi;
	int ret = -ENOSYS, cnt = 10;
	char buf[10];

	ts = to_nrf24_struct(port);
	if (ts == NULL) {
		return -ECHILD;
	}

	dev_info(&ts->spi->dev," Command 0x%x, pointer %x\n",cmd,(unsigned int)arg);

	spin_lock_irq(&ts->lock);
	spi = spi_dev_get(ts->spi);
	spin_unlock_irq(&ts->lock);



		switch (cmd)
		{
#if 1
		case 'c':
			dev_info(&spi->dev, TYPE_NAME " Flush buffers.\n");
			write_register(ts, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
			// Flush buffers
			flush_rx(ts);
			flush_tx(ts);
			ret = 0;
			break;
		case 's':
			while(cnt && ts->pending) {
				msleep(10);
				cnt--;
			}

			if (cnt > 0)
			{
			buf[0] = get_status(ts);
			dev_info(&spi->dev, TYPE_NAME " Status %x\n",(unsigned int)buf[0]);
			memcpy((char*)arg,buf,1);
			ret = 1;
			}
			else {
				ret = -EAGAIN;
			}
#endif
		case TIO_NRF24_ACKPAYLOAD:
			memcpy(&ts->nioctl, (void*)arg, sizeof(struct nrf24_ioctl));
			dev_dbg(&ts->spi->dev,"AP(%d): %d (%d)\n",ts->nioctl.pipe,(unsigned int)ts->nioctl.ackPayload,ts->nioctl.apLen);
			ts->ackPayload = true;
			nrf24_dowork(ts);
			ret = 0;
			break;
		case TIO_NRF24_PAYLOADSIZE:
			ts->payload_size = *(char*)arg;
			dev_dbg(&ts->spi->dev,"Set payload size %d\n", ts->payload_size);
			ret = 0;
			break;
		case TIO_NRF24_GETCONFIG:
			if (getConfiguration(ts) == 0) {
				memcpy((void*)arg,&ts->radioConfig,sizeof(struct nrf24_config));
				ret = 0;
			}
			else {
				ret = -EBUSY;
			}
			break;
		case TIO_NRF24_SETCHANNEL:
			ts->radioConfig.channel = *(char*)arg;
			if (assign_command(ts, NRF24_SETCHANNEL) == 0) {
				if (!request_comm(ts, 1000)) {
					nrf24_device_control(ts);
					release_comm(ts);
					ret = 0;
				}
				else {
					ret = -EBUSY;
				}
			}
			else {
				ret = -EBUSY;
			}
			break;
#if 0
		case TCGETS:
			ret = 0;
			break;
		case TCSETSW:
			ret = 0;
			break;
#endif
		default:
			ret = -ENOIOCTLCMD;;
			break;
		}

	return ret;
}


/* ******************************** INIT ********************************* */
void print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
  printk("\t%s =",name);
  while (qty--)
    printk(" 0x%02x ",read_register(p_ts, reg++));
  printk("\r\n");
}

void print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
  printk("\t%s =",name);

  while (qty--)
  {
    uint8_t buffer[5];
    uint8_t* bufptr;
    read_buffer_from_register(p_ts,reg++,buffer,sizeof buffer);

    printk(" 0x");
    bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer )
      printk("%02x",*bufptr);
  }

  printk("\r\n");
}

void printDetails(void)
{
  printk("Status %x\n",get_status(p_ts));

  print_address_register("RX_ADDR_P0-1",RX_ADDR_P0,2);
  print_byte_register("RX_ADDR_P2-5",RX_ADDR_P2,4);
  print_address_register("TX_ADDR",TX_ADDR,1);

  print_byte_register("RX_PW_P0-6",RX_PW_P0,6);
  print_byte_register("EN_AA",EN_AA,1);
  print_byte_register("EN_RXADDR",EN_RXADDR,1);
  print_byte_register("RF_CH",RF_CH,1);
  print_byte_register("RF_SETUP",RF_SETUP,1);
  print_byte_register("CONFIG",CONFIG,1);
  print_byte_register("DYNPD/FEATURE",DYNPD,2);

  printk("CRC Length\t = %x\r\n",getCRCLength(p_ts));
//  printk("PA Power\t = %x\r\n"),getPALevel());
/*
  printf_P(PSTR("Data Rate\t = %s\r\n"),rf24_datarate_e_str_P[getDataRate()]);
  printf_P(PSTR("Model\t\t = %s\r\n"),rf24_model_e_str_P[isPVariant()]);


*/
}


int default_configuration(struct nrf24_chip *ts)
{
	msleep(5);
	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
	//write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
	setRetries(ts, 5,15);

	// Restore our default PA level
	setPALevel(ts, RF24_PA_MAX ) ;
	// Determine if this is a p or non-p RF24 module and then
	// reset our data rate back to default value. This works
	// because a non-P variant won't allow the data rate to
	// be set to 250Kbps.
	if( setDataRate(ts, RF24_250KBPS ) )
	{
		ts->p_variant = true ;
	}

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	setDataRate(ts, RF24_1MBPS ) ;

	// Initialize CRC and request 2-byte (16bit) CRC
	setCRCLength(ts, RF24_CRC_16 ) ;

	// Disable dynamic payloads, to match dynamic_payloads_enabled setting
	toggle_features(ts);
	write_register(ts, DYNPD, 0);

	// Reset current status
	// Notice reset and flush is the last thing we do
	write_register(ts, STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	// enable dynamic payloads
	enableDynamicPayloads(ts);

	setAutoAck(ts, 1);
	enableAckPayload(ts);
	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	setChannel(ts, 0x6e);

	// Open pipes for communication
	openWritingPipe(ts, 0x00a5b4c370);
	openReadingPipe(ts, 1, 0x00a5b4c371);
	openReadingPipe(ts, 2, 0x00a5b4c372);
	openReadingPipe(ts, 3, 0x00a5b4c373);

	return 0;
}

int assign_command(struct nrf24_chip *ts, int command)
{
	int cnt = 20;

	while (ts->ctrl_cmd != 0 && cnt > 0) {
		msleep(100);
		cnt--;
	}
	if (cnt > 0) {
		ts->ctrl_cmd = command;
		/* Trigger work thread for handling the actual device command */
		//nrf24_dowork(ts);
//		nrf24_device_control(ts);
	}
	else {
		dev_err(&ts->spi->dev, "Failed to assign command %d.", command);
		return -1;
	}
	return 0;
}

static const char * nrf24_type(struct uart_port *port)
{
	printk("%s\n", __func__);
	return TYPE_NAME;
}

static void nrf24_release_port(struct uart_port *port)
{
	printk("%s\n", __func__);
}

static int nrf24_request_port(struct uart_port *port)
{
	printk("%s\n", __func__);
	return 0;
}

static void nrf24_config_port(struct uart_port *port, int flags)
{
	printk("%s\n", __func__);
	return;
}

static int
nrf24_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type == PORT_UNKNOWN || ser->type == 98)
		return 0;

	return -EINVAL;
}

static void nrf24_shutdown(struct uart_port *port)
{
	struct nrf24_chip *ts;

	printk("%s\n", __func__);

	ts = to_nrf24_struct(port);
	BUG_ON(!ts);
	if (!ts->pending) {
		if (ts->workqueue) {
			/* Flush and destroy work queue */
			flush_workqueue(ts->workqueue);
			destroy_workqueue(ts->workqueue);
			ts->workqueue = NULL;
		}
		printk("Workqueue removed\n");
	}
	return;
}

static unsigned int nrf24_tx_empty(struct uart_port *port)
{
	printk("%s\n", __func__);
	return TIOCSER_TEMT;
}

static unsigned int nrf24_get_mctrl(struct uart_port *port)
{
	printk("%s\n", __func__);
	return mcr;
}

static void nrf24_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	printk("%s\n", __func__);
	mcr = mctrl;
}

static void nrf24_enable_ms(struct uart_port *port)
{
	/* Do nothing */
	printk("%s\n", __func__);
}

static void nrf24_break_ctl(struct uart_port *port, int break_state)
{
	/* Do nothing */
	printk("%s\n", __func__);
}

static void nrf24_stop_tx(struct uart_port *port)
{
	/* Do nothing */
	printk("%s\n", __func__);
}

static void nrf24_start_tx(struct uart_port *port)
{
	struct nrf24_chip *ts;

	ts = to_nrf24_struct(port);
	if (ts == NULL)
			return;

	dev_info(&ts->spi->dev, "%s\n", __func__);

	/* Trigger work thread for sending data */
	nrf24_dowork(ts);
}

static void nrf24_stop_rx(struct uart_port *port)
{
	struct nrf24_chip *ts;

	ts = to_nrf24_struct(port);
	if (ts == NULL)
			return;

	dev_info(&ts->spi->dev, "%s\n", __func__);

	if (assign_command(ts, NRF24_POWERDOWN) == 0) {
		nrf24_dowork(ts);
	}
	else {
		dev_info(&ts->spi->dev, "Poweroff command failed\n");
	}
}

static void
nrf24_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct nrf24_chip *ts;
	unsigned long flags;

	printk("%s\n", __func__);
	ts = to_nrf24_struct(port);
	spin_lock_irqsave(&ts->uart.lock, flags);
	/* we are sending char from a workqueue so enable */
#ifdef KERNEL_PRE_3v9
	ts->uart.state->port.tty->low_latency = 1;
#else
	ts->uart.state->port.low_latency = 1;
#endif
	/* Update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, 57600);
	spin_unlock_irqrestore(&ts->uart.lock, flags);
	return;
}

static struct uart_ops nrf24_uart_ops = {
	.tx_empty	= nrf24_tx_empty,
	.set_mctrl	= nrf24_set_mctrl,
	.get_mctrl	= nrf24_get_mctrl,
	.stop_tx        = nrf24_stop_tx,
	.start_tx	= nrf24_start_tx,
	.stop_rx	= nrf24_stop_rx,
	.enable_ms  = nrf24_enable_ms,
	.break_ctl      = nrf24_break_ctl,

	.startup	= nrf24_open,
	.shutdown	= nrf24_shutdown,
	.set_termios	= nrf24_set_termios,

	.type		= nrf24_type,
	.release_port   = nrf24_release_port,
	.request_port   = nrf24_request_port,
	.config_port	= nrf24_config_port,
	.verify_port	= nrf24_verify_port,
	.ioctl		= nrf24dev_ioctl,
};

static struct uart_driver nrf24_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = "ttyRF",
	.nr             = 1,
};

static int nrf24_register_uart_port(struct nrf24_chip *ts,struct nrf24_platform_data *pdata, int ch)
{
	struct uart_port *uart = &ts->uart;

	uart->irq = ts->spi->irq;
	uart->uartclk = pdata->uartclk;
	uart->fifosize = FIFO_SIZE;
	uart->ops = &nrf24_uart_ops;
	uart->flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	uart->line = pdata->uart_base + ch;
	uart->type = 98; //PORT_16650;
	uart->dev = &ts->spi->dev;

	return uart_add_one_port(&nrf24_uart_driver, uart);
}

static int nrf24_probe(struct spi_device *spi)
{
	struct nrf24_chip *ts;
	struct nrf24_platform_data *pdata;
	int ret;
	int irq_num;
	struct device_node *np = spi->dev.of_node;

	ts = kzalloc(sizeof(struct nrf24_chip), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	/* TODO */
	p_ts = ts;
	ts->payload_size = 10;

	spi_set_drvdata(spi, ts);
	ts->spi = spi;
	if (np) {
		pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&spi->dev, "could not allocate memory for pdata\n");
			return (-ENOMEM);
		}
		pdata->ce_pin = of_get_named_gpio(np,"ce-gpio", 0);
		pdata->active_pipes = 7;
		pdata->combine_pipes = 1;
		pdata->uartclk = 16000000;
		pdata->uart_base = 0;
		dev_info(&spi->dev, TYPE_NAME " Device tree config\n");
		spi->dev.platform_data = pdata;
	}
	else {
		/* Get platform data for  module */
		pdata = (struct nrf24_platform_data*)spi->dev.platform_data;
	}
	if (pdata == NULL)
		return -ENOTTY;

	dev_info(&spi->dev, "Alias %s\n",spi->modalias);
	dev_info(&spi->dev, "IRQ pin %d (%d)\n",spi->irq, gpio_to_irq(spi->irq));
	dev_info(&spi->dev, "CE pin %d\n",pdata->ce_pin);
	dev_info(&spi->dev, "pipes %d\n",pdata->active_pipes);
	dev_info(&spi->dev, "combo %d\n",pdata->combine_pipes);
	dev_info(&spi->dev, "uclk %d\n",pdata->uartclk);
	dev_info(&spi->dev, "ubase %d\n",pdata->uart_base);
	ce_pin = pdata->ce_pin;


	if (!gpio_is_valid(ce_pin))
		return -EINVAL;

	if (gpio_request(ce_pin, "Radio_CE"))
		return	-EINVAL;

	if (gpio_direction_output(ce_pin,0))
		return -EINVAL;

	mutex_init(&ts->txlock);

	/* Allocate the async SPI transfer structures */
	ts->transfers = kzalloc(sizeof(struct spi_transfer)*SPI_MAX_TRANSFERS, GFP_KERNEL);
	/* Allocate the async SPI transfer buffer */
	ts->spiBuf = kzalloc(sizeof(u8)*(BUFFER_SIZE), GFP_KERNEL);
	ts->txbuf = kzalloc(sizeof(u8)*(FIFO_SIZE), GFP_KERNEL);

	ts->pending = 0;
	ts->queue = 0;
	ts->ctrl_cmd = NRF24_NO_COMMAND;

	default_configuration(ts);

	ret = nrf24_register_uart_port(ts, pdata, 0);
	if (ret)
		goto exit_destroy;

	/* Setup IRQ. Actually we have a low active IRQ, but we want
	 * one shot behaviour */
	irq_num = gpio_to_irq(ts->spi->irq);
	if (irq_num < 0) {
		dev_err(&ts->spi->dev, "GPIO to IRQ failed\n");
	}
	else {
		if (request_any_context_irq(irq_num, nrf24_irq,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_SHARED,
				"nrf24", ts)) {
			dev_err(&ts->spi->dev, "IRQ request failed\n");
			goto exit_uart0;
		}
	}

	dev_info(&spi->dev, TYPE_NAME " at CS%d (irq %d), 2.4GHz radio link\n",
			spi->chip_select, spi->irq);

	dev_info(&spi->dev, TYPE_NAME " active pipe mask: %x\n",pdata->active_pipes);

	return 0;

exit_uart0:
	uart_remove_one_port(&nrf24_uart_driver, &ts->uart);
exit_destroy:
	dev_set_drvdata(&spi->dev, NULL);
	kfree(ts);
	gpio_free(ce_pin);
	if (np) {
		kfree(pdata);
	}
	return -EBUSY;
}

static int nrf24_remove(struct spi_device *spi)
{
	struct nrf24_chip *ts = spi_get_drvdata(spi);
	int ret;
	struct nrf24_platform_data *pdata;

	if (ts == NULL)
		return -ENODEV;
	dev_info(&spi->dev,"ts %x\n",(int)ts);

	if (ts->spi == NULL)
		return -ENODEV;
	dev_info(&spi->dev,"ts->spi %x\n",(int)ts->spi);

	if (request_comm(ts, 2000)) {
		dev_err(&spi->dev,"Comm clear timeout at exit\n");
	}

	dev_info(&spi->dev,"ts->spi->irq %d\n",(int)ts->spi->irq);

	if (&ts->lock == NULL)
		return -ENODEV;
	dev_info(&spi->dev,"ts->lock %x\n",(int)&ts->lock);

	/* Free the interrupt */
	free_irq(gpio_to_irq(ts->spi->irq), ts);

	dev_info(&spi->dev,"IRQ free done\n");
	ret = uart_remove_one_port(&nrf24_uart_driver, &ts->uart);
	if (ret)
		return ret;

	dev_info(&spi->dev,"Uart remove done\n");
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&ts->lock);
	ts->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&ts->lock);

	kfree(ts->transfers); /* Free the async SPI transfer structures */
	kfree(ts->spiBuf); /* Free the async SPI transfer buffer */
	kfree(ts->txbuf); /* Free the transmit buffer */
	kfree(ts);
	gpio_free(ce_pin);
	if (spi->dev.of_node) {
		pdata = (struct nrf24_platform_data*)spi->dev.platform_data;
		kfree(pdata);
		spi->dev.platform_data = NULL;
	}
	return 0;
}

static const struct of_device_id nrf24spi_dt_ids[] = {
        { .compatible = "nrf24L01" },
        {},
};

MODULE_DEVICE_TABLE(of, nrf24spi_dt_ids);

/* Spi driver data */
static struct spi_driver nrf24_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(nrf24spi_dt_ids),
	},
	.probe		= nrf24_probe,
	.remove		= nrf24_remove,
};

/* Driver init function */
static int __init nrf24_init(void)
{
	int status;
	int ret = uart_register_driver(&nrf24_uart_driver);

	if (ret)
		return ret;

	status = spi_register_driver(&nrf24_spi_driver);

	return status;
}

/* Driver exit function */
static void __exit nrf24_exit(void)
{
	spi_unregister_driver(&nrf24_spi_driver);
	uart_unregister_driver(&nrf24_uart_driver);
}

/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(nrf24_init);
module_exit(nrf24_exit);

MODULE_AUTHOR("Ray");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("nRF24 radio link chip with SPI interface");
MODULE_ALIAS("spi:" DRIVER_NAME);
