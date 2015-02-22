#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>


#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "nrf24.h"
#include "nRF24L01.h"

#define DRIVER_NAME		"nrf24_spi"
#define TYPE_NAME		"2.4GHz_radio"

#define FIFO_SIZE		64
#define USLEEP_DELTA		500
#define SPI_MAX_TRANSFERS	8
#define BUFFER_SIZE		(FIFO_SIZE+4)


#define SPIDEV_MAJOR                    153     /* assigned */
#define N_SPI_MINORS                    32      /* ... up to 256 */

#define LOW	0
#define HIGH	1


#define _BV(x) (1<<(x))

void printDetails(void);

//static DECLARE_BITMAP(minors, N_SPI_MINORS);

struct nrf24_chip {
	struct spi_device *spi;
	dev_t                   devt;
//	struct nrf24_channel channel[6];
	spinlock_t              lock;
	u8		pending;	/* Async transfer active (only one at a time)*/
	u8		state;		/* Which state (async spi) is being handled */
	u8 		queue;		/* Queued interrupt */
        struct spi_message      message; /* Message struct for async transfer */
        struct spi_transfer     *transfers; /* Transfer structs for the async message */
	u8 		*spiBuf;	/* Buffer for message data */
	u8		*txbuf;
	bool		p_variant;
	uint64_t	pipe0_reading_address;
};

struct nrf24_platform_data {
	u8	dummy;
};

  uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
//  uint8_t csn_pin; /**< SPI Chip select */
  bool wide_band; /* 2Mbs data rate in use? */
//  bool p_variant; /* False for RF24L01 and true for RF24L01P */
  uint8_t payload_size; /**< Fixed size of payloads */
//  bool ack_payload_available; /**< Whether there is an ack payload waiting */
  bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
//  uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
//  uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */

static struct nrf24_chip *p_ts;

/* ******************************** SPI ********************************* */


/*
 * nrf24_write - Write a new register content (sync)
 *  <at> reg: Register offset
 */
static int nrf24_write(struct nrf24_chip *ts, u8 len)
{
	return spi_write(ts->spi, ts->txbuf, len);
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
  at91_set_gpio_value(ce_pin,level);
}

uint8_t read_buffer_from_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  struct nrf24_chip *ts = p_ts;
 // uint8_t* first = buf;
/*  int status = nrf24_read(ts,reg);

  while ( len-- )
    *buf++ = spi_w8r8(ts->spi,0xff);
*/
  ts->txbuf[0] = (R_REGISTER | ( REGISTER_MASK & reg ));
  spi_write_then_read(ts->spi, ts->txbuf, 1, buf, len);

  return len;
}

uint8_t read_register(uint8_t reg)
{
  struct nrf24_chip *ts = p_ts;
  int result = nrf24_read(ts,reg);
  if (result < 0)
	printk(KERN_ERR "SPI failed (%d)!\n",result);
  return (uint8_t)result;
}

uint8_t write_buffer_to_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;
  struct nrf24_chip *ts = p_ts;
  ts->txbuf[0] = (W_REGISTER | ( REGISTER_MASK & reg ));
  memcpy(&ts->txbuf[1],buf,len);
  status = nrf24_write(ts, len+1);

  return status;
}

uint8_t write_register(uint8_t reg, uint8_t val)
{
  uint8_t status;
  struct nrf24_chip *ts = p_ts;
  ts->txbuf[0] = (W_REGISTER | ( REGISTER_MASK & reg ));
  ts->txbuf[1] = val;
  status = nrf24_write(ts, 2);

  return status;
}

uint8_t write_payload(const void* buf, uint8_t len)
{
  uint8_t status;

//  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);
  uint8_t* payload = (uint8_t*)buf;

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  struct nrf24_chip *ts = p_ts;
  
  //printf("[Writing %u bytes %u blanks]",data_len,blank_len);

  ts->txbuf[0] = ( W_TX_PAYLOAD );
  status = nrf24_write(ts, 1);
  memcpy(ts->txbuf,payload,data_len);
  status = nrf24_write(ts, data_len);
  memset(ts->txbuf,0,blank_len);
  status = nrf24_write(ts, blank_len);

  return status;
}



/* ******************************** CONFIG ********************************* */

uint8_t flush_rx(void)
{
  uint8_t status;

  p_ts->txbuf[0] = ( FLUSH_RX );
  status = nrf24_write(p_ts, 1);

  return status;
}

uint8_t flush_tx(void)
{
  uint8_t status;

  p_ts->txbuf[0] = ( FLUSH_TX );
  status = nrf24_write(p_ts, 1);

  return status;
}

void powerUp(void)
{
  write_register(CONFIG,read_register(CONFIG) | _BV(PWR_UP));
}

void toggle_features(void)
{
  p_ts->txbuf[0] = ACTIVATE;
  p_ts->txbuf[1] = 0x73; 
  nrf24_write(p_ts, 2);
  return;
}

void enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

  write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

void enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system
  write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );
  }

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

void setAutoAckPipe( uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    write_register( EN_AA, en_aa ) ;
  }
}

void setChannel(uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const uint8_t max_channel = 127;
  write_register(RF_CH,min(channel,max_channel));
}

void setCRCLength(rf24_crclength_e length)
{
  uint8_t config = read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
  
  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  write_register( CONFIG, config ) ;
}


bool setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  write_register(RF_SETUP,setup);

  // Verify our result
  if ( read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}

void setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = read_register(RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  write_register( RF_SETUP, setup ) ;
}

void setPayloadSize(uint8_t size)
{
  const uint8_t max_payload_size = 32;
  payload_size = min(size,max_payload_size);
}

void setRetries(uint8_t delay, uint8_t count)
{
 write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

void openWritingPipe(uint64_t value)
{
  const uint8_t max_payload_size = 32;

  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.
  write_buffer_to_register(RX_ADDR_P0, (uint8_t*)(&value), 5);
  write_buffer_to_register(TX_ADDR, (uint8_t*)(&value), 5);

  write_register(RX_PW_P0,min(payload_size,max_payload_size));
}

static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void openReadingPipe(struct nrf24_chip *ts, uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
    ts->pipe0_reading_address = address;

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
 
	  if ( child < 2 ) {
		  write_buffer_to_register(child_pipe[child], (uint8_t*)(&address), 5);
	  }
	  else {
		  write_register(child_pipe[child], (uint8_t)address);
	  }
	  write_register(child_payload_size[child],payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(child_pipe_enable[child]));
  }
}

void startListening(struct nrf24_chip *ts)
{
  write_register(CONFIG, read_register(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
  write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (ts->pipe0_reading_address)
    write_buffer_to_register(RX_ADDR_P0, (uint8_t*)(&ts->pipe0_reading_address), 5);

  // Flush buffers
  flush_rx();
  flush_tx();

  // Go!
  ce(HIGH);

  // wait for the radio to come up (130us actually only needed)
  //delayMicroseconds(130);
}

void setAutoAck(bool enable)
{
  if ( enable )
    write_register(EN_AA, 0b111111);
  else
    write_register(EN_AA, 0);
}

rf24_crclength_e getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

/* ******************************** IRQ ********************************* */

int nrf24_write_read(struct nrf24_chip *ts, u8 *txbuf, unsigned n_tx, u8 *spibuf, unsigned n_rx);

#define RING_BUFFER_SIZE	256
uint8_t readIndex=0, writeIndex=0;
char ringBuf_rx[RING_BUFFER_SIZE];

int copy_to_ring_buffer(char *buf, int len)
{
	uint8_t copylen, rblen;
	rblen = (int)RING_BUFFER_SIZE - writeIndex;
	copylen = min(len, (int)rblen);
	memcpy(&ringBuf_rx[writeIndex], buf, copylen);
	writeIndex += copylen;
	copylen = (len-copylen);
	if (copylen > 0) {
		memcpy(&ringBuf_rx[writeIndex], buf, copylen);
		writeIndex += copylen;
	}
	return 0;
}

void nrf24_callback(void *data)
{
  struct nrf24_chip *ts = (struct nrf24_chip*)data;
  int reg = ts->spiBuf[0];
  unsigned long flags;
  static uint8_t chipStatus=0;

  printk("Callback (%x): %x\n", reg, ts->spiBuf[1]);
  //write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
  spin_lock_irqsave(&ts->lock, flags);

  switch (reg) 
  {
  case STATUS:
	  chipStatus = ts->spiBuf[1];
	  if (chipStatus & 0x40) {
		  ts->txbuf[0] = R_RX_PAYLOAD;
		  nrf24_write_read(ts,ts->txbuf,1,ts->spiBuf,payload_size);
	  }
	  else {
		  ts->txbuf[0] = ( W_REGISTER | ( REGISTER_MASK & STATUS) );
		  ts->txbuf[1] = 0x70;
		  nrf24_write_read(ts,ts->txbuf,2,ts->spiBuf,1);
	  }
	  break;
  case R_RX_PAYLOAD:
	  copy_to_ring_buffer(ts->spiBuf,payload_size);
	  if (chipStatus & 0x70) {
		  ts->txbuf[0] = ( W_REGISTER | ( REGISTER_MASK & STATUS) );
		  ts->txbuf[1] = 0x70;
		  nrf24_write_read(ts,ts->txbuf,2,ts->spiBuf,1);
	  }
	  break;
  default:
	  if (ts->queue) {
		  ts->queue = 0;
		  ts->txbuf[0] = ( R_REGISTER | ( REGISTER_MASK & STATUS) );
		  nrf24_write_read(ts,ts->txbuf,1,ts->spiBuf,1);
	  }
	  else {
		  ts->pending = 0;
		  ts->txbuf[0] = 0xFF;
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
/*
  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  ts->txbuf[0] = ( R_RX_PAYLOAD );
  status = sc_write_read(ts->txbuf,1,ts->spiBuf,data_len+blank_len);
*/
uint8_t get_status(struct nrf24_chip *ts)
{
  return read_register(STATUS);
}

int get_register_async(struct nrf24_chip *ts, uint8_t reg)
{
	ts->txbuf[0] = ( R_REGISTER | ( REGISTER_MASK & reg) );
	nrf24_write_read(ts,ts->txbuf,1,ts->spiBuf,1);
	return 0;
}

int get_status_async(struct nrf24_chip *ts)
{
	//ts->txbuf[0] = ( R_REGISTER | ( REGISTER_MASK & reg) );
	ts->txbuf[0] = NOP;
	nrf24_write_read(ts,ts->txbuf,1,ts->spiBuf,1);
	return 0;
}

static irqreturn_t nrf24_irq(int irq, void *data)
{
	struct nrf24_chip *ts = data;
	unsigned long flags;

	//printk("IRQ ");

	spin_lock_irqsave(&ts->lock, flags);
	if (ts->pending != 0) {
		ts->queue = 1;
	}
	else {
		ts->pending = 1;
		get_register_async(ts,STATUS);
//		get_status_async(ts);
	}
	spin_unlock_irqrestore(&ts->lock, flags);

  return IRQ_HANDLED;
}
/*
uint8_t RF24::read_payload(void* buf, uint8_t len)
{
  uint8_t status;
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Reading %u bytes %u blanks]",data_len,blank_len);
  
  csn(LOW);
  status = spi->transfer( R_RX_PAYLOAD );
  while ( data_len-- )
    *current++ = spi->transfer(0xff);
  while ( blank_len-- )
    spi->transfer(0xff);
  csn(HIGH);


  return status;
}
*/

/* ******************************** FOPS ********************************* */


static int nrf24_open(struct inode *inode, struct file *filp)
{
  // Flush buffers
//  flush_rx();
//  flush_tx();

  powerUp(); //Power up by default when open() is called

  msleep(2);
  filp->private_data =p_ts;

  startListening(p_ts);

  msleep(1);
  printDetails();

  return 0;
}

/* Read-only message with current device setup */
static ssize_t
nrf24dev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	size_t bytes=0;

	if(readIndex != writeIndex) {
		if(writeIndex > readIndex) {
			bytes = min((int)count,((int)writeIndex-readIndex));
			bytes = copy_to_user(buf,&ringBuf_rx[readIndex],bytes);
		}
	}
	return bytes;
}

/* Write-only message with current device setup */
static ssize_t
nrf24dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	return 0;
}

static long
nrf24dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct nrf24_chip *ts;
	struct spi_device	*spi;
	int ret = -1, cnt = 10;
	char buf[10];

	printk(" Command %d, pointer %x\n",cmd,(unsigned int)arg);

	ts = filp->private_data;
	spin_lock_irq(&ts->lock);
	spi = spi_dev_get(ts->spi);
	spin_unlock_irq(&ts->lock);


	while(cnt && ts->pending) {
		msleep(10);
		cnt--;
	}

	if (cnt > 0)
	{
		switch (cmd)
		{
		case 'c':
			dev_info(&spi->dev, TYPE_NAME " Flush buffers.\n");
			write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
			// Flush buffers
			flush_rx();
			flush_tx();
			ret = 0;
			break;
		case 's':
			buf[0] = get_status(ts);
			dev_info(&spi->dev, TYPE_NAME " Status %x\n",(unsigned int)buf[0]);
			memcpy((char*)arg,buf,1);
			ret = 1;
			break;
		default:
			ret = 0;
			break;
		}
	}
	else {
		printk("SPI Timeout\n");
	}

	return ret;
}
/*
static int nrf24_release(struct inode *inode, struct file *filp)
{
	write_register(CONFIG,read_register(CONFIG) & ~(_BV(PWR_UP)));
	return 0;
}
*/
static const struct file_operations nrf24_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	nrf24dev_write,
	.read =		nrf24dev_read,
	.unlocked_ioctl = nrf24dev_ioctl,
//	.compat_ioctl = spidev_compat_ioctl,
	.open =		nrf24_open,
//	.release =	nrf24_release,
//	.llseek =	no_llseek,
};

static struct class *nrf24_class;

/* ******************************** INIT ********************************* */
void print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
  printk("\t%s =",name);
  while (qty--)
    printk(" 0x%02x ",read_register(reg++));
  printk("\r\n");
}

void print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
  printk("\t%s =",name);

  while (qty--)
  {
    uint8_t buffer[5];
    uint8_t* bufptr;
    read_buffer_from_register(reg++,buffer,sizeof buffer);

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

  printk("CRC Length\t = %x\r\n",getCRCLength());
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
  setRetries(5,15);

  // Restore our default PA level
  setPALevel( RF24_PA_MAX ) ;
  // Determine if this is a p or non-p RF24 module and then
  // reset our data rate back to default value. This works
  // because a non-P variant won't allow the data rate to
  // be set to 250Kbps.
  if( setDataRate( RF24_250KBPS ) )
  {
    ts->p_variant = true ;
  }

  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  setCRCLength( RF24_CRC_16 ) ;
 
  // Disable dynamic payloads, to match dynamic_payloads_enabled setting
  toggle_features();
  //write_register(FEATURE,0 );
  write_register(DYNPD,0);

  // Reset current status
  // Notice reset and flush is the last thing we do
  write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

// enable dynamic payloads
	enableDynamicPayloads();

	setAutoAck(1);

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  setChannel(0x6e);

  // Open pipes for communication
  openWritingPipe(0x00a5b4c370);
  openReadingPipe(ts, 1, 0x00a5b4c371);
  openReadingPipe(ts, 2, 0x00a5b4c372);
  openReadingPipe(ts, 3, 0x00a5b4c373);



  return 0;
}

static int __devinit nrf24_probe(struct spi_device *spi)
{
	struct nrf24_chip *ts;
	struct nrf24_platform_data *pdata;
	struct device *dev;
	int minor = 2;
	//	int ret;

	/* Get platform data for  module */
	pdata = spi->dev.platform_data;

	ce_pin = 111; /* AT91_PIN_PD15 */

	at91_set_gpio_output(ce_pin,0);
	ce(LOW);

	ts = kzalloc(sizeof(struct nrf24_chip), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	/* TODO */
	p_ts = ts;
	payload_size = 10;

	spi_set_drvdata(spi, ts);
	ts->spi = spi;

	/* Allocate the async SPI transfer structures */
	ts->transfers = kzalloc(sizeof(struct spi_transfer)*SPI_MAX_TRANSFERS, GFP_KERNEL);
	/* Allocate the async SPI transfer buffer */
	ts->spiBuf = kzalloc(sizeof(u8)*(BUFFER_SIZE), GFP_KERNEL);
	ts->txbuf = kzalloc(sizeof(u8)*(FIFO_SIZE), GFP_KERNEL);

	ts->pending = 0;
	ts->queue = 0;

	default_configuration(ts);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */

	ts->devt = MKDEV(SPIDEV_MAJOR, minor);
	dev = device_create(nrf24_class, &spi->dev, ts->devt,
			ts, "radio%d",
			spi->chip_select);
	//		ret = IS_ERR(dev) ? PTR_ERR(dev) : 0;

	/* Setup IRQ. Actually we have a low active IRQ, but we want
	 * one shot behaviour */
	int irq_num = 105;//gpio_to_irq(ts->spi->irq);
	if (irq_num < 0) {
		dev_err(&ts->spi->dev, "GPIO to IRQ failed\n");
	}
	else {
		ts->spi->irq = irq_num;
		if (request_any_context_irq(irq_num, nrf24_irq,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_SHARED,
				"nrf24", ts)) {
			dev_err(&ts->spi->dev, "IRQ request failed\n");
			//		destroy_workqueue(chan->workqueue);
			//		chan->workqueue = NULL;
			//		return -EBUSY;
			goto exit_destroy;
		}
	}



	dev_info(&spi->dev, TYPE_NAME " at CS%d (irq %d), 2.4GHz radio link\n",
			spi->chip_select, spi->irq);


	dev_info(&spi->dev, TYPE_NAME " IRQ handler %x\n",(int)nrf24_irq);
	printk(" ts pointer %x\n",(int)ts);


	return 0;

	exit_destroy:
	dev_set_drvdata(&spi->dev, NULL);
	kfree(ts);
	return 0;
}

static int __devexit nrf24_remove(struct spi_device *spi)
{
	struct nrf24_chip *ts = spi_get_drvdata(spi);
//	int ret;

	if (ts == NULL)
		return -ENODEV;
	printk("ts %x\n",(int)ts);

	if (ts->spi == NULL)
		return -ENODEV;
	printk("ts->spi %x\n",(int)ts->spi);

	printk("ts->spi->irq %x\n",(int)ts->spi->irq);

	if (&ts->lock == NULL)
		return -ENODEV;
	printk("ts->lock %x\n",(int)&ts->lock);

	if (nrf24_class == NULL)
		return -ENODEV;
	printk("nrf24_class %x\n",(int)nrf24_class);

	printk("ts->devt %x\n",(int)ts->devt);

	/* Free the interrupt */
	free_irq(ts->spi->irq, ts);

	/* make sure ops on existing fds can abort cleanly */

	spin_lock_irq(&ts->lock);
	ts->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&ts->lock);

	/* prevent new opens */

	device_destroy(nrf24_class, ts->devt);

//	clear_bit(MINOR(ts->devt), minors);


	kfree(ts->transfers); /* Free the async SPI transfer structures */
	kfree(ts->spiBuf); /* Free the async SPI transfer buffer */
	kfree(ts->txbuf); /* Free the transmit buffer */
	kfree(ts);

	return 0;
}

/* Spi driver data */
static struct spi_driver nrf24_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe		= nrf24_probe,
	.remove		= __devexit_p(nrf24_remove),
};

/* Driver init function */
static int __init nrf24_init(void)
{
	int status;
	status = register_chrdev(SPIDEV_MAJOR, DRIVER_NAME, &nrf24_fops);
	if (status < 0)
		return status;
	nrf24_class = class_create(THIS_MODULE, "nrf24dev");
	if (IS_ERR(nrf24_class)) {
		unregister_chrdev(SPIDEV_MAJOR, nrf24_spi_driver.driver.name);
		return PTR_ERR(nrf24_class);
	}

	status = spi_register_driver(&nrf24_spi_driver);
	if (status < 0) {
		class_destroy(nrf24_class);
		unregister_chrdev(SPIDEV_MAJOR, nrf24_spi_driver.driver.name);
	}
	return status;
}

/* Driver exit function */
static void __exit nrf24_exit(void)
{
	printk("driver name %s\n",nrf24_spi_driver.driver.name);
	spi_unregister_driver(&nrf24_spi_driver);
	printk("Exit  nrf24_class %x\n",(int)nrf24_class);
	class_destroy(nrf24_class);
	unregister_chrdev(SPIDEV_MAJOR, nrf24_spi_driver.driver.name);
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
