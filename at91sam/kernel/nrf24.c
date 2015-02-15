#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
//#include <linux/irqreturn.h>
#include "nRF24L01.h"

#define DRIVER_NAME		"nrf24_spi"
#define TYPE_NAME		"2.4GHz_radio"

#define FIFO_SIZE		64
#define USLEEP_DELTA		500
#define SPI_MAX_TRANSFERS	8
#define BUFFER_SIZE		(FIFO_SIZE+4)

struct nrf24_chip {
	struct spi_device *spi;
//	struct nrf24_channel channel[6];
	spinlock_t              lock;
	u8		pending;	/* Async transfer active (only one at a time)*/
	u8		state;		/* Which state (async spi) is being handled */
	u8 		queue;		/* Queued interrupt */
        struct spi_message      message; /* Message struct for async transfer */
        struct spi_transfer     *transfers; /* Transfer structs for the async message */
	u8 		*spiBuf;	/* Buffer for message data */
	u8		*txbuf;
};

struct nrf24_platform_data {
	u8	dummy;
};

  uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
//  uint8_t csn_pin; /**< SPI Chip select */
//  bool wide_band; /* 2Mbs data rate in use? */
//  bool p_variant; /* False for RF24L01 and true for RF24L01P */
  uint8_t payload_size; /**< Fixed size of payloads */
//  bool ack_payload_available; /**< Whether there is an ack payload waiting */
  bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
//  uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
//  uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */

struct nrf24_chip *p_ts;

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

uint8_t read_register(uint8_t reg)
{
  struct nrf24_chip *ts = p_ts;
  int result = nrf24_read(ts,reg);
  if (result < 0)
	printk(KERN_ERR "SPI failed (%d)!\n",result);
  return (uint8_t)result;
}

uint8_t write_buffer(uint8_t reg, const uint8_t* buf, uint8_t len)
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

void nrf24_callback(void *data)
{
  struct nrf24_chip *ts = (struct nrf24_chip*)data;
  int reg = ts->txbuf[0];

  switch (reg) 
  {
    case STATUS:
      break;
  }  
}

int nrf24_write_read(struct nrf24_chip *ts, /*struct sc16is7x2_channel *chan,*/
                u8 *txbuf, unsigned n_tx,
                u8 *spibuf, unsigned n_rx)
{
        int                     status;
        struct spi_message      *message;
        struct spi_transfer     *x;
//	struct sc16is7x2_chip *ts = chan->chip;
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

static irqreturn_t nrf24_irq(int irq, void *data)
{
  struct nrf24_chip *ts = data;
  ts->txbuf[0] = ( STATUS );
  nrf24_write_read(ts,ts->txbuf,1,ts->spiBuf,1);

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

/* Read-only message with current device setup */
static ssize_t
nrf24dev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	return 0;
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
#if 0
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	dev_info(&spi->dev, TYPE_NAME " status %d\n",get_status(ts));
#endif
/*
	switch (cmd)
	{
	  case 0:
		
	}
*/
	return 0;
}

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
//	.open =		spidev_open,
//	.release =	spidev_release,
//	.llseek =	no_llseek,
};


/* ******************************** INIT ********************************* */

static int __devinit nrf24_probe(struct spi_device *spi)
{
	struct nrf24_chip *ts;
	struct nrf24_platform_data *pdata;
	int ret;

	/* Get platform data for  module */
	pdata = spi->dev.platform_data;


	ts = kzalloc(sizeof(struct nrf24_chip), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

/* TODO */
p_ts = ts;
payload_size = 8;

	spi_set_drvdata(spi, ts);
	ts->spi = spi;

	/* Allocate the async SPI transfer structures */
	ts->transfers = kzalloc(sizeof(struct spi_transfer)*SPI_MAX_TRANSFERS, GFP_KERNEL);
	/* Allocate the async SPI transfer buffer */
	ts->spiBuf = kzalloc(sizeof(u8)*(BUFFER_SIZE), GFP_KERNEL);
	ts->txbuf = kzalloc(sizeof(u8)*(FIFO_SIZE), GFP_KERNEL);
#if 0
	/* Reset the chip */
	sc16is7x2_write(ts, REG_IOC, 0, IOC_SRESET);
	/* Force RTS lines low to keep tranmitter OFF */
	sc16is7x2_write(ts, UART_MCR, 0, UART_MCR_RTS);
	sc16is7x2_write(ts, UART_MCR, 1, UART_MCR_RTS);

#endif

	/* Setup IRQ. Actually we have a low active IRQ, but we want
	 * one shot behaviour */
	if (request_irq(ts->spi->irq, nrf24_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_SHARED,
			"nrf24", ts)) {
		dev_err(&ts->spi->dev, "IRQ request failed\n");
//		destroy_workqueue(chan->workqueue);
//		chan->workqueue = NULL;
//		return -EBUSY;
		goto exit_destroy;
	}


	dev_info(&spi->dev, TYPE_NAME " at CS%d (irq %d), 2.4GHz radio link\n",
			spi->chip_select, spi->irq);


	dev_info(&spi->dev, TYPE_NAME " status %d\n",get_status(ts));
#if 0
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}

	status = register_chrdev(SPIDEV_MAJOR, "nrf24", &nrf24_fops);
	if (status < 0)
		return status;
#endif

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

	/* Free the interrupt */
	free_irq(ts->spi->irq, chan);

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
	return spi_register_driver(&nrf24_spi_driver);
}

/* Driver exit function */
static void __exit nrf24_exit(void)
{
	spi_unregister_driver(&nrf24_spi_driver);
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
