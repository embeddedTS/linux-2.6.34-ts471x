#include <linux/version.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/semaphore.h>
#include <mach/irqs.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <mach/dma.h>
#include <asm/atomic.h>


#include <linux/version.h>

#include <linux/clk.h>

#include <mach/hardware.h>
#include <linux/jiffies.h>

#include <linux/time.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include "ca.h"

static u8 *writebuf;
static u8 *readbuf;

static volatile unsigned long last = 0;
static volatile u16 readnum;
static long timeout;
static long timeout_end;
static struct clk *uartclk = NULL;


static inline void MFP_Set_UART_Rx()
{
	UART3TRX = 0xc3 ; /* RX */
}

static inline void MFP_Set_UART_Tx()
{
	UART3TRX = 0xc2 ; /* TX */
}

static int uart_init(int divisor)
{
	int temp;
	/* switch receiver and transmitter off */
	SerialLCR = 0;
	SerialIER = 0;
	SerialFIFO = 0;

	/* Gain access to divisor latch */
	SerialLCR = LCR_WLS0 | LCR_WLS1 | LCR_DLAB;

	/* Load baud rate divisor in two steps, lsb, then msb of value */

	SerialDATA = divisor & 0xff;
	SerialIER = (divisor >> 8) & 0xff;


	/* set the port to sensible defaults (no break, no interrupts,
	 * no parity, 8 databits, 1 stopbit, transmitter and receiver
	 * enabled), reset dlab bit:
	 */
	SerialLCR = 0x1f ;
	/* SerialLCR = 0x13; */
	SerialFIFO = 0x1 | 0x2 | 0x4 | 0x80;

	/* SerialLCR = 0x4; */
	/* turn the receiver and transmitter back on */
	SerialIER = IER_UUE | IER_RAVIE | 0x4 | 0x10;
	/* SerialIER = 0x55; */

	SerialMCR = 0x8;
	return 0;
}

int uart_read(int num)
{
	u16 i;
	int rev;
	long interval;

	readnum = 0;

	MFP_Set_UART_Rx();

	last = jiffies;

	while (1) {
		msleep(5);

		if (readnum >= num)
			break;
		interval = jiffies - last ;
		if (readnum >= 2) {
			if (interval >= timeout_end) {
				break;
			}
		} else {
			if (interval >= timeout) {
				break;
			}
		}
	}
	return 0;

}

static irqreturn_t smd_irq(int irq, void *ptr)
{
	while (SerialLSR & LSR_DR) {
		last = jiffies;
		readbuf[readnum] = SerialDATA & 0xff ;
		readnum++;
		if (readnum >= CA_BUFFER_SIZE)
			readnum = 0;
	}
	return IRQ_HANDLED;
}

int uart_write(u8 *buf, u16 length, u16 num)
{
	u16 i;
	MFP_Set_UART_Tx();
	for (i = 0; i < length; i++) {
		SerialDATA = buf[i];
		while (!(SerialLSR & LSR_TEMT))
			;
	}
	uart_read(num);
	return i;
}


static void smd_hw_reset()
{
	GPIO_RST_PDR |= 0x8;

	GPIO_RST_PSR &= 0xFFFFFFF7;/* low */

	PWM2_APBC = 0x3;
	PWM2_CR = 0x0;
	PWM2_DCR = 0x0;
	PWM2_PCR = 0x0;
	msleep(30);

	PWM2_APBC = 0x3;
	PWM2_CR = 0x0;
	PWM2_DCR = 0x1;
	PWM2_PCR = 0x1;

	GPIO_RST_PSR |= 0x8;/* high */
}

static int smd_reset(u8 ATR[])
{
	int atrlen;
	u8 BitMap;
	int iByteOffset;
	int i;

	ATRParser gATRParser;

	smd_hw_reset();

	uart_read(MAX_ATR_LEN);

	atrlen = readnum;
	for (i = 0; i < readnum; i++) {
		ATR[i] = readbuf[i];
	}

	memset((void *)&gATRParser, 0, sizeof(ATRParser));

	gATRParser.T0 = ATR[0];
	gATRParser.TS = ATR[1];

	BitMap = gATRParser.T0 & 0xF0;

	gATRParser.LengthOfHistoricalBytes = gATRParser.T0 & 0x0F;
	gATRParser.LengthOfInterfaceBytes = 0;

	iByteOffset = 2;

	for (i = 1; BitMap && gATRParser.LengthOfInterfaceBytes <=
			(ATR_Max_Interface_Bytes - gATRParser.LengthOfHistoricalBytes); i++) {
		if (BitMap == 0)
			break;

		if (BitMap & TA_bit_Mask) { /* TA */
			gATRParser.LengthOfTA++;
			gATRParser.TA[i] = ATR[iByteOffset++];
		}

		if (BitMap & TB_bit_Mask) { /* TB */
			gATRParser.LengthOfTB++;
			gATRParser.TB[i] = ATR[iByteOffset++];
		}

		if (BitMap & TC_bit_Mask) { /* TC */
			gATRParser.LengthOfTC++;
			gATRParser.TC[i] = ATR[iByteOffset++];
		}


		if (BitMap & TD_bit_Mask) { /* TD */
			gATRParser.LengthOfTD++;
			gATRParser.TD[i] = ATR[iByteOffset++];
			gATRParser.ProtocolType[gATRParser.LengthOfProtocol++] = gATRParser.TD[i] & 0xF;
		}

		BitMap = gATRParser.TD[i] & 0xF0;

		/* Store the correct number of interface bytes */
		gATRParser.LengthOfInterfaceBytes = gATRParser.LengthOfTA + gATRParser.LengthOfTB +
							gATRParser.LengthOfTC + gATRParser.LengthOfTD;
	}

	for (i = 0; i < gATRParser.LengthOfHistoricalBytes; ++i) {
		gATRParser.HistoricalBytes[i] = ATR[iByteOffset++];
	}

	/* Step 2:  Change the T1 to T0 Protocol using the PTS command. */
	if (((gATRParser.TA[1]&0xFF) != 0x11) && ((gATRParser.TA[1]&0xFF) != 0x01) &&
			((gATRParser.TA[1]&0xFF) != 0x0)) {
		u8 SentBuf[4];
		/* UINT8 RecvBuf[4]; */

		SentBuf[0] = 0xff;
		SentBuf[1] = 0x10;
		SentBuf[2] = 0x13; /* 1--F  3--D */
		SentBuf[3] = SentBuf[0] ^ SentBuf[1] ^ SentBuf[2];

		uart_write(SentBuf, 4, 4);
		uart_init(52);
	}
	return atrlen;

}

static ssize_t ca_read(struct file *filp, char *buf, size_t count , loff_t *f_pos)
{
	int remained;
	int nrecv;

	if (count > readnum)
		nrecv = readnum;
	else
		nrecv = count;
	remained = nrecv ;

	do {
		remained = copy_to_user(buf + nrecv - remained, readbuf + nrecv - remained, remained);
	} while (remained != 0);


	return nrecv;
}

static ssize_t ca_write(struct file *filp, char *buf, size_t count , loff_t *f_pos)
{
	int sentlen, remained;

	sentlen = GET_WRITE_NUM(count);

	remained = sentlen;

	do {
		remained = copy_from_user(writebuf + sentlen - remained, buf + sentlen - remained, remained);
	} while (remained != 0);

	sentlen = uart_write(writebuf, sentlen, GET_READ_NUM(count));

	return sentlen;

}

static int ca_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int result;
	int ret = 0;
	int atrlen;
	u8 ATR[MAX_ATR_LEN];

	switch (cmd) {
	case CMD_CA_RESET:
		atrlen = smd_reset(ATR);
		copy_to_user((u8 *)arg, ATR, atrlen);
		ret = atrlen;
		break;
	case CMD_CA_TIMEOUT:
		timeout = arg;
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static int ca_open(struct inode *inode, struct file *filp)
{
	int ret;

	timeout = 30;
	timeout_end = 2;

	writebuf = (u8 *)kmalloc(CA_BUFFER_SIZE , GFP_KERNEL);
	if (writebuf == NULL)
	{
		printk("kmalloc error\n");
	}

	readbuf = (u8 *)kmalloc(CA_BUFFER_SIZE , GFP_KERNEL);
	if (readbuf == NULL) {
		printk("kmalloc error\n");
	}

	clk_enable(uartclk);
	clk_set_rate(uartclk, 58500000);
	uart_init(206);

	ret = request_irq(IRQ_PXA168_UART3, smd_irq, 0, "CAUART", NULL);
	if (ret) {
		printk(KERN_ERR "%s: can't request uart3 irq\n", __FUNCTION__);
		return -1;
	}

	MFP_Set_UART_Rx();

	return 0;
}

static int ca_release(struct inode *inode, struct file *filp)
{
	free_irq(IRQ_PXA168_UART3, NULL);

	kfree(writebuf);
	kfree(readbuf);

	return 0;
}

static struct file_operations ca_fops = {
	.open		= ca_open,
	.read		= ca_read,
	.write		= ca_write,
	.release	= ca_release,
	.ioctl		= ca_ioctl,
	.owner		= THIS_MODULE
};

static struct miscdevice ca_miscdev = {
	MINOR_NUMBER,
	"pxa168-ca",
	&ca_fops,
};



static int __init ca_init(void)
{
	int ret = 0;

	ret = misc_register(&ca_miscdev);
	if (ret) {
		printk("cannot register ca driver\n");
		return ret;
	}

	uartclk = clk_get(ca_miscdev.this_device, "UART3CLK");
	if (IS_ERR(uartclk)) {
		dev_err(ca_miscdev.this_device, "unable to get CCICRSTCLK");
		return PTR_ERR(uartclk);
	}

	return ret;
}

static void __exit ca_exit(void)
{
	misc_deregister(&ca_miscdev);
	clk_disable(uartclk);

}

module_init(ca_init);
module_exit(ca_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell CA Driver");
