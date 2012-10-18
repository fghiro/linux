/*
 * I2C master mode driver for Nomadik STN8815 application processor
 *
 * (2012) Written by Fabrizio Ghiringhelli <fghiro@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include <linux/i2c-stn8815.h>


/* Driver name and version */
#define DRIVER_NAME	"stn8815_i2c"
#define DRIVER_VERSION	"1.0"


/* I2C register map */
#define	I2C_CR			0x000	/* R/W Control Register */
#define I2C_SCR			0x004	/* R/W Slave Control Register */
#define I2C_HSMCR		0x008	/* R/W High Speed Master Code Reg. */
#define I2C_MCR			0x00C	/* R/W Master Control Register */
#define I2C_TFR			0x010	/* W Transmit FIFO Register */
#define I2C_SR			0x010	/* R Status Register */
#define I2C_RFR			0x018	/* R Receive FIFO Register */
#define I2C_TFTR		0x01C	/* R/W Transmit FIFO Threshold Reg. */
#define I2C_RFTR		0x020	/* R/W Receive FIFO Threshold Reg. */
#define I2C_DMAR		0x024	/* R/W DMA Register */
#define I2C_BRCR		0x028	/* R/W Baud-Rate Counter Register */
#define I2C_IMSCR		0x02C	/* R/W Interrupt Mask Set/Clear Reg. */
#define I2C_RISR		0x030	/* R RAW Interrupt Status Register */
#define I2C_MISR		0x034	/* R Masked Interrupt Status Register */
#define I2C_ICR			0x038	/* R/W Interrupt Clear Register */
#define I2C_PERIPH_ID0		0xFE0	/* R Peripheral Identification Reg. 0 */
#define I2C_PERIPH_ID1		0xFE4	/* R Peripheral Identification Reg. 1 */
#define I2C_PERIPH_ID2		0xFE8	/* R Peripheral Identification Reg. 2 */
#define I2C_PERIPH_ID3		0xFEC	/* R Peripheral Identification Reg. 3 */
#define I2C_PCELL_ID0		0xFF0	/* R PCell Identification Register 0 */
#define I2C_PCELL_ID1		0xFF4	/* R PCell Identification Register 1 */
#define I2C_PCELL_ID2		0xFF8	/* R PCell Identification Register 2 */
#define I2C_PCELL_ID3		0xFFC	/* R PCell Identification Register 3 */

/* I2C Control Register (I2C_CR) */
#define I2C_CR_FON_MASK		(3 << 13)	/* Filtering On */
#define I2C_CR_LM		(1 << 12)	/* Loopback mode */
#define I2C_CR_DMA_SLE		(1 << 11)	/* DMA sync. logic enable */
#define I2C_CR_DMA_RX_EN	(1 << 10)	/* DMA Rx enable */
#define I2C_CR_DMA_TX_EN	(1 << 9)	/* DMA Tx enable */
#define I2C_CR_FRX		(1 << 8)	/* Flush receive */
#define I2C_CR_FTX		(1 << 7)	/* Flush transmit */
#define I2C_CR_SGCM		(1 << 6)	/* Slave general call mode */
#define I2C_CR_SM_MASK		(3 << 4)	/* Speed mode: */
#define I2C_CR_SM_STANDARD	(0 << 4)	/*   Standard mode */
#define I2C_CR_SM_FAST		(1 << 4)	/*   Fast mode */
#define I2C_CR_SM_HS		(2 << 4)	/*   High-speed mode */
#define I2C_CR_SAM		(1 << 3)	/* Slave addressing mode */
#define I2C_CR_OM_MASK		(3 << 1)	/* Operating mode: */
#define I2C_CR_OM_SLAVE		(0 << 1)	/*   Slave mode */
#define I2C_CR_OM_MASTER	(1 << 1)	/*   Master mode */
#define I2C_CR_OM_MS		(2 << 1)	/*   Master/Slave mode */
#define I2C_CR_PE		(1 << 0)	/* Peripheral enable */

/* I2C Slave Control Register (I2C_SCR) */
#define I2C_SCR_SLSU_MASK	(0xFFFF << 16)	/* Slave data setup time */
#define I2C_SCR_ESA10_MASK	(7 << 7)	/* Extended 10-bit Slave addr */
#define I2C_SCR_SA7_MASK	(0x7F << 0)	/* 7-bit Slave address */

/* High Speed Master Code Register (I2C_HSMCR) */
#define	I2C_HSMCR_MC_MASK	(7 << 0)	/* Master code */

/* Master Control Register (I2C_MCR) */
#define I2C_MCR_LENGTH_MASK	(0x7FF << 15)	/* Transaction length */
#define	I2C_MCR_P		(1 << 14)	/* Stop condition */
#define	I2C_MCR_AM_MASK		(3 << 12)	/* Address type: */
#define	I2C_MCR_AM_GCCMD	(0 << 12)	/*   General call command */
#define	I2C_MCR_AM_A7		(1 << 12)	/*   7-bit address */
#define	I2C_MCR_AM_A10		(2 << 12)	/*   10-bit address */
#define	I2C_MCR_SB		(1 << 11)	/* Start byte */
#define	I2C_MCR_EA10_MASK	(7 << 8)	/* Extended 10-bit address */
#define I2C_MCR_A7_MASK		(0x7F << 1)	/* 7-bit address */
#define I2C_MCR_A10_MASK	(I2C_MCR_A7_MASK | I2C_MCR_EA10_MASK)
#define	I2C_MCR_OP_MASK		(1 << 0)	/* Operation: Wr(0) Rd(1) */
#define	I2C_MCR_OP_WR		(0 << 0)	/*   Write */
#define	I2C_MCR_OP_RD		(1 << 0)	/*   Read  */

/* Transmit FIFO Register (I2C_TFR) */
#define I2C_TFR_TDATA_MASK	0xFF		/* Transmission data */

/* Status Register (I2C_SR) */
#define I2C_SR_LENGTH_MASK	(0x7FF << 9)	/* Transfer length */
#define I2C_SR_TYPE_MASK	(3 << 7)	/* Receive type: */
#define I2C_SR_TYPE_FRAME	(0 << 7)	/*   Normal frame */
#define I2C_SR_TYPE_GCALL	(1 << 7)	/*   General call */
#define I2C_SR_TYPE_HW_GCALL	(2 << 7)	/*   Hardware general call */
#define I2C_SR_CAUSE_MASK	(7 << 4)	/* Abort cause: */
#define I2C_SR_CAUSE_NACK_ADDR	(0 << 4)	/*   NACK after addr */
#define I2C_SR_CAUSE_NACK_DATA	(1 << 4)	/*   NACK during data phase */
#define I2C_SR_CAUSE_ACK_MCODE	(2 << 4)	/*   ACK after master code */
#define I2C_SR_CAUSE_ARB_LOST	(3 << 4)	/*   Master arbitration lost */
#define I2C_SR_CAUSE_BERR_START	(4 << 4)	/*   Slave restart */
#define I2C_SR_CAUSE_BERR_STOP	(5 << 4)	/*   Slave reset */
#define I2C_SR_CAUSE_OVFL	(6 << 4)	/*   Slave overflow */
#define I2C_SR_STATUS_MASK	(3 << 2)	/* Controller status: */
#define I2C_SR_STATUS_NOP	(0 << 2)	/*   No operation in progess */
#define I2C_SR_STATUS_ONGOING	(1 << 2)	/*   An operation is ongoing */
#define I2C_SR_STATUS_OK	(2 << 2)	/*   Operation completed ok */
#define I2C_SR_STATUS_ABORT	(3 << 2)	/*   Operation aborted */
#define I2C_SR_OP_MASK		(3 << 0)	/* Operation: */
#define I2C_SR_OP_MW		(0 << 0)	/*   Master Write */
#define I2C_SR_OP_MR		(1 << 0)	/*   Master Read */
#define I2C_SR_OP_RFS		(2 << 0)	/*   Read From Slave */
#define I2C_SR_OP_WTS		(3 << 0)	/*   Write To Slave */

/* Receive FIFO Register (I2C_RFR) */
#define I2C_RFR_RDATA_MASK	0xFF		/* Receive data */

/* Transmit FIFO Threshold Register (I2C_TFTR) */
#define I2C_TFTR_THRTX_MASK	0x03		/* Threshold Tx (in bytes) */

/* Receive FIFO Threshold Register (I2C_RFTR) */
#define I2C_RFTR_THRRX_MASK	0x03		/* Threshold Rx (in bytes) */

/* DMA Register (I2C_DMAR) */
#define I2C_DMAR_BURST_TX	(1 << 11)	/* Burst Tx */
#define I2C_DMAR_DBSIZE_TX_MASK	(3 << 8)	/* Destination burst size Tx */
#define I2C_DMAR_BURST_RX	(1 << 3)	/* Burst Rx */
#define I2C_DMAR_SBSIZE_RX_MASK	(3 << 0)	/* Source burst size Rx */

/* Baud-Rate Counter Register (I2C_BRCR) */
#define I2C_BRCR_BRCNT1_MASK	(0xFFFF << 16)	/* Baud rate counter 1 */
#define I2C_BRCR_BRCNT2_MASK	(0xFFFF << 0)	/* Baud rate counter 2 */

/* Interrupt register flags. Applicable to the following registers:
 * - Interrupt Mask Set/Clear Register (I2C_IMSCR)
 * - RAW Interrupt Status Register (I2C_RISR)
 * - Masked Interrupt Status Register (I2C_MISR)
 * - Interrupt Clear Register (I2C_ICR) - all but the ones marked by (#) */
#define I2C_INT_BERR		(1 << 25)	/* Bus error */
#define I2C_INT_MAL		(1 << 24)	/* Master arbitration lost */
#define I2C_INT_STD		(1 << 20)	/* Slave transaction done */
#define I2C_INT_MTD		(1 << 19)	/* Master transaction done */
#define I2C_INT_WTSR		(1 << 18)	/* Write-To-Slave request */
#define I2C_INT_RFSE		(1 << 17)	/* Read-from-Slave empty */
#define I2C_INT_RFSR		(1 << 16)	/* Read-from-Slave request */
#define I2C_INT_RXFF		(1 << 6)	/* Rx fifo full (#) */
#define I2C_INT_RXFNF		(1 << 5)	/* Rx fifo nearly full (#) */
#define I2C_INT_RXFE		(1 << 4)	/* Rx fifo empty (#) */
#define I2C_INT_TXFOVR		(1 << 3)	/* Tx fifo overrun */
#define I2C_INT_TXFF		(1 << 2)	/* Tx fifo full (#) */
#define I2C_INT_TXFNE		(1 << 1)	/* Tx fifo nearly empty (#) */
#define I2C_INT_TXFE		(1 << 0)	/* Tx fifo empty (#) */

/* All software-clearable interrupts (I2C_ICR) */
#define I2C_ICR_ALL		(I2C_INT_BERR | I2C_INT_MAL | I2C_INT_STD |  \
				 I2C_INT_MTD | I2C_INT_WTSR | I2C_INT_RFSE | \
				 I2C_INT_RFSR | I2C_INT_TXFOVR)

/* Buad-Rate counter init values (I2C kernel IP clocked @ 48MHz) */
#define I2C_BRCR_BRCNT1_SM	240		/* Standard (counter #1)*/
#define I2C_BRCR_BRCNT1_FM	80		/* Fast (counter #1) */
#define I2C_BRCR_BRCNT2_HS	10		/* High-Speed (counter #2) */

/* Timeout waiting for the controller to respond */
#define I2C_TIMEOUT		(msecs_to_jiffies(1000))

/* STN8815 I2C device structure */
struct stn8815_i2c_dev {
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	struct i2c_adapter	*adapter;
	struct completion	msg_complete;

	u16			msg_err;	/* message errors */
	struct i2c_msg		*msg;

	/* platform data */
	unsigned char		speed;
	unsigned char		filter;
};

/* STN8815-I2C platform data default */
static struct i2c_stn8815_platform_data nhk8815_i2c_default_init = {
	.filter	= I2C_STN8815_FILTER_NONE,
	.speed	= I2C_STN8815_SPEED_STANDARD,
	.master_code = 0,
};

/* Speed mode description messages */
static const char * const speed_desc[] = {
	"standard mode (100 Kb/s)",
	"fast mode (400 Kb/s)",
	"high-speed mode (3.4 Mb/s)",
	""
};
#define	get_speed_desc(x)	(speed_desc[x])

/* Digital filter description messages */
static const char * const filter_desc[] = {
	"no",
	"1 clock-wide-spikes",
	"2 clock-wide-spikes",
	"3 clock-wide-spikes"
};
#define	get_filter_desc(x)	(filter_desc[x])

/* Low-level register write function */
static inline void stn8815_i2c_wr_reg(struct stn8815_i2c_dev *dev,
	int reg, u32 val)
{
	__raw_writel(val, dev->base + reg);
}

/* Low-level register read function */
static inline u32 stn8815_i2c_rd_reg(struct stn8815_i2c_dev *dev, int reg)
{
	return __raw_readl(dev->base + reg);
}

/* Clear the specified interrupt */
static inline void stn8815_i2c_clear_int(struct stn8815_i2c_dev *dev, u32 msk)
{
	stn8815_i2c_wr_reg(dev, I2C_ICR, msk);
}

/* Enable the controller */
static inline void stn8815_i2c_enable(struct stn8815_i2c_dev *dev)
{
	u32 cr = stn8815_i2c_rd_reg(dev, I2C_CR);
	cr |= I2C_CR_PE;
	stn8815_i2c_wr_reg(dev, I2C_CR, cr);
}

/* Disable the controller */
static inline void stn8815_i2c_disable(struct stn8815_i2c_dev *dev)
{
	u32 cr = stn8815_i2c_rd_reg(dev, I2C_CR);
	cr &= ~I2C_CR_PE;
	stn8815_i2c_wr_reg(dev, I2C_CR, cr);
}

/* Rx and Tx FIFO flushing */
static inline short stn8815_i2c_fifo_flush(struct stn8815_i2c_dev *dev)
{
	u32 cr;
	int loop_cntr = 1000;

	/* Start flushing Rx and Tx FIFOs */
	cr = stn8815_i2c_rd_reg(dev, I2C_CR);
	cr |= I2C_CR_FRX | I2C_CR_FTX;
	stn8815_i2c_wr_reg(dev, I2C_CR, cr);

	/* Wait for completion */
	do {
		udelay(10);
		loop_cntr--;
	} while (stn8815_i2c_rd_reg(dev, I2C_CR) & (I2C_CR_FRX | I2C_CR_FTX) &&
		 loop_cntr > 0);

	/* Check exit status */
	if (loop_cntr <= 0) {
		dev_warn(dev->dev, "Timeout waiting for fifo flushing\n");
		return -ETIMEDOUT;
	}
	return 0;
}

/* I2C controller initialization */
static void __devinit stn8815_i2c_hwinit(struct stn8815_i2c_dev *dev)
{
	u32 reg;
	struct i2c_stn8815_platform_data *pdata;

	/* Get platform data: if not provided use defaults */
	pdata = dev->dev->platform_data;
	if (!pdata) {
		dev_info(dev->dev, "no platform data, using defaults\n");
		pdata = &nhk8815_i2c_default_init;
	}
	dev->speed = pdata->speed & I2C_STN8815_SPEED_MASK;
	dev->filter = pdata->filter & I2C_STN8815_FILTER_MASK;

	/* Program the Baud-Rate */
	reg = (pdata->speed == I2C_STN8815_SPEED_FAST ?
		(I2C_BRCR_BRCNT2_HS << 16) | I2C_BRCR_BRCNT1_FM :
		(I2C_BRCR_BRCNT2_HS << 16) | I2C_BRCR_BRCNT1_SM);
	stn8815_i2c_wr_reg(dev, I2C_BRCR, reg);

	/* Set up the high-speed master code */
	reg = pdata->master_code & I2C_HSMCR_MC_MASK;
	stn8815_i2c_wr_reg(dev, I2C_HSMCR, reg);

	/* Program the controller */
	reg = (dev->filter << 13) |		/* digital filter */
	      (dev->speed << 4) |		/* speed mode */
	       I2C_CR_OM_MASTER |		/* single master */
	       I2C_CR_PE;			/* peripheral enable */
	stn8815_i2c_wr_reg(dev, I2C_CR, reg);

	/* Rx and Tx FIFO flushing */
	stn8815_i2c_fifo_flush(dev);

	/* Enable interrupts: Bus-ERRor and Master-Transaction-Done */
	stn8815_i2c_clear_int(dev, I2C_ICR_ALL);
	stn8815_i2c_wr_reg(dev, I2C_IMSCR, I2C_INT_BERR | I2C_INT_MTD);
}

/* Wait for bus release */
static short stn8815_i2c_wait_bus_release(struct stn8815_i2c_dev *dev)
{
	unsigned long timeout;

	timeout = jiffies + I2C_TIMEOUT;
	while (stn8815_i2c_rd_reg(dev, I2C_SR) & I2C_SR_STATUS_ONGOING) {
		if (time_after(jiffies, timeout)) {
			dev_warn(dev->dev, "Timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}

	return 0;
}

/* ISR */
static irqreturn_t stn8815_i2c_isr(int irq, void *dev_id)
{
	struct stn8815_i2c_dev *dev = dev_id;
	struct i2c_msg *msg = dev->msg;
	u32 misr;
	int i;

	/* Check the adapter suspend status */
	if (pm_runtime_suspended(dev->dev))
		return IRQ_NONE;

	/* Read interrupt source */
	misr = stn8815_i2c_rd_reg(dev, I2C_MISR);
	if ((misr & I2C_INT_MTD) != 0) {

		/* Transmission succesfully completed */
		if ((msg->flags & I2C_M_RD) != 0) {
			/* Master read */
			for (i = 0; i < msg->len; i++)
				msg->buf[i] = stn8815_i2c_rd_reg(dev, I2C_RFR) &
					I2C_RFR_RDATA_MASK;
		}

		/* Clear interrupt (the controller will send Ack/Nack) */
		stn8815_i2c_clear_int(dev, I2C_INT_MTD);
	} else if ((misr & I2C_INT_BERR) != 0) {

		/* Transmission error */
		dev->msg_err = (stn8815_i2c_rd_reg(dev, I2C_SR) & I2C_SR_CAUSE_MASK) >> 4;
		stn8815_i2c_clear_int(dev, I2C_INT_BERR);
	} else {
		/* Unknown interrupt: clear all interrupt flags */
		stn8815_i2c_clear_int(dev, I2C_ICR_ALL);
		return IRQ_NONE;
	}

	/* Complete */
	complete(&dev->msg_complete);
	return IRQ_HANDLED;
}

/* I2C master read */
static int stn8815_i2c_xfer_rd(struct i2c_adapter *adap, struct i2c_msg *pmsg,
			bool stop)
{
	struct stn8815_i2c_dev *dev = i2c_get_adapdata(adap);
	u32 mcr;
	int i;

	if (pmsg->len == 0)
		return -EINVAL;

	/* Initialize completion */
	init_completion(&dev->msg_complete);
	dev->msg = pmsg;
	dev->msg_err = 0;

	/* Set up the master transmission parameters */
	mcr = (pmsg->len << 15 & I2C_MCR_LENGTH_MASK) |	/* message length */
	      (stop ? I2C_MCR_P : 0) |			/* stop bit */
	      I2C_MCR_OP_RD |				/* read operation */
	      (pmsg->flags & I2C_M_TEN ?		/* slave address */
		I2C_MCR_AM_A10 | (pmsg->addr << 1 & I2C_MCR_A10_MASK) :
		I2C_MCR_AM_A7 | (pmsg->addr << 1 & I2C_MCR_A7_MASK));
	stn8815_i2c_wr_reg(dev, I2C_MCR, mcr);

	/* Wait for completion */
	i = wait_for_completion_timeout(&dev->msg_complete, I2C_TIMEOUT);
	if (i < 0)
		return i;
	if (i == 0) {
		dev_err(dev->dev, "Controller timeout\n");
		return -ETIMEDOUT;
	}

	if (likely(dev->msg_err == 0))
		return 0;

	/* There is an error */
	dev_err(dev->dev, "Controller error 0x%04X\n", dev->msg_err);
	return -EIO;
}

/* I2C master write */
static int stn8815_i2c_xfer_wr(struct i2c_adapter *adap, struct i2c_msg *pmsg,
			bool stop)
{
	struct stn8815_i2c_dev *dev = i2c_get_adapdata(adap);
	u32 mcr;
	int i;

	if (pmsg->len == 0)
		return -EINVAL;

	/* Load the Tx FIFO */
	for (i = 0; i < pmsg->len; i++)
		stn8815_i2c_wr_reg(dev, I2C_TFR, pmsg->buf[i]);

	/* Initialize completion */
	init_completion(&dev->msg_complete);
	dev->msg = pmsg;
	dev->msg_err = 0;

	/* Set up the master transmission parameters */
	mcr = (pmsg->len << 15 & I2C_MCR_LENGTH_MASK) |	/* message length */
	      (stop ? I2C_MCR_P : 0) |			/* stop bit */
	      I2C_MCR_OP_WR |				/* write operation */
	      (pmsg->flags & I2C_M_TEN ?		/* slave address */
		I2C_MCR_AM_A10 | (pmsg->addr << 1 & I2C_MCR_A10_MASK) :
		I2C_MCR_AM_A7 | (pmsg->addr << 1 & I2C_MCR_A7_MASK));
	stn8815_i2c_wr_reg(dev, I2C_MCR, mcr);

	/* Wait for completion */
	i = wait_for_completion_timeout(&dev->msg_complete, I2C_TIMEOUT);
	if (i < 0)
		return i;
	if (i == 0) {
		dev_err(dev->dev, "Controller timeout\n");
		return -ETIMEDOUT;
	}

	if (likely(dev->msg_err == 0))
		return 0;

	/* There is an error */
	dev_err(dev->dev, "Controller error 0x%04X\n", dev->msg_err);
	return -EIO;
}

/* Generic I2C master transfer entrypoint */
static int stn8815_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			int num)
{
	struct stn8815_i2c_dev *dev = i2c_get_adapdata(adap);
	struct i2c_msg *pmsg;
	int i, r;
	bool stop;

	/* Power management: increment device usage counter */
	pm_runtime_get_sync(dev->dev);

	/* Wait for bus release */
	/* FIXME: really necessary in single master mode? */
	r = stn8815_i2c_wait_bus_release(i2c_get_adapdata(adap));
	if (r)
		goto out;

	/* Cycle through each message */
	for (i = 0, r = 0; i < num && !r; i++) {
		stop = (i == (num - 1));
		pmsg = &msgs[i];
		if (pmsg->flags & I2C_M_RD)
			r = stn8815_i2c_xfer_rd(adap, pmsg, stop);
		else
			r = stn8815_i2c_xfer_wr(adap, pmsg, stop);
	}

out:
	/* Power management: decrement device usage counter */
	pm_runtime_put(dev->dev);

	return r == 0 ? num : r;
}

/* I2C supported functionality */
static u32 stn8815_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/* I2C algorithm structure */
static struct i2c_algorithm stn8815_i2c_algo = {
	.master_xfer = stn8815_i2c_xfer,
	.functionality = stn8815_i2c_func,
};

/* Probe function */
static int __devinit stn8815_i2c_probe(struct platform_device *pdev)
{
	struct stn8815_i2c_dev *dev;
	struct i2c_adapter *adap;
	struct resource *mem, *irq;
	int err;

	/* Get resources */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "unable to find mem resource\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "unable to find irq resource\n");
		return -ENODEV;
	}

	/* Request I2C memory region */
	if (!request_mem_region(mem->start, resource_size(mem), pdev->name)) {
		dev_err(&pdev->dev, "unable to get I2C memory region\n");
		return -EBUSY;
	}

	/* Allocate memory for the STN8815 I2C device data */
	dev = kzalloc(sizeof(struct stn8815_i2c_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		err = -ENOMEM;
		goto err_release_region;
	}
	platform_set_drvdata(pdev, dev);

	/* Setup STN8815 I2C device data */
	dev->base = ioremap(mem->start, resource_size(mem));
	if (!dev->base) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		err = -ENOMEM;
		goto err_free_mem;
	}
	dev->dev = &pdev->dev;	/* or: dev->dev = get_device(&pdev->dev); */
	dev->irq = irq->start;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap) {
		dev_err(&pdev->dev, "failed to allocate adapter\n");
		err = -ENOMEM;
		goto err_io_unmap;
	}
	dev->adapter = adap;

	/* Power management: enable device and increment device usage counter */
	pm_runtime_enable(dev->dev);
	pm_runtime_get_sync(dev->dev);

	/* Initialize I2C controller */
	stn8815_i2c_hwinit(dev);

	/* Get IRQ */
	err = request_irq(dev->irq, stn8815_i2c_isr, 0, pdev->name, dev);
	if (err) {
		dev_err(&pdev->dev, "can't register IRQ %d\n", dev->irq);
		err = -ENOMEM;
		goto err_free_adap;
	}

	/* Power management: decrement device usage counter */
	pm_runtime_put(dev->dev);

	/* Setup I2C adapter */
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strlcpy(adap->name, "STN8815 I2C adapter", sizeof(adap->name));
	adap->algo = &stn8815_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->nr = pdev->id;
	i2c_set_adapdata(adap, dev);

	err = i2c_add_numbered_adapter(adap);
	if (err) {
		dev_err(&pdev->dev, "failed to register adapter\n");
		goto err_free_irq;
	}

	dev_info(dev->dev, "bus %d, rev %s, %s, %s filter\n", pdev->id,
		DRIVER_VERSION, get_speed_desc(dev->speed),
		get_filter_desc(dev->filter));

	return 0;

err_free_irq:
	free_irq(dev->irq, dev);

err_free_adap:
	kfree(adap);
	pm_runtime_put(dev->dev);
	pm_runtime_disable(dev->dev);

err_io_unmap:
	iounmap(dev->base);

err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(dev);

err_release_region:
	release_mem_region(mem->start, resource_size(mem));

	return err;
}

/* Remove function */
static int __devexit stn8815_i2c_remove(struct platform_device *pdev)
{
	struct stn8815_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource *mem;

	free_irq(dev->irq, dev);

	i2c_del_adapter(dev->adapter);
	kfree(dev->adapter);
	pm_runtime_disable(dev->dev);
	iounmap(dev->base);

	platform_set_drvdata(pdev, NULL);
	kfree(dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
/* Runtime suspend */
static int stn8815_i2c_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct stn8815_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	stn8815_i2c_disable(i2c_dev);	/* Disable the controller */
	return 0;
}

/* Runtime resume */
static int stn8815_i2c_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct stn8815_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	stn8815_i2c_enable(i2c_dev);	/* Enable the controller */
	return 0;
}
#endif	/* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops stn8815_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(stn8815_i2c_runtime_suspend,
			   stn8815_i2c_runtime_resume,
			   NULL)
};

static struct platform_driver stn8815_i2c_driver = {
	.probe		= stn8815_i2c_probe,
	.remove		= __devexit_p(stn8815_i2c_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= &stn8815_i2c_pm_ops,
	},
};

module_platform_driver(stn8815_i2c_driver);

MODULE_DESCRIPTION("NOMADIK STN8815 I2C bus adapter");
MODULE_AUTHOR("Fabrizio Ghiringhelli <fghiro@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS("platform:" DRIVER_NAME);
