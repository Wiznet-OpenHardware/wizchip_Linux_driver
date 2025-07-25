// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Ethernet driver for the WIZnet W5100 , W5200 , W5500, W6100 , W6300 chip.
 *
 * Copyright (C) 2006-2008 WIZnet Co.,Ltd.
 * Copyright (C) 2012 Mike Sinkovsky <msink@permonline.ru>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/platform_data/wiznet.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include "wizchip_driver.h"

#define priv  netdev_priv(ndev)
#include "reg_define.h"
#undef priv




MODULE_DESCRIPTION("WIZnet W5100 Ethernet driver v"DRV_VERSION);
MODULE_AUTHOR("Mike Sinkovsky <msink@permonline.ru>");
MODULE_ALIAS("platform:"DRV_NAME);
MODULE_LICENSE("GPL");


/*
 * Device driver private data structure
 */

struct wizchip_priv {
	const struct wizchip_ops *ops;
 	
#if 1
	/* Socket 0 register offset address */
	u32 s0_regs;
	/* Socket 0 TX buffer offset address and size */
	u32 s0_tx_buf;
	u16 s0_tx_buf_size;
	/* Socket 0 RX buffer offset address and size */
	u32 s0_rx_buf;
	u16 s0_rx_buf_size;
#endif 
	const u32 *map;

	int irq;
	int link_irq;
	int link_gpio;

	struct napi_struct napi;
	struct net_device *ndev;
	bool promisc;
	u32 msg_enable;

	struct workqueue_struct *xfer_wq;
	struct work_struct rx_work;
	struct sk_buff *tx_skb;
	struct work_struct tx_work;
	struct work_struct setrx_work;
	struct work_struct restart_work;
};
struct wizchip_priv *priv = NULL;
 
/************************************************************************
 *
 *  Lowlevel I/O functions
 *
 ***********************************************************************/

struct wizchip_mmio_priv {
	void __iomem *base;
	/* Serialize access in indirect address mode */
	spinlock_t reg_lock;
};

static inline struct wizchip_mmio_priv *wizchip_mmio_priv(struct net_device *dev)
{
	return wizchip_ops_priv(dev);
}

static inline void __iomem *wizchip_mmio(struct net_device *ndev)
{
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);
	return mmio_priv->base;
}

/*
 * In direct address mode host system can directly access W5100 registers
 * after mapping to Memory-Mapped I/O space.
 *
 * 0x8000 bytes are required for memory space.
 */
static inline int wizchip_read_direct(struct net_device *ndev, u32 addr)
{
	return ioread8(wizchip_mmio(ndev) + (addr << CONFIG_WIZNET_BUS_SHIFT));
}

static inline int __wizchip_write_direct(struct net_device *ndev, u32 addr,
				       u8 data)
{
	iowrite8(data, wizchip_mmio(ndev) + (addr << CONFIG_WIZNET_BUS_SHIFT));

	return 0;
}

static inline int wizchip_write_direct(struct net_device *ndev, u32 addr, u8 data)
{
	__wizchip_write_direct(ndev, addr, data);

	return 0;
}

static int wizchip_read16_direct(struct net_device *ndev, u32 addr)
{
	u16 data;
	data  = wizchip_read_direct(ndev, addr) << 8;
	data |= wizchip_read_direct(ndev, addr + 1);
	return data;
}

static int wizchip_write16_direct(struct net_device *ndev, u32 addr, u16 data)
{
	__wizchip_write_direct(ndev, addr, data >> 8);
	__wizchip_write_direct(ndev, addr + 1, data);

	return 0;
}

static int wizchip_readbulk_direct(struct net_device *ndev, u32 addr, u8 *buf,
				 int len)
{
	int i;

	for (i = 0; i < len; i++, addr++)
		*buf++ = wizchip_read_direct(ndev, addr);

	return 0;
}

static int wizchip_writebulk_direct(struct net_device *ndev, u32 addr,
				  const u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++, addr++)
		__wizchip_write_direct(ndev, addr, *buf++);

	return 0;
}

static int wizchip_mmio_init(struct net_device *ndev)
{
	struct platform_device *pdev = to_platform_device(ndev->dev.parent);
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);

	spin_lock_init(&mmio_priv->reg_lock);

	mmio_priv->base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(mmio_priv->base))
		return PTR_ERR(mmio_priv->base);

	return 0;
}

static const struct wizchip_ops wizchip_mmio_direct_ops = {
	.chip_id = W5100,
	.read = wizchip_read_direct,
	.write = wizchip_write_direct,
	.read16 = wizchip_read16_direct,
	.write16 = wizchip_write16_direct,
	.readbulk = wizchip_readbulk_direct,
	.writebulk = wizchip_writebulk_direct,
	.init = wizchip_mmio_init,
};

/*
 * In indirect address mode host system indirectly accesses registers by
 * using Indirect Mode Address Register (IDM_AR) and Indirect Mode Data
 * Register (IDM_DR), which are directly mapped to Memory-Mapped I/O space.
 * Mode Register (MR) is directly accessible.
 *
 * Only 0x04 bytes are required for memory space.
 */
#define W5100_IDM_AR		0x01   /* Indirect Mode Address Register */
#define W5100_IDM_DR		0x03   /* Indirect Mode Data Register */

static int wizchip_read_indirect(struct net_device *ndev, u32 addr)
{
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);
	unsigned long flags;
	u8 data;

	spin_lock_irqsave(&mmio_priv->reg_lock, flags);
	wizchip_write16_direct(ndev, W5100_IDM_AR, addr);
	data = wizchip_read_direct(ndev, W5100_IDM_DR);
	spin_unlock_irqrestore(&mmio_priv->reg_lock, flags);

	return data;
}

static int wizchip_write_indirect(struct net_device *ndev, u32 addr, u8 data)
{
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);
	unsigned long flags;

	spin_lock_irqsave(&mmio_priv->reg_lock, flags);
	wizchip_write16_direct(ndev, W5100_IDM_AR, addr);
	wizchip_write_direct(ndev, W5100_IDM_DR, data);
	spin_unlock_irqrestore(&mmio_priv->reg_lock, flags);

	return 0;
}

static int wizchip_read16_indirect(struct net_device *ndev, u32 addr)
{
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);
	unsigned long flags;
	u16 data;

	spin_lock_irqsave(&mmio_priv->reg_lock, flags);
	wizchip_write16_direct(ndev, W5100_IDM_AR, addr);
	data  = wizchip_read_direct(ndev, W5100_IDM_DR) << 8;
	data |= wizchip_read_direct(ndev, W5100_IDM_DR);
	spin_unlock_irqrestore(&mmio_priv->reg_lock, flags);

	return data;
}

static int wizchip_write16_indirect(struct net_device *ndev, u32 addr, u16 data)
{
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);
	unsigned long flags;

	spin_lock_irqsave(&mmio_priv->reg_lock, flags);
	wizchip_write16_direct(ndev, W5100_IDM_AR, addr);
	__wizchip_write_direct(ndev, W5100_IDM_DR, data >> 8);
	wizchip_write_direct(ndev, W5100_IDM_DR, data);
	spin_unlock_irqrestore(&mmio_priv->reg_lock, flags);

	return 0;
}

static int wizchip_readbulk_indirect(struct net_device *ndev, u32 addr, u8 *buf,
				   int len)
{
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&mmio_priv->reg_lock, flags);
	wizchip_write16_direct(ndev, W5100_IDM_AR, addr);

	for (i = 0; i < len; i++)
		*buf++ = wizchip_read_direct(ndev, W5100_IDM_DR);

	spin_unlock_irqrestore(&mmio_priv->reg_lock, flags);

	return 0;
}

static int wizchip_writebulk_indirect(struct net_device *ndev, u32 addr,
				    const u8 *buf, int len)
{
	struct wizchip_mmio_priv *mmio_priv = wizchip_mmio_priv(ndev);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&mmio_priv->reg_lock, flags);
	wizchip_write16_direct(ndev, W5100_IDM_AR, addr);

	for (i = 0; i < len; i++)
		__wizchip_write_direct(ndev, W5100_IDM_DR, *buf++);

	spin_unlock_irqrestore(&mmio_priv->reg_lock, flags);

	return 0;
}

static int w5100_reset_indirect(struct net_device *ndev)
{

	wizchip_write_direct(ndev, MR, MR_RST);
	mdelay(5);
	wizchip_write_direct(ndev, MR, MR_PB | MR_AI | MR_IND);

	return 0;
}

static const struct wizchip_ops wizchip_mmio_indirect_ops = {
	.chip_id = W5100,
	.read = wizchip_read_indirect,
	.write = wizchip_write_indirect,
	.read16 = wizchip_read16_indirect,
	.write16 = wizchip_write16_indirect,
	.readbulk = wizchip_readbulk_indirect,
	.writebulk = wizchip_writebulk_indirect,
	.init = wizchip_mmio_init,
	.reset = w5100_reset_indirect,
};

#if defined(CONFIG_WIZNET_BUS_DIRECT)

static int wizchip_read(struct wizchip_priv *priv, u32 addr)
{
	return priv->ops->read(priv->ndev, addr);
}

static int wizchip_write(struct wizchip_priv *priv, u32 addr, u8 data)
{
	return priv->ops->write(priv->ndev, addr, data);
}

static int wizchip_read16(struct wizchip_priv *priv, u32 addr)
{
	return priv->ops->read16(priv->ndev, addr);
}

static int wizchip_write16(struct wizchip_priv *priv, u32 addr, u16 data)
{
	return priv->ops->write16(priv->ndev, addr, data);
}

static int wizchip_readbulk(struct wizchip_priv *priv, u32 addr, u8 *buf, int len)
{
	return priv->ops->readbulk(priv->ndev, addr, buf, len);
}

static int wizchip_writebulk(struct wizchip_priv *priv, u32 addr, const u8 *buf,
			   int len)
{
	return priv->ops->writebulk(priv->ndev, addr, buf, len);
}

#elif defined(CONFIG_WIZNET_BUS_INDIRECT)

static int wizchip_read(struct wizchip_priv *priv, u32 addr)
{
	return priv->ops->read(priv->ndev, addr);
}

static int wizchip_write(struct wizchip_priv *priv, u32 addr, u8 data)
{
	return priv->ops->write(priv->ndev, addr, data);
}

static int wizchip_read16(struct wizchip_priv *priv, u32 addr)
{
	return priv->ops->read16(priv->ndev, addr);
}

static int wizchip_write16(struct wizchip_priv *priv, u32 addr, u16 data)
{
	return priv->ops->write16(priv->ndev, addr, data);
}

static int wizchip_readbulk(struct wizchip_priv *priv, u32 addr, u8 *buf, int len)
{
	return priv->ops->readbulk(priv->ndev, addr, buf, len);
}

static int wizchip_writebulk(struct wizchip_priv *priv, u32 addr, const u8 *buf,
			   int len)
{
	return priv->ops->writebulk(priv->ndev, addr, buf, len);
}

#else /* CONFIG_WIZNET_BUS_ANY */

static int wizchip_read(struct wizchip_priv *priv, u32 addr)
{
	return priv->ops->read(priv->ndev, addr);
}

static int wizchip_write(struct wizchip_priv *priv, u32 addr, u8 data)
{
	return priv->ops->write(priv->ndev, addr, data);
}

static int wizchip_read16(struct wizchip_priv *priv, u32 addr)
{
	return priv->ops->read16(priv->ndev, addr);
}

static int wizchip_write16(struct wizchip_priv *priv, u32 addr, u16 data)
{
	return priv->ops->write16(priv->ndev, addr, data);
}

static int wizchip_readbulk(struct wizchip_priv *priv, u32 addr, u8 *buf, int len)
{
	return priv->ops->readbulk(priv->ndev, addr, buf, len);
}

static int wizchip_writebulk(struct wizchip_priv *priv, u32 addr, const u8 *buf,
			   int len)
{
	return priv->ops->writebulk(priv->ndev, addr, buf, len);
}

#endif

static int wizchip_readbuf(struct wizchip_priv *priv, u16 offset, u8 *buf, int len)
{
	u32 addr;
	int remain = 0;
	int ret;
	const u32 mem_start = priv->s0_rx_buf;
	const u16 mem_size = priv->s0_rx_buf_size;

//	printk(KERN_INFO "1rd offset = 0x%1d \n", offset);

	offset %= mem_size;
//	printk(KERN_INFO "2rd offset = 0x%1d \n", offset);
	addr = mem_start + offset;
       if (offset + len > mem_size) {
               remain = (offset + len) % mem_size;
               len = mem_size - offset;
       }


	ret = wizchip_readbulk(priv, addr, buf, len);
	if (ret || !remain)
//	if (remain>0)
		return ret;

	return wizchip_readbulk(priv, mem_start, buf + len, remain);
}

static int wizchip_writebuf(struct wizchip_priv *priv, u16 offset, const u8 *buf,
			  int len)
{
	u32 addr;
	int ret;
	int remain = 0;

	const u32 mem_start =TX_MEM_START;
	const u16 mem_size =TX_MEM_SIZE;

	offset %= mem_size;
	addr = mem_start + offset;

	if (offset + len > mem_size) {
		remain = (offset + len) % mem_size;
		len = mem_size - offset;
//		len = len - remain;
	}
//		remain = ((int)offset + len) %0x4000;
//		len = len - remain;

	ret = wizchip_writebulk(priv, addr, buf, len);
	if (ret || !remain)
		return ret;

	return wizchip_writebulk(priv, mem_start , buf + len, remain);
}

static int wizchip_reset(struct wizchip_priv *priv)
{
	if (priv->ops->reset)
		return priv->ops->reset(priv->ndev);

	wizchip_write(priv, MR, MR_RST);
	mdelay(5);
	wizchip_write(priv, MR, MR_PB);

	return 0;



}

static int wizchip_command(struct wizchip_priv *priv, u16 cmd)
{
	unsigned long timeout;
       // printk(KERN_INFO "   wizchip_command 0x%08x // 0x%04x \n", W5100_S0_CR(priv), cmd );

	wizchip_write(priv, S0_CR(priv), cmd);

	timeout = jiffies + msecs_to_jiffies(100);

	while (wizchip_read(priv, S0_CR(priv)) != 0) {
		if (time_after(jiffies, timeout))
			return -EIO;
		cpu_relax();
	}

	return 0;
}

static void wizchip_write_macaddr(struct wizchip_priv *priv)
{
	struct net_device *ndev = priv->ndev;

	wizchip_writebulk(priv, SHAR, ndev->dev_addr, ETH_ALEN);
}

static void wizchip_socket_intr_mask(struct wizchip_priv *priv, u8 mask)
{
	u32 imr;

	if (priv->ops->chip_id == W5500)
		imr = SIMR;
	else if (priv->ops->chip_id == W6100)
		imr = SIMR;
	else
		imr = IMR;

	wizchip_write(priv, imr, mask);
}

static void wizchip_enable_intr(struct wizchip_priv *priv)
{
	wizchip_socket_intr_mask(priv, IR_S0);
}

static void wizchip_disable_intr(struct wizchip_priv *priv)
{
	wizchip_socket_intr_mask(priv, 0);
}


static void w5100_memory_configure(struct wizchip_priv *priv)
{
	/* Configure 16K of internal memory
	 * as 8K RX buffer and 8K TX buffer
	 */
	wizchip_write(priv, RMSR, 0x03);
	wizchip_write(priv, TMSR, 0x03);
}

static void w5200_memory_configure(struct wizchip_priv *priv)
{
	int i;

	/* Configure internal RX memory as 16K RX buffer and
	 * internal TX memory as 16K TX buffer
	 */
	wizchip_write(priv, W5200_Sn_RXMEM_SIZE(0), 0x10);
	wizchip_write(priv, W5200_Sn_TXMEM_SIZE(0), 0x10);

	for (i = 1; i < 8; i++) {
		wizchip_write(priv, W5200_Sn_RXMEM_SIZE(i), 0);
		wizchip_write(priv, W5200_Sn_TXMEM_SIZE(i), 0);
	}
}



static void w5500_memory_configure(struct wizchip_priv *priv)
{
	int i;

	/* Configure internal RX memory as 16K RX buffer and
	 * internal TX memory as 16K TX buffer
	 */
	wizchip_write(priv, W5500_Sn_RXMEM_SIZE(0), 0x10);
	wizchip_write(priv, W5500_Sn_TXMEM_SIZE(0), 0x10);

	for (i = 1; i < 8; i++) {
		wizchip_write(priv, W5500_Sn_RXMEM_SIZE(i), 0);
		wizchip_write(priv, W5500_Sn_TXMEM_SIZE(i), 0);
	}
}

static void w6100_memory_configure(struct wizchip_priv *priv)
{
	int i;

	printk(KERN_INFO "wizchip_read(priviiiii , 0x0010) =  0x%02X \n",wizchip_read(priv ,0x080010));
	printk(KERN_INFO "wizchip_read(priviiiii , 0x0030) =  0x%02X \n",wizchip_read(priv ,0x080030));
	/* Configure internal RX memory as 16K RX buffer and
	 * internal TX memory as 16K TX buffer
	 */
	wizchip_write(priv, W6100_Sn_RXMEM_SIZE(0), 0x10);
	wizchip_write(priv, W6100_Sn_TXMEM_SIZE(0), 0x10);

	for (i = 1; i < 8; i++) 
	{
		printk(KERN_INFO " Sn_RXMEM_SIZE =  0x%08X \n", W6100_Sn_RXMEM_SIZE(i));
		wizchip_write(priv, W6100_Sn_RXMEM_SIZE(i), 0);
		wizchip_write(priv, W6100_Sn_TXMEM_SIZE(i), 0);
		printk(KERN_INFO "wizchip_read W6100_Sn_RXMEM_SIZE(i) = 0x%02X \n",wizchip_read(priv ,W6100_Sn_RXMEM_SIZE(i)));
		printk(KERN_INFO "wizchip_read W6100_Sn_TXMEM_SIZE(i) = 0x%02X \n",  wizchip_read(priv , W6100_Sn_TXMEM_SIZE(i)));
	}
}

// TODO :: will be modified for optimal performance... by lihan
static int wizchip_hw_reset(struct wizchip_priv *priv)
{
	u32 rtr;

	wizchip_reset(priv);

	wizchip_disable_intr(priv);
	wizchip_write_macaddr(priv);

	// PHY 설정 (Auto Negotiation + PHY Reset 한번 줌)
	// 1. 강제 설정
	wizchip_write(priv, PHYCFG,0xE8); // Auto + Reset bit set
	udelay(10);
	wizchip_write(priv, PHYCFG, PHYCFG_AUTONEG);  // Reset bit clear (AutoNeg 시작)

	//	wizchip_write(priv, PHYCFG, PHYCFG_AUTONEG);
	//	udelay(10);  // 10us 정도 잠깐 대기
	//	wizchip_write(priv, PHYCFG, 0xC8);
	switch (priv->ops->chip_id) {
		case W5100:
			printk(KERN_INFO "   switch (priv->ops->chip_id) 5100  \n" );
			w5100_memory_configure(priv);
			rtr = RTR;
			break;
		case W5200:
			printk(KERN_INFO "   switch (priv->ops->chip_id) 5200  \n" );
			w5200_memory_configure(priv);
			rtr = RTR;
			break;
		case W5500:
			printk(KERN_INFO "   switch (priv->ops->chip_id) 5500  \n" );
			w5500_memory_configure(priv);
			rtr = RTR;
			break;
		case W6100:
			printk(KERN_INFO "   switch (priv->ops->chip_id) 6100  \n" );
			w6100_memory_configure(priv);
			rtr = RTR;
			break;
		default:
			return -EINVAL;
	}
	int reset = wizchip_read16(priv, rtr) ; 
	if (reset != RTR_DEFAULT)
	{	
		printk(KERN_INFO "reset =  %d RTR_DEFAULT = %d  \n" , reset , RTR_DEFAULT);
		return -ENODEV;
	}
	udelay(1000);
	
	u8 phycfg = wizchip_read(priv, 0x3000);
	printk(KERN_INFO "W6100 PHYCFG = 0x%02X (Link %s)\n", phycfg, (phycfg & 0x01) ? "UP" : "DOWN");

	return 0;
}

//TODO :: will be modified for all Chips... lihan
static void wizchip_hw_start(struct wizchip_priv *priv)
{

#if 0 //original code 

	if (!priv->promisc) {
		if (priv->ops->chip_id == W5500)
			mode |= S0_MR_MF;
		else
			mode |= S0_MR_MF;
	}
	w5100_write(priv, W5100_S0_MR(priv), mode);
	w5100_command(priv, S0_CR_OPEN);
	w5100_enable_intr(priv);
#endif 
	int test = wizchip_read(priv ,0x000000);

	test = wizchip_read(priv ,0x002004);
	test = wizchip_read(priv ,0x080000);

	test = wizchip_read(priv ,0x080024);
	//wizchip_write(priv, 0x080000, mode);
	wizchip_write(priv, 0x080000, 0x07);

	mdelay(500);
	mdelay(500);

	test = wizchip_read(priv ,0x080000);
	test = wizchip_read(priv ,0x080024);

	wizchip_command(priv, S0_CR_OPEN);
	 test = wizchip_read(priv ,0x080024);
	 test = wizchip_read(priv ,0x080000);
	wizchip_enable_intr(priv);


	 test = wizchip_read(priv ,0x080108);
	 test = wizchip_read(priv ,0x080200);
	 test = wizchip_read(priv ,0x080220);

}

static void wizchip_hw_close(struct wizchip_priv *priv)
{
	wizchip_disable_intr(priv);
	wizchip_command(priv, S0_CR_CLOSE);
}

/***********************************************************************
 *
 *   Device driver functions / callbacks
 *
 ***********************************************************************/

static void wizchip_get_drvinfo(struct net_device *ndev,
		struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, dev_name(ndev->dev.parent),
			sizeof(info->bus_info));
}

static u32 wizchip_get_link(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	if (gpio_is_valid(priv->link_gpio))
		return !!gpio_get_value(priv->link_gpio);

	return 1;
}

static u32 wizchip_get_msglevel(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	return priv->msg_enable;
}

static void wizchip_set_msglevel(struct net_device *ndev, u32 value)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	priv->msg_enable = value;
}

static int wizchip_get_regs_len(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);
    pr_info("[DBG] get_regs_len(): ndev=%p, priv=%p, map=%p\n",
            ndev, priv, priv ? priv->map : NULL);
	return COMMON_REGS_LEN + S0_REGS_LEN;
}				 

static void wizchip_get_regs(struct net_device *ndev,
		struct ethtool_regs *regs, void *buf)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	regs->version = 1;
	wizchip_readbulk(priv, COMMON_REGS, buf, COMMON_REGS_LEN);
	buf += COMMON_REGS_LEN;
	wizchip_readbulk(priv, S0_REGS(priv), buf, S0_REGS_LEN);
}

static void wizchip_restart(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	wizchip_hw_reset(priv);
	wizchip_hw_start(priv);
	ndev->stats.tx_errors++;
	netif_trans_update(ndev);
	netif_wake_queue(ndev);
}

static void wizchip_restart_work(struct work_struct *work)
{
	struct wizchip_priv *priv = container_of(work, struct wizchip_priv,
					       restart_work);

	wizchip_restart(priv->ndev);
}

static void wizchip_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	if (priv->ops->may_sleep)
		schedule_work(&priv->restart_work);
	else
		wizchip_restart(ndev);
}

static void wizchip_tx_skb(struct net_device *ndev, struct sk_buff *skb)
{
    struct wizchip_priv *priv = netdev_priv(ndev);
    u16 offset;

    // 1. TX 버퍼 여유 확인
    while (wizchip_read16(priv, S0_TX_FSR(priv)) < 0x2000)
        cpu_relax();

    // 2. 데이터 write
    offset = wizchip_read16(priv, S0_TX_WR(priv));
    wizchip_writebuf(priv, offset, skb->data, skb->len);
    wizchip_write16(priv, S0_TX_WR(priv), offset + skb->len);

    ndev->stats.tx_bytes += skb->len;
    ndev->stats.tx_packets++;
    dev_kfree_skb(skb);

    // 3. SEND 명령 (내부 대기 포함)
    wizchip_command(priv, S0_CR_SEND);

	// 4. (선택) SEND_OK 비트 클리어
	// if (wizchip_read(priv, S0_IR(priv)), S0_IR_SENDOK)
	// wizchip_write(priv, S0_IR(priv), S0_IR_SENDOK);
}

static void wizchip_tx_work(struct work_struct *work)
{
	struct wizchip_priv *priv = container_of(work, struct wizchip_priv,
					       tx_work);
	struct sk_buff *skb = priv->tx_skb;

	priv->tx_skb = NULL;

	if (WARN_ON(!skb))
		return;
	wizchip_tx_skb(priv->ndev, skb);
}
#if 0
static netdev_tx_t wizchip_start_tx(struct sk_buff *skb, struct net_device *ndev)
{
    struct wizchip_priv *priv = netdev_priv(ndev);
    unsigned long flags;

    /* 송신 큐 정지 */
    netif_stop_queue(ndev);

    /* 동기 전송 경로로 바로 전송 */
    wizchip_tx_skb(ndev, skb);

    /* 전송 완료 후 송신 큐 재개 */
    netif_wake_queue(ndev);

    return NETDEV_TX_OK;
}
#else
static netdev_tx_t wizchip_start_tx(struct sk_buff *skb, struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);

	if (priv->ops->may_sleep) {
		WARN_ON(priv->tx_skb);
		priv->tx_skb = skb;
		queue_work(priv->xfer_wq, &priv->tx_work);
	} else {
		wizchip_tx_skb(ndev, skb);
	}

	return NETDEV_TX_OK;
}
#endif 

static struct sk_buff *wizchip_rx_skb(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);
	struct sk_buff *skb;
	u16 rx_len;
	u16 offset;
	u8 header[2];
	u16 rx_buf_len = wizchip_read16(priv, S0_RX_RSR(priv));
	
	if (rx_buf_len == 0)
		return NULL;

	offset = wizchip_read16(priv, S0_RX_RD(priv));

	wizchip_readbuf(priv, offset, header, 2);
	rx_len = get_unaligned_be16(header) - 2;

	skb = netdev_alloc_skb_ip_align(ndev, rx_len);
	if (unlikely(!skb)) {
		wizchip_write16(priv, S0_RX_RD(priv), offset + rx_buf_len);
		wizchip_command(priv, S0_CR_RECV);
		ndev->stats.rx_dropped++;
		return NULL;
	}

	skb_put(skb, rx_len);
	wizchip_readbuf(priv, offset + 2, skb->data, rx_len);

	wizchip_write16(priv, S0_RX_RD(priv), offset + 2 + rx_len);
	wizchip_command(priv, S0_CR_RECV);
	skb->protocol = eth_type_trans(skb, ndev);

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += rx_len;

	return skb;
}

static void wizchip_rx_work(struct work_struct *work)
{
	struct wizchip_priv *priv = container_of(work, struct wizchip_priv,
					       rx_work);
	struct sk_buff *skb;

	while ((skb = wizchip_rx_skb(priv->ndev)))
		netif_rx(skb);

	wizchip_enable_intr(priv);
}

static int wizchip_napi_poll(struct napi_struct *napi, int budget)
{
	struct wizchip_priv *priv = container_of(napi, struct wizchip_priv, napi);
	int rx_count;

	for (rx_count = 0; rx_count < budget; rx_count++) {
		struct sk_buff *skb = wizchip_rx_skb(priv->ndev);

		if (skb)
			netif_receive_skb(skb);
		else
			break;
	}

	if (rx_count < budget) {
		napi_complete_done(napi, rx_count);
		wizchip_enable_intr(priv);
	}

	return rx_count;
}

static irqreturn_t wizchip_interrupt(int irq, void *ndev_instance)
{
	struct net_device *ndev = ndev_instance;
	struct wizchip_priv *priv = netdev_priv(ndev);

	int ir = wizchip_read(priv, S0_IR(priv));
	if (!ir)
		return IRQ_NONE;
//	wizchip_write(priv, S0_IR(priv), ir);
	wizchip_write(priv, set_S0_IR(priv), ir);

	if (ir & S0_IR_SENDOK) {
		netif_dbg(priv, tx_done, ndev, "tx done\n");
		netif_wake_queue(ndev);
	}

	if (ir & S0_IR_RECV) {
		wizchip_disable_intr(priv);

		if (priv->ops->may_sleep)
			queue_work(priv->xfer_wq, &priv->rx_work);
		else if (napi_schedule_prep(&priv->napi))
			__napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

static irqreturn_t wizchip_detect_link(int irq, void *ndev_instance)
{
	struct net_device *ndev = ndev_instance;
	struct wizchip_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		if (gpio_get_value(priv->link_gpio) != 0) {
			netif_info(priv, link, ndev, "link is up\n");
			netif_carrier_on(ndev);
		} else {
			netif_info(priv, link, ndev, "link is down\n");
			netif_carrier_off(ndev);
		}
	}

	return IRQ_HANDLED;
}

static void wizchip_setrx_work(struct work_struct *work)
{
	struct wizchip_priv *priv = container_of(work, struct wizchip_priv,
					       setrx_work);

	wizchip_hw_start(priv);
}

static void wizchip_set_rx_mode(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);
	bool set_promisc = (ndev->flags & IFF_PROMISC) != 0;

	if (priv->promisc != set_promisc) {
		priv->promisc = set_promisc;

		if (priv->ops->may_sleep)
			schedule_work(&priv->setrx_work);
		else
			wizchip_hw_start(priv);
	}
}


static int wizchip_set_macaddr(struct net_device *ndev, void *addr)
{
	struct wizchip_priv *priv = netdev_priv(ndev);
	struct sockaddr *sock_addr = addr;

	if (!is_valid_ether_addr(sock_addr->sa_data))
		return -EADDRNOTAVAIL;
	// memcpy(ndev->dev_addr, sock_addr->sa_data, ETH_ALEN);
	eth_hw_addr_set(ndev, sock_addr->sa_data);
	wizchip_write_macaddr(priv);
	return 0;
}

static int wizchip_open(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	netif_info(priv, ifup, ndev, "enabling\n");
	wizchip_hw_start(priv);
	napi_enable(&priv->napi);
	netif_start_queue(ndev);
	if (!gpio_is_valid(priv->link_gpio) ||
	    gpio_get_value(priv->link_gpio) != 0)
		netif_carrier_on(ndev);
	return 0;
}

static int wizchip_stop(struct net_device *ndev)
{
	struct wizchip_priv *priv = netdev_priv(ndev);

	netif_info(priv, ifdown, ndev, "shutting down\n");
	wizchip_hw_close(priv);
	netif_carrier_off(ndev);
	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	return 0;
}

static const struct ethtool_ops wizchip_ethtool_ops = {
	.get_drvinfo		= wizchip_get_drvinfo,
	.get_msglevel		= wizchip_get_msglevel,
	.set_msglevel		= wizchip_set_msglevel,
	.get_link			= wizchip_get_link,
	.get_regs_len		= wizchip_get_regs_len,
	.get_regs			= wizchip_get_regs,
};

static const struct net_device_ops wizchip_netdev_ops = {
	.ndo_open		= wizchip_open,
	.ndo_stop		= wizchip_stop,
	.ndo_start_xmit		= wizchip_start_tx,
	.ndo_tx_timeout		= wizchip_tx_timeout,
	.ndo_set_rx_mode	= wizchip_set_rx_mode,
	.ndo_set_mac_address	= wizchip_set_macaddr,
	.ndo_validate_addr	= eth_validate_addr,
};

static int wizchip_mmio_probe(struct platform_device *pdev)
{
	struct wiznet_platform_data *data = dev_get_platdata(&pdev->dev);
	const void *mac_addr = NULL;
	struct resource *mem;
	const struct wizchip_ops *ops;
	int irq;

	if (data && is_valid_ether_addr(data->mac_addr))
		mac_addr = data->mac_addr;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;
	if (resource_size(mem) < W5100_BUS_DIRECT_SIZE)
		ops = &wizchip_mmio_indirect_ops;
	else
		ops = &wizchip_mmio_direct_ops;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	return wizchip_probe(&pdev->dev, ops, sizeof(struct wizchip_mmio_priv),
			   mac_addr, irq, data ? data->link_gpio : -EINVAL);
}

static int wizchip_mmio_remove(struct platform_device *pdev)
{
	return wizchip_remove(&pdev->dev);
}

void *wizchip_ops_priv(const struct net_device *ndev)
{
	return netdev_priv(ndev) +
	       ALIGN(sizeof(struct wizchip_priv), NETDEV_ALIGN);
}
EXPORT_SYMBOL_GPL(wizchip_ops_priv);

int wizchip_probe(struct device *dev, const struct wizchip_ops *ops,
		int sizeof_ops_priv, const void *mac_addr, int irq,
		int link_gpio)
{

	struct wizchip_priv *priv;
	struct net_device *ndev;
	int err;
	size_t alloc_size;

	printk(KERN_INFO "probe start" );
	alloc_size = sizeof(*priv);
	if (sizeof_ops_priv) {
		alloc_size = ALIGN(alloc_size, NETDEV_ALIGN);
		alloc_size += sizeof_ops_priv;
	}
	alloc_size += NETDEV_ALIGN - 1;

	ndev = alloc_etherdev(alloc_size);
	if (!ndev)
	{
		return -ENOMEM;

		printk(KERN_INFO "probe error");
	
	}	
		printk(KERN_INFO "probe no error  ");

	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);
	priv = netdev_priv(ndev);

	switch (ops->chip_id) {
	case W5100:
		priv->map = wiz5100_map;
		priv->s0_regs = wiz5100_map[REG_S0_REGS];
		priv->s0_tx_buf = TX_MEM_START;
		priv->s0_tx_buf_size = TX_MEM_SIZE;
		priv->s0_rx_buf = RX_MEM_START;
		priv->s0_rx_buf_size = RX_MEM_SIZE;
		break;
	case W5200:
		priv->map = wiz5200_map;
		priv->s0_regs = wiz5200_map[REG_S0_REGS];
		priv->s0_tx_buf = TX_MEM_START;
		priv->s0_tx_buf_size = TX_MEM_SIZE;
		priv->s0_rx_buf = RX_MEM_START;
		priv->s0_rx_buf_size = RX_MEM_SIZE;
		break;
	case W5500:
		priv->map = wiz5500_map;
		priv->s0_regs =wiz5100_map[REG_S0_REGS];
		priv->s0_tx_buf = TX_MEM_START;
		priv->s0_tx_buf_size = TX_MEM_SIZE;
		priv->s0_rx_buf = RX_MEM_START;
		priv->s0_rx_buf_size = RX_MEM_SIZE;
		break;
	case W6100:
		priv->map = wiz6100_map;
		priv->s0_regs = wiz5100_map[REG_S0_REGS];
		priv->s0_tx_buf = TX_MEM_START;
		priv->s0_tx_buf_size = TX_MEM_SIZE;
		priv->s0_rx_buf = RX_MEM_START;
		priv->s0_rx_buf_size = RX_MEM_SIZE;
		break;
	case W6300:
		priv->map = wiz6300_map;
		priv->s0_regs = wiz5100_map[REG_S0_REGS];
		priv->s0_tx_buf = TX_MEM_START;
		priv->s0_tx_buf_size = TX_MEM_SIZE;
		priv->s0_rx_buf = RX_MEM_START;
		priv->s0_rx_buf_size = RX_MEM_SIZE;
		break;
	default:
		printk(KERN_ERR "Unsupported WIZnet chip ID:\n");
		printk(KERN_ERR "Unsupported WIZnet chip ID:\n");
		printk(KERN_ERR "Unsupported WIZnet chip ID:\n");
		printk(KERN_ERR "Unsupported WIZnet chip ID:\n");
		printk(KERN_ERR "Unsupported WIZnet chip ID:\n");
		err = -EINVAL;
		goto err_register;
	}

	printk(KERN_INFO "probe TEST2 = %04x",SHAR );
	printk(KERN_INFO "probe TEST2 = %04x",IR );
	printk(KERN_INFO "probe TEST2 = %04x",IMR );
	printk(KERN_INFO "probe TES2T = %04x",COMMON_REGS_LEN );
	printk(KERN_INFO "probe TEST2 2= %04x",COMMON_REGS_LEN );
	printk(KERN_INFO "probe TES22T = %04x",S0_REGS_LEN );

	priv->ndev = ndev;
	priv->ops = ops;
	priv->irq = irq;
	priv->link_gpio = link_gpio;



	ndev->netdev_ops = &wizchip_netdev_ops;
	ndev->ethtool_ops = &wizchip_ethtool_ops;
	netif_napi_add(ndev, &priv->napi, wizchip_napi_poll);
	

	/* This chip doesn't support VLAN packets with normal MTU,
	 * so disable VLAN for this device.
	 */
	ndev->features |= NETIF_F_VLAN_CHALLENGED;

	err = register_netdev(ndev);
	if (err < 0){
		goto err_register;
	}
	priv->xfer_wq = alloc_workqueue("%s", WQ_MEM_RECLAIM, 0,
					netdev_name(ndev));
	if (!priv->xfer_wq) {
		err = -ENOMEM;
		goto err_wq;
	}
	printk(KERN_INFO "probe TEST = %04x",SHAR );
	printk(KERN_INFO "probe TEST = %04x",IR );
	printk(KERN_INFO "probe TEST = %04x",IMR );


	INIT_WORK(&priv->rx_work, wizchip_rx_work);
	INIT_WORK(&priv->tx_work,   wizchip_tx_work);
	INIT_WORK(&priv->setrx_work, wizchip_setrx_work);
	INIT_WORK(&priv->restart_work, wizchip_restart_work);

	if (mac_addr)
		//memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
		eth_hw_addr_set(ndev, mac_addr);
	else
		eth_hw_addr_random(ndev);

	if (priv->ops->init) {
		err = priv->ops->init(priv->ndev);
		if (err){
			goto err_hw;}
	}

	err = wizchip_hw_reset(priv);

	if (err){

		goto err_hw;
	}
	if (ops->may_sleep) {
		err = request_threaded_irq(priv->irq, NULL, wizchip_interrupt,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   netdev_name(ndev), ndev);
	} else {
		err = request_irq(priv->irq, wizchip_interrupt,
				  IRQF_TRIGGER_LOW, netdev_name(ndev), ndev);
	}
	printk(KERN_INFO "int-ok" );
	
	if (err){
		goto err_hw;
		}

	if (gpio_is_valid(priv->link_gpio)) {
		char *link_name = devm_kzalloc(dev, 16, GFP_KERNEL);

		if (!link_name) {
			err = -ENOMEM;
			goto err_gpio;
		}
		snprintf(link_name, 16, "%s-link", netdev_name(ndev));
		priv->link_irq = gpio_to_irq(priv->link_gpio);
		if (request_any_context_irq(priv->link_irq, wizchip_detect_link,
					    IRQF_TRIGGER_RISING |
					    IRQF_TRIGGER_FALLING,
					    link_name, priv->ndev) < 0)
			priv->link_gpio = -EINVAL;
	}

	return 0;

err_gpio:
	free_irq(priv->irq, ndev);
err_hw:
	destroy_workqueue(priv->xfer_wq);
err_wq:
	unregister_netdev(ndev);
err_register:
	free_netdev(ndev);
	return err;
}
EXPORT_SYMBOL_GPL(wizchip_probe);

int wizchip_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct wizchip_priv *priv = netdev_priv(ndev);

	wizchip_hw_reset(priv);
	free_irq(priv->irq, ndev);
	if (gpio_is_valid(priv->link_gpio))
		free_irq(priv->link_irq, ndev);

	flush_work(&priv->setrx_work);
	flush_work(&priv->restart_work);
	destroy_workqueue(priv->xfer_wq);

	unregister_netdev(ndev);
	free_netdev(ndev);
	return 0;
}
EXPORT_SYMBOL_GPL(wizchip_remove);

#ifdef CONFIG_PM_SLEEP
static int wizchip_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct wizchip_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_carrier_off(ndev);
		netif_device_detach(ndev);

		wizchip_hw_close(priv);
	}
	return 0;
}

static int wizchip_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct wizchip_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		wizchip_hw_reset(priv);
		wizchip_hw_start(priv);

		netif_device_attach(ndev);
		if (!gpio_is_valid(priv->link_gpio) ||
		    gpio_get_value(priv->link_gpio) != 0)
			netif_carrier_on(ndev);
	}
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

SIMPLE_DEV_PM_OPS(wizchip_pm_ops, wizchip_suspend, wizchip_resume);
EXPORT_SYMBOL_GPL(wizchip_pm_ops);

static struct platform_driver wizchip_mmio_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.pm	= &wizchip_pm_ops,
	},
	.probe		= wizchip_mmio_probe,
	.remove		= wizchip_mmio_remove,
};
module_platform_driver(wizchip_mmio_driver);
