// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Ethernet driver for the WIZnet W5100/W5500 chip.
 *
 * Copyright (C) 2016 Akinobu Mita <akinobu.mita@gmail.com>
 *
 * Datasheet:
 * http://www.wiznet.co.kr/wp-content/uploads/wiznethome/Chip/W5100/Document/W5100_Datasheet_v1.2.6.pdf
 * http://wizwiki.net/wiki/lib/exe/fetch.php?media=products:w5500:w5500_ds_v106e_141230.pdf
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/of_net.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

#include "wizchip_driver.h"



/******************************************************************************
 * W5100 SPI Interface Implementation
 * - Basic SPI read/write operations for W5100 chip  
 * - Simple byte-by-byte operations for bulk transfers
 * - Uses 0xF0/0x0F opcodes for write/read operations
 *****************************************************************************/

#define W5100_SPI_WRITE_OPCODE 0xf0
#define W5100_SPI_READ_OPCODE 0x0f

// ...existing w5100 functions...
static int w5100_spi_read(struct net_device *ndev, u32 addr)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[3] = { W5100_SPI_READ_OPCODE, addr >> 8, addr & 0xff };
	u8 data;
	int ret;

	ret = spi_write_then_read(spi, cmd, sizeof(cmd), &data, 1);

	return ret ? ret : data;
}

static int w5100_spi_write(struct net_device *ndev, u32 addr, u8 data)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[4] = { W5100_SPI_WRITE_OPCODE, addr >> 8, addr & 0xff, data};

	return spi_write_then_read(spi, cmd, sizeof(cmd), NULL, 0);
}

static int w5100_spi_read16(struct net_device *ndev, u32 addr)
{
	u16 data;
	int ret;

	ret = w5100_spi_read(ndev, addr);
	if (ret < 0)
		return ret;
	data = ret << 8;
	ret = w5100_spi_read(ndev, addr + 1);

	return ret < 0 ? ret : data | ret;
}

static int w5100_spi_write16(struct net_device *ndev, u32 addr, u16 data)
{
	int ret;

	ret = w5100_spi_write(ndev, addr, data >> 8);
	if (ret)
		return ret;

	return w5100_spi_write(ndev, addr + 1, data & 0xff);
}

static int w5100_spi_readbulk(struct net_device *ndev, u32 addr, u8 *buf,
			      int len)
{
	int i;

	for (i = 0; i < len; i++) {
		int ret = w5100_spi_read(ndev, addr + i);

		if (ret < 0)
			return ret;
		buf[i] = ret;
	}

	return 0;
}

static int w5100_spi_writebulk(struct net_device *ndev, u32 addr, const u8 *buf,
			       int len)
{
	int i;

	for (i = 0; i < len; i++) {
		int ret = w5100_spi_write(ndev, addr + i, buf[i]);

		if (ret)
			return ret;
	}

	return 0;
}

static const struct wizchip_ops w5100_spi_ops = {
	.may_sleep = true,
	.chip_id = W5100,
	.read = w5100_spi_read,
	.write = w5100_spi_write,
	.read16 = w5100_spi_read16,
	.write16 = w5100_spi_write16,
	.readbulk = w5100_spi_readbulk,
	.writebulk = w5100_spi_writebulk,
};



/* End of W5100 Implementation */



/******************************************************************************
 * W5500 SPI Interface Implementation
 * - Block Select Bit (BSB) based addressing scheme
 * - Advanced SPI control with 5-bit block selection  
 * - Optimized for high-speed SPI operations
 * - Uses bit manipulation for read/write control
 *****************************************************************************/


#define W5500_SPI_BLOCK_SELECT(addr) (((addr) >> 16) & 0x1f)
#define W5500_SPI_READ_CONTROL(addr) (W5500_SPI_BLOCK_SELECT(addr) << 3)
#define W5500_SPI_WRITE_CONTROL(addr)	\
	((W5500_SPI_BLOCK_SELECT(addr) << 3) | BIT(2))
// ...existing w5500 functions...


struct w5500_spi_priv {
	/* Serialize access to cmd_buf */
	struct mutex cmd_lock;

	/* DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8 cmd_buf[3] ____cacheline_aligned;
};

static struct w5500_spi_priv *w5500_spi_priv(struct net_device *ndev)
{
	return wizchip_ops_priv(ndev);
}

static int w5500_spi_init(struct net_device *ndev)
{
	struct w5500_spi_priv *spi_priv = w5500_spi_priv(ndev);

	mutex_init(&spi_priv->cmd_lock);

	return 0;
}

static int w5500_spi_read(struct net_device *ndev, u32 addr)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[3] = {
		addr >> 8,
		addr,
		W5500_SPI_READ_CONTROL(addr)
	};
	u8 data;
	int ret;

	ret = spi_write_then_read(spi, cmd, sizeof(cmd), &data, 1);

	return ret ? ret : data;
}

static int w5500_spi_write(struct net_device *ndev, u32 addr, u8 data)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[4] = {
		addr >> 8,
		addr,
		W5500_SPI_WRITE_CONTROL(addr),
		data
	};

	return spi_write_then_read(spi, cmd, sizeof(cmd), NULL, 0);
}

static int w5500_spi_read16(struct net_device *ndev, u32 addr)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[3] = {
		addr >> 8,
		addr,
		W5500_SPI_READ_CONTROL(addr)
	};
	__be16 data;
	int ret;

	ret = spi_write_then_read(spi, cmd, sizeof(cmd), &data, sizeof(data));

	return ret ? ret : be16_to_cpu(data);
}

static int w5500_spi_write16(struct net_device *ndev, u32 addr, u16 data)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[5] = {
		addr >> 8,
		addr,
		W5500_SPI_WRITE_CONTROL(addr),
		data >> 8,
		data
	};

	return spi_write_then_read(spi, cmd, sizeof(cmd), NULL, 0);
}

static int w5500_spi_readbulk(struct net_device *ndev, u32 addr, u8 *buf,
			      int len)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	struct w5500_spi_priv *spi_priv = w5500_spi_priv(ndev);
	struct spi_transfer xfer[] = {
		{
			.tx_buf = spi_priv->cmd_buf,
			.len = sizeof(spi_priv->cmd_buf),
		},
		{
			.rx_buf = buf,
			.len = len,
		},
	};
	int ret;

	mutex_lock(&spi_priv->cmd_lock);

	spi_priv->cmd_buf[0] = addr >> 8;
	spi_priv->cmd_buf[1] = addr;
	spi_priv->cmd_buf[2] = W5500_SPI_READ_CONTROL(addr);
	ret = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));

	mutex_unlock(&spi_priv->cmd_lock);

	return ret;
}

static int w5500_spi_writebulk(struct net_device *ndev, u32 addr, const u8 *buf,
			       int len)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	struct w5500_spi_priv *spi_priv = w5500_spi_priv(ndev);
	struct spi_transfer xfer[] = {
		{
			.tx_buf = spi_priv->cmd_buf,
			.len = sizeof(spi_priv->cmd_buf),
		},
		{
			.tx_buf = buf,
			.len = len,
		},
	};
	int ret;

	mutex_lock(&spi_priv->cmd_lock);

	spi_priv->cmd_buf[0] = addr >> 8;
	spi_priv->cmd_buf[1] = addr;
	spi_priv->cmd_buf[2] = W5500_SPI_WRITE_CONTROL(addr);
	ret = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));

	mutex_unlock(&spi_priv->cmd_lock);

	return ret;
}

static const struct wizchip_ops w5500_ops = {
	.may_sleep = true,
	.chip_id = W5500,
	.read = w5500_spi_read,
	.write = w5500_spi_write,
	.read16 = w5500_spi_read16,
	.write16 = w5500_spi_write16,
	.readbulk = w5500_spi_readbulk,
	.writebulk = w5500_spi_writebulk,
	.init = w5500_spi_init,
};

/* End of W5500 Implementation */

/******************************************************************************
 * W6100 SPI Interface Implementation
 * - Next generation WIZnet chip with enhanced features
 * - Extended 8-bit block select addressing 
 * - Advanced bulk transfer operations with dynamic memory allocation
 * - Improved performance for high-throughput applications
 *****************************************************************************/


#define W6100_SPI_BLOCK_SELECT(addr) (((addr) >> 16) & 0xff)
#define W6100_SPI_READ_CONTROL(addr) (W6100_SPI_BLOCK_SELECT(addr) )
#define W6100_SPI_WRITE_CONTROL(addr)	\
	((W6100_SPI_BLOCK_SELECT(addr)) | 0x04)
	
// ...existing w6100 functions...

struct w6100_spi_priv {
	/* Serialize access to cmd_buf */
	struct mutex cmd_lock;

	/* DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8 cmd_buf[3] ____cacheline_aligned;
};

static struct w6100_spi_priv *w6100_spi_priv(struct net_device *ndev)
{
	return wizchip_ops_priv(ndev);
}

static int w6100_spi_init(struct net_device *ndev)
{
	struct w6100_spi_priv *spi_priv = w6100_spi_priv(ndev);

	mutex_init(&spi_priv->cmd_lock);

	return 0;
}

static int w6100_spi_read(struct net_device *ndev, u32 addr)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
//	printk(KERN_INFO "w6100_spi_read add =0x%08X i\n",addr);
	u8 cmd[3] = {
		(addr >> 8)& 0xff ,
		addr &0xff,
		W6100_SPI_READ_CONTROL(addr)
	};
	u8 data;
	int ret;

	ret = spi_write_then_read(spi, cmd, sizeof(cmd), &data, 1);

	return ret ? ret : data;
}

static int w6100_spi_write(struct net_device *ndev, u32 addr, u8 data)
{
//	printk(KERN_INFO "w6100_spi_write add =  0x%08X i\n",addr );
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[4] = {
		(addr >> 8)& 0xff ,
		addr &0xff,
		W6100_SPI_WRITE_CONTROL(addr),
		data
	};
//	printk(KERN_INFO "wcmd[4] = 0x%02X |0x%02X | 0x%02X | 0x%02X  n", cmd[0], cmd[1], cmd[2], cmd[3]);
	return spi_write_then_read(spi, cmd, sizeof(cmd), NULL, 0);
}

static int w6100_spi_read16(struct net_device *ndev, u32 addr)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[3] = {
		addr >> 8,
		addr,
		W6100_SPI_READ_CONTROL(addr)
	};
	__be16 data;
	int ret;

//	printk(KERN_INFO "w6100_spi_read16 = 0x%02X |0x%02X | 0x%02X | 0x%02X  n", cmd[0], cmd[1], cmd[2], cmd[3]);
	ret = spi_write_then_read(spi, cmd, sizeof(cmd), &data, sizeof(data));

//	printk(KERN_INFO "result = 0x%02X |0x%02X \n", data & 0xff , (data>>8 )& 0xff);
	return ret ? ret : be16_to_cpu(data);
}

static int w6100_spi_write16(struct net_device *ndev, u32 addr, u16 data)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[5] = {
		addr >> 8,
		addr,
		W6100_SPI_WRITE_CONTROL(addr),
		data >> 8,
		data
	};

//	printk(KERN_INFO " w6100_spi_write16  = 0x%02X |0x%02X | 0x%02X | 0x%02X | 0x%02x \n", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
	return spi_write_then_read(spi, cmd, sizeof(cmd), NULL, 0);
}

static int w6100_spi_readbulk(struct net_device *ndev, u32 addr, u8 *buf, int len)
{
    struct spi_device *spi = to_spi_device(ndev->dev.parent);
    struct w6100_spi_priv *spi_priv = w6100_spi_priv(ndev);
    int ret;
    int cmd_len = 3;
    int total_len = cmd_len + len;

    u8 *tx_buf = NULL;
    u8 *rx_buf = NULL;

    mutex_lock(&spi_priv->cmd_lock);

    // 명령어 구성
    spi_priv->cmd_buf[0] = addr >> 8;
    spi_priv->cmd_buf[1] = addr;
    spi_priv->cmd_buf[2] = W6100_SPI_READ_CONTROL(addr);

    tx_buf = kzalloc(total_len, GFP_KERNEL);
    rx_buf = kzalloc(total_len, GFP_KERNEL);
    if (!tx_buf || !rx_buf) {
        ret = -ENOMEM;
        goto out_free;
    }

    // 전송 버퍼 구성: command + dummy
    memcpy(tx_buf, spi_priv->cmd_buf, cmd_len);
    {
        struct spi_transfer xfer = {
            .tx_buf = tx_buf,
            .rx_buf = rx_buf,
            .len = total_len,
        };
        ret = spi_sync_transfer(spi, &xfer, 1);
        if (ret == 0)
            memcpy(buf, rx_buf + cmd_len, len);
    }

out_free:
    kfree(tx_buf);
    kfree(rx_buf);
    mutex_unlock(&spi_priv->cmd_lock);
    return ret;
}

static int w6100_spi_writebulk(struct net_device *ndev, u32 addr, const u8 *buf, int len)
{
    struct spi_device *spi = to_spi_device(ndev->dev.parent);
    struct w6100_spi_priv *spi_priv = w6100_spi_priv(ndev);
    int ret;
    int cmd_len = 3;
    int total_len = cmd_len + len;

    u8 *tx_buf = NULL;

    mutex_lock(&spi_priv->cmd_lock);

    // 명령어 구성
    spi_priv->cmd_buf[0] = addr >> 8;
    spi_priv->cmd_buf[1] = addr;
    spi_priv->cmd_buf[2] = W6100_SPI_WRITE_CONTROL(addr);

    tx_buf = kzalloc(total_len, GFP_KERNEL);
    if (!tx_buf) {
        ret = -ENOMEM;
        goto out_unlock;
    }

    // command + data 결합
    memcpy(tx_buf, spi_priv->cmd_buf, cmd_len);
    memcpy(tx_buf + cmd_len, buf, len);

    {
        struct spi_transfer xfer = {
            .tx_buf = tx_buf,
            .len = total_len,
        };
        ret = spi_sync_transfer(spi, &xfer, 1);
    }

    kfree(tx_buf);

out_unlock:
    mutex_unlock(&spi_priv->cmd_lock);
    return ret;
}


static const struct wizchip_ops w6100_ops = {
	.may_sleep = true,
	.chip_id = W6100,
	.read = w6100_spi_read,
	.write = w6100_spi_write,
	.read16 = w6100_spi_read16,
	.write16 = w6100_spi_write16,
	.readbulk = w6100_spi_readbulk,
	.writebulk = w6100_spi_writebulk,
	.init = w6100_spi_init,
};


/* End of W6100 Implementation */

/******************************************************************************
 * W6300 SPI Interface Implementation
 * - Latest WIZnet chip with advanced networking features
 * - Similar to W6100 but with enhanced capabilities
 * - Optimized bulk operations for high-performance applications  
 * - Enterprise-grade features and improved reliability
 *****************************************************************************/

#define w6300_SPI_BLOCK_SELECT(addr) (((addr) >> 16) & 0xff)
#define w6300_SPI_READ_CONTROL(addr) (w6300_SPI_BLOCK_SELECT(addr) )
#define w6300_SPI_WRITE_CONTROL(addr)	\
	((w6300_SPI_BLOCK_SELECT(addr)) | 0x04)

// ...existing w6300 functions...

struct w6300_spi_priv {
	/* Serialize access to cmd_buf */
	struct mutex cmd_lock;

	/* DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8 cmd_buf[3] ____cacheline_aligned;
};

static struct w6300_spi_priv *w6300_spi_priv(struct net_device *ndev)
{
	return wizchip_ops_priv(ndev);
}

static int w6300_spi_init(struct net_device *ndev)
{
	struct w6300_spi_priv *spi_priv = w6300_spi_priv(ndev);

	mutex_init(&spi_priv->cmd_lock);

	return 0;
}

static int w6300_spi_read(struct net_device *ndev, u32 addr)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
//	printk(KERN_INFO "w6300_spi_read add =0x%08X i\n",addr);
	u8 cmd[3] = {
		(addr >> 8)& 0xff ,
		addr &0xff,
		w6300_SPI_READ_CONTROL(addr)
	};
	u8 data;
	int ret;

	ret = spi_write_then_read(spi, cmd, sizeof(cmd), &data, 1);

	return ret ? ret : data;
}

static int w6300_spi_write(struct net_device *ndev, u32 addr, u8 data)
{
//	printk(KERN_INFO "w6300_spi_write add =  0x%08X i\n",addr );
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[4] = {
		(addr >> 8)& 0xff ,
		addr &0xff,
		w6300_SPI_WRITE_CONTROL(addr),
		data
	};
//	printk(KERN_INFO "wcmd[4] = 0x%02X |0x%02X | 0x%02X | 0x%02X  n", cmd[0], cmd[1], cmd[2], cmd[3]);
	return spi_write_then_read(spi, cmd, sizeof(cmd), NULL, 0);
}

static int w6300_spi_read16(struct net_device *ndev, u32 addr)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[3] = {
		addr >> 8,
		addr,
		w6300_SPI_READ_CONTROL(addr)
	};
	__be16 data;
	int ret;

//	printk(KERN_INFO "w6300_spi_read16 = 0x%02X |0x%02X | 0x%02X | 0x%02X  n", cmd[0], cmd[1], cmd[2], cmd[3]);
	ret = spi_write_then_read(spi, cmd, sizeof(cmd), &data, sizeof(data));

//	printk(KERN_INFO "result = 0x%02X |0x%02X \n", data & 0xff , (data>>8 )& 0xff);
	return ret ? ret : be16_to_cpu(data);
}

static int w6300_spi_write16(struct net_device *ndev, u32 addr, u16 data)
{
	struct spi_device *spi = to_spi_device(ndev->dev.parent);
	u8 cmd[5] = {
		addr >> 8,
		addr,
		w6300_SPI_WRITE_CONTROL(addr),
		data >> 8,
		data
	};

//	printk(KERN_INFO " w6300_spi_write16  = 0x%02X |0x%02X | 0x%02X | 0x%02X | 0x%02x \n", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
	return spi_write_then_read(spi, cmd, sizeof(cmd), NULL, 0);
}


static int w6300_spi_readbulk(struct net_device *ndev, u32 addr, u8 *buf, int len)
{
    struct spi_device *spi = to_spi_device(ndev->dev.parent);
    struct w6300_spi_priv *spi_priv = w6300_spi_priv(ndev);
    int ret;
    int cmd_len = 3;
    int total_len = cmd_len + len;

    u8 *tx_buf = NULL;
    u8 *rx_buf = NULL;

    mutex_lock(&spi_priv->cmd_lock);

    // 명령어 구성
    spi_priv->cmd_buf[0] = addr >> 8;
    spi_priv->cmd_buf[1] = addr;
    spi_priv->cmd_buf[2] = w6300_SPI_READ_CONTROL(addr);

    tx_buf = kzalloc(total_len, GFP_KERNEL);
    rx_buf = kzalloc(total_len, GFP_KERNEL);
    if (!tx_buf || !rx_buf) {
        ret = -ENOMEM;
        goto out_free;
    }

    // 전송 버퍼 구성: command + dummy
    memcpy(tx_buf, spi_priv->cmd_buf, cmd_len);

    {
        struct spi_transfer xfer = {
            .tx_buf = tx_buf,
            .rx_buf = rx_buf,
            .len = total_len,
        };
        ret = spi_sync_transfer(spi, &xfer, 1);
        if (ret == 0)
            memcpy(buf, rx_buf + cmd_len, len);
    }

out_free:
    kfree(tx_buf);
    kfree(rx_buf);
    mutex_unlock(&spi_priv->cmd_lock);
    return ret;
}

static int w6300_spi_writebulk(struct net_device *ndev, u32 addr, const u8 *buf, int len)
{
    struct spi_device *spi = to_spi_device(ndev->dev.parent);
    struct w6300_spi_priv *spi_priv = w6300_spi_priv(ndev);
    int ret;
    int cmd_len = 3;
    int total_len = cmd_len + len;

    u8 *tx_buf = NULL;

    mutex_lock(&spi_priv->cmd_lock);

    // 명령어 구성
    spi_priv->cmd_buf[0] = addr >> 8;
    spi_priv->cmd_buf[1] = addr;
    spi_priv->cmd_buf[2] = w6300_SPI_WRITE_CONTROL(addr);

    tx_buf = kzalloc(total_len, GFP_KERNEL);
    if (!tx_buf) {
        ret = -ENOMEM;
        goto out_unlock;
    }

    // command + data 결합
    memcpy(tx_buf, spi_priv->cmd_buf, cmd_len);
    memcpy(tx_buf + cmd_len, buf, len);

    {
        struct spi_transfer xfer = {
            .tx_buf = tx_buf,
            .len = total_len,
        };
        ret = spi_sync_transfer(spi, &xfer, 1);
    }

    kfree(tx_buf);

out_unlock:
    mutex_unlock(&spi_priv->cmd_lock);
    return ret;
}



static const struct wizchip_ops w6300_ops = {
	.may_sleep = true,
	.chip_id = W6300,
	.read = w6300_spi_read,
	.write = w6300_spi_write,
	.read16 = w6300_spi_read16,
	.write16 = w6300_spi_write16,
	.readbulk = w6300_spi_readbulk,
	.writebulk = w6300_spi_writebulk,
	.init = w6300_spi_init,
};
/**********************/


/* End of W6300 Implementation */

/******************************************************************************
 * Common Driver Infrastructure
 * - Device tree matching and probe functions
 * - Unified driver registration and management
 * - Support for all WIZnet chip variants (W5100/W5500/W6100/W6300)
 *****************************************************************************/


static const struct of_device_id wizchip_of_match[] = {
	{ .compatible = "wiznet,w5100", .data = (const void*)W5100, },
	{ .compatible = "wiznet,w5500", .data = (const void*)W5500, },
	{ .compatible = "wiznet,w6100", .data = (const void*)W6100, },
	{ .compatible = "wiznet,w6300", .data = (const void*)W6300, },
	{ },
};
MODULE_DEVICE_TABLE(of, wizchip_of_match);

static int wizchip_spi_probe(struct spi_device *spi)
{

	const struct of_device_id *of_id;
	const struct wizchip_ops *ops;
	kernel_ulong_t driver_data;
	const void *mac = NULL;
	u8 tmpmac[ETH_ALEN];
	int priv_size;
	int ret;
	
			printk(KERN_INFO "wiznet chip test - probe start\n");
	ret = of_get_mac_address(spi->dev.of_node, tmpmac);
	if (!ret)
		mac = tmpmac;

	if (spi->dev.of_node) {
		of_id = of_match_device(wizchip_of_match, &spi->dev);
		if (!of_id){
			printk(KERN_INFO "wiznet chip test - of_id\n");
			return -ENODEV;
		}
		driver_data = (kernel_ulong_t)of_id->data;
	} else {
		driver_data = spi_get_device_id(spi)->driver_data;
	}

	switch (driver_data) {
	case W5100:
		printk(KERN_INFO "wiznet chip test - driver_data = W5100\n");
		ops = &w5100_spi_ops;
		priv_size = 0;
		break;
	case W5500:
		printk(KERN_INFO "wiznet chip test - driver_data = W5500\n");
		ops = &w5500_ops;
		priv_size = sizeof(struct w5500_spi_priv);
		break;
	case W6100:
		printk(KERN_INFO "wiznet chip test - driver_data = W6100\n");
		ops = &w6100_ops;
		priv_size = sizeof(struct w6100_spi_priv);
		break;
	case W6300:
		printk(KERN_INFO "wiznet chip test - driver_data = W6300\n");
		ops = &w6300_ops;
		priv_size = sizeof(struct w6300_spi_priv);
		break;
	default:

		printk(KERN_INFO "wiznet chip test - driver_data = default\n");
		return -EINVAL;
	}

	return wizchip_probe(&spi->dev, ops, priv_size, mac, spi->irq, -EINVAL);
}

static void wizchip_spi_remove(struct spi_device *spi)
{
//	return wizchip_remove(&spi->dev);
	wizchip_remove(&spi->dev);
}

static const struct spi_device_id wizchip_spi_ids[] = {
	{ "w5100", W5100 },
	{ "w5500", W5500 },
	{ "w6100", W6100 },
	{ "w6300", W6300 },
	{}
};
MODULE_DEVICE_TABLE(spi, wizchip_spi_ids);

static struct spi_driver wizchip_spi_driver = {
	.driver		= {
		.name	= "wizchip",
	
		.pm	= &wizchip_pm_ops,
		.of_match_table = wizchip_of_match,
	},
	.probe		= wizchip_spi_probe,
	.remove		= wizchip_spi_remove,
	.id_table	= wizchip_spi_ids,
};
module_spi_driver(wizchip_spi_driver);

MODULE_DESCRIPTION("WIZnet W5100/W5500/w6100/w6300 Ethernet driver for SPI mode");
MODULE_AUTHOR("Akinobu Mita <akinobu.mita@gmail.com>");
MODULE_LICENSE("GPL");
