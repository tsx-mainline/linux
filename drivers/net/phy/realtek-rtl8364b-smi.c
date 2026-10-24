// SPDX-License-Identifier: GPL-2.0
/*
 * Register access to a Realtek RTL8364B switch over SMI-over-MDIO.
 *
 * On the Crestron UC-P10 the RK3399 GMAC has no PHY. Its RGMII port goes to
 * an RTL8364B 2-port switch (LAN port and PC port). The switch registers are
 * reached through the Realtek "SMI over MDIO" indirect protocol on the GMAC
 * MDIO bus. This driver binds to that MDIO address (29 on the UC-P10). It
 * gives the switch registers to userspace through one sysfs file, so that a
 * userspace tool can configure the switch before the MAC is opened. The tool
 * (rtl8363-init) is built from the GPL Realtek rtl8367c SDK. The order is
 * important: the MAC DMA reset needs the RGMII RX clock, and the switch
 * drives that clock only after its RGMII port is configured.
 *
 *   echo 0x1300 > smi            select a register
 *   cat smi                      read it: "0x6511"
 *   echo "0x13c2 0x0249" > smi   write a register
 *
 * The protocol is the same as in drivers/net/dsa/realtek/realtek-mdio.c and
 * the stock vendor kernel:
 *   write: w(31,0x000E) w(23,reg) w(24,val) w(21,0x0003)
 *   read:  w(31,0x000E) w(23,reg) w(21,0x0001) r(25)
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/phy.h>

#define SMI_CTRL0_REG		31
#define SMI_CTRL1_REG		21
#define SMI_ADDRESS_REG		23
#define SMI_DATA_WRITE_REG	24
#define SMI_DATA_READ_REG	25
#define SMI_ADDR_OP		0x000e
#define SMI_READ_OP		0x0001
#define SMI_WRITE_OP		0x0003

struct rtl8364b_smi {
	struct mdio_device *mdiodev;
	struct mutex lock;	/* protects sel */
	u16 sel;
};

static int rtl8364b_smi_read(struct mdio_device *mdiodev, u16 reg, u16 *val)
{
	struct mii_bus *bus = mdiodev->bus;
	int addr = mdiodev->addr;
	int ret;

	mutex_lock(&bus->mdio_lock);
	ret = __mdiobus_write(bus, addr, SMI_CTRL0_REG, SMI_ADDR_OP);
	if (!ret)
		ret = __mdiobus_write(bus, addr, SMI_ADDRESS_REG, reg);
	if (!ret)
		ret = __mdiobus_write(bus, addr, SMI_CTRL1_REG, SMI_READ_OP);
	if (!ret) {
		ret = __mdiobus_read(bus, addr, SMI_DATA_READ_REG);
		if (ret >= 0) {
			*val = ret;
			ret = 0;
		}
	}
	mutex_unlock(&bus->mdio_lock);
	return ret;
}

static int rtl8364b_smi_write(struct mdio_device *mdiodev, u16 reg, u16 val)
{
	struct mii_bus *bus = mdiodev->bus;
	int addr = mdiodev->addr;
	int ret;

	mutex_lock(&bus->mdio_lock);
	ret = __mdiobus_write(bus, addr, SMI_CTRL0_REG, SMI_ADDR_OP);
	if (!ret)
		ret = __mdiobus_write(bus, addr, SMI_ADDRESS_REG, reg);
	if (!ret)
		ret = __mdiobus_write(bus, addr, SMI_DATA_WRITE_REG, val);
	if (!ret)
		ret = __mdiobus_write(bus, addr, SMI_CTRL1_REG, SMI_WRITE_OP);
	mutex_unlock(&bus->mdio_lock);
	return ret;
}

static ssize_t smi_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rtl8364b_smi *p = dev_get_drvdata(dev);
	u16 val;
	int ret;

	mutex_lock(&p->lock);
	ret = rtl8364b_smi_read(p->mdiodev, p->sel, &val);
	mutex_unlock(&p->lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "0x%04x\n", val);
}

static ssize_t smi_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct rtl8364b_smi *p = dev_get_drvdata(dev);
	unsigned int reg, val;
	int n, ret = 0;

	n = sscanf(buf, "%x %x", &reg, &val);
	if (n < 1 || reg > 0xffff || (n == 2 && val > 0xffff))
		return -EINVAL;
	mutex_lock(&p->lock);
	p->sel = reg;
	if (n == 2)
		ret = rtl8364b_smi_write(p->mdiodev, reg, val);
	mutex_unlock(&p->lock);
	return ret ? ret : count;
}

static DEVICE_ATTR_RW(smi);

static struct attribute *rtl8364b_smi_attrs[] = {
	&dev_attr_smi.attr,
	NULL
};
ATTRIBUTE_GROUPS(rtl8364b_smi);

static int rtl8364b_smi_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct rtl8364b_smi *p;
	u16 id = 0, ver = 0;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->mdiodev = mdiodev;
	mutex_init(&p->lock);
	dev_set_drvdata(dev, p);

	/* The chip ID reads back after a write of the magic value to 0x13c2. */
	ret = rtl8364b_smi_write(mdiodev, 0x13c2, 0x0249);
	if (!ret)
		ret = rtl8364b_smi_read(mdiodev, 0x1300, &id);
	if (!ret)
		ret = rtl8364b_smi_read(mdiodev, 0x1301, &ver);
	if (!ret)
		ret = rtl8364b_smi_write(mdiodev, 0x13c2, 0x0000);
	if (ret)
		dev_warn(dev, "switch SMI access failed: %d\n", ret);
	else
		dev_info(dev, "Realtek switch chip id 0x%04x ver 0x%04x (SMI @ MDIO %d)\n",
			 id, ver, mdiodev->addr);
	return 0;
}

static const struct of_device_id rtl8364b_smi_of_match[] = {
	{ .compatible = "realtek,rtl8364b-smi" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl8364b_smi_of_match);

static struct mdio_driver rtl8364b_smi_driver = {
	.mdiodrv.driver = {
		.name = "realtek-rtl8364b-smi",
		.of_match_table = rtl8364b_smi_of_match,
		.dev_groups = rtl8364b_smi_groups,
	},
	.probe = rtl8364b_smi_probe,
};
mdio_module_driver(rtl8364b_smi_driver);

MODULE_DESCRIPTION("Realtek RTL8364B switch SMI register access");
MODULE_LICENSE("GPL");
