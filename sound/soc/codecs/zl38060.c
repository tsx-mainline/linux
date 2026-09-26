// SPDX-License-Identifier: GPL-2.0-only
//
// Codec driver for Microsemi ZL38060 Connected Home Audio Processor and the
// ZL38051 voice processor (same "Timberwolf" host bus interface).
//
// Copyright(c) 2020 Sven Van Asbroeck

// The ZL38060 is very flexible and configurable. This driver implements only a
// tiny subset of the chip's possible configurations:
//
// - DSP block bypassed: DAI        routed straight to DACs
//                       microphone routed straight to DAI
// - chip's internal clock is driven by a 12 MHz external crystal
// - chip's DAI connected to CPU is I2S, and bit + frame clock master
// - chip must be strapped for "host boot": in this mode, firmware will be
//   provided by this driver.
//
// With "mscc,flash-boot" the chip instead boots its firmware and its
// configuration record from its own flash. The driver then loads nothing,
// leaves the cross-point, clock and DSP configuration alone, and only
// follows the DAI format and rate chosen by the machine driver (which must
// match the flash configuration: the configured TDMA rate is the only rate
// offered). The AEC mute/bypass/gain controls are exposed in that mode.
//
// The host bus interface (HBI) is reachable over SPI or I2C with the same
// command words: [0x80 | offset, words - 1] for page 0 ("direct"),
// [0xFE, page - 1, offset, words - 1] otherwise, bit 7 of the length byte
// set for writes. Over I2C a read is that command written, then the data
// read after a repeated start (Microsemi zl380tw driver, MICROSEMI_HBI_I2C).

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/ihex.h>

#include <sound/pcm_params.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define DRV_NAME		"zl38060"

#define ZL38_RATES		(SNDRV_PCM_RATE_8000  |\
				SNDRV_PCM_RATE_16000 |\
				SNDRV_PCM_RATE_48000)
#define ZL38_FORMATS		SNDRV_PCM_FMTBIT_S16_LE

#define HBI_FIRMWARE_PAGE	0xFF
#define ZL38_MAX_RAW_XFER	0x100

#define REG_TDMA_CFG_CLK	0x0262
#define CFG_CLK_PCLK_SHIFT	4
#define CFG_CLK_PCLK_MASK	(0x7ff << CFG_CLK_PCLK_SHIFT)
#define CFG_CLK_PCLK(bits)	((bits - 1) << CFG_CLK_PCLK_SHIFT)
#define CFG_CLK_MASTER		BIT(15)
#define CFG_CLK_FSRATE_MASK	0x7
#define CFG_CLK_FSRATE_8KHZ	0x1
#define CFG_CLK_FSRATE_16KHZ	0x2
#define CFG_CLK_FSRATE_48KHZ	0x6

#define REG_CLK_CFG		0x0016
#define CLK_CFG_SOURCE_XTAL	BIT(15)

#define REG_CLK_STATUS		0x0014
#define CLK_STATUS_HWRST	BIT(0)

#define REG_PARAM_RESULT	0x0034
#define PARAM_RESULT_READY	0xD3D3

#define REG_PG255_BASE_HI	0x000C
#define REG_PG255_OFFS(addr)	((HBI_FIRMWARE_PAGE << 8) | (addr & 0xFF))
#define REG_FWR_EXEC		0x012C

#define REG_CMD			0x0032
#define REG_HW_REV		0x0020
#define REG_FW_PROD		0x0022
#define REG_FW_REV		0x0024

#define REG_SEMA_FLAGS		0x0006
#define SEMA_FLAGS_BOOT_CMD	BIT(0)
#define SEMA_FLAGS_APP_REBOOT	BIT(1)

#define REG_HW_REV		0x0020
#define REG_FW_PROD		0x0022
#define REG_FW_REV		0x0024
#define REG_GPIO_DIR		0x02DC
#define REG_GPIO_DAT		0x02DA

#define BOOTCMD_LOAD_COMPLETE	0x000D
#define BOOTCMD_FW_GO		0x0008

#define REG_AEC_CTRL0		0x0300
#define AEC_CTRL0_BYPASS_SHIFT	4
#define AEC_CTRL0_MUTE_SOUT_SHIFT	8
#define REG_AEC_SEND_GAIN	0x030C
#define AEC_SEND_GAIN_SOUT_SHIFT	8

#define FIRMWARE_MAJOR		2
#define FIRMWARE_MINOR		2

/*
 * Bring-up aid: the Microsemi driver soft-resets a flash-booted chip on
 * every hw_params. This driver only does so when it changed a register;
 * zl38060.flash_boot_reset=1 restores the vendor behaviour.
 */
static bool flash_boot_reset;
module_param(flash_boot_reset, bool, 0644);
MODULE_PARM_DESC(flash_boot_reset,
		 "flash boot: software reset on every hw_params (vendor behaviour)");

struct zl38_variant {
	const char *name;
	const char *firmware;
	bool check_fw_version;
};

static const struct zl38_variant zl38060_variant = {
	.name = "zl38060",
	.firmware = "zl38060.fw",
	.check_fw_version = true,
};

static const struct zl38_variant zl38051_variant = {
	.name = "zl38051",
	.firmware = "zl38051.fw",
	.check_fw_version = false,
};

struct zl38_codec_priv {
	struct device *dev;
	struct regmap *regmap;
	bool is_stream_in_use[2];
	struct gpio_chip *gpio_chip;
	const struct zl38_variant *variant;
	bool flash_boot;
	bool need_reset;	/* flash boot: a DAI register changed */
	struct snd_soc_dai_driver dai;
};

static int zl38_fw_issue_command(struct regmap *regmap, u16 cmd)
{
	unsigned int val;
	int err;

	err = regmap_read_poll_timeout(regmap, REG_SEMA_FLAGS, val,
				       !(val & SEMA_FLAGS_BOOT_CMD), 10000,
				       10000 * 100);
	if (err)
		return err;
	err = regmap_write(regmap, REG_CMD, cmd);
	if (err)
		return err;
	err = regmap_update_bits(regmap, REG_SEMA_FLAGS, SEMA_FLAGS_BOOT_CMD,
				 SEMA_FLAGS_BOOT_CMD);
	if (err)
		return err;

	return regmap_read_poll_timeout(regmap, REG_CMD, val, !val, 10000,
					10000 * 100);
}

static int zl38_fw_go(struct regmap *regmap)
{
	int err;

	err = zl38_fw_issue_command(regmap, BOOTCMD_LOAD_COMPLETE);
	if (err)
		return err;

	return zl38_fw_issue_command(regmap, BOOTCMD_FW_GO);
}

static int zl38_fw_enter_boot_mode(struct regmap *regmap)
{
	unsigned int val;
	int err;

	err = regmap_update_bits(regmap, REG_CLK_STATUS, CLK_STATUS_HWRST,
				 CLK_STATUS_HWRST);
	if (err)
		return err;

	return regmap_read_poll_timeout(regmap, REG_PARAM_RESULT, val,
					val == PARAM_RESULT_READY, 1000, 50000);
}

static int
zl38_fw_send_data(struct regmap *regmap, u32 addr, const void *data, u16 len)
{
	__be32 addr_base = cpu_to_be32(addr & ~0xFF);
	int err;

	err = regmap_raw_write(regmap, REG_PG255_BASE_HI, &addr_base,
			       sizeof(addr_base));
	if (err)
		return err;
	return regmap_raw_write(regmap, REG_PG255_OFFS(addr), data, len);
}

static int zl38_fw_send_xaddr(struct regmap *regmap, const void *data)
{
	/* execution address from ihex: 32-bit little endian.
	 * device register expects 32-bit big endian.
	 */
	u32 addr = le32_to_cpup(data);
	__be32 baddr = cpu_to_be32(addr);

	return regmap_raw_write(regmap, REG_FWR_EXEC, &baddr, sizeof(baddr));
}

static int zl38_load_firmware(struct device *dev, struct regmap *regmap,
			      const char *name)
{
	const struct ihex_binrec *rec;
	const struct firmware *fw;
	u32 addr;
	u16 len;
	int err;

	/* how to get this firmware:
	 * 1. request and download chip firmware from Microsemi
	 *    (provided by Microsemi in srec format)
	 * 2. convert downloaded firmware from srec to ihex. Simple tool:
	 *    https://gitlab.com/TheSven73/s3-to-irec
	 * 3. convert ihex to binary (.fw) using ihex2fw tool which is included
	 *    with the Linux kernel sources
	 */
	err = request_ihex_firmware(&fw, name, dev);
	if (err)
		return err;
	err = zl38_fw_enter_boot_mode(regmap);
	if (err)
		goto out;
	rec = (const struct ihex_binrec *)fw->data;
	while (rec) {
		addr = be32_to_cpu(rec->addr);
		len = be16_to_cpu(rec->len);
		if (addr) {
			/* regular data ihex record */
			err = zl38_fw_send_data(regmap, addr, rec->data, len);
		} else if (len == 4) {
			/* execution address ihex record */
			err = zl38_fw_send_xaddr(regmap, rec->data);
		} else {
			err = -EINVAL;
		}
		if (err)
			goto out;
		/* next ! */
		rec = ihex_next_binrec(rec);
	}
	err = zl38_fw_go(regmap);

out:
	release_firmware(fw);
	return err;
}


static int zl38_software_reset(struct regmap *regmap)
{
	unsigned int val;
	int err;

	err = regmap_update_bits(regmap, REG_SEMA_FLAGS, SEMA_FLAGS_APP_REBOOT,
				 SEMA_FLAGS_APP_REBOOT);
	if (err)
		return err;

	/* wait for host bus interface to settle.
	 * Not sure if this is required: Microsemi's vendor driver does this,
	 * but the firmware manual does not mention it. Leave it in, there's
	 * little downside, apart from a slower reset.
	 */
	msleep(50);

	return regmap_read_poll_timeout(regmap, REG_SEMA_FLAGS, val,
					!(val & SEMA_FLAGS_APP_REBOOT), 10000,
					10000 * 100);
}

static int zl38_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct zl38_codec_priv *priv = snd_soc_dai_get_drvdata(dai);
	bool changed = false;
	int err;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* firmware default is normal i2s */
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		/* firmware default is normal bitclock and frame */
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		/* always 32 bits per frame (= 16 bits/channel, 2 channels) */
		err = regmap_update_bits_check(priv->regmap, REG_TDMA_CFG_CLK,
					       CFG_CLK_MASTER | CFG_CLK_PCLK_MASK,
					       CFG_CLK_MASTER | CFG_CLK_PCLK(32),
					       &changed);
		if (err)
			return err;
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		/*
		 * Clock consumer: the frame length (PCLK) is the one the chip
		 * was configured with (flash configuration or firmware
		 * default); only the direction is changed here.
		 */
		err = regmap_update_bits_check(priv->regmap, REG_TDMA_CFG_CLK,
					       CFG_CLK_MASTER, 0, &changed);
		if (err)
			return err;
		break;
	default:
		return -EINVAL;
	}

	if (changed)
		priv->need_reset = true;

	return 0;
}

static int zl38_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct zl38_codec_priv *priv = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int fsrate;
	bool changed;
	int err;

	/* We cannot change hw_params while the dai is already in use - the
	 * software reset will corrupt the audio. However, this is not required,
	 * as the chip's TDM buses are fully symmetric, which mandates identical
	 * rates, channels, and samplebits for record and playback.
	 */
	if (priv->is_stream_in_use[!tx])
		goto skip_setup;

	switch (params_rate(params)) {
	case 8000:
		fsrate = CFG_CLK_FSRATE_8KHZ;
		break;
	case 16000:
		fsrate = CFG_CLK_FSRATE_16KHZ;
		break;
	case 48000:
		fsrate = CFG_CLK_FSRATE_48KHZ;
		break;
	default:
		return -EINVAL;
	}

	err = regmap_update_bits_check(priv->regmap, REG_TDMA_CFG_CLK,
				       CFG_CLK_FSRATE_MASK, fsrate, &changed);
	if (err)
		return err;

	/*
	 * chip requires a software reset to apply audio register changes.
	 * A chip running its flash configuration is only reset when this
	 * driver changed something, so an unchanged stream does not glitch.
	 */
	if (!priv->flash_boot || changed || priv->need_reset ||
	    flash_boot_reset) {
		err = zl38_software_reset(priv->regmap);
		if (err)
			return err;
		priv->need_reset = false;
	}

skip_setup:
	priv->is_stream_in_use[tx] = true;

	return 0;
}

static int zl38_hw_free(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct zl38_codec_priv *priv = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	priv->is_stream_in_use[tx] = false;

	return 0;
}

/* stereo bypass with no AEC */
static const struct reg_sequence cp_config_stereo_bypass[] = {
	/* interconnects must be programmed first */
	{ 0x0210, 0x0005 },	/* DAC1   in <= I2S1-L */
	{ 0x0212, 0x0006 },	/* DAC2   in <= I2S1-R */
	{ 0x0214, 0x0001 },	/* I2S1-L in <= MIC1   */
	{ 0x0216, 0x0001 },	/* I2S1-R in <= MIC1   */
	{ 0x0224, 0x0000 },	/* AEC-S  in <= n/a    */
	{ 0x0226, 0x0000 },	/* AEC-R  in <= n/a    */
	/* output enables must be programmed next */
	{ 0x0202, 0x000F },	/* enable I2S1 + DAC   */
};

static const struct snd_soc_dai_ops zl38_dai_ops = {
	.set_fmt = zl38_set_fmt,
	.hw_params = zl38_hw_params,
	.hw_free = zl38_hw_free,
};

static struct snd_soc_dai_driver zl38_dai = {
	.name = "zl38060-tdma",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ZL38_RATES,
		.formats = ZL38_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ZL38_RATES,
		.formats = ZL38_FORMATS,
	},
	.ops = &zl38_dai_ops,
	.symmetric_rate = 1,
	.symmetric_sample_bits = 1,
	.symmetric_channels = 1,
};

static const struct snd_soc_dapm_widget zl38_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("DAC1"),
	SND_SOC_DAPM_OUTPUT("DAC2"),

	SND_SOC_DAPM_INPUT("DMICL"),
};

static const struct snd_soc_dapm_route zl38_dapm_routes[] = {
	{ "DAC1",  NULL, "Playback" },
	{ "DAC2",  NULL, "Playback" },

	{ "Capture",  NULL, "DMICL" },
};

/*
 * AEC controls, only for a chip running its own (flash) DSP configuration.
 * Register meaning from the Microsemi zl380tw driver (ZL38040_AEC_CTRL_REG0,
 * ZL38040_SYSGAIN): changes to the mute and gain bits apply at once (the
 * vendor driver toggles the mute bits while streaming), a change of the
 * AEC bypass bit needs a software reset (brief audio drop-out).
 */
static int zl38_aec_switch_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);
	int ret, err;

	ret = snd_soc_put_volsw(kcontrol, ucontrol);
	if (ret <= 0)
		return ret;

	err = zl38_software_reset(priv->regmap);

	return err ? err : ret;
}

/* SOUT digital gain: 0 = -24 dB .. 15 = +21 dB, 3 dB steps */
static const DECLARE_TLV_DB_SCALE(zl38_sout_tlv, -2400, 300, 0);

static const struct snd_kcontrol_new zl38_aec_controls[] = {
	SOC_SINGLE("Mic Capture Switch", REG_AEC_CTRL0,
		   AEC_CTRL0_MUTE_SOUT_SHIFT, 1, 1),
	SOC_SINGLE_TLV("Mic Capture Volume", REG_AEC_SEND_GAIN,
		       AEC_SEND_GAIN_SOUT_SHIFT, 15, 0, zl38_sout_tlv),
	SOC_SINGLE_EXT("AEC Switch", REG_AEC_CTRL0, AEC_CTRL0_BYPASS_SHIFT,
		       1, 1, snd_soc_get_volsw, zl38_aec_switch_put),
};

static int zl38_component_probe(struct snd_soc_component *component)
{
	struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);

	if (!priv->flash_boot)
		return 0;

	return snd_soc_add_component_controls(component, zl38_aec_controls,
					      ARRAY_SIZE(zl38_aec_controls));
}

static const struct snd_soc_component_driver zl38_component_dev = {
	.probe			= zl38_component_probe,
	.dapm_widgets		= zl38_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(zl38_dapm_widgets),
	.dapm_routes		= zl38_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(zl38_dapm_routes),
	.endianness		= 1,
};

static int chip_gpio_set(struct gpio_chip *c, unsigned int offset, int val)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);

	return regmap_update_bits(regmap, REG_GPIO_DAT, mask, val ? mask : 0);
}

static int chip_gpio_get(struct gpio_chip *c, unsigned int offset)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);
	unsigned int val;
	int err;

	err = regmap_read(regmap, REG_GPIO_DAT, &val);
	if (err)
		return err;

	return !!(val & mask);
}

static int chip_direction_input(struct gpio_chip *c, unsigned int offset)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);

	return regmap_update_bits(regmap, REG_GPIO_DIR, mask, 0);
}

static int
chip_direction_output(struct gpio_chip *c, unsigned int offset, int val)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);
	int ret;

	ret = chip_gpio_set(c, offset, val);
	if (ret)
		return ret;

	return regmap_update_bits(regmap, REG_GPIO_DIR, mask, mask);
}

static const struct gpio_chip template_chip = {
	.owner = THIS_MODULE,
	.label = DRV_NAME,

	.base = -1,
	.ngpio = 14,
	.direction_input = chip_direction_input,
	.direction_output = chip_direction_output,
	.get = chip_gpio_get,
	.set = chip_gpio_set,

	.can_sleep = true,
};

static int zl38_check_revision(struct device *dev, struct regmap *regmap,
			       bool check_version)
{
	unsigned int hwrev, fwprod, fwrev;
	int fw_major, fw_minor, fw_micro;
	int err;

	err = regmap_read(regmap, REG_HW_REV, &hwrev);
	if (err)
		return err;
	err = regmap_read(regmap, REG_FW_PROD, &fwprod);
	if (err)
		return err;
	err = regmap_read(regmap, REG_FW_REV, &fwrev);
	if (err)
		return err;

	fw_major = (fwrev >> 12) & 0xF;
	fw_minor = (fwrev >>  8) & 0xF;
	fw_micro = fwrev & 0xFF;
	dev_info(dev, "hw rev 0x%x, fw product code %d, firmware rev %d.%d.%d",
		 hwrev & 0x1F, fwprod, fw_major, fw_minor, fw_micro);

	if (!check_version)
		return 0;

	if (fw_major != FIRMWARE_MAJOR || fw_minor < FIRMWARE_MINOR) {
		dev_err(dev, "unsupported firmware. driver supports %d.%d",
			FIRMWARE_MAJOR, FIRMWARE_MINOR);
		return -EINVAL;
	}

	return 0;
}

/* build the HBI command for a transfer of @val_size bytes, return its length */
static size_t zl38_hbi_cmd(u8 *cmd, const u8 *reg_buf8, size_t val_size,
			   bool write)
{
	u8 offs = reg_buf8[1] >> 1;
	u8 page = reg_buf8[0];
	u8 words = (val_size / 2 - 1) | (write ? 0x80 : 0);
	size_t len = 0;

	if (page) {
		cmd[len++] = 0xFE;
		cmd[len++] = page == HBI_FIRMWARE_PAGE ? 0xFF : page - 1;
		cmd[len++] = offs;
	} else {
		cmd[len++] = offs | 0x80;
	}
	cmd[len++] = words;

	return len;
}

static int zl38_spi_bus_read(void *context,
			     const void *reg_buf, size_t reg_size,
			     void *val_buf, size_t val_size)
{
	struct spi_device *spi = context;
	u8 txbuf[4];
	size_t len;

	if (reg_size != 2 || val_size > ZL38_MAX_RAW_XFER)
		return -EINVAL;

	len = zl38_hbi_cmd(txbuf, reg_buf, val_size, false);

	return spi_write_then_read(spi, txbuf, len, val_buf, val_size);
}

static int zl38_spi_bus_write(void *context, const void *data, size_t count)
{
	struct spi_device *spi = context;
	u8 buf[4 + ZL38_MAX_RAW_XFER];
	size_t val_len, len;
	const u8 *data8 = data;

	if (count > (2 + ZL38_MAX_RAW_XFER) || count < 4)
		return -EINVAL;
	val_len = count - 2;

	len = zl38_hbi_cmd(buf, data8, val_len, true);
	memcpy(buf + len, data8 + 2, val_len);
	len += val_len;

	return spi_write(spi, buf, len);
}

static const struct regmap_bus zl38_spi_regmap_bus = {
	.read = zl38_spi_bus_read,
	.write = zl38_spi_bus_write,
	.max_raw_write = ZL38_MAX_RAW_XFER,
	.max_raw_read = ZL38_MAX_RAW_XFER,
};

static int zl38_i2c_bus_read(void *context,
			     const void *reg_buf, size_t reg_size,
			     void *val_buf, size_t val_size)
{
	struct i2c_client *i2c = context;
	struct i2c_msg msgs[2];
	u8 txbuf[4];
	int ret;

	if (reg_size != 2 || val_size > ZL38_MAX_RAW_XFER)
		return -EINVAL;

	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = zl38_hbi_cmd(txbuf, reg_buf, val_size, false);
	msgs[0].buf = txbuf;
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = val_size;
	msgs[1].buf = val_buf;

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == (int)ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static int zl38_i2c_bus_write(void *context, const void *data, size_t count)
{
	struct i2c_client *i2c = context;
	u8 buf[4 + ZL38_MAX_RAW_XFER];
	size_t val_len, len;
	const u8 *data8 = data;
	int ret;

	if (count > (2 + ZL38_MAX_RAW_XFER) || count < 4)
		return -EINVAL;
	val_len = count - 2;

	len = zl38_hbi_cmd(buf, data8, val_len, true);
	memcpy(buf + len, data8 + 2, val_len);
	len += val_len;

	ret = i2c_master_send(i2c, buf, len);
	if (ret < 0)
		return ret;

	return (size_t)ret == len ? 0 : -EIO;
}

static const struct regmap_bus zl38_i2c_regmap_bus = {
	.read = zl38_i2c_bus_read,
	.write = zl38_i2c_bus_write,
	.max_raw_write = ZL38_MAX_RAW_XFER,
	.max_raw_read = ZL38_MAX_RAW_XFER,
};

static const struct regmap_config zl38_regmap_conf = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 2,
	.use_single_read = true,
	.use_single_write = true,
};

/*
 * Flash boot: the chip loaded its firmware and configuration by itself.
 * Offer only the TDMA rate of that configuration.
 */
static int zl38_flash_boot_setup(struct zl38_codec_priv *priv)
{
	unsigned int clkcfg, rate;
	int err;

	err = regmap_read(priv->regmap, REG_TDMA_CFG_CLK, &clkcfg);
	if (err)
		return err;

	switch (clkcfg & CFG_CLK_FSRATE_MASK) {
	case CFG_CLK_FSRATE_8KHZ:
		rate = SNDRV_PCM_RATE_8000;
		break;
	case CFG_CLK_FSRATE_16KHZ:
		rate = SNDRV_PCM_RATE_16000;
		break;
	case CFG_CLK_FSRATE_48KHZ:
		rate = SNDRV_PCM_RATE_48000;
		break;
	default:
		dev_warn(priv->dev, "flash config: unsupported TDMA rate code %u\n",
			 clkcfg & CFG_CLK_FSRATE_MASK);
		return 0;
	}

	dev_info(priv->dev, "flash boot, TDMA clock config 0x%04x (%s, %u bit clocks per frame)\n",
		 clkcfg, clkcfg & CFG_CLK_MASTER ? "master" : "slave",
		 ((clkcfg & CFG_CLK_PCLK_MASK) >> CFG_CLK_PCLK_SHIFT) + 1);

	priv->dai.playback.rates = rate;
	priv->dai.capture.rates = rate;

	return 0;
}

static int zl38_probe(struct device *dev, struct regmap *regmap)
{
	struct zl38_codec_priv *priv;
	struct gpio_desc *reset_gpio;
	int err;

	priv = dev_get_drvdata(dev);
	priv->regmap = regmap;

	/* get the chip to a known state by putting it in reset */
	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return PTR_ERR(reset_gpio);
	if (reset_gpio) {
		/* datasheet: need > 10us for a digital + analog reset */
		usleep_range(15, 50);
		/* take the chip out of reset */
		gpiod_set_value_cansleep(reset_gpio, 0);
		/* datasheet: need > 3ms for digital section to become stable */
		usleep_range(3000, 10000);
		/* a chip booting from its flash needs longer */
		if (priv->flash_boot)
			msleep(500);
	}

	if (!priv->flash_boot) {
		err = zl38_load_firmware(dev, regmap, priv->variant->firmware);
		if (err)
			return err;
	}

	err = zl38_check_revision(dev, regmap,
				  !priv->flash_boot &&
				  priv->variant->check_fw_version);
	if (err)
		return err;

	priv->gpio_chip = devm_kmemdup(dev, &template_chip,
				       sizeof(template_chip), GFP_KERNEL);
	if (!priv->gpio_chip)
		return -ENOMEM;
	priv->gpio_chip->parent = dev;
	err = devm_gpiochip_add_data(dev, priv->gpio_chip, priv->regmap);
	if (err)
		return err;

	if (priv->flash_boot) {
		err = zl38_flash_boot_setup(priv);
		if (err)
			return err;
	} else {
		/* setup the cross-point switch for stereo bypass */
		err = regmap_multi_reg_write(priv->regmap,
					     cp_config_stereo_bypass,
					     ARRAY_SIZE(cp_config_stereo_bypass));
		if (err)
			return err;
		/* setup for 12MHz crystal connected to the chip */
		err = regmap_update_bits(priv->regmap, REG_CLK_CFG,
					 CLK_CFG_SOURCE_XTAL,
					 CLK_CFG_SOURCE_XTAL);
		if (err)
			return err;
	}

	return devm_snd_soc_register_component(dev, &zl38_component_dev,
					       &priv->dai, 1);
}

static struct zl38_codec_priv *zl38_priv_alloc(struct device *dev,
					       const struct zl38_variant *variant)
{
	struct zl38_codec_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	priv->dev = dev;
	priv->variant = variant ?: &zl38060_variant;
	priv->flash_boot = device_property_read_bool(dev, "mscc,flash-boot");
	priv->dai = zl38_dai;
	dev_set_drvdata(dev, priv);

	return priv;
}

static const struct of_device_id zl38_dt_ids[] __maybe_unused = {
	{ .compatible = "mscc,zl38060", .data = &zl38060_variant },
	{ .compatible = "mscc,zl38051", .data = &zl38051_variant },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zl38_dt_ids);

#if IS_ENABLED(CONFIG_SPI_MASTER)
static int zl38_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct regmap *regmap;

	if (!zl38_priv_alloc(dev, spi_get_device_match_data(spi)))
		return -ENOMEM;

	regmap = devm_regmap_init(dev, &zl38_spi_regmap_bus, spi,
				  &zl38_regmap_conf);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return zl38_probe(dev, regmap);
}

static const struct spi_device_id zl38_spi_ids[] = {
	{ "zl38060", (kernel_ulong_t)&zl38060_variant },
	{ "zl38051", (kernel_ulong_t)&zl38051_variant },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, zl38_spi_ids);

static struct spi_driver zl38060_spi_driver = {
	.driver	= {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(zl38_dt_ids),
	},
	.probe = zl38_spi_probe,
	.id_table = zl38_spi_ids,
};
#endif

#if IS_ENABLED(CONFIG_I2C)
static int zl38_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap *regmap;

	if (!zl38_priv_alloc(dev, i2c_get_match_data(i2c)))
		return -ENOMEM;

	regmap = devm_regmap_init(dev, &zl38_i2c_regmap_bus, i2c,
				  &zl38_regmap_conf);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return zl38_probe(dev, regmap);
}

static const struct i2c_device_id zl38_i2c_ids[] = {
	{ "zl38060", (kernel_ulong_t)&zl38060_variant },
	{ "zl38051", (kernel_ulong_t)&zl38051_variant },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, zl38_i2c_ids);

static struct i2c_driver zl38060_i2c_driver = {
	.driver	= {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(zl38_dt_ids),
	},
	.probe = zl38_i2c_probe,
	.id_table = zl38_i2c_ids,
};
#endif

static int __init zl38_init(void)
{
	int ret = 0;

#if IS_ENABLED(CONFIG_I2C)
	ret = i2c_add_driver(&zl38060_i2c_driver);
	if (ret)
		return ret;
#endif
#if IS_ENABLED(CONFIG_SPI_MASTER)
	ret = spi_register_driver(&zl38060_spi_driver);
#if IS_ENABLED(CONFIG_I2C)
	if (ret)
		i2c_del_driver(&zl38060_i2c_driver);
#endif
#endif
	return ret;
}
module_init(zl38_init);

static void __exit zl38_exit(void)
{
#if IS_ENABLED(CONFIG_SPI_MASTER)
	spi_unregister_driver(&zl38060_spi_driver);
#endif
#if IS_ENABLED(CONFIG_I2C)
	i2c_del_driver(&zl38060_i2c_driver);
#endif
}
module_exit(zl38_exit);

MODULE_DESCRIPTION("ASoC ZL38060/ZL38051 driver");
MODULE_AUTHOR("Sven Van Asbroeck <TheSven73@gmail.com>");
MODULE_LICENSE("GPL v2");
