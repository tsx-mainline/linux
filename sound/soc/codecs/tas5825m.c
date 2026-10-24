// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal driver for the TI TAS5825M class-D speaker amplifier.
 *
 * The amplifier's DSP is configured entirely by a register-write sequence
 * produced by TI PurePath Console 3 (PPC3). This driver loads that sequence
 * as firmware ("tas5825m_<ti,dsp-config-name>.bin": a flat list of
 * <reg, value> byte pairs; register 0xfe is the PPC3 "delay <value> ms"
 * meta-command) and replays it at probe. It then holds the output in Hi-Z
 * and only enters PLAY while a stream is running, so the device never sits
 * in PLAY without I2S clocks (clock-error fault).
 *
 * Volume is the book 0 / page 0 DIG_VOL register (0x4c, 0.5 dB/step,
 * 0x30 = 0 dB, 0xff = mute). The control is capped at 0 dB.
 *
 * Written for the Crestron UC-P10. Its stock kernel holds the PPC3
 * sequence. Nothing in this driver is board specific.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define TAS5825M_REG_PAGE		0x00
#define TAS5825M_REG_DEVICE_CTRL_2	0x03
#define TAS5825M_REG_DIG_VOL		0x4c
#define TAS5825M_REG_CHAN_FAULT		0x70
#define TAS5825M_REG_GLOBAL_FAULT1	0x71
#define TAS5825M_REG_GLOBAL_FAULT2	0x72
#define TAS5825M_REG_FAULT_CLEAR	0x78
#define TAS5825M_REG_BOOK		0x7f

#define TAS5825M_META_DELAY		0xfe

#define DCTRL2_MODE_HIZ			0x02
#define DCTRL2_MODE_PLAY		0x03
#define DCTRL2_MUTE			0x08

#define TAS5825M_FAULT_CLEAR		0x80

struct tas5825m_priv {
	struct device *dev;
	struct regmap *regmap;
	struct regulator *pvdd;
	struct gpio_desc *pdn_gpio;	/* PDN pin, active low */
	struct gpio_desc *mute_gpio;	/* MUTEZ pin, logical 1 = muted */
	u8 *seq;
	size_t seq_len;

	struct mutex lock;		/* protects is_playing/is_muted/configured */
	struct work_struct work;
	bool is_playing;
	bool is_muted;
	bool configured;		/* PPC3 sequence written successfully */
};

static const struct regmap_config tas5825m_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int tas5825m_write_seq(struct tas5825m_priv *tas)
{
	size_t i;
	int ret;

	for (i = 0; i + 1 < tas->seq_len; i += 2) {
		u8 reg = tas->seq[i], val = tas->seq[i + 1];

		if (reg == TAS5825M_META_DELAY) {
			msleep(val);
			continue;
		}
		ret = regmap_write(tas->regmap, reg, val);
		if (ret) {
			dev_err(tas->dev, "config write %zu (reg 0x%02x) failed: %d\n",
				i / 2, reg, ret);
			return ret;
		}
	}

	/* Back to book 0 / page 0 and hold the output in Hi-Z. */
	regmap_write(tas->regmap, TAS5825M_REG_PAGE, 0x00);
	regmap_write(tas->regmap, TAS5825M_REG_BOOK, 0x00);
	regmap_write(tas->regmap, TAS5825M_REG_PAGE, 0x00);
	regmap_write(tas->regmap, TAS5825M_REG_FAULT_CLEAR, TAS5825M_FAULT_CLEAR);
	return regmap_write(tas->regmap, TAS5825M_REG_DEVICE_CTRL_2, DCTRL2_MODE_HIZ);
}

static void tas5825m_refresh(struct tas5825m_priv *tas)
{
	unsigned int chan = 0, g1 = 0, g2 = 0;
	u8 mode;

	/*
	 * If the amplifier did not answer at probe (e.g. its supply came up
	 * late), retry the configuration before the first PLAY. Until it is
	 * configured the output is never taken out of Hi-Z.
	 */
	if (tas->is_playing && !tas->configured) {
		if (tas5825m_write_seq(tas)) {
			dev_warn_ratelimited(tas->dev, "amplifier not responding, output stays off\n");
			if (tas->mute_gpio)
				gpiod_set_value_cansleep(tas->mute_gpio, 1);
			return;
		}
		tas->configured = true;
		dev_info(tas->dev, "configured on stream start\n");
	}

	if (!tas->configured) {
		if (tas->mute_gpio)
			gpiod_set_value_cansleep(tas->mute_gpio, 1);
		return;
	}

	if (tas->is_playing) {
		regmap_read(tas->regmap, TAS5825M_REG_CHAN_FAULT, &chan);
		regmap_read(tas->regmap, TAS5825M_REG_GLOBAL_FAULT1, &g1);
		regmap_read(tas->regmap, TAS5825M_REG_GLOBAL_FAULT2, &g2);
		if (chan || g1 || g2)
			dev_warn(tas->dev, "fault: chan 0x%02x global 0x%02x 0x%02x, clearing\n",
				 chan, g1, g2);
		regmap_write(tas->regmap, TAS5825M_REG_FAULT_CLEAR, TAS5825M_FAULT_CLEAR);
		mode = DCTRL2_MODE_PLAY | (tas->is_muted ? DCTRL2_MUTE : 0);
	} else {
		mode = DCTRL2_MODE_HIZ;
	}

	regmap_write(tas->regmap, TAS5825M_REG_DEVICE_CTRL_2, mode);
	if (tas->mute_gpio)
		gpiod_set_value_cansleep(tas->mute_gpio, !tas->is_playing || tas->is_muted);
}

static void tas5825m_work(struct work_struct *work)
{
	struct tas5825m_priv *tas = container_of(work, struct tas5825m_priv, work);

	mutex_lock(&tas->lock);
	tas5825m_refresh(tas);
	mutex_unlock(&tas->lock);
}

static int tas5825m_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	struct tas5825m_priv *tas = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	/* Atomic context: just record the state; I2C writes go to the worker. */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		tas->is_playing = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		tas->is_playing = false;
		break;
	default:
		return -EINVAL;
	}
	schedule_work(&tas->work);
	return 0;
}

static int tas5825m_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct tas5825m_priv *tas = snd_soc_component_get_drvdata(dai->component);

	mutex_lock(&tas->lock);
	tas->is_muted = mute;
	tas5825m_refresh(tas);
	mutex_unlock(&tas->lock);
	return 0;
}

static int tas5825m_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* The PPC3 config sets I2S input; the amplifier is always a clock consumer. */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops tas5825m_dai_ops = {
	.set_fmt	= tas5825m_set_fmt,
	.trigger	= tas5825m_trigger,
	.mute_stream	= tas5825m_mute,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver tas5825m_dai = {
	.name = "tas5825m-amplifier",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				  SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				  SNDRV_PCM_RATE_96000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &tas5825m_dai_ops,
};

/* reg 0xff = mute ... 0x30 = 0 dB; 0.5 dB steps; capped at 0 dB */
static const DECLARE_TLV_DB_SCALE(tas5825m_vol_tlv, -10350, 50, 1);

static const struct snd_kcontrol_new tas5825m_controls[] = {
	SOC_SINGLE_RANGE_TLV("Speaker Digital Playback Volume", TAS5825M_REG_DIG_VOL,
			     0, 0x30, 0xff, 1, tas5825m_vol_tlv),
};

static const struct snd_soc_dapm_widget tas5825m_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("DAC IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route tas5825m_dapm_routes[] = {
	{ "DAC", NULL, "DAC IN" },
	{ "OUT", NULL, "DAC" },
};

static const struct snd_soc_component_driver tas5825m_component = {
	.controls		= tas5825m_controls,
	.num_controls		= ARRAY_SIZE(tas5825m_controls),
	.dapm_widgets		= tas5825m_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas5825m_dapm_widgets),
	.dapm_routes		= tas5825m_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tas5825m_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int tas5825m_load_config(struct tas5825m_priv *tas)
{
	const struct firmware *fw;
	const char *name;
	char filename[64];
	int ret;

	if (device_property_read_string(tas->dev, "ti,dsp-config-name", &name))
		name = "default";
	snprintf(filename, sizeof(filename), "tas5825m_%s.bin", name);

	ret = request_firmware(&fw, filename, tas->dev);
	if (ret)
		return dev_err_probe(tas->dev, ret, "cannot load %s\n", filename);

	if (!fw->size || (fw->size & 1)) {
		release_firmware(fw);
		return dev_err_probe(tas->dev, -EINVAL, "%s: bad size %zu\n",
				     filename, fw->size);
	}

	tas->seq = devm_kmemdup(tas->dev, fw->data, fw->size, GFP_KERNEL);
	tas->seq_len = fw->size;
	release_firmware(fw);
	if (!tas->seq)
		return -ENOMEM;

	dev_info(tas->dev, "%s: %zu register writes\n", filename, tas->seq_len / 2);
	return 0;
}

static int tas5825m_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct tas5825m_priv *tas;
	int ret;

	tas = devm_kzalloc(dev, sizeof(*tas), GFP_KERNEL);
	if (!tas)
		return -ENOMEM;

	tas->dev = dev;
	tas->is_muted = true;
	mutex_init(&tas->lock);
	INIT_WORK(&tas->work, tas5825m_work);
	i2c_set_clientdata(i2c, tas);

	tas->regmap = devm_regmap_init_i2c(i2c, &tas5825m_regmap);
	if (IS_ERR(tas->regmap))
		return dev_err_probe(dev, PTR_ERR(tas->regmap), "regmap init failed\n");

	tas->pvdd = devm_regulator_get(dev, "pvdd");
	if (IS_ERR(tas->pvdd))
		return dev_err_probe(dev, PTR_ERR(tas->pvdd), "failed to get pvdd\n");

	/* Hold the part in power-down until PVDD is up; keep the output muted. */
	tas->pdn_gpio = devm_gpiod_get(dev, "pdn", GPIOD_OUT_HIGH);
	if (IS_ERR(tas->pdn_gpio))
		return dev_err_probe(dev, PTR_ERR(tas->pdn_gpio), "failed to get pdn gpio\n");

	tas->mute_gpio = devm_gpiod_get_optional(dev, "mute", GPIOD_OUT_HIGH);
	if (IS_ERR(tas->mute_gpio))
		return dev_err_probe(dev, PTR_ERR(tas->mute_gpio), "failed to get mute gpio\n");

	ret = tas5825m_load_config(tas);
	if (ret)
		return ret;

	ret = regulator_enable(tas->pvdd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable pvdd\n");

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(tas->pdn_gpio, 0);	/* release power-down */
	usleep_range(10000, 15000);

	/*
	 * Do not fail the probe if the amplifier does not answer yet: the rest
	 * of the sound card (the other codec on the same I2S link) must still
	 * come up. The configuration is retried at the first stream start.
	 */
	ret = tas5825m_write_seq(tas);
	if (ret)
		dev_warn(dev, "amplifier not responding at probe (%d), will retry on stream start\n",
			 ret);
	else
		tas->configured = true;

	ret = devm_snd_soc_register_component(dev, &tas5825m_component, &tas5825m_dai, 1);
	if (ret)
		goto err_power;

	return 0;

err_power:
	gpiod_set_value_cansleep(tas->pdn_gpio, 1);
	regulator_disable(tas->pvdd);
	return ret;
}

static void tas5825m_i2c_remove(struct i2c_client *i2c)
{
	struct tas5825m_priv *tas = i2c_get_clientdata(i2c);

	cancel_work_sync(&tas->work);
	regmap_write(tas->regmap, TAS5825M_REG_DEVICE_CTRL_2, DCTRL2_MODE_HIZ);
	if (tas->mute_gpio)
		gpiod_set_value_cansleep(tas->mute_gpio, 1);
	gpiod_set_value_cansleep(tas->pdn_gpio, 1);
	regulator_disable(tas->pvdd);
}

static const struct i2c_device_id tas5825m_i2c_id[] = {
	{ "tas5825m" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5825m_i2c_id);

static const struct of_device_id tas5825m_of_match[] = {
	{ .compatible = "ti,tas5825m" },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5825m_of_match);

static struct i2c_driver tas5825m_i2c_driver = {
	.probe		= tas5825m_i2c_probe,
	.remove		= tas5825m_i2c_remove,
	.id_table	= tas5825m_i2c_id,
	.driver		= {
		.name		= "tas5825m",
		.of_match_table	= tas5825m_of_match,
	},
};
module_i2c_driver(tas5825m_i2c_driver);

MODULE_DESCRIPTION("TI TAS5825M amplifier driver (PPC3 sequence replay)");
MODULE_LICENSE("GPL");
