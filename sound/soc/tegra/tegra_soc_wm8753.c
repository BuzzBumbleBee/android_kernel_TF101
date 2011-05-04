 /*
  * tegra_soc_wm8753.c  --  SoC audio for tegra
  *
  * Copyright  2011 Nvidia Graphics Pvt. Ltd.
  *
  * Author: Sachin Nikam
  *             snikam@nvidia.com
  *  http://www.nvidia.com
  *
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful, but WITHOUT
  * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  * more details.
  *
  * You should have received a copy of the GNU General Public License along
  * with this program; if not, write to the Free Software Foundation, Inc.,
  * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include "tegra_soc.h"
#include "../codecs/wm8753.h"
#include <linux/regulator/consumer.h>

#include <linux/types.h>
#include <sound/jack.h>
#include <linux/switch.h>
#include <mach/gpio.h>
#include <mach/audio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define WM8753_PWR1_VMIDSEL_1		1<<8
#define WM8753_PWR1_VMIDSEL_0		1<<7
#define WM8753_PWR1_VREF		1<<6
#define WM8753_PWR1_MICB		1<<5
#define WM8753_PWR1_DACL		1<<3
#define WM8753_PWR1_DACR		1<<2

#define WM8753_PWR2_MICAMP1EN		1<<8
#define WM8753_PWR2_MICAMP2EN		1<<7
#define WM8753_PWR2_ALCMIX		1<<6
#define WM8753_PWR2_PGAL		1<<5
#define WM8753_PWR2_PGAR		1<<4
#define WM8753_PWR2_ADCL		1<<3
#define WM8753_PWR2_ADCR		1<<2
#define WM8753_PWR2_RXMIX		1<<1
#define WM8753_PWR2_LINEMIX		1<<0

#define WM8753_PWR3_LOUT1		1<<8
#define WM8753_PWR3_ROUT1		1<<7
#define WM8753_PWR3_LOUT2		1<<6
#define WM8753_PWR3_ROUT2		1<<5
#define WM8753_PWR3_OUT3		1<<4
#define WM8753_PWR3_OUT4		1<<3
#define WM8753_PWR3_MONO1		1<<2
#define WM8753_PWR3_MONO2		1<<1

#define WM8753_PWR4_RECMIX		1<<3
#define WM8753_PWR4_MONOMIX		1<<2
#define WM8753_PWR4_RIGHTMIX		1<<1
#define WM8753_PWR4_LEFTMIX		1<<0

#define WM8753_IOCTL_VXCLKTRI		1<<7
#define WM8753_IOCTL_BCCLKTRI		1<<6
#define WM8753_IOCTL_VXDTRI		1<<5
#define WM8753_IOCTL_ADCTRI		1<<4
#define WM8753_IOCTL_IFMODE_1		1<<3
#define WM8753_IOCTL_IFMODE_0		1<<2
#define WM8753_IOCTL_VXFSOE		1<<1
#define WM8753_IOCTL_LRCOE		1<<0

#define WM8753_LOUTM1_LD2LO		1<<8

#define WM8753_ROUTM1_RD2RO		1<<8

#define WM8753_ADCIN_MONOMIX_1		1<<5
#define WM8753_ADCIN_MONOMIX_0		1<<4
#define WM8753_ADCIN_RADCSEL_1		1<<3
#define WM8753_ADCIN_RADCSEL_0		1<<2
#define WM8753_ADCIN_LADCSEL_1		1<<1
#define WM8753_ADCIN_LADCSEL_0		1<<0

#define WM8753_INCTL1_MIC2BOOST_1	1<<8
#define WM8753_INCTL1_MIC2BOOST_0	1<<7

#define WM8753_INCTL2_MICMUX_1		1<<5
#define WM8753_INCTL2_MICMUX_0		1<<4

#define WM8753_ADC_DATSEL_1		1<<8
#define WM8753_ADC_DATSEL_0		1<<7
#define WM8753_ADC_ADCPOL_1		1<<6
#define WM8753_ADC_ADCPOL_0		1<<5
#define WM8753_ADC_VXFILT		1<<4
#define WM8753_ADC_HPMODE_1		1<<3
#define WM8753_ADC_HPMODE_0		1<<2
#define WM8753_ADC_HPOR			1<<1
#define WM8753_ADC_ADCHPD		1<<0

#define WM8753_LINVOL_MAX		0x11F

#define WM8753_RINVOL_MAX		0x11F

#define WM8753_GPIO2_GP2M_2		1<<5
#define WM8753_GPIO2_GP2M_1 	1<<4
#define WM8753_GPIO2_GP2M_0 	1<<3

#define WM8753_GPIO1_INTCON_1 	1<<8
#define WM8753_GPIO1_INTCON_0 	1<<7

#define WM8753_INTPOL_GPIO4IPOL 1<<4

#define WM8753_INTEN_MICSHTEN	1<<0
#define WM8753_INTEN_MICDETEN	1<<1
#define WM8753_INTEN_GPIO3IEN	1<<3
#define WM8753_INTEN_GPIO4IEN	1<<4
#define WM8753_INTEN_GPIO5IEN	1<<5
#define WM8753_INTEN_HPSWIEN	1<<6
#define WM8753_INTEN_TSDIEN 	1<<7

/* Board Specific GPIO configuration for Whistler */
#define TEGRA_GPIO_PW3 			179

static struct wm8753_headphone_jack
{
	struct snd_jack *jack;
	int gpio;
	struct work_struct work;
	struct snd_soc_codec* pcodec;
};

static struct wm8753_headphone_jack* wm8753_jack = NULL;

static struct platform_device *tegra_snd_device;
static struct regulator* wm8753_reg;

extern struct snd_soc_dai tegra_i2s_dai[];
extern struct snd_soc_dai tegra_generic_codec_dai[];
extern struct snd_soc_platform tegra_soc_platform;

static int tegra_hifi_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int err;
	unsigned int value;
	unsigned int channels, rate, bit_size = 16;
	struct snd_soc_codec *codec = codec_dai->codec;

	rate	 = params_rate(params);		/* Sampling Rate in Hz */
	channels = params_channels(params);	/* Number of channels */

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_size = 16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_size = 20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		bit_size = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		bit_size = 32;
		break;
	default:
		pr_err(KERN_ERR "Invalid pcm format size\n");
		return EINVAL;
	}

	err = snd_soc_dai_set_fmt(codec_dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		pr_err(KERN_ERR "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		pr_err(KERN_ERR "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, bit_size*rate*channels*2*4,
			SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err(KERN_ERR "codec_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, bit_size*rate*channels*2,
			SND_SOC_CLOCK_IN);
	if (err < 0) {
		pr_err(KERN_ERR "cpu_dai clock not set\n");
		return err;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Enables MICBIAS, VMIDSEL and VREF, DAC-L and DAC-R */
		value = snd_soc_read(codec_dai->codec, WM8753_PWR1);
		value |= (WM8753_PWR1_VMIDSEL_0 | WM8753_PWR1_VREF |
				WM8753_PWR1_MICB | WM8753_PWR1_DACL |
				WM8753_PWR1_DACR);
		value &= ~(WM8753_PWR1_VMIDSEL_1);
		snd_soc_write(codec_dai->codec, WM8753_PWR1, value);

		/* Enables Lout1 and Rout1 */
		value = snd_soc_read(codec_dai->codec, WM8753_PWR3);
		value |= (WM8753_PWR3_LOUT1 | WM8753_PWR3_ROUT1);
		snd_soc_write(codec_dai->codec, WM8753_PWR3, value);

		/* Left and Right Mix Enabled */
		value = snd_soc_read(codec_dai->codec, WM8753_PWR4);
		value |= (WM8753_PWR4_RIGHTMIX | WM8753_PWR4_LEFTMIX);
		snd_soc_write(codec_dai->codec, WM8753_PWR4, value);

		/* Mode set to HiFi over HiFi interface and VXDOUT,ADCDAT,
			VXCLK and BCLK pin enabled */
		value = snd_soc_read(codec_dai->codec, WM8753_IOCTL);
		value |= (WM8753_IOCTL_IFMODE_1);
		value &= ~(WM8753_IOCTL_VXCLKTRI | WM8753_IOCTL_BCCLKTRI |
				WM8753_IOCTL_VXDTRI | WM8753_IOCTL_ADCTRI |
				WM8753_IOCTL_IFMODE_0 | WM8753_IOCTL_VXFSOE |
				WM8753_IOCTL_LRCOE);
		snd_soc_write(codec_dai->codec, WM8753_IOCTL, value);

		/* L-DAC to L-Mix */
		value = snd_soc_read(codec_dai->codec, WM8753_LOUTM1);
		value |= WM8753_LOUTM1_LD2LO;
		snd_soc_write(codec_dai->codec, WM8753_LOUTM1, value);

		/* R-DAC to R-Mix */
		value = snd_soc_read(codec_dai->codec, WM8753_ROUTM1);
		value |= WM8753_ROUTM1_RD2RO;
		snd_soc_write(codec_dai->codec, WM8753_ROUTM1, value);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* PGA i/p's to L and R ADC and operation in stero mode */
		value = snd_soc_read(codec_dai->codec, WM8753_ADCIN);
		value &= ~(WM8753_ADCIN_MONOMIX_1 | WM8753_ADCIN_MONOMIX_0 |
				WM8753_ADCIN_RADCSEL_1 | WM8753_ADCIN_RADCSEL_0 |
				WM8753_ADCIN_LADCSEL_1 | WM8753_ADCIN_LADCSEL_0);
		snd_soc_write(codec_dai->codec, WM8753_ADCIN, value);

		/* 24 db boost for Mic2 */
		value = snd_soc_read(codec_dai->codec, WM8753_INCTL1);
		value |= (WM8753_INCTL1_MIC2BOOST_1);
		value &= ~(WM8753_INCTL1_MIC2BOOST_0);
		snd_soc_write(codec_dai->codec, WM8753_INCTL1, value);

		/* Side-Tone-Mic2 preamp o/p */
		value = snd_soc_read(codec_dai->codec, WM8753_INCTL2);
		value |= (WM8753_INCTL2_MICMUX_1);
		value &= ~(WM8753_INCTL2_MICMUX_0);
		snd_soc_write(codec_dai->codec, WM8753_INCTL2, value);

		/* L and R data from R-ADC */
		value = snd_soc_read(codec_dai->codec, WM8753_ADC);
		value |= (WM8753_ADC_DATSEL_1);
		value &= ~(WM8753_ADC_DATSEL_0 | WM8753_ADC_ADCPOL_1 |
				WM8753_ADC_ADCPOL_0 | WM8753_ADC_VXFILT |
				WM8753_ADC_HPMODE_1 | WM8753_ADC_HPMODE_0 |
				WM8753_ADC_HPOR | WM8753_ADC_ADCHPD);
		snd_soc_write(codec_dai->codec, WM8753_ADC, value);

		/* Disable Mute and set L-PGA Vol to max */
		snd_soc_write(codec, WM8753_LINVOL, WM8753_LINVOL_MAX);

		/* Disable Mute and set R-PGA Vol to max */
		snd_soc_write(codec, WM8753_RINVOL, WM8753_RINVOL_MAX);

		/* Enables MICBIAS, VMIDSEL and VREF */
		value = snd_soc_read(codec_dai->codec, WM8753_PWR1);
		value |= (WM8753_PWR1_VMIDSEL_0|WM8753_PWR1_VREF|
				WM8753_PWR1_MICB);
		value &= ~(WM8753_PWR1_VMIDSEL_1);
		snd_soc_write(codec_dai->codec, WM8753_PWR1, value);

		/* Enable Mic2 preamp, PGA-R and ADC-R (Mic1 preamp,ALC Mix,
			PGA-L , ADC-L, RXMIX and LINEMIX disabled) */
		value = snd_soc_read(codec_dai->codec, WM8753_PWR2);
		value |= (WM8753_PWR2_MICAMP2EN | WM8753_PWR2_PGAR|
					WM8753_PWR2_ADCR);
		value &= ~(WM8753_PWR2_MICAMP1EN | WM8753_PWR2_ALCMIX |
					WM8753_PWR2_PGAL | WM8753_PWR2_ADCL |
					WM8753_PWR2_RXMIX | WM8753_PWR2_LINEMIX);
		snd_soc_write(codec, WM8753_PWR2, value);

		/* Mode set to HiFi over HiFi interface and VXDOUT,ADCDAT,
			VXCLK and BCLK pin enabled */
		value = snd_soc_read(codec_dai->codec, WM8753_IOCTL);
		value |= (WM8753_IOCTL_IFMODE_1);
		value &= ~(WM8753_IOCTL_VXCLKTRI | WM8753_IOCTL_BCCLKTRI |
				WM8753_IOCTL_VXDTRI | WM8753_IOCTL_ADCTRI |
				WM8753_IOCTL_IFMODE_0 | WM8753_IOCTL_VXFSOE |
				WM8753_IOCTL_LRCOE);
		snd_soc_write(codec_dai->codec, WM8753_IOCTL, value);
	}

	return 0;
}

static int tegra_hifi_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}


static int tegra_voice_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)

{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int err;

	err = snd_soc_dai_set_fmt(cpu_dai,
					   SND_SOC_DAIFMT_DSP_A | \
					   SND_SOC_DAIFMT_NB_NF | \
					   SND_SOC_DAIFMT_CBS_CFS);

	if (err < 0) {
		  pr_err("%s:cpu_dai fmt not set \n", __func__);
		  return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, I2S2_CLK, SND_SOC_CLOCK_IN);

	if (err < 0) {
		  pr_err("%s:cpu_dai clock not set\n", __func__);
		  return err;
	}

	return 0;
}

static int tegra_voice_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops tegra_hifi_ops = {
	.hw_params = tegra_hifi_hw_params,
	.hw_free   = tegra_hifi_hw_free,
};

static struct snd_soc_ops tegra_voice_ops = {
	.hw_params = tegra_voice_hw_params,
	.hw_free   = tegra_voice_hw_free,
};

static void wm8753_intr_work(struct work_struct *work)
{
	unsigned int value;

	/* Do something here */
	mutex_lock(&wm8753_jack->pcodec->mutex);

	/* GPIO4 interrupt disable (also disable other interrupts) */
	value = snd_soc_read(wm8753_jack->pcodec, WM8753_INTEN);
	value &= ~(WM8753_INTEN_MICSHTEN | WM8753_INTEN_MICDETEN |
			  WM8753_INTEN_GPIO3IEN | WM8753_INTEN_HPSWIEN |
			  WM8753_INTEN_GPIO5IEN | WM8753_INTEN_TSDIEN |
			  WM8753_INTEN_GPIO4IEN);
	snd_soc_write(wm8753_jack->pcodec, WM8753_INTEN, value);

	/* Invert GPIO4 interrupt polarity */
	value = snd_soc_read(wm8753_jack->pcodec, WM8753_INTPOL);
	value &= WM8753_INTPOL_GPIO4IPOL;
	if ((value) & ((WM8753_INTPOL_GPIO4IPOL))) {
		tegra_switch_set_state(SND_JACK_HEADPHONE);
		snd_jack_report(wm8753_jack->jack, SND_JACK_HEADPHONE);
		value &= ~(WM8753_INTPOL_GPIO4IPOL);
	}
	else {
		tegra_switch_set_state(0);
		snd_jack_report(wm8753_jack->jack, 0);
		value |= (WM8753_INTPOL_GPIO4IPOL);
	}
	snd_soc_write(wm8753_jack->pcodec, WM8753_INTPOL, value);

	/* GPIO4 interrupt enable */
	value = snd_soc_read(wm8753_jack->pcodec, WM8753_INTEN);
	value |= (WM8753_INTEN_GPIO4IEN);
	value &= ~(WM8753_INTEN_MICSHTEN | WM8753_INTEN_MICDETEN |
			  WM8753_INTEN_GPIO3IEN | WM8753_INTEN_HPSWIEN |
			  WM8753_INTEN_GPIO5IEN | WM8753_INTEN_TSDIEN);
	snd_soc_write(wm8753_jack->pcodec, WM8753_INTEN, value);

	mutex_unlock(&wm8753_jack->pcodec->mutex);
}

static irqreturn_t wm8753_irq(int irq, void *data)
{
	schedule_work(&wm8753_jack->work);
	return IRQ_HANDLED;
}

static int tegra_codec_init(struct snd_soc_codec *codec)
{
	int ret;
	unsigned int value;

	ret =  tegra_controls_init(codec);
	if (ret < 0)
		goto failed;

	if (!wm8753_jack) {

		wm8753_jack = kzalloc(sizeof(*wm8753_jack), GFP_KERNEL);
		if (!wm8753_jack) {
			pr_err("failed to allocate wm8753-jack\n");
			return -ENOMEM;
		}

		wm8753_jack->gpio = TEGRA_GPIO_PW3;
		wm8753_jack->pcodec = codec;

		INIT_WORK(&wm8753_jack->work, wm8753_intr_work);

		ret = snd_jack_new(codec->card, "Headphone Jack", SND_JACK_HEADPHONE,
			&wm8753_jack->jack);
		if (ret < 0)
			goto failed;

		ret = gpio_request(wm8753_jack->gpio, "headphone-detect-gpio");
		if (ret)
			goto failed;

		ret = gpio_direction_input(wm8753_jack->gpio);
		if (ret)
			goto gpio_failed;

		tegra_gpio_enable(wm8753_jack->gpio);

		ret = request_irq(gpio_to_irq(wm8753_jack->gpio),
				wm8753_irq,
				IRQF_TRIGGER_FALLING,
				"wm8753",
				wm8753_jack);

		if (ret)
			goto gpio_failed;

		/* Configure GPIO2 pin to generate the interrupt */
		value = snd_soc_read(codec, WM8753_GPIO2);
		value |= (WM8753_GPIO2_GP2M_0 | WM8753_GPIO2_GP2M_1);
		value &= ~(WM8753_GPIO2_GP2M_2);
		snd_soc_write(codec, WM8753_GPIO2, value);

		/* Active low Interrupt */
		value = snd_soc_read(codec, WM8753_GPIO1);
		value |= (WM8753_GPIO1_INTCON_1 | WM8753_GPIO1_INTCON_0);
		snd_soc_write(codec, WM8753_GPIO1, value);

		/* GPIO4 interrupt polarity -- interupt when low i.e Headphone connected */
		value = snd_soc_read(codec, WM8753_INTPOL);
		value |= (WM8753_INTPOL_GPIO4IPOL);
		snd_soc_write(codec, WM8753_INTPOL, value);

		/* GPIO4 interrupt enable and disable other interrupts */
		value = snd_soc_read(codec, WM8753_INTEN);
		value |= (WM8753_INTEN_GPIO4IEN);
		value &= ~(WM8753_INTEN_MICSHTEN | WM8753_INTEN_MICDETEN |
			      WM8753_INTEN_GPIO3IEN | WM8753_INTEN_HPSWIEN |
			      WM8753_INTEN_GPIO5IEN | WM8753_INTEN_TSDIEN);
		snd_soc_write(codec, WM8753_INTEN, value);
	}

	return ret;

gpio_failed:
	gpio_free(wm8753_jack->gpio);
failed:
	kfree(wm8753_jack);
	wm8753_jack = NULL;
	return ret;
}

static struct snd_soc_dai_link tegra_soc_dai[] = {
	{
		.name = "WM8753",
		.stream_name = "WM8753 HiFi",
		.cpu_dai = &tegra_i2s_dai[0],
		.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
		.init = tegra_codec_init,
		.ops = &tegra_hifi_ops,
	},
	{
		.name = "Tegra-generic",
		.stream_name = "Tegra Generic Voice",
		.cpu_dai = &tegra_i2s_dai[1],
		.codec_dai = &tegra_generic_codec_dai[0],
		.init = tegra_codec_init,
		.ops = &tegra_voice_ops,
	},
};

static struct snd_soc_card tegra_snd_soc = {
	.name = "tegra",
	.platform = &tegra_soc_platform,
	.dai_link = tegra_soc_dai,
	.num_links = ARRAY_SIZE(tegra_soc_dai),
};

static struct snd_soc_device tegra_snd_devdata = {
	.card = &tegra_snd_soc,
	.codec_dev = &soc_codec_dev_wm8753,
};

static int __init tegra_init(void)
{
	int ret = 0;

	tegra_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tegra_snd_device) {
		pr_err("failed to allocate soc-audio \n");
		return ENOMEM;
	}

	platform_set_drvdata(tegra_snd_device, &tegra_snd_devdata);
	tegra_snd_devdata.dev = &tegra_snd_device->dev;
	ret = platform_device_add(tegra_snd_device);
	if (ret) {
		pr_err("audio device could not be added \n");
		goto fail;
	}

	wm8753_reg = regulator_get(NULL, "avddio_audio");
	if (IS_ERR(wm8753_reg)) {
		ret = PTR_ERR(wm8753_reg);
		pr_err("unable to get wm8753 regulator\n");
		goto fail;
	}

	ret = regulator_enable(wm8753_reg);
	if (ret) {
		pr_err("wm8753 regulator enable failed\n");
		goto err_put_regulator;
	}

	return 0;

fail:
	if (tegra_snd_device) {
		platform_device_put(tegra_snd_device);
		tegra_snd_device = 0;
	}

	return ret;

err_put_regulator:
	regulator_put(wm8753_reg);
	return ret;
}

static void __exit tegra_exit(void)
{
	tegra_controls_exit();
	platform_device_unregister(tegra_snd_device);
	regulator_disable(wm8753_reg);
	regulator_put(wm8753_reg);

	if (wm8753_jack) {
		gpio_free(wm8753_jack->gpio);
		kfree(wm8753_jack);
		wm8753_jack = NULL;
	}
}

module_init(tegra_init);
module_exit(tegra_exit);

/* Module information */
MODULE_DESCRIPTION("Tegra ALSA SoC");
MODULE_LICENSE("GPL");

