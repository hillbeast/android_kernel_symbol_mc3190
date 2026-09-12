/*
 * sound/soc/pxa/mc3190.c -- Board specific driver for WM9714 audio codec
 *
 * Based on zylonite.c
 *
 * Copyright 2008 Wolfson Microelectronics PLC.
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <linux/mc3190.h>
#include "../codecs/wm9713.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-ac97.h"
#include "pxa-ssp.h"

/*
 * There is a physical switch SW15 on the board which changes the MCLK
 * for the WM9713 between the standard AC97 master clock and the
 * output of the CLK_POUT signal from the PXA.
 */
static int clk_pout;
module_param(clk_pout, int, 0);
MODULE_PARM_DESC(clk_pout, "Use CLK_POUT as WM9713 MCLK (SW15 on board).");

static struct clk *pout;

static struct snd_soc_card mc3190;

static int mc3190_spk_amp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_update_bits(codec, 0x1c, 0x3f00, 0x1c00);
		snd_soc_update_bits(codec, 0x1e, 0x1c00, 0x0800);
		mc3190_cpld_write(MC3190_CPLD_AUDIOAMP_ON_BIT, MC3190_CPLD_REG_AUDIO);
		msleep(5);
	} else {
		mc3190_cpld_write(MC3190_CPLD_AUDIOAMP_ON_BIT, MC3190_CPLD_REG_AUDIO + 2);
	}
	
	return 0;
}

static const struct snd_soc_dapm_widget mc3190_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Microphone", NULL),
	SND_SOC_DAPM_MIC("Handset Microphone", NULL),
	SND_SOC_DAPM_SPK("Multiactor", mc3190_spk_amp_event),
	SND_SOC_DAPM_SPK("Headset Earpiece", NULL),
};

/* Currently supported audio map */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Headphone output connected to HPL/HPR */
	{ "Headphone", NULL,  "HPL" },
	{ "Headphone", NULL,  "HPR" },

	/* On-board earpiece */
	{ "Headset Earpiece", NULL, "OUT3" },

	/* Headphone mic */
	{ "MIC2A", NULL, "Mic Bias" },
	{ "Mic Bias", NULL, "Headset Microphone" },

	/* On-board mic */
	{ "MIC1", NULL, "Mic Bias" },
	{ "Mic Bias", NULL, "Handset Microphone" },

	/* Multiactor differentially connected over SPKL/SPKR */
	{ "Multiactor", NULL, "SPKL" },
	{ "Multiactor", NULL, "SPKR" },
};

static int mc3190_wm9713_init(struct snd_soc_codec *codec)
{
	if (clk_pout)
		snd_soc_dai_set_pll(&codec->dai[0], 0, 0,
				    clk_get_rate(pout), 0);

	snd_soc_dapm_new_controls(codec, mc3190_dapm_widgets,
				  ARRAY_SIZE(mc3190_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* Static setup for now */
	snd_soc_dapm_enable_pin(codec, "Headphone");
	snd_soc_dapm_enable_pin(codec, "Headset Earpiece");
	snd_soc_dapm_enable_pin(codec, "Multiactor"); 

	snd_soc_dapm_sync(codec);
	return 0;
}

static int mc3190_voice_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0;
	unsigned int wm9713_div = 0;
	int ret = 0;
	int rate = params_rate(params);
	int width = snd_pcm_format_physical_width(params_format(params));

	/* Only support ratios that we can generate neatly from the AC97
	 * based master clock - in particular, this excludes 44.1kHz.
	 * In most applications the voice DAC will be used for telephony
	 * data so multiples of 8kHz will be the common case.
	 */
	switch (rate) {
	case 8000:
		wm9713_div = 12;
		break;
	case 16000:
		wm9713_div = 6;
		break;
	case 48000:
		wm9713_div = 2;
		break;
	default:
		/* Don't support OSS emulation */
		return -EINVAL;
	}

	/* Add 1 to the width for the leading clock cycle */
	pll_out = rate * (width + 1) * 8;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA_SSP_CLK_AUDIO, 0, 1);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, pll_out);
	if (ret < 0)
		return ret;

	if (clk_pout)
		ret = snd_soc_dai_set_clkdiv(codec_dai, WM9713_PCMCLK_PLL_DIV,
					     WM9713_PCMDIV(wm9713_div));
	else
		ret = snd_soc_dai_set_clkdiv(codec_dai, WM9713_PCMCLK_DIV,
					     WM9713_PCMDIV(wm9713_div));
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops mc3190_voice_ops = {
	.hw_params = mc3190_voice_hw_params,
};

static struct snd_soc_dai_link mc3190_dai[] = {
{
    .name = "AC97",
    .stream_name = "AC97 HiFi",
    .cpu_dai = &pxa_ac97_dai[PXA2XX_DAI_AC97_HIFI],
    .codec_dai = &wm9713_dai[WM9713_DAI_AC97_HIFI],
    .init = mc3190_wm9713_init,
//    .ops = &mc3190_ops,
},
{
    .name = "AC97 Auxiliary",
    .stream_name = "AC97 Aux",
    .cpu_dai = &pxa_ac97_dai[PXA2XX_DAI_AC97_AUX],
    .codec_dai = &wm9713_dai[WM9713_DAI_AC97_AUX],
    .ops = &mc3190_voice_ops,
},
};

static int mc3190_probe(struct platform_device *pdev)
{
	int ret;

	if (clk_pout) {
		pout = clk_get(NULL, "CLK_POUT");
		if (IS_ERR(pout)) {
			dev_err(&pdev->dev, "Unable to obtain CLK_POUT: %ld\n",
				PTR_ERR(pout));
			return PTR_ERR(pout);
		}

		ret = clk_enable(pout);
		if (ret != 0) {
			dev_err(&pdev->dev, "Unable to enable CLK_POUT: %d\n",
				ret);
			clk_put(pout);
			return ret;
		}

		dev_dbg(&pdev->dev, "MCLK enabled at %luHz\n",
			clk_get_rate(pout));
	}

	return 0;
}

static int mc3190_remove(struct platform_device *pdev)
{
	if (clk_pout) {
		clk_disable(pout);
		clk_put(pout);
	}

	return 0;
}

static int mc3190_suspend_post(struct platform_device *pdev,
				 pm_message_t state)
{
	if (clk_pout)
		clk_disable(pout);

	return 0;
}

static int mc3190_resume_pre(struct platform_device *pdev)
{
	int ret = 0;

	if (clk_pout) {
		ret = clk_enable(pout);
		if (ret != 0)
			dev_err(&pdev->dev, "Unable to enable CLK_POUT: %d\n",
				ret);
	}

	return ret;
}

static struct snd_soc_card mc3190 = {
	.name = "MC3190",
	.probe = &mc3190_probe,
	.remove = &mc3190_remove,
	.suspend_post = &mc3190_suspend_post,
	.resume_pre = &mc3190_resume_pre,
	.platform = &pxa2xx_soc_platform,
	.dai_link = mc3190_dai,
	.num_links = ARRAY_SIZE(mc3190_dai),
};

static struct snd_soc_device mc3190_snd_ac97_devdata = {
	.card = &mc3190,
	.codec_dev = &soc_codec_dev_wm9713,
};

static struct platform_device *mc3190_snd_ac97_device;

static int __init mc3190_init(void)
{
    int ret;

    printk(KERN_INFO "MC3190 Audio: Initializing soc-audio device...\n");

    mc3190_snd_ac97_device = platform_device_alloc("soc-audio", -1);
    if (!mc3190_snd_ac97_device)
        return -ENOMEM;

    platform_set_drvdata(mc3190_snd_ac97_device,
                 &mc3190_snd_ac97_devdata);
    mc3190_snd_ac97_devdata.dev = &mc3190_snd_ac97_device->dev;

    ret = platform_device_add(mc3190_snd_ac97_device);
    if (ret != 0) {
        printk(KERN_ERR "MC3190 Audio: platform_device_add failed! ret=%d\n", ret);
        platform_device_put(mc3190_snd_ac97_device);
    } else {
        printk(KERN_INFO "MC3190 Audio: platform_device_add succeeded.\n");
    }

    return ret;
}

static void __exit mc3190_exit(void)
{
	platform_device_unregister(mc3190_snd_ac97_device);
}

module_init(mc3190_init);
module_exit(mc3190_exit);

MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_DESCRIPTION("ALSA SoC WM9714 MC3190");
MODULE_LICENSE("GPL");
