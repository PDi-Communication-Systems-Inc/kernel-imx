/*
 * imx-wm8960.c
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/kthread.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <mach/audmux.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>

#include "imx-ssi.h"
#include "../codecs/wm8960.h"

struct imx_priv {
	int sysclk;         /*mclk from the outside*/
	int codec_sysclk;
	int dai_hifi;
	int hp_irq;
	int hp_status;
	int amic_irq;
	int amic_status;
	struct platform_device *pdev;
};
unsigned int sample_format = SNDRV_PCM_FMTBIT_S16_LE;
static struct imx_priv card_priv;
static struct snd_soc_card snd_soc_card_imx;
static struct snd_soc_codec *gcodec;
static int suspend_hp_flag;
static int suspend_mic_flag;
static int hp_irq;
static int mic_irq;

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat = priv->pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(1);

	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat = priv->pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(0);

	return;
}

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	unsigned int channels = params_channels(params);
	unsigned int sample_rate = 44100;
	int ret = 0;
	u32 dai_format;
	int dacdiv;
	unsigned sysclk;

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	sample_rate = params_rate(params);
	sample_format = params_format(params);

	ret = snd_soc_dai_set_clkdiv(rtd->codec_dai, WM8960_SYSCLKDIV, WM8960_SYSCLK_DIV_2);
	if(ret < 0){
		pr_err("-%s(): Codec SYSCLKDIV setting error\n", __FUNCTION__);
		return ret;
	}

	switch ( sample_rate ) {
	case 44100:
		dacdiv = WM8960_DAC_DIV_1;
		sysclk = 11289600;
		break;
	case 48000:
		dacdiv = WM8960_DAC_DIV_1;
		sysclk = 12288000;
		break;
	default:
		pr_err("-%s(): SND RATE ERROR (%d)\n", __FUNCTION__, sample_rate);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8960_DACDIV, dacdiv);
	if(ret < 0){
		pr_err("-%s(): Codec DACDIV setting error, %d\n", __FUNCTION__, ret );
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, priv->sysclk, sysclk * 2);
	if( ret < 0 ){
		pr_err("-%s(): Codec SYSCLK setting error, %d\n", __FUNCTION__, ret);
		return ret;
	}

	pr_debug(" *** sample_rate = %d, bclk = %d, sysclk = %d\n", sample_rate, priv->sysclk, sysclk);

	return ret;
}

/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_dapm_widgets[] = {
#if 0
	SND_SOC_DAPM_SPK("Speaker_R", NULL),
	SND_SOC_DAPM_SPK("Speaker_L", NULL),
#endif
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("MIC Jack", NULL),
};

/* imx machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
#if 0
	{ "Speaker_R", NULL, "SPK_RP" },
	{ "Speaker_R", NULL, "SPK_RN" },

	{ "Speaker_L", NULL, "SPK_LP" },
	{ "Speaker_L", NULL, "SPK_LN" },
#endif
	{ "Headphone Jack", NULL, "HP_L" },
	{ "Headphone Jack", NULL, "HP_R" },

	{ "LINPUT1", NULL, "MICB" },
	{ "MICB", NULL, "MIC Jack" },
};

static int imx_wm8960_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret = 0;

  	//snd_soc_dapm_nc_pin(&codec->dapm, "LINPUT1");
  	snd_soc_dapm_nc_pin(&codec->dapm, "RINPUT1");
  	snd_soc_dapm_nc_pin(&codec->dapm, "LINPUT2");
  	snd_soc_dapm_nc_pin(&codec->dapm, "RINPUT2");
  	snd_soc_dapm_nc_pin(&codec->dapm, "LINPUT3");
  	snd_soc_dapm_nc_pin(&codec->dapm, "RINPUT3");
  	snd_soc_dapm_nc_pin(&codec->dapm, "OUT3");

	/* Add imx specific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_dapm_widgets,
				  ARRAY_SIZE(imx_dapm_widgets));

	/* Set up imx specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	//snd_soc_dapm_enable_pin(&codec->dapm, "Speaker_L");
  	//snd_soc_dapm_enable_pin(&codec->dapm, "Speaker_R");
  	//snd_soc_dapm_disable_pin(&codec->dapm, "Speaker_L");
	snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(&codec->dapm, "MIC Jack");

	snd_soc_dapm_sync(&codec->dapm);

	return 0;
}

static struct snd_soc_ops imx_hifi_ops = {
	/* .startup = imx_hifi_startup,
	.shutdown = imx_hifi_shutdown, */
	.hw_params = imx_hifi_hw_params,
};

static struct snd_soc_dai_link imx_dai[] = {
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.codec_dai_name	= "wm8960-hifi",
		.codec_name	= "wm8960-codec.0-001a",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_wm8960_init,
		.ops		= &imx_hifi_ops,
	},
};

static struct snd_soc_card snd_soc_card_imx = {
	.name		= "wm8960-audio",
	.dai_link	= imx_dai,
	.num_links	= ARRAY_SIZE(imx_dai),
};

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_wm8960_probe(struct platform_device *pdev)
{

	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_priv *priv = &card_priv;
	int ret = 0;

	priv->pdev = pdev;

	imx_audmux_config(plat->src_port, plat->ext_port);

	if (plat->init && plat->init()) {
		ret = -EINVAL;
		return ret;
	}

	priv->sysclk = plat->sysclk;

	return ret;
}

static int __devexit imx_wm8960_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_priv *priv = &card_priv;

	if (plat->finit)
		plat->finit();

	if (priv->hp_irq)
		free_irq(priv->hp_irq, priv);
	if (priv->amic_irq)
		free_irq(priv->amic_irq, priv);

	return 0;
}

static struct platform_driver imx_wm8960_driver = {
	.probe = imx_wm8960_probe,
	.remove = imx_wm8960_remove,
	.driver = {
		   .name = "imx-wm8960",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_snd_device;

static int __init imx_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_wm8960_driver);
	if (ret < 0)
		goto exit;

	imx_dai[0].codec_name = "wm8960-codec.0-001a";

	imx_snd_device = platform_device_alloc("soc-audio", 5);
	if (!imx_snd_device)
		goto err_device_alloc;

	platform_set_drvdata(imx_snd_device, &snd_soc_card_imx);

	ret = platform_device_add(imx_snd_device);

	if (0 == ret)
		goto exit;

	platform_device_put(imx_snd_device);

err_device_alloc:
	platform_driver_unregister(&imx_wm8960_driver);
exit:
	return ret;
}

static void __exit imx_asoc_exit(void)
{
	platform_driver_unregister(&imx_wm8960_driver);
	platform_device_unregister(imx_snd_device);
}

module_init(imx_asoc_init);
module_exit(imx_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC imx wm8960");
MODULE_LICENSE("GPL");
