/*
 * Pinctrl driver for Microchip GSC32 SoCs
 * Copyright (c) 2015 Microchip Technology Inc.
 * Written by Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <dm/pinctrl.h>
#include <mach/gsc32.h>

DECLARE_GLOBAL_DATA_PTR;

/* GSC32 has 10 peripheral ports with 16 pins each.
 * Ports are marked PORTA-PORTK or PORT0-PORT9.
 */
enum {
	GSC32_PORT_A = 0,
	GSC32_PORT_B = 1,
	GSC32_PORT_C = 2,
	GSC32_PORT_D = 3,
	GSC32_PORT_E = 4,
	GSC32_PORT_F = 5,
	GSC32_PORT_G = 6,
	GSC32_PORT_H = 7,
	GSC32_PORT_J = 8, /* no PORT_I */
	GSC32_PORT_K = 9,
	GSC32_PINS_PER_PORT = 16,
};

#define PIN_CONFIG_GSC32_DIGITAL	(PIN_CONFIG_END + 1)
#define PIN_CONFIG_GSC32_ANALOG		(PIN_CONFIG_END + 2)

/* pin configuration descriptor */
struct gsc32_pin_config {
	u16 port;	/* port number */
	u16 pin;	/* pin number in the port */
	u32 config;	/* one of PIN_CONFIG_* */
};
#define PIN_CONFIG(_prt, _pin, _cfg) \
	{.port = (_prt), .pin = (_pin), .config = (_cfg), }

/* In GSC32 muxing is performed at pin-level through two
 * different set of registers - one set for input functions,
 * and other for output functions.
 * Pin configuration is handled through port register.
 */
/* Port control registers */
struct gsc32_reg_port {
	struct gsc32_reg_atomic ansel;
	struct gsc32_reg_atomic tris;
	struct gsc32_reg_atomic port;
	struct gsc32_reg_atomic lat;
	struct gsc32_reg_atomic odc;
	struct gsc32_reg_atomic cnpu;
	struct gsc32_reg_atomic cnpd;
	struct gsc32_reg_atomic cncon;
	struct gsc32_reg_atomic unused[8];
};

/* Input function mux registers */
struct gsc32_reg_in_mux {
	u32 unused0;
	u32 int1[4];
	u32 unused1;
	u32 t2ck[8];
	u32 ic1[9];
	u32 unused2;
	u32 ocfar;
	u32 unused3;
	u32 u1rx;
	u32 u1cts;
	u32 u2rx;
	u32 u2cts;
	u32 u3rx;
	u32 u3cts;
	u32 u4rx;
	u32 u4cts;
	u32 u5rx;
	u32 u5cts;
	u32 u6rx;
	u32 u6cts;
	u32 unused4;
	u32 sdi1;
	u32 ss1;
	u32 unused5;
	u32 sdi2;
	u32 ss2;
	u32 unused6;
	u32 sdi3;
	u32 ss3;
	u32 unused7;
	u32 sdi4;
	u32 ss4;
	u32 unused8;
	u32 sdi5;
	u32 ss5;
	u32 unused9;
	u32 sdi6;
	u32 ss6;
	u32 c1rx;
	u32 c2rx;
	u32 refclki1;
	u32 refclki2;
	u32 refclki3;
	u32 refclki4;
};

/* output mux register offset */
#define PPS_OUT(__port, __pin) \
	(((__port) * GSC32_PINS_PER_PORT + (__pin)) << 2)


struct gsc32_pinctrl_priv {
	struct gsc32_reg_in_mux *mux_in; /* mux input function */
	struct gsc32_reg_port *pinconf; /* pin configuration*/
	void __iomem *mux_out;	/* mux output function */
};

enum {
	PERIPH_ID_UART1,
	PERIPH_ID_UART2,
	PERIPH_ID_ETH,
	PERIPH_ID_USB,
	PERIPH_ID_SDHCI,
	PERIPH_ID_I2C1,
	PERIPH_ID_I2C2,
	PERIPH_ID_SPI1,
	PERIPH_ID_SPI2,
	PERIPH_ID_SQI,
};

static int gsc32_pinconfig_one(struct gsc32_pinctrl_priv *priv,
			       u32 port_nr, u32 pin, u32 param)
{
	struct gsc32_reg_port *port;

	port = &priv->pinconf[port_nr];
	switch (param) {
	case PIN_CONFIG_GSC32_DIGITAL:
		writel(BIT(pin), &port->ansel.clr);
		break;
	case PIN_CONFIG_GSC32_ANALOG:
		writel(BIT(pin), &port->ansel.set);
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		writel(BIT(pin), &port->tris.set);
		break;
	case PIN_CONFIG_OUTPUT:
		writel(BIT(pin), &port->tris.clr);
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		writel(BIT(pin), &port->cnpu.set);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		writel(BIT(pin), &port->cnpd.set);
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		writel(BIT(pin), &port->odc.set);
		break;
	default:
		break;
	}

	return 0;
}

static int gsc32_pinconfig_set(struct gsc32_pinctrl_priv *priv,
			       const struct gsc32_pin_config *list, int count)
{
	int i;

	for (i = 0 ; i < count; i++)
		gsc32_pinconfig_one(priv, list[i].port,
				    list[i].pin, list[i].config);

	return 0;
}

static void gsc32_eth_pin_config(struct udevice *dev)
{
	struct gsc32_pinctrl_priv *priv = dev_get_priv(dev);
	const struct gsc32_pin_config configs[] = {
		/* EMDC - D11 */
		PIN_CONFIG(GSC32_PORT_D, 11, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_D, 11, PIN_CONFIG_OUTPUT),
		/* ETXEN */
		PIN_CONFIG(GSC32_PORT_D, 6, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_D, 6, PIN_CONFIG_OUTPUT),
		/* ECRSDV */
		PIN_CONFIG(GSC32_PORT_H, 13, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_H, 13, PIN_CONFIG_INPUT_ENABLE),
		/* ERXD0 */
		PIN_CONFIG(GSC32_PORT_H, 8, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_H, 8, PIN_CONFIG_INPUT_ENABLE),
		PIN_CONFIG(GSC32_PORT_H, 8, PIN_CONFIG_BIAS_PULL_DOWN),
		/* ERXD1 */
		PIN_CONFIG(GSC32_PORT_H, 5, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_H, 5, PIN_CONFIG_INPUT_ENABLE),
		PIN_CONFIG(GSC32_PORT_H, 5, PIN_CONFIG_BIAS_PULL_DOWN),
		/* EREFCLK */
		PIN_CONFIG(GSC32_PORT_J, 11, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_J, 11, PIN_CONFIG_INPUT_ENABLE),
		/* ETXD1 */
		PIN_CONFIG(GSC32_PORT_J, 9, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_J, 9, PIN_CONFIG_OUTPUT),
		/* ETXD0 */
		PIN_CONFIG(GSC32_PORT_J, 8, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_J, 8, PIN_CONFIG_OUTPUT),
		/* EMDIO */
		PIN_CONFIG(GSC32_PORT_J, 1, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_J, 1, PIN_CONFIG_INPUT_ENABLE),
		/* ERXERR */
		PIN_CONFIG(GSC32_PORT_F, 3, PIN_CONFIG_GSC32_DIGITAL),
		PIN_CONFIG(GSC32_PORT_F, 3, PIN_CONFIG_INPUT_ENABLE),
	};

	gsc32_pinconfig_set(priv, configs, ARRAY_SIZE(configs));
}

static int gsc32_pinctrl_request(struct udevice *dev, int func, int flags)
{
	struct gsc32_pinctrl_priv *priv = dev_get_priv(dev);

	switch (func) {
	case PERIPH_ID_UART2:
		/* PPS for U2 RX/TX */
		writel(0x02, priv->mux_out + PPS_OUT(GSC32_PORT_G, 9));
		writel(0x05, &priv->mux_in->u2rx); /* B0 */
		/* set digital mode */
		gsc32_pinconfig_one(priv, GSC32_PORT_G, 9,
				    PIN_CONFIG_GSC32_DIGITAL);
		gsc32_pinconfig_one(priv, GSC32_PORT_B, 0,
				    PIN_CONFIG_GSC32_DIGITAL);
		break;
	case PERIPH_ID_ETH:
		gsc32_eth_pin_config(dev);
		break;
	default:
		debug("%s: unknown-unhandled case\n", __func__);
		break;
	}

	return 0;
}

static int gsc32_pinctrl_get_periph_id(struct udevice *dev,
				       struct udevice *periph)
{
	int ret;
	u32 cell[2];

	ret = fdtdec_get_int_array(gd->fdt_blob, periph->of_offset,
				   "interrupts", cell, ARRAY_SIZE(cell));
	if (ret < 0)
		return -EINVAL;

	/* interrupt number */
	switch (cell[0]) {
	case 112 ... 114:
		return PERIPH_ID_UART1;
	case 145 ... 147:
		return PERIPH_ID_UART2;
	case 109 ... 111:
		return PERIPH_ID_SPI1;
	case 142 ... 144:
		return PERIPH_ID_SPI2;
	case 115 ... 117:
		return PERIPH_ID_I2C1;
	case 148 ... 150:
		return PERIPH_ID_I2C2;
	case 132 ... 133:
		return PERIPH_ID_USB;
	case 169:
		return PERIPH_ID_SQI;
	case 191:
		return PERIPH_ID_SDHCI;
	case 153:
		return PERIPH_ID_ETH;
	default:
		break;
	}

	return -ENOENT;
}

static int gsc32_pinctrl_set_state_simple(struct udevice *dev,
					  struct udevice *periph)
{
	int func;

	debug("%s: periph %s\n", __func__, periph->name);
	func = gsc32_pinctrl_get_periph_id(dev, periph);
	if (func < 0)
		return func;
	return gsc32_pinctrl_request(dev, func, 0);
}

static struct pinctrl_ops gsc32_pinctrl_ops = {
	.set_state_simple	= gsc32_pinctrl_set_state_simple,
	.request		= gsc32_pinctrl_request,
	.get_periph_id		= gsc32_pinctrl_get_periph_id,
};

static int gsc32_pinctrl_probe(struct udevice *dev)
{
	struct gsc32_pinctrl_priv *priv = dev_get_priv(dev);
	struct fdt_resource res;
	void *fdt = (void *)gd->fdt_blob;
	int node = dev->of_offset;
	int ret;

	ret = fdt_get_named_resource(fdt, node, "reg", "reg-names",
				     "ppsin", &res);
	if (ret < 0) {
		printf("pinctrl: resource \"ppsin\" not found\n");
		return ret;
	}
	priv->mux_in = ioremap(res.start, fdt_resource_size(&res));

	ret = fdt_get_named_resource(fdt, node, "reg", "reg-names",
				     "ppsout", &res);
	if (ret < 0) {
		printf("pinctrl: resource \"ppsout\" not found\n");
		return ret;
	}
	priv->mux_out = ioremap(res.start, fdt_resource_size(&res));

	ret = fdt_get_named_resource(fdt, node, "reg", "reg-names",
				     "port", &res);
	if (ret < 0) {
		printf("pinctrl: resource \"port\" not found\n");
		return ret;
	}
	priv->pinconf = ioremap(res.start, fdt_resource_size(&res));

	return 0;
}

static const struct udevice_id gsc32_pinctrl_ids[] = {
	{ .compatible = "godson,gsc32-pinctrl" },
	{ }
};

U_BOOT_DRIVER(pinctrl_gsc32) = {
	.name		= "pinctrl_gsc32",
	.id		= UCLASS_PINCTRL,
	.of_match	= gsc32_pinctrl_ids,
	.ops		= &gsc32_pinctrl_ops,
	.probe		= gsc32_pinctrl_probe,
	.bind		= dm_scan_fdt_dev,
	.priv_auto_alloc_size = sizeof(struct gsc32_pinctrl_priv),
};
