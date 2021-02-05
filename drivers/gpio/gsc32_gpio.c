/*
 * Copyright (c) 2015 Microchip Technology Inc
 * Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <malloc.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/compat.h>
#include <mach/gsc32.h>

DECLARE_GLOBAL_DATA_PTR;

/* Peripheral Pin Control */
struct gsc32_reg_port {
	struct gsc32_reg_atomic ansel;
	struct gsc32_reg_atomic tris;
	struct gsc32_reg_atomic port;
	struct gsc32_reg_atomic lat;
	struct gsc32_reg_atomic open_drain;
	struct gsc32_reg_atomic cnpu;
	struct gsc32_reg_atomic cnpd;
	struct gsc32_reg_atomic cncon;
};

enum {
	GODSON_GPIO_DIR_OUT,
	GODSON_GPIO_DIR_IN,
	GODSON_GPIOS_PER_BANK = 16,
};

struct gsc32_gpio_priv {
	struct gsc32_reg_port *regs;
	char name[2];
};

static int gsc32_gpio_get_value(struct udevice *dev, unsigned offset)
{
	struct gsc32_gpio_priv *priv = dev_get_priv(dev);

	return !!(readl(&priv->regs->port.raw) & BIT(offset));
}

static int gsc32_gpio_set_value(struct udevice *dev, unsigned offset,
				int value)
{
	struct gsc32_gpio_priv *priv = dev_get_priv(dev);
	int mask = BIT(offset);

	if (value)
		writel(mask, &priv->regs->port.set);
	else
		writel(mask, &priv->regs->port.clr);

	return 0;
}

static int gsc32_gpio_direction(struct udevice *dev, unsigned offset)
{
	struct gsc32_gpio_priv *priv = dev_get_priv(dev);

	/* pin in analog mode ? */
	if (readl(&priv->regs->ansel.raw) & BIT(offset))
		return -EPERM;

	if (readl(&priv->regs->tris.raw) & BIT(offset))
		return GODSON_GPIO_DIR_IN;
	else
		return GODSON_GPIO_DIR_OUT;
}

static int gsc32_gpio_direction_input(struct udevice *dev, unsigned offset)
{
	struct gsc32_gpio_priv *priv = dev_get_priv(dev);
	int mask = BIT(offset);

	writel(mask, &priv->regs->ansel.clr);
	writel(mask, &priv->regs->tris.set);

	return 0;
}

static int gsc32_gpio_direction_output(struct udevice *dev,
				       unsigned offset, int value)
{
	struct gsc32_gpio_priv *priv = dev_get_priv(dev);
	int mask = BIT(offset);

	writel(mask, &priv->regs->ansel.clr);
	writel(mask, &priv->regs->tris.clr);

	gsc32_gpio_set_value(dev, offset, value);
	return 0;
}

static int gsc32_gpio_get_function(struct udevice *dev, unsigned offset)
{
	int ret = GPIOF_UNUSED;

	switch (gsc32_gpio_direction(dev, offset)) {
	case GODSON_GPIO_DIR_OUT:
		ret = GPIOF_OUTPUT;
		break;
	case GODSON_GPIO_DIR_IN:
		ret = GPIOF_INPUT;
		break;
	default:
		ret = GPIOF_UNUSED;
		break;
	}
	return ret;
}

static const struct dm_gpio_ops gpio_gsc32_ops = {
	.direction_input	= gsc32_gpio_direction_input,
	.direction_output	= gsc32_gpio_direction_output,
	.get_value		= gsc32_gpio_get_value,
	.set_value		= gsc32_gpio_set_value,
	.get_function		= gsc32_gpio_get_function,
};

static int gsc32_gpio_probe(struct udevice *dev)
{
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct gsc32_gpio_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;
	fdt_size_t size;
	char *end;
	int bank;

	addr = fdtdec_get_addr_size(gd->fdt_blob, dev->of_offset, "reg", &size);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->regs = ioremap(addr, size);

	uc_priv->gpio_count = GODSON_GPIOS_PER_BANK;
	/* extract bank name */
	end = strrchr(dev->name, '@');
	bank = trailing_strtoln(dev->name, end);
	priv->name[0] = 'A' + bank;
	uc_priv->bank_name = priv->name;

	return 0;
}

static const struct udevice_id gsc32_gpio_ids[] = {
	{ .compatible = "godson,gsc32-gpio" },
	{ }
};

U_BOOT_DRIVER(gpio_gsc32) = {
	.name		= "gpio_gsc32",
	.id		= UCLASS_GPIO,
	.of_match	= gsc32_gpio_ids,
	.ops		= &gpio_gsc32_ops,
	.probe		= gsc32_gpio_probe,
	.priv_auto_alloc_size	= sizeof(struct gsc32_gpio_priv),
};
