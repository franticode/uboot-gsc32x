/*
 * (c) 2015 Paul Thacker <paul.thacker@microchip.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */
#include <common.h>
#include <clk.h>
#include <dm.h>
#include <serial.h>
#include <wait_bit.h>
#include <mach/gsc32.h>
#include <dt-bindings/clock/godson,clock.h>

DECLARE_GLOBAL_DATA_PTR;

/* UART Control Registers */
#define U_MOD		0x00
#define U_MODCLR	(U_MOD + _CLR_OFFSET)
#define U_MODSET	(U_MOD + _SET_OFFSET)
#define U_STA		0x10
#define U_STACLR	(U_STA + _CLR_OFFSET)
#define U_STASET	(U_STA + _SET_OFFSET)
#define U_TXR		0x20
#define U_RXR		0x30
#define U_BRG		0x40

/* U_MOD bits */
#define UART_ENABLE		BIT(15)

/* U_STA bits */
#define UART_RX_ENABLE		BIT(12)
#define UART_TX_BRK		BIT(11)
#define UART_TX_ENABLE		BIT(10)
#define UART_TX_FULL		BIT(9)
#define UART_TX_EMPTY		BIT(8)
#define UART_RX_OVER		BIT(1)
#define UART_RX_DATA_AVAIL	BIT(0)

struct gsc32_uart_priv {
	void __iomem *base;
	ulong uartclk;
};

/*
 * Initialize the serial port with the given baudrate.
 * The settings are always 8 data bits, no parity, 1 stop bit, no start bits.
 */
static int gsc32_serial_init(void __iomem *base, ulong clk, u32 baudrate)
{
	u32 div = DIV_ROUND_CLOSEST(clk, baudrate * 16);

	/* wait for TX FIFO to empty */
	wait_for_bit(__func__, base + U_STA, UART_TX_EMPTY,
		     true, CONFIG_SYS_HZ, false);

	/* send break */
	writel(UART_TX_BRK, base + U_STASET);

	/* disable and clear mode */
	writel(0, base + U_MOD);
	writel(0, base + U_STA);

	/* set baud rate generator */
	writel(div - 1, base + U_BRG);

	/* enable the UART for TX and RX */
	writel(UART_TX_ENABLE | UART_RX_ENABLE, base + U_STASET);

	/* enable the UART */
	writel(UART_ENABLE, base + U_MODSET);
	return 0;
}

/* Check whether any char pending in RX fifo */
static int gsc32_uart_pending_input(void __iomem *base)
{
	/* check if rx buffer overrun error has occurred */
	if (readl(base + U_STA) & UART_RX_OVER) {
		readl(base + U_RXR);

		/* clear overrun error to keep receiving */
		writel(UART_RX_OVER, base + U_STACLR);
	}

	/* In GSC32 there is no way to know number of outstanding
	 * chars in rx-fifo. Only it can be known whether there is any.
	 */
	return readl(base + U_STA) & UART_RX_DATA_AVAIL;
}

static int gsc32_uart_pending(struct udevice *dev, bool input)
{
	struct gsc32_uart_priv *priv = dev_get_priv(dev);

	if (input)
		return gsc32_uart_pending_input(priv->base);

	return !(readl(priv->base + U_STA) & UART_TX_EMPTY);
}

static int gsc32_uart_setbrg(struct udevice *dev, int baudrate)
{
	struct gsc32_uart_priv *priv = dev_get_priv(dev);

	return gsc32_serial_init(priv->base, priv->uartclk, baudrate);
}

static int gsc32_uart_putc(struct udevice *dev, const char ch)
{
	struct gsc32_uart_priv *priv = dev_get_priv(dev);

	/* Check if Tx FIFO is full */
	if (readl(priv->base + U_STA) & UART_TX_FULL)
		return -EAGAIN;

	/* pump the char to tx buffer */
	writel(ch, priv->base + U_TXR);

	return 0;
}

static int gsc32_uart_getc(struct udevice *dev)
{
	struct gsc32_uart_priv *priv = dev_get_priv(dev);

	/* return error if RX fifo is empty */
	if (!gsc32_uart_pending_input(priv->base))
		return -EAGAIN;

	/* read the character from rx buffer */
	return readl(priv->base + U_RXR) & 0xff;
}

static int gsc32_uart_probe(struct udevice *dev)
{
	struct gsc32_uart_priv *priv = dev_get_priv(dev);
	struct clk clk;
	fdt_addr_t addr;
	fdt_size_t size;
	int ret;

	/* get address */
	addr = fdtdec_get_addr_size(gd->fdt_blob, dev->of_offset, "reg", &size);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->base = ioremap(addr, size);

	/* get clock rate */
	ret = clk_get_by_index(dev, 0, &clk);
	if (ret < 0)
		return ret;
	priv->uartclk = clk_get_rate(&clk);
	clk_free(&clk);

	/* initialize serial */
	return gsc32_serial_init(priv->base, priv->uartclk, CONFIG_BAUDRATE);
}

static const struct dm_serial_ops gsc32_uart_ops = {
	.putc		= gsc32_uart_putc,
	.pending	= gsc32_uart_pending,
	.getc		= gsc32_uart_getc,
	.setbrg		= gsc32_uart_setbrg,
};

static const struct udevice_id gsc32_uart_ids[] = {
	{ .compatible = "godson,gsc32-uart" },
	{}
};

U_BOOT_DRIVER(gsc32_serial) = {
	.name		= "gsc32-uart",
	.id		= UCLASS_SERIAL,
	.of_match	= gsc32_uart_ids,
	.probe		= gsc32_uart_probe,
	.ops		= &gsc32_uart_ops,
	.flags		= DM_FLAG_PRE_RELOC,
	.priv_auto_alloc_size = sizeof(struct gsc32_uart_priv),
};

#ifdef CONFIG_DEBUG_UART_GSC32
#include <debug_uart.h>

static inline void _debug_uart_init(void)
{
	void __iomem *base = (void __iomem *)CONFIG_DEBUG_UART_BASE;

	gsc32_serial_init(base, CONFIG_DEBUG_UART_CLOCK, CONFIG_BAUDRATE);
}

static inline void _debug_uart_putc(int ch)
{
	writel(ch, CONFIG_DEBUG_UART_BASE + U_TXR);
}

DEBUG_UART_FUNCS
#endif
