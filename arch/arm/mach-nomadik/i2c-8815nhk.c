/*
 *  linux/arch/arm/mach-nomadik/i2c-8815nhk.c
 *
 *  Updated 2012 Fabrizio Ghiringhelli <fghiro@gmail.com>
 *
 *  NHK15 board specifc i2c driver definition
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <plat/gpio-nomadik.h>

#include <linux/i2c-stn8815.h>

/*
 * There are two busses in the 8815NHK.
 * They can be driven either by the hardware component, or by bit-bang through
 * GPIO.
 */

#define	I2C_SCL0_PIN	62
#define	I2C_SDA0_PIN	63
#define	I2C_SCL1_PIN	53
#define	I2C_SDA1_PIN	54

#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
static struct i2c_gpio_platform_data nhk8815_i2c_data0 = {
	/* keep defaults for timeouts; pins are push-pull bidirectional */
	.scl_pin = I2C_SCL0_PIN,
	.sda_pin = I2C_SDA0_PIN,
};

static struct i2c_gpio_platform_data nhk8815_i2c_data1 = {
	/* keep defaults for timeouts; pins are push-pull bidirectional */
	.scl_pin = I2C_SCL1_PIN,
	.sda_pin = I2C_SDA1_PIN,
};

/* first bus: GPIO XX and YY */
static struct platform_device nhk8815_i2c_dev0 = {
	.name	= "i2c-gpio",
	.id	= 0,
	.dev	= {
		.platform_data = &nhk8815_i2c_data0,
	},
};
/* second bus: GPIO XX and YY */
static struct platform_device nhk8815_i2c_dev1 = {
	.name	= "i2c-gpio",
	.id	= 1,
	.dev	= {
		.platform_data = &nhk8815_i2c_data1,
	},
};

static int __init nhk8815_i2c_init(void)
{
	nmk_gpio_set_mode(nhk8815_i2c_data0.scl_pin, NMK_GPIO_ALT_GPIO);
	nmk_gpio_set_mode(nhk8815_i2c_data0.sda_pin, NMK_GPIO_ALT_GPIO);
	platform_device_register(&nhk8815_i2c_dev0);

	nmk_gpio_set_mode(nhk8815_i2c_data1.scl_pin, NMK_GPIO_ALT_GPIO);
	nmk_gpio_set_mode(nhk8815_i2c_data1.sda_pin, NMK_GPIO_ALT_GPIO);
	platform_device_register(&nhk8815_i2c_dev1);

	return 0;
}
arch_initcall(nhk8815_i2c_init);


#elif defined(CONFIG_I2C_NOMADIK_STN8815) || (CONFIG_I2C_NOMADIK_STN8815_MODULE)

static struct resource nhk8815_i2c_resources[] = {
	{
		.start = NOMADIK_I2C0_BASE,
		.end = NOMADIK_I2C0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_I2C0,
		.end = IRQ_I2C0,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = NOMADIK_I2C1_BASE,
		.end = NOMADIK_I2C1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_I2C1,
		.end = IRQ_I2C1,
		.flags = IORESOURCE_IRQ,
	}
};

/* first bus: i2c0 */
static struct i2c_stn8815_platform_data nhk8815_i2c_dev0_data = {
	.filter	= I2C_STN8815_FILTER_1_CLK,	/* 1 clock-wide-spikes filter */
	.speed	= I2C_STN8815_SPEED_FAST,	/* fast mode (up to 400Kb/s)  */
	.master_code = 0,
};

static struct platform_device nhk8815_i2c_dev0 = {
	.name		= "stn8815_i2c",
	.id		= 0,
	.resource	= &nhk8815_i2c_resources[0],
	.num_resources	= 2,
	.dev		= {
		.platform_data	= &nhk8815_i2c_dev0_data,
	},
};

/* second bus: i2c1 */
static struct platform_device nhk8815_i2c_dev1 = {
	.name		= "stn8815_i2c",
	.id		= 1,
	.resource	= &nhk8815_i2c_resources[2],
	.num_resources	= 2,
	/* No platform data: use driver defaults */
};

static int __init nhk8815_i2c_init(void)
{
	int err;

	err =	nmk_gpio_set_mode(I2C_SCL0_PIN, NMK_GPIO_ALT_A) +
		nmk_gpio_set_mode(I2C_SDA0_PIN, NMK_GPIO_ALT_A);
	platform_device_register(&nhk8815_i2c_dev0);

	err +=	nmk_gpio_set_mode(I2C_SCL1_PIN, NMK_GPIO_ALT_A) +
		nmk_gpio_set_mode(I2C_SDA1_PIN, NMK_GPIO_ALT_A);
	platform_device_register(&nhk8815_i2c_dev1);

	if (err != 0)
		printk(KERN_ERR "I2C: error on setting pin mode\n");

	return 0;
}
arch_initcall(nhk8815_i2c_init);


#else

static int __init nhk8815_i2c_init(void) {}
arch_initcall(nhk8815_i2c_init);

#endif

