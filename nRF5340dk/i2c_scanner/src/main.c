/*
 * Copyright (c) 2018 Tavish Naruka <tavishnaruka@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>


/**
 * @file This app scans I2C bus for any devices present
 */

void main(void)
{
	struct device *i2c_dev;

	printk("Starting i2c scanner\n");

	i2c_dev = device_get_binding("I2C_1");
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	// if (i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_FAST)) != 0)	{
	// 	printk("I2C: Configuration failed.\n");
	// }

	struct i2c_msg msgs[1];
	uint8_t dst;

	/* Send the address to read from */
	msgs[0].buf = &dst;
	msgs[0].len = 0U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	//int transferResult = i2c_transfer(i2c_dev, &msgs[0], 1, 0x57);
	//printk("transfer result: %d \n", transferResult);

	// if (transferResult == 0) {
	// 	printk("FOUND\n");
	// }

	for (uint8_t i = 7; i <= 0x10; i++) {
		
		struct i2c_msg msgs[1];
		uint8_t dst;

		/* Send the address to read from */
		msgs[0].buf = &dst;
		msgs[0].len = 0U;
		msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

		if (i2c_transfer(i2c_dev, &msgs[0], 1, i) == 0) {
			printk("0x%2x FOUND\n", i);
		}
		k_usleep(20);
	}

	printk("Scanning complete. ");
}
