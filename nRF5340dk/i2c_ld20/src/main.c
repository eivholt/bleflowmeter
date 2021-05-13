/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>

#define I2C_DEV "I2C_1"

/**
 * @file Sample app using I2C.
 */

#define LD20_I2C_ADDRESS	0x08

static const uint16_t LD20_STARTCONTINOUS_CMD1 = 	0x36;
static const uint16_t LD20_STARTCONTINOUS_CMD2 = 	0x08;
static const uint16_t LD20_PARTNAME_CMD = 			0x367C;
static const uint16_t LD20_SERIAL_CMD = 			0xE102;
static const uint8_t LD20_RESET_CMD = 				0x06;

const float SCALE_FACTOR_FLOW = 1200.0;
const float SCALE_FACTOR_TEMP = 200.0;
const char *UNIT_FLOW = " ml/h";
const char *UNIT_TEMP = " deg C";

static int write_bytes(const struct device *i2c_dev, uint16_t addr,
		       uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, LD20_I2C_ADDRESS);
}

static int read_bytes(const struct device *i2c_dev, uint16_t addr,
		      uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, LD20_I2C_ADDRESS);
}

void main(void)
{
	const struct device *i2c_dev;
	uint8_t product_info[4];
	//uint8_t data[16];
	int i, ret;

	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	/* Do one-byte read/write */

	struct i2c_msg msgs[1];
	uint8_t data[] = {LD20_RESET_CMD};
	msgs[0].buf = data;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	
	ret = i2c_transfer(i2c_dev, &msgs[0], 1, 0x00);
	if (ret) {
		printk("Error writing LD20_RESET_CMD to LD20! error code (%d)\n", ret);
		//return;
	} else {
		printk("Wrote LD20_RESET_CMD to address 0x00.\n");
	}

	k_msleep(25);

	struct i2c_msg msgs2[1];
	uint8_t data2[] = {LD20_STARTCONTINOUS_CMD1, LD20_STARTCONTINOUS_CMD2};
	msgs2[0].buf = data2;
	msgs2[0].len = 2;
	msgs2[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	
	ret = i2c_transfer(i2c_dev, &msgs2[0], 2, LD20_I2C_ADDRESS);
	if (ret) {
		printk("Error writing LD20_STARTCONTINOUS to LD20! error code (%d)\n", ret);
		//return;
	} else {
		printk("Wrote LD20_STARTCONTINOUS to address 0x08.\n");
	}

	k_msleep(120);

	while(true) {
		
		// Byte1: Flow 8msb
		// Byte2: Flow 8lsb
		// Byte3: CRC
		// Byte4: Temp 8msb
		// Byte5: Temp 8lsb
		// Byte6: CRC
		// Byte7: Signaling flags 8msb
		// Byte8: Signaling flags 8lsb
		// Byte9: CRC

		// Bit, Signaling flags (set to high = 1, set to low = 0)
		// 0 Air-in-Line flag
		// 1 High Flow flag
		// 2-4 Unused, reserved for future use.
		// 5 Exponential smoothing active
		// 6-15 Unused, reserved for future use


		uint16_t sensor_flow_value, sensor_temperature_value, sensor_signalingflags_value;
		int16_t signed_flow_value, signed_temperature_value;
		float scaled_flow_value, scaled_temperature_value;
		uint8_t sensor_flow_crc, sensor_temperature_crc, sensor_signalingflags_crc;
		uint8_t flag_air_in_line, flag_high_flow, flag_exp_smooth;


		struct i2c_msg msgs3[1];
		uint8_t data3[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		msgs3[0].buf = data3;
		msgs3[0].len = 9;
		msgs3[0].flags = I2C_MSG_READ | I2C_MSG_STOP;
		
		ret = i2c_transfer(i2c_dev, &msgs3[0], 9, LD20_I2C_ADDRESS);

		//printk("Read 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X \n", data3[0], data3[1], data3[3], data3[4], data3[6], data3[7]);

		sensor_flow_value = data3[0] << 8;
		sensor_flow_value |= data3[1];
		sensor_flow_crc = data3[2];
		sensor_temperature_value = data3[3] << 8;
		sensor_temperature_value |= data3[4];
		sensor_temperature_crc = data3[5];
		sensor_signalingflags_value = data3[6] << 8;
		sensor_signalingflags_value |= data3[7];
		sensor_signalingflags_crc = data3[8];

		signed_flow_value = (int16_t) sensor_flow_value;
		scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;

		signed_temperature_value = (int16_t) sensor_temperature_value;
		scaled_temperature_value = ((float) signed_temperature_value) / SCALE_FACTOR_TEMP;

		flag_air_in_line = ((data3[7] >> 0) & 0x01);
		flag_high_flow = ((data3[7] >> 1) & 0x01);
		flag_exp_smooth = ((data3[7] >> 5) & 0x01);


		printk("Read flow: %.2f temperature: %.2f flags: Air in line: %x High flow: %x Exponential smoothing active: %x \n", scaled_flow_value, scaled_temperature_value, flag_air_in_line, flag_high_flow, flag_exp_smooth);

		k_msleep(100);
	}
	
}
