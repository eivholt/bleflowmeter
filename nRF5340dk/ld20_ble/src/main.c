/* main.c - Application main entry point */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

#include <device.h>
#include <drivers/i2c.h>

/* Custom Service Variables */
static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0x9f,0xf1,0xbe,0xf5,0x7b,0x60,0xa4,0x4f,0x85,0x00,0xf9,0xe3,0x08,0x31,0xb4,0xa1);

static struct bt_uuid_16 vnd_ld20_flow_uuid = BT_UUID_INIT_16(
	0x01);

static struct bt_uuid_16 vnd_ld20_flags_uuid = BT_UUID_INIT_16(
	0x02);

static const struct bt_uuid_16 vnd_long_uuid = BT_UUID_INIT_16(
	0xFF);

static uint8_t vnd_value[] = { 'V', 'e', 'n', 'd', 'o', 'r' };
static uint8_t vnd_value2[] = { 'S', 'e', 'n' };

#define I2C_DEV "I2C_1"
#define LD20_I2C_ADDRESS	0x08

const struct device *i2c_dev;

static const uint16_t LD20_STARTCONTINOUS_CMD1 = 	0x36;
static const uint16_t LD20_STARTCONTINOUS_CMD2 = 	0x08;
static const uint16_t LD20_PARTNAME_CMD = 			0x367C;
static const uint16_t LD20_SERIAL_CMD = 			0xE102;
static const uint8_t LD20_RESET_CMD = 				0x06;

const float SCALE_FACTOR_FLOW = 1200.0;
const float SCALE_FACTOR_TEMP = 200.0;
const char *UNIT_FLOW = " ml/h";
const char *UNIT_TEMP = " deg C";

uint8_t flag_air_in_line, flag_high_flow, flag_exp_smooth;
float scaled_flow_value, scaled_temperature_value;

static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(vnd_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static uint8_t simulate_vnd;
static uint8_t indicating;
static uint8_t flow_value = 1U;

static struct bt_gatt_indicate_params ind_params_flow;
static struct bt_gatt_indicate_params ind_params_flags;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	printk("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	//printk("Indication complete\n");
	indicating = 0U;
}

#define MAX_DATA 74
static uint8_t vnd_long_value[] = {
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '1',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '2',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '3',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '4',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '5',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '6',
		  '.', ' ' };

static ssize_t read_long_vnd(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(vnd_long_value));
}

static ssize_t write_long_vnd(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, const void *buf,
			      uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset + len > sizeof(vnd_long_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static struct bt_gatt_cep vnd_long_cep = {
	.properties = BT_GATT_CEP_RELIABLE_WRITE,
};

static const struct bt_uuid_128 vnd_write_cmd_uuid = BT_UUID_INIT_128(
	0x3c,0xf5,0x87,0x74,0xe5,0xdd,0xa2,0x45,0xa4,0xc9,0x7a,0xcb,0xd2,0xd2,0xd1,0xbb);

static uint8_t mfg_data[] = { 0xff, 0xff, 0x00 };

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(vnd_svc,
	BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
	BT_GATT_CHARACTERISTIC(&vnd_ld20_flow_uuid.uuid,
			       BT_GATT_CHRC_READ |
			       BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_READ,
			       read_vnd, NULL, vnd_value2),
	BT_GATT_CCC(vnd_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&vnd_ld20_flags_uuid.uuid,
			       BT_GATT_CHRC_READ |
			       BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_READ,
			       read_vnd, NULL, vnd_value),
	BT_GATT_CCC(vnd_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid, BT_GATT_CHRC_READ |
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE |
			       BT_GATT_PERM_PREPARE_WRITE,
			       read_long_vnd, write_long_vnd, &vnd_long_value),
	BT_GATT_CEP(&vnd_long_cep)
);

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 3),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x51,0xd4,0x41,0xab,0x50,0xa2,0xae,0x41,0xbe,0x4b,0x5d,0x82,0xd8,0xf5,0xd1,0xea),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}


static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

static void ld20_reset()
{
	int ret;
	struct i2c_msg msgs[1];
	uint8_t data[] = {LD20_RESET_CMD};
	msgs[0].buf = data;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	
	ret = i2c_transfer(i2c_dev, &msgs[0], 1, 0x00);
	if (ret) {
		printk("Error writing LD20_RESET_CMD to LD20! error code (%d)\n", ret);
		return;
	} else {
		printk("Wrote LD20_RESET_CMD to address 0x00.\n");
	}

	// Give LD20 time to soft reboot
	k_msleep(25);
}

static void ld20_startcontinous()
{
	int ret;
	struct i2c_msg msgs[1];
	uint8_t data2[] = {LD20_STARTCONTINOUS_CMD1, LD20_STARTCONTINOUS_CMD2};
	msgs[0].buf = data2;
	msgs[0].len = 2;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	
	ret = i2c_transfer(i2c_dev, &msgs[0], 2, LD20_I2C_ADDRESS);
	if (ret) {
		printk("Error writing LD20_STARTCONTINOUS to LD20! error code (%d)\n", ret);
		//return; Ignore error message, works anyway.
	} else {
		printk("Wrote LD20_STARTCONTINOUS to address 0x08.\n");
	}

	k_msleep(120);
}

void main(void)
{
	//printk("sizeof(int): %x, sizeof(float): %x\n", sizeof(int), sizeof(float));

	int ret;
	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	ld20_reset();
	ld20_startcontinous();

	int err;
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();
	bt_conn_cb_register(&conn_callbacks);
	
	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		k_sleep(K_SECONDS(1));

		/* Battery level simulation */
		bas_notify();

		/* Vendor indication simulation */
		if (simulate_vnd) {
			if (indicating) {
				continue;
			}

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
			uint8_t sensor_flow_crc, sensor_temperature_crc, sensor_signalingflags_crc;
		
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

			printk("Read flow: %.2f temperature: %.2f flags: Air in line: %x High flow: %x Exponential smoothing active: %x \n", 
				scaled_flow_value, 
				scaled_temperature_value, 
				flag_air_in_line, 
				flag_high_flow, 
				flag_exp_smooth);

			uint8_t sensor_flags[] =  { flag_air_in_line, flag_high_flow, flag_exp_smooth };
			//printk("Flow as hex:%a", scaled_flow_value);

			ind_params_flags.uuid = &vnd_ld20_flags_uuid.uuid;
			ind_params_flags.attr = &vnd_svc.attrs[0];
			ind_params_flags.func = indicate_cb;
			ind_params_flags.destroy = indicate_destroy;
			ind_params_flags.data = &sensor_flags;
			ind_params_flags.len = sizeof(sensor_flags);

			if (bt_gatt_indicate(NULL, &ind_params_flags) == 0) {
				indicating = 1U;
			}

			while(indicating){
				printk(" * ");
			}

			ind_params_flow.uuid = &vnd_ld20_flow_uuid.uuid;
			ind_params_flow.attr = &vnd_svc.attrs[0];
			ind_params_flow.func = indicate_cb;
			ind_params_flow.destroy = indicate_destroy;
			ind_params_flow.data = &scaled_flow_value;
			ind_params_flow.len = sizeof(scaled_flow_value);

			// if(bt_gatt_notify_multiple(NULL, 2, ) == 0) {
			// 	notifying = 1U;
			// }

			if (bt_gatt_indicate(NULL, &ind_params_flow) == 0) {
				indicating = 1U;
			}
		}
	}
}
