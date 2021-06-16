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
// static const uint16_t LD20_ENDCONTINOUS_CMD1 = 		0x3F;
// static const uint16_t LD20_ENDCONTINOUS_CMD2 = 		0xF9;
// static const uint16_t LD20_PARTNAME_CMD = 			0x367C;
// static const uint16_t LD20_SERIAL_CMD = 			0xE102;
static const uint8_t LD20_RESET_CMD = 				0x06;

const float SCALE_FACTOR_FLOW = 1200.0;
const float SCALE_FACTOR_TEMP = 200.0;
const char *UNIT_FLOW = " ml/h";
const char *UNIT_TEMP = " deg C";

static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}

static uint8_t simulate_vnd;
static uint8_t indicating;

static struct bt_gatt_indicate_params ind_params_flow;
static struct bt_gatt_indicate_params ind_params_flags;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	//printk("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	//printk("Indication complete\n");
	indicating = 0U;
}

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

static void ld20_reset()
{
	int ret;
	uint8_t i2c_cmd = LD20_RESET_CMD;

	ret = i2c_write(i2c_dev, &i2c_cmd, 1, 0x00);
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
	uint8_t i2c_cmd[] = {LD20_STARTCONTINOUS_CMD1, LD20_STARTCONTINOUS_CMD2};
	
	ret = i2c_write(i2c_dev, &i2c_cmd[0], 2, LD20_I2C_ADDRESS);
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
			uint8_t flag_air_in_line, flag_high_flow, flag_exp_smooth;
			float scaled_flow_value, scaled_temperature_value;
		
			uint8_t i2c_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
			
			ret = i2c_read(i2c_dev, &i2c_data[0], 9, LD20_I2C_ADDRESS);

			if (ret) {
				printk("Error reading measurement from LD20! error code (%d)\n", ret);
				k_msleep(50);
				// Might be warming up, keep trying.
			} else {
				printk("Read measurement from address 0x08.\n");
			}
			printk("Read 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X \n", i2c_data[0], i2c_data[1], i2c_data[3], i2c_data[4], i2c_data[6], i2c_data[7]);

			sensor_flow_value = i2c_data[0] << 8;
			sensor_flow_value |= i2c_data[1];
			sensor_flow_crc = i2c_data[2];
			sensor_temperature_value = i2c_data[3] << 8;
			sensor_temperature_value |= i2c_data[4];
			sensor_temperature_crc = i2c_data[5];
			sensor_signalingflags_value = i2c_data[6] << 8;
			sensor_signalingflags_value |= i2c_data[7];
			sensor_signalingflags_crc = i2c_data[8];

			signed_flow_value = (int16_t) sensor_flow_value;
			scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;

			signed_temperature_value = (int16_t) sensor_temperature_value;
			scaled_temperature_value = ((float) signed_temperature_value) / SCALE_FACTOR_TEMP;

			flag_air_in_line = ((i2c_data[7] >> 0) & 0x01);
			flag_high_flow = ((i2c_data[7] >> 1) & 0x01);
			flag_exp_smooth = ((i2c_data[7] >> 5) & 0x01);

			printk("Scaled flow: %.4f, sensor flow value: %ld, Unscaled flow: %d, Temperature: %.2f, Flags: AiL: %x, HiF: %x, ExpSmooth: %x \n", 
				scaled_flow_value,
				sensor_flow_value,
				signed_flow_value,
				scaled_temperature_value, 
				flag_air_in_line, 
				flag_high_flow, 
				flag_exp_smooth);

			uint8_t sensor_flags[] =  { flag_air_in_line, flag_high_flow, flag_exp_smooth };

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
				k_msleep(50);
				//printk(" - ");
			}

			ind_params_flow.uuid = &vnd_ld20_flow_uuid.uuid;
			ind_params_flow.attr = &vnd_svc.attrs[0];
			ind_params_flow.func = indicate_cb;
			ind_params_flow.destroy = indicate_destroy;
			ind_params_flow.data = &signed_flow_value;
			ind_params_flow.len = sizeof(signed_flow_value);

			if (bt_gatt_indicate(NULL, &ind_params_flow) == 0) {
				indicating = 1U;
			}
		}
	}
}
