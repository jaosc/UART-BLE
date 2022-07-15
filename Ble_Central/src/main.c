#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <sys/slist.h>
#include <bluetooth/conn.h>
#include <bluetooth/att.h>
#include <console/console.h>
#include <stdint.h>


static void start_scan(void);
static struct bt_conn *default_conn;

#define START_HANDLE 0x0001
#define END_HANDLE 0xFFFF

static struct bt_uuid_128 ble_uart_uppercase = BT_UUID_INIT_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00);

static struct bt_uuid_128 ble_uart_receive_data = BT_UUID_INIT_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03,0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00);

static struct bt_uuid_128 ble_uart_notify = BT_UUID_INIT_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03,0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x11);


static struct bt_uuid *primary_uuid = &ble_uart_uppercase.uuid;

#define CREATE_FLAG(flag) static atomic_t flag = (atomic_t)false
#define SET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)true)
#define UNSET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)false)
#define WAIT_FOR_FLAG(flag) \
	while (!(bool)atomic_get(&flag)) { \
		(void)k_sleep(K_MSEC(1)); \
	}

CREATE_FLAG(is_connected);
CREATE_FLAG(is_discover_complete);

static uint16_t chrc_handle;

//callback of discover procedure
static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params)
{
	int err;

	if (attr == NULL) {
		if (chrc_handle == 0) {
			printk("Did not discover long_chrc (%x)",chrc_handle);
		}

		(void)memset(params, 0, sizeof(*params));

		SET_FLAG(is_discover_complete);

		return BT_GATT_ITER_STOP;
	}

	//printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
	    bt_uuid_cmp(params->uuid, &ble_uart_uppercase.uuid) == 0) {
		printk("Found service\n");
		params->uuid = NULL;
		params->start_handle = attr->handle + 1;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, params);
		if (err != 0) {
			printk("Discover failed (err %d)\n", err);
		}

		return BT_GATT_ITER_STOP;
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

		if (bt_uuid_cmp(chrc->uuid, &ble_uart_receive_data.uuid) == 0) {
			printk("Found TX_CHRC\n");
			chrc_handle = chrc->value_handle;
		}

	}

	return BT_GATT_ITER_CONTINUE;
}


//discover services and chrcs provided for a certain peripheral
static void gatt_discover(void)
{
	int err;

	printk("Discovering services and characteristics\n");

	static struct bt_gatt_discover_params discover_params;

	discover_params.uuid = primary_uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = START_HANDLE;
	discover_params.end_handle = END_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(default_conn, &discover_params);
	if (err != 0) {
		printk("Discover failed(err %d)\n", err);
	}

	WAIT_FOR_FLAG(is_discover_complete);
	printk("Discover complete\n");
}

//write in a certain chrc without response
static void write_without_response(uint16_t handle, char* chrc_data)
{
	int err;

	printk("----sending data----\n");
	
	err = bt_gatt_write_without_response(default_conn, handle, chrc_data, strlen(chrc_data), 0);
	if (err != 0) {
		printk("bt_gatt_write_without_response failed: %d\n", err);
	}

	printk("write success\n");
}


//send data to peripherical every 1second
static void start_communication(void)
{
	char *my_string = {"MY_DATA"};		//Data to send
	bool can_discover = true;
    while(true){
		
		//wait connection to start discover services and chrcs
		WAIT_FOR_FLAG(is_connected);
		if(can_discover){
			gatt_discover();
			can_discover = false;
		}
		
		//when connected send data to peripheral 
		write_without_response(chrc_handle, my_string);
		(void)k_sleep(K_MSEC(1000));
		}
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	/* connect only to devices in close proximity */
	if (rssi < -70) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%u)\n", addr_str, err);
		start_scan();
	}
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	SET_FLAG(is_connected);
	printk("Connected: %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;
	
	UNSET_FLAG(is_connected);
	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	start_scan();
	start_communication();
}