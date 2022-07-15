#include <peripheral.h>

static ble_uart_service_rx_callback rx_callback = NULL;
#define BLE_UART_SERVICE_TX_CHAR_OFFSET    3

#define CHRC_SIZE 100
static uint8_t received_data[CHRC_SIZE];

#define CREATE_FLAG(flag) static atomic_t flag = (atomic_t)false
#define SET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)true)
#define UNSET_FLAG(flag) (void)atomic_set(&flag, (atomic_t)false)
#define WAIT_FOR_FLAG(flag) \
	while (!(bool)atomic_get(&flag)) { \
		(void)k_sleep(K_MSEC(1)); \
	}

CREATE_FLAG(flag_long_subscribe);

static struct bt_uuid_128 ble_uart_uppercase = BT_UUID_INIT_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00);

static struct bt_uuid_128 ble_uart_receive_data = BT_UUID_INIT_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03,0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00);

static struct bt_uuid_128 ble_uart_notify = BT_UUID_INIT_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03,0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x11);

static struct bt_gatt_attr attrs[] = { 
        BT_GATT_PRIMARY_SERVICE(&ble_uart_uppercase),
		BT_GATT_CHARACTERISTIC(&ble_uart_receive_data.uuid, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
													 BT_GATT_PERM_WRITE, NULL, show_received_data, NULL),
        BT_GATT_CHARACTERISTIC(&ble_uart_notify.uuid, BT_GATT_CHRC_NOTIFY,
													 BT_GATT_PERM_NONE, NULL, NULL, NULL),
        BT_GATT_CCC(ble_uart_ccc_changed, BT_GATT_PERM_WRITE),
};


static struct bt_gatt_service peripheral = BT_GATT_SERVICE(attrs);

static ssize_t show_received_data(struct bt_conn *conn,
				    const struct bt_gatt_attr *attr,
				    const void *buf, uint16_t len,
				    uint16_t offset, uint8_t flags)
{

    uint8_t string[len+1];
    for(int i = 0; i < len;i++){
        string[i] = *((char*)buf+i);
    }
    string[len] = '\0';
    printk("\nReceived data: %s\n",string);


	(void)memcpy(received_data + offset, buf, len);

    if(rx_callback) {
        rx_callback((const uint8_t *)buf,len);
    }
    
    buf = "";

	return len;
}

static void ble_uart_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value){
    const bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	if (notif_enabled) {
		SET_FLAG(flag_long_subscribe);
	}

	printk("Notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

int ble_uart_service_register(const ble_uart_service_rx_callback callback) {
    rx_callback = callback;
	return 	bt_gatt_service_register(&peripheral);
}

int ble_uart_service_transmit(const uint8_t *buffer, size_t len) {
  
    /*
	if(!buffer || !len) {
		return -1;
	}

    struct bt_conn *conn = ble_get_connection_ref();
    if(conn) {
       return ( bt_gatt_notify(conn,
                            &attrs[2],
                            string,
                            len));
    } else {
        return -1;
    }
    */
}