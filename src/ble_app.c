#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "data_manager.h"
#include "storage_manager.h"

LOG_MODULE_REGISTER(ble_app, LOG_LEVEL_INF);

/* 1. Define Service and Characteristic UUIDs */
static struct bt_uuid_128 hope_svc_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0));

static struct bt_uuid_128 hope_data_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static struct bt_uuid_128 char_profile_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5));

static struct bt_uuid_128 char_config_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef6));

static struct bt_uuid_128 char_info_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef7));

/* Characteristic #5: User Profile (5 Bytes) */
typedef struct __attribute__((packed)) {
    uint8_t age;          /* Years */
    uint8_t height_cm;    /* cm (0-255) */
    uint16_t weight_dg;   /* Weight in decigrams (kg * 10) e.g., 75.5kg -> 755 */
    uint8_t gender;       /* 0=Male, 1=Female, 2=Other */
} user_profile_t;

/* Characteristic #6: Device Config (4 Bytes) */
typedef struct __attribute__((packed)) {
    uint8_t op_mode;      /* 0=Continuous, 1=Periodic */
    uint16_t interval_s;  /* Sampling interval in seconds (30, 60, 300...) */
    uint8_t led_current;  /* LED brightness (0-255) */
} device_config_t;

/* Default Values */
static user_profile_t current_profile = { 0, 0, 0, 0 };
static device_config_t current_config = { 0, 0, 50 }; /* Continuous, 0s, 50mA */

static bool notify_enabled = false;

/* 2. Notification Enabled Callback */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BLE Notifications: %s", notify_enabled ? "ON" : "OFF");
}

/* --- Callback for User Profile (Char #5) --- */
static ssize_t read_profile(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_profile, sizeof(current_profile));
}

static ssize_t write_profile(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != sizeof(user_profile_t)) { return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN); }
    
    memcpy(&current_profile, buf, len);
    LOG_INF("User Updated: Age=%d, Wt=%d", current_profile.age, current_profile.weight_dg);
    
    /* TODO: Save 'current_profile' to NVS/Flash here */
    return len;
}

/* --- Callback for Device Config (Char #6) --- */
static ssize_t read_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_config, sizeof(current_config));
}

static ssize_t write_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != sizeof(device_config_t)) { return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN); }
    
    memcpy(&current_config, buf, len);
    LOG_INF("Config Updated: Mode=%d, Interval=%ds", current_config.op_mode, current_config.interval_s);
    
    /* Apply logic immediately */
    if (current_config.op_mode == 1) {
        LOG_INF("Switching to Periodic Mode (%d sec)", current_config.interval_s);
        /* Call a function in data_manager.c to set sampling rate */
    } else {
        LOG_INF("Switching to Continuous Mode");
    }

    /* TODO: Save 'current_config' to NVS/Flash here */
    return len;
}

/* 3. Define GATT Service */
BT_GATT_SERVICE_DEFINE(hope_svc,
    BT_GATT_PRIMARY_SERVICE(&hope_svc_uuid),
    
    /* 1. Vitals Data (Notify) */
    BT_GATT_CHARACTERISTIC(&hope_data_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* 5. User Profile (Read/Write) */
    BT_GATT_CHARACTERISTIC(&char_profile_uuid.uuid, 
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, 
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
                           read_profile, write_profile, NULL),

    /* 6. Device Config (Read/Write) */
    BT_GATT_CHARACTERISTIC(&char_config_uuid.uuid, 
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, 
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
                           read_config, write_config, NULL),
                           
    /* 7. Device Info (Read Only) */
    BT_GATT_CHARACTERISTIC(&char_info_uuid.uuid, 
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, 
                           BT_GATT_PERM_READ, 
                           NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* 4. Advertisement Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "Neso Monitor", 12),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                  0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

static void bt_ready(int err)
{
    if (err) { LOG_ERR("BLE init failed (err %d)", err); return; }
    
    /* Pass 'sd' (Scan Response) as 4th and 5th arguments */
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising started as 'Neso Monitor'");
}

/* 5. Main BLE Worker Thread */
void start_ble_thread(void)
{
    /* Initialize Storage */
    if (storage_init() != 0) { LOG_ERR("Storage Init Failed!"); }

    /* Enable Bluetooth */
    bt_enable(bt_ready);

    patient_batch_t live_batch;
    patient_batch_t hist_batch;

    while (1) {
        /* Wait for new data (Blocking) */
        if (data_manager_get_ble_batch(&live_batch) == 0) {
            
            if (notify_enabled) {
                /* --- CONNECTED --- */
                
                /* A. Send Live Data Immediately */
                LOG_INF("BLE: Sending LIVE batch...");
                bt_gatt_notify(NULL, &hope_svc.attrs[1], &live_batch, sizeof(live_batch));
                
                /* B. Check for stored history and send ONE old batch to catch up */
                if (storage_has_data()) {
                    if (storage_get_next_batch(&hist_batch) == 0) {
                        k_sleep(K_MSEC(100)); // Slight delay to prevent congestion
                        LOG_INF("BLE: Forwarding HISTORY batch...");
                        bt_gatt_notify(NULL, &hope_svc.attrs[1], &hist_batch, sizeof(hist_batch));
                    }
                }
            } 
            else {
                /* --- DISCONNECTED --- */
                LOG_WRN("BLE Disconnected. Saving batch to NVM...");
                storage_save_batch(&live_batch);
            }
        }
    }
}
