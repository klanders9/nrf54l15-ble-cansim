#include "TelemetryService.hpp"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(ble_telemetry, LOG_LEVEL_INF);

// ─── UUIDs ────────────────────────────────────────────────────────────────
// Service:  a0a1a2a3-a4a5-a6a7-a8a9-aaabacadaeaf
// Accel:    b0b1b2b3-b4b5-b6b7-b8b9-babbbcbdbebf
// Gyro:     c0c1c2c3-c4c5-c6c7-c8c9-cacbcccdcecf

#define TELEMETRY_SVC_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xa0a1a2a3, 0xa4a5, 0xa6a7, 0xa8a9, 0xaaabacadaeafULL))

#define ACCEL_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xb0b1b2b3, 0xb4b5, 0xb6b7, 0xb8b9, 0xbabbbcbdbebfULL))

#define GYRO_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xc0c1c2c3, 0xc4c5, 0xc6c7, 0xc8c9, 0xcacbcccdcecfULL))

// ─── Singleton pointers — used by static C callbacks ─────────────────────
static TelemetryService *instance;
static ImuSensor        *imu_ptr;

// ─── Notification enable flags — written from BLE RX thread, read from work queue ──
static atomic_t accel_notify_enabled;
static atomic_t gyro_notify_enabled;

// ─── CCC callbacks ────────────────────────────────────────────────────────
static void accel_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    atomic_set(&accel_notify_enabled, value == BT_GATT_CCC_NOTIFY ? 1 : 0);
    LOG_INF("Accel notify %s", value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled");
}

static void gyro_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    atomic_set(&gyro_notify_enabled, value == BT_GATT_CCC_NOTIFY ? 1 : 0);
    LOG_INF("Gyro notify %s", value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled");
}

// ─── GATT service definition ──────────────────────────────────────────────
// Attribute index layout:
//   [0] Primary service declaration
//   [1] Accel characteristic declaration
//   [2] Accel characteristic value   ← notify target
//   [3] Accel CCC
//   [4] Gyro characteristic declaration
//   [5] Gyro characteristic value    ← notify target
//   [6] Gyro CCC
BT_GATT_SERVICE_DEFINE(telemetry_svc,
    BT_GATT_PRIMARY_SERVICE(TELEMETRY_SVC_UUID),
    BT_GATT_CHARACTERISTIC(ACCEL_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(accel_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(GYRO_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(gyro_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// ─── Advertising ──────────────────────────────────────────────────────────
// ad[]: flags + service UUID go in the primary advertising payload.
// sd[]: device name goes in the scan response (Zephyr 4.x dropped BT_LE_ADV_OPT_USE_NAME).
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        BT_UUID_128_ENCODE(0xa0a1a2a3, 0xa4a5, 0xa6a7, 0xa8a9, 0xaaabacadaeafULL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void start_advertising()
{
    // BT_LE_ADV_CONN_FAST_2 = BT_LE_ADV_OPT_CONN, 100-150ms interval.
    int rc = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
                             ad, ARRAY_SIZE(ad),
                             sd, ARRAY_SIZE(sd));
    if (rc) {
        LOG_ERR("bt_le_adv_start: %d", rc);
    } else {
        LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
    }
}

// ─── Connection callbacks ─────────────────────────────────────────────────
static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed: %d", err);
        return;
    }
    LOG_INF("BLE connected");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE disconnected (reason %d)", reason);
    atomic_set(&accel_notify_enabled, 0);
    atomic_set(&gyro_notify_enabled, 0);
    // Do NOT call start_advertising() here — the conn object isn't recycled yet.
    // bt_le_adv_start() called from on_disconnected races the stack's cleanup and
    // returns an error silently. Use the recycled callback instead.
}

static void on_recycled(void)
{
    start_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
    .recycled     = on_recycled,
};

// ─── Timer → work queue pipeline ─────────────────────────────────────────
static void notify_work_handler(struct k_work *work);
static void notify_timer_cb(struct k_timer *timer);

K_WORK_DEFINE(notify_work, notify_work_handler);
K_TIMER_DEFINE(notify_timer, notify_timer_cb, NULL);

static void notify_timer_cb(struct k_timer *timer)
{
    k_work_submit(&notify_work);
}

static void notify_work_handler(struct k_work *work)
{
    if (!instance || !imu_ptr) {
        return;
    }
    if (imu_ptr->sample()) {
        instance->notify(imu_ptr->data());
    }
}

// ─── TelemetryService public methods ─────────────────────────────────────
bool TelemetryService::init(ImuSensor &imu)
{
    instance = this;
    imu_ptr  = &imu;

    int rc = bt_enable(NULL);
    if (rc) {
        LOG_ERR("bt_enable: %d", rc);
        return false;
    }

    start_advertising();
    k_timer_start(&notify_timer, K_MSEC(20), K_MSEC(20));

    LOG_INF("TelemetryService ready");
    return true;
}

void TelemetryService::notify(const ImuData &d)
{
    if (atomic_get(&accel_notify_enabled)) {
        int16_t buf[3] = {
            static_cast<int16_t>(d.accel_x * 100.0f),
            static_cast<int16_t>(d.accel_y * 100.0f),
            static_cast<int16_t>(d.accel_z * 100.0f),
        };
        bt_gatt_notify(NULL, &telemetry_svc.attrs[2], buf, sizeof(buf));
    }

    if (atomic_get(&gyro_notify_enabled)) {
        int16_t buf[3] = {
            static_cast<int16_t>(d.gyro_x * 100.0f),
            static_cast<int16_t>(d.gyro_y * 100.0f),
            static_cast<int16_t>(d.gyro_z * 100.0f),
        };
        bt_gatt_notify(NULL, &telemetry_svc.attrs[5], buf, sizeof(buf));
    }
}
