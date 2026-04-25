#include "CanGatewayService.hpp"

#include <cstring>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(can_gw, LOG_LEVEL_INF);

// ─── UUIDs ────────────────────────────────────────────────────────────────────
// Service:  d0d1d2d3-d4d5-d6d7-d8d9-dadbdcdddedf
// EEC1:     e0e1e2e3-e4e5-e6e7-e8e9-eaebecedeeef  (PGN 61444 — engine RPM)
// ET1:      f0f1f2f3-f4f5-f6f7-f8f9-fafbfcfdfeff  (PGN 65262 — coolant temp)
// EFLP1:    10111213-1415-1617-1819-1a1b1c1d1e1f  (PGN 65263 — fuel pressure)

#define CAN_GW_SVC_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xd0d1d2d3, 0xd4d5, 0xd6d7, 0xd8d9, 0xdadbdcdddedfULL))

#define EEC1_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xe0e1e2e3, 0xe4e5, 0xe6e7, 0xe8e9, 0xeaebecedeeefULL))

#define ET1_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xf0f1f2f3, 0xf4f5, 0xf6f7, 0xf8f9, 0xfafbfcfdfeffULL))

#define EFLP1_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x10111213, 0x1415, 0x1617, 0x1819, 0x1a1b1c1d1e1fULL))

// ─── Singleton and notify-enable flags ────────────────────────────────────────
static CanSimulator *sim_ptr;

static atomic_t eec1_notify_enabled;
static atomic_t et1_notify_enabled;
static atomic_t eflp1_notify_enabled;

// Last-notified frames — compared each tick to decide whether to notify.
// Reset to zero on disconnect so the first tick of a new connection always fires.
static J1939Frame last_eec1{};
static J1939Frame last_et1{};
static J1939Frame last_eflp1{};

// ─── CCC callbacks ────────────────────────────────────────────────────────────
static void eec1_ccc_changed(const struct bt_gatt_attr *, uint16_t value)
{
    atomic_set(&eec1_notify_enabled, value == BT_GATT_CCC_NOTIFY ? 1 : 0);
    LOG_INF("EEC1 notify %s", value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled");
}

static void et1_ccc_changed(const struct bt_gatt_attr *, uint16_t value)
{
    atomic_set(&et1_notify_enabled, value == BT_GATT_CCC_NOTIFY ? 1 : 0);
    LOG_INF("ET1 notify %s", value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled");
}

static void eflp1_ccc_changed(const struct bt_gatt_attr *, uint16_t value)
{
    atomic_set(&eflp1_notify_enabled, value == BT_GATT_CCC_NOTIFY ? 1 : 0);
    LOG_INF("EFLP1 notify %s", value == BT_GATT_CCC_NOTIFY ? "enabled" : "disabled");
}

// ─── GATT service definition ──────────────────────────────────────────────────
// Attribute index layout:
//   [0] Primary service declaration
//   [1] EEC1 characteristic declaration
//   [2] EEC1 characteristic value    ← notify target
//   [3] EEC1 CCC
//   [4] ET1 characteristic declaration
//   [5] ET1 characteristic value     ← notify target
//   [6] ET1 CCC
//   [7] EFLP1 characteristic declaration
//   [8] EFLP1 characteristic value   ← notify target
//   [9] EFLP1 CCC
BT_GATT_SERVICE_DEFINE(can_gw_svc,
    BT_GATT_PRIMARY_SERVICE(CAN_GW_SVC_UUID),
    BT_GATT_CHARACTERISTIC(EEC1_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(eec1_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(ET1_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(et1_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(EFLP1_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(eflp1_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// ─── Connection callbacks ─────────────────────────────────────────────────────
static void on_disconnected(struct bt_conn *, uint8_t reason)
{
    LOG_INF("CAN GW: BLE disconnected (reason %d), clearing state", reason);
    atomic_set(&eec1_notify_enabled,  0);
    atomic_set(&et1_notify_enabled,   0);
    atomic_set(&eflp1_notify_enabled, 0);
    // Zero last frames so the first tick of the next connection notifies immediately.
    last_eec1  = {};
    last_et1   = {};
    last_eflp1 = {};
}

BT_CONN_CB_DEFINE(can_gw_conn_cbs) = {
    .disconnected = on_disconnected,
};

// ─── Payload packing ──────────────────────────────────────────────────────────
// 12 bytes: PGN (3 bytes LE) | source address (1 byte) | J1939 data (8 bytes)
static void pack_payload(uint8_t out[12], const J1939Frame &f)
{
    out[0] = static_cast<uint8_t>(f.pgn & 0xFF);
    out[1] = static_cast<uint8_t>((f.pgn >> 8) & 0xFF);
    out[2] = static_cast<uint8_t>((f.pgn >> 16) & 0xFF);
    out[3] = f.src_addr;
    memcpy(&out[4], f.data, 8);
}

// ─── Change detection ─────────────────────────────────────────────────────────
// EEC1: RPM in bytes 3-4 (uint16 LE). Threshold 200 raw = 25 RPM.
static bool eec1_changed(const J1939Frame &a, const J1939Frame &b)
{
    int32_t ra = a.data[3] | (a.data[4] << 8);
    int32_t rb = b.data[3] | (b.data[4] << 8);
    int32_t d  = ra - rb;
    return d > 200 || d < -200;
}

// ET1: coolant in byte 0, 1°C/LSB. Any LSB change triggers.
static bool et1_changed(const J1939Frame &a, const J1939Frame &b)
{
    return a.data[0] != b.data[0];
}

// EFLP1: fuel pressure in byte 0, 4 kPa/LSB. Threshold 3 LSB = 12 kPa.
static bool eflp1_changed(const J1939Frame &a, const J1939Frame &b)
{
    int32_t d = static_cast<int32_t>(a.data[0]) - static_cast<int32_t>(b.data[0]);
    return d > 3 || d < -3;
}

// ─── Timer → work queue pipeline ─────────────────────────────────────────────
static void gw_work_handler(struct k_work *work);
static void gw_timer_cb(struct k_timer *timer);
K_WORK_DEFINE(gw_work, gw_work_handler);
K_TIMER_DEFINE(gw_timer, gw_timer_cb, NULL);

static void gw_timer_cb(struct k_timer *) { k_work_submit(&gw_work); }

static void gw_work_handler(struct k_work *)
{
    if (!sim_ptr) {
        return;
    }

    sim_ptr->tick();

    J1939Frame eec1  = sim_ptr->eec1();
    J1939Frame et1   = sim_ptr->et1();
    J1939Frame eflp1 = sim_ptr->eflp1();

    uint8_t payload[12];

    if (atomic_get(&eec1_notify_enabled) && eec1_changed(eec1, last_eec1)) {
        pack_payload(payload, eec1);
        bt_gatt_notify(NULL, &can_gw_svc.attrs[2], payload, sizeof(payload));
        last_eec1 = eec1;
    }

    if (atomic_get(&et1_notify_enabled) && et1_changed(et1, last_et1)) {
        pack_payload(payload, et1);
        bt_gatt_notify(NULL, &can_gw_svc.attrs[5], payload, sizeof(payload));
        last_et1 = et1;
    }

    if (atomic_get(&eflp1_notify_enabled) && eflp1_changed(eflp1, last_eflp1)) {
        pack_payload(payload, eflp1);
        bt_gatt_notify(NULL, &can_gw_svc.attrs[8], payload, sizeof(payload));
        last_eflp1 = eflp1;
    }
}

// ─── CanGatewayService public methods ────────────────────────────────────────
bool CanGatewayService::init(CanSimulator &sim)
{
    sim_ptr = &sim;
    k_timer_start(&gw_timer, K_MSEC(100), K_MSEC(100));
    LOG_INF("CanGatewayService ready");
    return true;
}
