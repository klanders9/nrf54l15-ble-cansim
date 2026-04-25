#include "LedIndicator.hpp"

#include <dk_buttons_and_leds.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led_indicator, LOG_LEVEL_INF);

static int chase_idx;

static void chase_work_handler(struct k_work *);
static void chase_timer_cb(struct k_timer *);

K_WORK_DEFINE(chase_work, chase_work_handler);
K_TIMER_DEFINE(chase_timer, chase_timer_cb, NULL);

static void chase_timer_cb(struct k_timer *) { k_work_submit(&chase_work); }

static const int chase_order[] = {0, 1, 3, 2};

static void chase_work_handler(struct k_work *)
{
    dk_set_leds(BIT(chase_order[chase_idx]));
    chase_idx = (chase_idx + 1) % 4;
}

static void start_chase()
{
    chase_idx = 0;
    k_timer_start(&chase_timer, K_MSEC(250), K_MSEC(250));
}

static void on_connected(struct bt_conn *, uint8_t err)
{
    if (err) return;
    k_timer_stop(&chase_timer);
    dk_set_leds(DK_ALL_LEDS_MSK);
}

static void on_disconnected(struct bt_conn *, uint8_t)
{
    start_chase();
}

BT_CONN_CB_DEFINE(led_conn_cbs) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

bool LedIndicator::init()
{
    int rc = dk_leds_init();
    if (rc) {
        LOG_ERR("dk_leds_init: %d", rc);
        return false;
    }
    start_chase();
    return true;
}
