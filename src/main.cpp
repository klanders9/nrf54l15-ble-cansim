#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensor/ImuSensor.hpp"
#include "ble/TelemetryService.hpp"
#include "can/CanSimulator.hpp"
#include "can/CanGatewayService.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static ImuSensor        imu;
static TelemetryService ble;
static CanSimulator     can_sim;
static CanGatewayService can_gw;

int main()
{
    if (!imu.init()) {
        LOG_ERR("IMU init failed — check wiring and overlay");
        return -1;
    }

    if (!ble.init(imu)) {
        LOG_ERR("BLE init failed");
        return -1;
    }

    if (!can_sim.init()) {
        LOG_ERR("CAN sim init failed");
        return -1;
    }

    if (!can_gw.init(can_sim)) {
        LOG_ERR("CAN gateway init failed");
        return -1;
    }

    k_sleep(K_FOREVER);
    return 0;
}
