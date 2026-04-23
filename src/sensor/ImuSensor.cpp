#include "ImuSensor.hpp"

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_sensor, LOG_LEVEL_INF);

// LSM6DS3TR-C is register-compatible with the lsm6dsl driver.
// DEVICE_DT_GET_ONE resolves at link time — no dynamic lookup cost.
static const struct device *const dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

bool ImuSensor::init()
{
    if (!device_is_ready(dev)) {
        LOG_ERR("LSM6DSL not ready");
        return false;
    }
    initialized_ = true;
    LOG_INF("LSM6DSL ready");
    return true;
}

bool ImuSensor::sample()
{
    if (!initialized_) {
        return false;
    }

    int rc = sensor_sample_fetch(dev);
    if (rc != 0) {
        LOG_ERR("sensor_sample_fetch: %d", rc);
        return false;
    }

    struct sensor_value val[3];

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, val);
    data_.accel_x = static_cast<float>(sensor_value_to_double(&val[0]));
    data_.accel_y = static_cast<float>(sensor_value_to_double(&val[1]));
    data_.accel_z = static_cast<float>(sensor_value_to_double(&val[2]));

    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, val);
    data_.gyro_x = static_cast<float>(sensor_value_to_double(&val[0]));
    data_.gyro_y = static_cast<float>(sensor_value_to_double(&val[1]));
    data_.gyro_z = static_cast<float>(sensor_value_to_double(&val[2]));

    return true;
}
