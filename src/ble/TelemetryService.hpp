#pragma once

#include "sensor/ImuSensor.hpp"

class TelemetryService {
public:
    bool init(ImuSensor &imu);
    void notify(const ImuData &data);
};
