#pragma once

struct ImuData {
    float accel_x, accel_y, accel_z;  // m/s²
    float gyro_x,  gyro_y,  gyro_z;   // rad/s
};

class ImuSensor {
public:
    bool init();
    bool sample();
    ImuData data() const { return data_; }

private:
    ImuData data_{};
    bool initialized_{false};
};
