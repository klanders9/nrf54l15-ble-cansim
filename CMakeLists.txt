cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nRF54L15_telemetry)

target_sources(app PRIVATE
    src/main.cpp
    src/sensor/ImuSensor.cpp
    src/ble/TelemetryService.cpp
    src/can/CanSimulator.cpp
    src/can/CanGatewayService.cpp
)

target_include_directories(app PRIVATE src)
