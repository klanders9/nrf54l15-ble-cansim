#include "CanSimulator.hpp"

#include <algorithm>

#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(can_sim, LOG_LEVEL_INF);

// Returns a float uniformly in [-range, +range].
static float rand_sym(float range)
{
    return (static_cast<float>(sys_rand32_get() % 1000) / 500.0f - 1.0f) * range;
}

bool CanSimulator::init()
{
    initialized_ = true;
    LOG_INF("CAN simulator ready");
    return true;
}

void CanSimulator::tick()
{
    if (!initialized_) {
        return;
    }

    ++tick_count_;

    // RPM target wanders ±40 RPM every ~2 s (20 ticks × 100 ms)
    if (tick_count_ % 20 == 0) {
        rpm_target_ = std::clamp(rpm_target_ + rand_sym(40.0f), 700.0f, 2000.0f);
    }

    // Coolant target wanders ±4°C every ~10 s (100 ticks)
    if (tick_count_ % 100 == 0) {
        coolant_target_ = std::clamp(coolant_target_ + rand_sym(4.0f), 80.0f, 95.0f);
    }

    // Low-pass track toward targets, then add per-tick noise
    rpm_ += (rpm_target_ - rpm_) * 0.08f;
    rpm_ = std::clamp(rpm_ + rand_sym(3.0f), 0.0f, 8031.0f);

    coolant_c_ += (coolant_target_ - coolant_c_) * 0.015f;
    coolant_c_ = std::clamp(coolant_c_ + rand_sym(0.3f), -40.0f, 210.0f);

    // Fuel pressure loosely coupled to RPM: 200–550 kPa
    float fuel_base = 200.0f + (rpm_ / 2000.0f) * 350.0f;
    fuel_pressure_kpa_ += (fuel_base - fuel_pressure_kpa_) * 0.12f;
    fuel_pressure_kpa_ = std::clamp(fuel_pressure_kpa_ + rand_sym(5.0f), 0.0f, 1000.0f);
}

// ─── Frame encoders ───────────────────────────────────────────────────────────

J1939Frame CanSimulator::eec1() const
{
    J1939Frame f{};
    f.pgn      = 61444;
    f.priority = 3;
    f.src_addr = 0x00;

    // Bytes 0-2: torque fields — not simulated
    f.data[0] = 0xFF;
    f.data[1] = 0xFF;
    f.data[2] = 0xFF;

    // Bytes 3-4: engine speed, uint16 LE, 0.125 rpm/bit → raw = rpm * 8
    auto raw = static_cast<uint16_t>(rpm_ * 8.0f);
    f.data[3] = static_cast<uint8_t>(raw & 0xFF);
    f.data[4] = static_cast<uint8_t>(raw >> 8);

    f.data[5] = 0xFF;
    f.data[6] = 0xFF;
    f.data[7] = 0xFF;

    return f;
}

J1939Frame CanSimulator::et1() const
{
    J1939Frame f{};
    f.pgn      = 65262;
    f.priority = 6;
    f.src_addr = 0x00;

    // Byte 0: engine coolant temp, uint8, 1°C/bit, offset -40°C → raw = temp + 40
    f.data[0] = static_cast<uint8_t>(std::clamp(coolant_c_, -40.0f, 210.0f) + 40.0f);

    // Bytes 1-7: other temp fields — not simulated
    for (int i = 1; i < 8; ++i) {
        f.data[i] = 0xFF;
    }

    return f;
}

J1939Frame CanSimulator::eflp1() const
{
    J1939Frame f{};
    f.pgn      = 65263;
    f.priority = 6;
    f.src_addr = 0x00;

    // Byte 0: fuel delivery pressure, uint8, 4 kPa/bit → raw = kPa / 4
    f.data[0] = static_cast<uint8_t>(std::clamp(fuel_pressure_kpa_, 0.0f, 1000.0f) / 4.0f);

    // Bytes 1-7: other fluid fields — not simulated
    for (int i = 1; i < 8; ++i) {
        f.data[i] = 0xFF;
    }

    return f;
}
