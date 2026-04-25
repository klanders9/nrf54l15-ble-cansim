#pragma once
#include <cstdint>

struct J1939Frame {
    uint32_t pgn;
    uint8_t  priority;
    uint8_t  src_addr;
    uint8_t  data[8];
};

class CanSimulator {
public:
    bool init();
    void tick();  // advance simulation one step; call every 100ms

    J1939Frame eec1()  const;  // PGN 61444 — engine RPM
    J1939Frame et1()   const;  // PGN 65262 — coolant temp
    J1939Frame eflp1() const;  // PGN 65263 — fuel delivery pressure

private:
    float    rpm_{800.0f};
    float    coolant_c_{75.0f};
    float    fuel_pressure_kpa_{300.0f};
    float    rpm_target_{1200.0f};
    float    coolant_target_{85.0f};
    uint32_t tick_count_{0};
    bool     initialized_{false};
};
