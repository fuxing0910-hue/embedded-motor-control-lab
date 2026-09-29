#include "Arduino.h"

namespace {
unsigned long simulated_time_ms = 0;
}

unsigned long millis() {
    return simulated_time_ms;
}

namespace motorlab {
void set_simulated_time_ms(unsigned long value) {
    simulated_time_ms = value;
}
}
