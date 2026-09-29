#ifndef MOTOR_LAB_HOST_ARDUINO_H
#define MOTOR_LAB_HOST_ARDUINO_H

// Minimal host substitute: the upstream PID library only needs millis().
// This is a deterministic simulation clock, not wall time or a hardware timer.
unsigned long millis();

namespace motorlab {
void set_simulated_time_ms(unsigned long value);
}

#endif
