#include "motor_sim.h"

#include "Arduino.h"
#include "PID_v1.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motorlab {

const char* scenario_name(Scenario scenario) {
    switch (scenario) {
    case Scenario::Step: return "step";
    case Scenario::Load: return "load";
    case Scenario::Saturation: return "saturation";
    }
    throw std::invalid_argument("unknown scenario");
}

Scenario parse_scenario(const std::string& name) {
    if (name == "step") return Scenario::Step;
    if (name == "load") return Scenario::Load;
    if (name == "saturation") return Scenario::Saturation;
    throw std::invalid_argument("scenario must be step, load, or saturation");
}

void validate_config(const Config& config) {
    (void)scenario_name(config.scenario);
    const double gains[] = {config.kp, config.ki, config.kd};
    for (double gain : gains) {
        // A finite bounded teaching range also prevents arithmetic overflow.
        if (!std::isfinite(gain) || gain < 0.0 || gain > 1000.0) {
            throw std::invalid_argument("kp, ki, and kd must be finite numbers in [0, 1000]");
        }
    }
    if (!std::isfinite(config.duration_s) || config.duration_s < 0.01 ||
        config.duration_s > 120.0) {
        throw std::invalid_argument("duration must be finite and in [0.01, 120] seconds");
    }
    const double steps = config.duration_s * 1000.0 / kSimulationStepMs;
    if (std::abs(steps - std::round(steps)) > 1e-7) {
        throw std::invalid_argument("duration must be a multiple of 0.01 seconds");
    }
}

std::vector<Sample> run_simulation(const Config& config) {
    validate_config(config);
    set_simulated_time_ms(0);
    double speed = 0.0;
    double pwm = 0.0;
    double setpoint = 0.0;
    // This object is the unmodified upstream Arduino PID controller.
    PID controller(&speed, &pwm, &setpoint, config.kp, config.ki, config.kd, DIRECT);
    controller.SetOutputLimits(0.0, kMaxPwm);
    controller.SetSampleTime(static_cast<int>(kPidSampleMs));
    controller.SetMode(AUTOMATIC);

    const auto steps = static_cast<unsigned long>(
        std::llround(config.duration_s * 1000.0 / kSimulationStepMs));
    std::vector<Sample> samples;
    samples.reserve(steps + 1);
    const double decay = std::exp(-(kSimulationStepMs / 1000.0) / kTimeConstantSeconds);

    for (unsigned long step = 0; step <= steps; ++step) {
        const unsigned long time_ms = step * kSimulationStepMs;
        set_simulated_time_ms(time_ms);
        setpoint = time_ms < 500 ? 0.0 : 1500.0;
        if (config.scenario == Scenario::Saturation && time_ms >= 500 && time_ms < 4000) {
            setpoint = 4500.0; // Intentionally above the model's 3000 rpm limit.
        }
        const double load = config.scenario == Scenario::Load &&
            time_ms >= 4000 && time_ms < 8000 ? 600.0 : 0.0;

        // Call on every simulation tick; the library itself enforces 20 ms sampling.
        const bool updated = controller.Compute();
        if (!std::isfinite(pwm)) {
            throw std::runtime_error("controller produced a non-finite output");
        }
        samples.push_back({time_ms, setpoint, speed, setpoint - speed, pwm, load, updated});

        // First-order ideal motor: tau * d(speed)/dt = target_speed - speed.
        // load is an equivalent steady-state speed reduction, not measured torque.
        // Use the exact solution for constant input during each 10 ms interval.
        // PWM is a continuous 0..255 command; quantization/noise/delay are omitted.
        const double target_speed = kNoLoadMaxRpm * pwm / kMaxPwm - load;
        speed = std::max(0.0, target_speed + (speed - target_speed) * decay);
    }
    return samples;
}

}
