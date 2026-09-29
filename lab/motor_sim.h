#ifndef MOTOR_LAB_MOTOR_SIM_H
#define MOTOR_LAB_MOTOR_SIM_H

#include <string>
#include <vector>

namespace motorlab {

constexpr unsigned long kSimulationStepMs = 10;
constexpr unsigned long kPidSampleMs = 20;
constexpr double kMaxPwm = 255.0;
constexpr double kNoLoadMaxRpm = 3000.0;
constexpr double kTimeConstantSeconds = 0.35;

enum class Scenario { Step, Load, Saturation };

struct Config {
    Scenario scenario = Scenario::Step;
    double duration_s = 12.0;
    double kp = 0.08;
    double ki = 0.25;
    double kd = 0.001;
};

struct Sample {
    unsigned long time_ms;
    double setpoint_rpm;
    double speed_rpm;
    double error_rpm;
    double pwm;
    double load_rpm;
    bool pid_updated;
};

const char* scenario_name(Scenario scenario);
Scenario parse_scenario(const std::string& name);
void validate_config(const Config& config);

// One run owns the fake clock. Runs may be repeated sequentially, not concurrently.
// Rows contain the current speed and the output to hold over the next 10 ms.
std::vector<Sample> run_simulation(const Config& config);

}
#endif
