#include "motor_sim.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

double max_error(const std::vector<motorlab::Sample>& rows,
                 unsigned long from_ms, unsigned long until_ms) {
    double result = 0.0;
    bool found = false;
    for (const auto& row : rows) {
        if (row.time_ms >= from_ms && row.time_ms < until_ms) {
            result = std::max(result, std::abs(row.error_rpm));
            found = true;
        }
    }
    require(found, "test window has no samples");
    return result;
}

void sampling_and_hold() {
    motorlab::Config config;
    config.duration_s = 1.0;
    const auto rows = motorlab::run_simulation(config);
    require(rows.size() == 101, "1 second must include 101 samples, including both endpoints");
    for (std::size_t index = 0; index < rows.size(); ++index) {
        const auto& row = rows[index];
        require(row.time_ms == index * 10, "simulation tick must be 10 ms");
        require(row.pid_updated == (row.time_ms % 20 == 0), "PID must update only every 20 ms");
        if (!row.pid_updated) {
            require(row.pwm == rows[index - 1].pwm, "output must hold between PID updates");
        }
    }
}

void step_tracking_and_limits() {
    const auto rows = motorlab::run_simulation(motorlab::Config{});
    double peak = 0.0;
    for (const auto& row : rows) {
        require(std::isfinite(row.speed_rpm) && std::isfinite(row.pwm), "states must remain finite");
        require(row.pwm >= 0.0 && row.pwm <= 255.0, "PWM must stay in [0,255]");
        require(row.speed_rpm >= 0.0 && row.speed_rpm <= 3000.0, "model speed must remain physical");
        if (row.time_ms < 500) require(row.speed_rpm == 0.0, "motor must remain stopped before the step");
        peak = std::max(peak, row.speed_rpm);
    }
    require(peak < 1575.0, "default step overshoot must remain below 5 percent");
    require(max_error(rows, 3000, 12001) < 15.0, "step must settle within 1 percent by 3 seconds");
    require(max_error(rows, 10000, 12001) < 1.0, "final step error must be below 1 rpm");
}

void load_rejection() {
    motorlab::Config config;
    config.scenario = motorlab::Scenario::Load;
    const auto rows = motorlab::run_simulation(config);
    double dip_error = 0.0;
    double loaded_pwm = 0.0;
    for (const auto& row : rows) {
        require(row.pwm >= 0.0 && row.pwm <= 255.0, "load scenario PWM must remain bounded");
        const double expected_load = row.time_ms >= 4000 && row.time_ms < 8000 ? 600.0 : 0.0;
        require(row.load_rpm == expected_load, "load boundaries must be applied at 4 and 8 seconds");
        if (row.time_ms > 4000 && row.time_ms < 5000) dip_error = std::max(dip_error, row.error_rpm);
        if (row.time_ms == 7900) loaded_pwm = row.pwm;
    }
    require(dip_error > 50.0, "load must cause a visible speed dip before compensation");
    require(max_error(rows, 7000, 8000) < 3.0, "controller must recover while load is still present");
    require(loaded_pwm > 170.0 && loaded_pwm < 185.0, "compensating PWM must match the loaded equilibrium");
    require(max_error(rows, 11000, 12001) < 3.0, "controller must recover after the load is removed");
}

void saturation_and_recovery() {
    motorlab::Config config;
    config.scenario = motorlab::Scenario::Saturation;
    const auto rows = motorlab::run_simulation(config);
    for (const auto& row : rows) {
        require(row.pwm >= 0.0 && row.pwm <= 255.0, "saturation scenario PWM must remain bounded");
        require(row.speed_rpm >= 0.0 && row.speed_rpm <= 3000.0, "unreachable request must not bypass motor limit");
        if (row.time_ms >= 3000 && row.time_ms < 4000) {
            require(row.pwm == 255.0, "unreachable request must saturate PWM");
            require(row.speed_rpm > 2950.0 && row.error_rpm >= 1500.0,
                    "unreachable target must leave persistent speed error");
        }
    }
    require(max_error(rows, 7000, 12001) < 15.0,
            "after setpoint returns to 1500 rpm, recovery must be within 1 percent by 7 seconds");
    require(max_error(rows, 10000, 12001) < 1.0, "final post-saturation error must be below 1 rpm");
}

void tuning_changes_behavior() {
    motorlab::Config config;
    config.kp = config.ki = config.kd = 0.0;
    const auto rows = motorlab::run_simulation(config);
    require(rows.back().setpoint_rpm == 1500.0, "test needs a nonzero demand");
    for (const auto& row : rows) {
        require(row.pwm == 0.0 && row.speed_rpm == 0.0, "zero gains must produce no control effort");
    }
}

void both_output_limits() {
    motorlab::Config config;
    config.kp = 10.0; // Deliberately poor tuning makes the command hit both rails.
    config.ki = config.kd = 0.0;
    const auto rows = motorlab::run_simulation(config);
    bool hit_upper = false;
    bool hit_lower_after_step = false;
    for (const auto& row : rows) {
        require(row.pwm >= 0.0 && row.pwm <= 255.0, "aggressive controller must still obey limits");
        if (row.time_ms >= 1000) {
            hit_upper = hit_upper || row.pwm == 255.0;
            hit_lower_after_step = hit_lower_after_step || row.pwm == 0.0;
        }
    }
    require(hit_upper && hit_lower_after_step, "test must exercise upper and lower clipping under demand");
}

void invalid_parameters() {
    const auto rejects = [](const motorlab::Config& config) {
        try {
            (void)motorlab::run_simulation(config);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    const double invalid_gains[] = {-1.0, 1001.0, std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::quiet_NaN()};
    for (double gain : invalid_gains) {
        motorlab::Config config;
        config.kp = gain;
        require(rejects(config), "invalid kp must be rejected");
        config = motorlab::Config{};
        config.ki = gain;
        require(rejects(config), "invalid ki must be rejected");
        config = motorlab::Config{};
        config.kd = gain;
        require(rejects(config), "invalid kd must be rejected");
    }
    const double invalid_durations[] = {0.0, -1.0, 120.01, 1.005,
                                       std::numeric_limits<double>::infinity(),
                                       std::numeric_limits<double>::quiet_NaN()};
    for (double duration : invalid_durations) {
        motorlab::Config config;
        config.duration_s = duration;
        require(rejects(config), "invalid duration must be rejected");
    }
    motorlab::Config config;
    config.scenario = static_cast<motorlab::Scenario>(99);
    require(rejects(config), "invalid scenario must be rejected");
}
}

int main() {
    const std::pair<const char*, std::function<void()>> tests[] = {
        {"20 ms PID sampling and output hold", sampling_and_hold},
        {"step tracking and actuator limits", step_tracking_and_limits},
        {"load disturbance and recovery", load_rejection},
        {"unreachable setpoint and recovery", saturation_and_recovery},
        {"gain selection changes control", tuning_changes_behavior},
        {"upper and lower output clipping", both_output_limits},
        {"invalid parameters rejected", invalid_parameters}
    };
    int failures = 0;
    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "PASS: " << test.first << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
        }
    }
    constexpr int test_count = sizeof(tests) / sizeof(tests[0]);
    std::cout << (test_count - failures) << '/' << test_count
              << " behavior tests passed (ideal software model only).\n";
    return failures == 0 ? 0 : 1;
}
