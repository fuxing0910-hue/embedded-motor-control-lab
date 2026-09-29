#include "motor_sim.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <set>
#include <stdexcept>
#include <string>

namespace {
double parse_number(const std::string& value, const std::string& option) {
    std::size_t consumed = 0;
    try {
        const double result = std::stod(value, &consumed);
        if (consumed == value.size()) return result;
    } catch (const std::exception&) {
        // Convert the library parser's errors into a useful CLI message.
    }
    throw std::invalid_argument(option + " needs a complete numeric value");
}

void print_help() {
    std::cout << "Ideal first-order motor simulation using the upstream Arduino PID library.\n"
        << "Usage: motor_sim [--scenario step|load|saturation] [--duration SECONDS]\n"
        << "                 [--kp NUMBER] [--ki NUMBER] [--kd NUMBER] [--output FILE]\n"
        << "Defaults: step, 12 seconds, kp=0.08, ki=0.25, kd=0.001.\n"
        << "Gains: finite [0,1000]. Duration: [0.01,120], in 0.01 second increments.\n"
        << "Omit --output or use '-' for CSV on stdout. The parent directory must exist.\n"
        << "Simulation tick=10 ms; PID sample time=20 ms. This is not hardware data.\n";
}

void write_csv(std::ostream& stream, const motorlab::Config& config,
               const std::vector<motorlab::Sample>& samples) {
    stream.imbue(std::locale::classic());
    stream << std::setprecision(std::numeric_limits<double>::max_digits10);
    stream << "scenario,time_s,setpoint_rpm,speed_rpm,error_rpm,pwm,load_rpm,"
        << "pid_updated,kp,ki,kd,sample_time_ms\n";
    for (const auto& row : samples) {
        stream << motorlab::scenario_name(config.scenario) << ',' << row.time_ms / 1000.0
            << ',' << row.setpoint_rpm << ',' << row.speed_rpm << ',' << row.error_rpm
            << ',' << row.pwm << ',' << row.load_rpm << ',' << (row.pid_updated ? 1 : 0)
            << ',' << config.kp << ',' << config.ki << ',' << config.kd
            << ',' << motorlab::kPidSampleMs << '\n';
    }
    stream.flush();
    if (!stream) throw std::runtime_error("failed to write CSV output");
}
}

int main(int argc, char* argv[]) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            print_help();
            return 0;
        }
        motorlab::Config config;
        std::string output = "-";
        std::set<std::string> seen;
        for (int index = 1; index < argc; ++index) {
            const std::string option = argv[index];
            if (option != "--scenario" && option != "--duration" && option != "--kp" &&
                option != "--ki" && option != "--kd" && option != "--output") {
                throw std::invalid_argument("unknown option: " + option);
            }
            if (!seen.insert(option).second) {
                throw std::invalid_argument("duplicate option: " + option);
            }
            if (++index >= argc) throw std::invalid_argument("missing value for " + option);
            const std::string value = argv[index];
            if (value.empty()) throw std::invalid_argument("empty value for " + option);
            if (option == "--scenario") config.scenario = motorlab::parse_scenario(value);
            else if (option == "--duration") config.duration_s = parse_number(value, option);
            else if (option == "--kp") config.kp = parse_number(value, option);
            else if (option == "--ki") config.ki = parse_number(value, option);
            else if (option == "--kd") config.kd = parse_number(value, option);
            else output = value;
        }
        // Validate and run before opening a file, so invalid options do not truncate it.
        const auto samples = motorlab::run_simulation(config);
        if (output == "-") {
            write_csv(std::cout, config, samples);
        } else {
            std::ofstream stream(output);
            if (!stream) throw std::runtime_error("cannot open output file: " + output);
            write_csv(stream, config, samples);
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "motor_sim: " << error.what() << '\n';
        return 2;
    }
}
