#include <Client.hpp>
#include <msp_msg.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {

struct Stats {
    size_t count = 0;
    size_t failures = 0;
    double mean_us = 0;
    double min_us = 0;
    double p50_us = 0;
    double p95_us = 0;
    double p99_us = 0;
    double max_us = 0;
    double stddev_us = 0;
};

Stats computeStats(std::vector<double>& samples, size_t failures = 0)
{
    Stats s;
    s.count = samples.size();
    s.failures = failures;
    if (s.count == 0) return s;

    std::sort(samples.begin(), samples.end());
    s.min_us = samples.front();
    s.max_us = samples.back();
    s.p50_us = samples[s.count * 50 / 100];
    s.p95_us = samples[s.count * 95 / 100];
    s.p99_us = samples[s.count * 99 / 100];

    const double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    s.mean_us = sum / static_cast<double>(s.count);

    double sq_sum = 0.0;
    for (const double v : samples) {
        const double d = v - s.mean_us;
        sq_sum += d * d;
    }
    s.stddev_us = std::sqrt(sq_sum / static_cast<double>(s.count));

    return s;
}

void printStats(const std::string& label, const Stats& s)
{
    std::cout << "\n--- " << label << " ---\n";
    if (s.failures > 0) {
        std::cout << "  WARNINGS: " << s.failures << " timeout(s) excluded from stats\n";
    }
    if (s.count == 0) {
        std::cout << "  No successful samples.\n";
        return;
    }
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  n:       " << s.count << "\n";
    std::cout << "  mean:    " << s.mean_us   << " us  (" << s.mean_us / 1000.0 << " ms)\n";
    std::cout << "  min:     " << s.min_us    << " us\n";
    std::cout << "  p50:     " << s.p50_us    << " us\n";
    std::cout << "  p95:     " << s.p95_us    << " us\n";
    std::cout << "  p99:     " << s.p99_us    << " us\n";
    std::cout << "  max:     " << s.max_us    << " us\n";
    std::cout << "  stddev:  " << s.stddev_us << " us\n";
    std::cout << std::setprecision(1)
              << "  max rate (from p50): " << (1e6 / s.p50_us) << " Hz\n";
}

using Clock = std::chrono::steady_clock;
using us = std::chrono::duration<double, std::micro>;

} // namespace

int main(int argc, char* argv[])
{
    const std::string device   = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const size_t baudrate      = (argc > 2) ? std::stoul(argv[2]) : 115200;
    const int n_samples        = (argc > 3) ? std::stoi(argv[3]) : 1000;
    const bool write_csv       = (argc > 4) && std::string(argv[4]) == "--csv";

    const int n_warmup    = 50;
    const double timeout_s = 0.5;
    const msp::FirmwareVariant fw = msp::FirmwareVariant::BETAFLIGHT;

    std::cout << "MSP Latency Test\n"
              << "  device:   " << device    << "\n"
              << "  baudrate: " << baudrate  << "\n"
              << "  samples:  " << n_samples << " (+warmup " << n_warmup << ")\n";

    msp::client::Client client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(fw);
    if (!client.start(device, baudrate)) {
        std::cerr << "Failed to connect to " << device << "\n";
        return 1;
    }

    std::vector<double> imu_samples, mtel_samples, combined_samples;
    imu_samples.reserve(n_samples);
    mtel_samples.reserve(n_samples);
    combined_samples.reserve(n_samples);
    size_t imu_failures = 0, mtel_failures = 0;

    // ----- MSP_RAW_IMU -----
    std::cout << "\nRunning MSP_RAW_IMU...";
    std::cout.flush();
    for (int i = 0; i < n_warmup + n_samples; ++i) {
        msp::msg::RawImu msg(fw);
        auto t0 = Clock::now();
        bool ok = client.sendMessage(msg, timeout_s);
        auto t1 = Clock::now();
        if (i < n_warmup) continue;
        if (ok) {
            imu_samples.push_back(us(t1 - t0).count());
        } else {
            ++imu_failures;
        }
    }
    std::cout << " done\n";

    // ----- MSP_MOTOR_TELEMETRY -----
    std::cout << "Running MSP_MOTOR_TELEMETRY...";
    std::cout.flush();
    for (int i = 0; i < n_warmup + n_samples; ++i) {
        msp::msg::MotorTelemetry msg(fw);
        auto t0 = Clock::now();
        bool ok = client.sendMessage(msg, timeout_s);
        auto t1 = Clock::now();
        if (i < n_warmup) continue;
        if (ok) {
            mtel_samples.push_back(us(t1 - t0).count());
        } else {
            ++mtel_failures;
        }
    }
    std::cout << " done\n";

    // ----- Combined (IMU then MotorTelemetry), simulates one control-cycle poll -----
    std::cout << "Running Combined (IMU + MotorTelemetry)...";
    std::cout.flush();
    for (int i = 0; i < n_warmup + n_samples; ++i) {
        msp::msg::RawImu      imu_msg(fw);
        msp::msg::MotorTelemetry mtel_msg(fw);
        auto t0 = Clock::now();
        client.sendMessage(imu_msg,  timeout_s);
        client.sendMessage(mtel_msg, timeout_s);
        auto t1 = Clock::now();
        if (i < n_warmup) continue;
        combined_samples.push_back(us(t1 - t0).count());
    }
    std::cout << " done\n";

    client.stop();

    // ----- Print stats -----
    const auto imu_stats      = computeStats(imu_samples,      imu_failures);
    const auto mtel_stats     = computeStats(mtel_samples,     mtel_failures);
    const auto combined_stats = computeStats(combined_samples);

    printStats("MSP_RAW_IMU",               imu_stats);
    printStats("MSP_MOTOR_TELEMETRY",        mtel_stats);
    printStats("Combined (IMU+MotorTelemetry)", combined_stats);

    if (combined_stats.count > 0) {
        std::cout << "\n=== Recommended config value ===\n"
                  << std::fixed << std::setprecision(3)
                  << "  imu_latency_offset_ms: "
                  << (combined_stats.p50_us / 2000.0)
                  << "  # p50 RTT / 2 = best estimate of measurement age\n";
    }

    // ----- Optional CSV -----
    if (write_csv) {
        const std::string csv_path = "msp_latency_log.csv";
        std::ofstream f(csv_path);
        if (f) {
            f << "index,imu_us,mtel_us,combined_us\n";
            for (int i = 0; i < n_samples; ++i) {
                f << i << ","
                  << (i < static_cast<int>(imu_samples.size())      ? imu_samples[i]      : 0.0) << ","
                  << (i < static_cast<int>(mtel_samples.size())     ? mtel_samples[i]     : 0.0) << ","
                  << (i < static_cast<int>(combined_samples.size()) ? combined_samples[i] : 0.0) << "\n";
            }
            std::cout << "\nCSV written to " << csv_path << "\n";
        } else {
            std::cerr << "Could not write " << csv_path << "\n";
        }
    }

    return 0;
}
