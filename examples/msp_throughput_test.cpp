#include <Client.hpp>
#include <msp_msg.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

// Measures the maximum sustained callback rate for RawImu and MotorTelemetry
// running concurrently via Client::subscribe().
//
// The subscription period is set intentionally shorter than the expected RTT
// to saturate the FC. The achieved callback rate (not the request rate) is
// what matters: it reflects the real bandwidth ceiling of the serial link
// carrying both message types simultaneously.
//
// Inter-arrival interval stats show jitter at the achieved rate. Per-second
// window counts show whether the rate is stable over time.
//
// Usage: msp_throughput_test [device] [baudrate] [request_hz] [duration_s] [--csv]

namespace {

struct Stats {
    size_t count     = 0;
    double mean_us   = 0;
    double min_us    = 0;
    double p50_us    = 0;
    double p95_us    = 0;
    double p99_us    = 0;
    double max_us    = 0;
    double stddev_us = 0;
};

Stats computeStats(std::vector<double>& samples)
{
    Stats s;
    s.count = samples.size();
    if(s.count == 0) return s;

    std::sort(samples.begin(), samples.end());
    s.min_us = samples.front();
    s.max_us = samples.back();
    s.p50_us = samples[s.count * 50 / 100];
    s.p95_us = samples[s.count * 95 / 100];
    s.p99_us = samples[s.count * 99 / 100];

    const double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    s.mean_us        = sum / double(s.count);

    double sq = 0.0;
    for(const double v : samples) {
        const double d = v - s.mean_us;
        sq += d * d;
    }
    s.stddev_us = std::sqrt(sq / double(s.count));
    return s;
}

void printStats(const std::string& label, const Stats& s)
{
    std::cout << "\n--- " << label << " ---\n";
    if(s.count == 0) {
        std::cout << "  No data.\n";
        return;
    }
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  n:      " << s.count << "\n";
    std::cout << "  mean:   " << s.mean_us   << " us\n";
    std::cout << "  min:    " << s.min_us    << " us\n";
    std::cout << "  p50:    " << s.p50_us    << " us\n";
    std::cout << "  p95:    " << s.p95_us    << " us\n";
    std::cout << "  p99:    " << s.p99_us    << " us\n";
    std::cout << "  max:    " << s.max_us    << " us\n";
    std::cout << "  stddev: " << s.stddev_us << " us\n";
    if(s.p50_us > 0.0)
        std::cout << "  implied rate (from p50): "
                  << std::setprecision(1) << (1e6 / s.p50_us) << " Hz\n";
}

// Counts elements whose value falls in [t_lo, t_hi).
size_t countInWindow(const std::vector<double>& v, double t_lo, double t_hi)
{
    return size_t(std::count_if(v.begin(), v.end(),
        [t_lo, t_hi](double t) { return t >= t_lo && t < t_hi; }));
}

using Clock   = std::chrono::steady_clock;
using us_dur  = std::chrono::duration<double, std::micro>;

} // namespace

int main(int argc, char* argv[])
{
    const std::string device = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const size_t baudrate    = (argc > 2) ? std::stoul(argv[2]) : 115200;
    const double request_hz  = (argc > 3) ? std::stod(argv[3]) : 500.0;
    const double duration_s  = (argc > 4) ? std::stod(argv[4]) : 10.0;
    const bool write_csv     = (argc > 5) && std::string(argv[5]) == "--csv";

    const msp::FirmwareVariant fw = msp::FirmwareVariant::BTFL;

    std::cout << "MSP Throughput Test\n"
              << "  device:      " << device     << "\n"
              << "  baudrate:    " << baudrate   << "\n"
              << "  request_hz:  " << request_hz << " (subscription fire rate)\n"
              << "  duration:    " << duration_s << " s\n";

    msp::client::Client client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(fw);
    if(!client.start(device, baudrate)) {
        std::cerr << "Failed to connect to " << device << "\n";
        return 1;
    }

    const auto t_start = Clock::now();

    // Arrival timestamps in microseconds relative to t_start.
    // Both vectors are written from the ASIO read thread (via processOneMessage
    // → subscription.decode), so they need mutex protection.
    std::vector<double> imu_times, mtel_times;
    std::mutex imu_mtx, mtel_mtx;

    const size_t reserve = size_t(request_hz * duration_s * 1.1 + 256);
    imu_times.reserve(reserve);
    mtel_times.reserve(reserve);

    const double period = 1.0 / request_hz;

    client.subscribe<msp::msg::RawImu>(
        [&](const msp::msg::RawImu&) {
            const double t = us_dur(Clock::now() - t_start).count();
            std::lock_guard<std::mutex> lock(imu_mtx);
            imu_times.push_back(t);
        },
        period
    );

    client.subscribe<msp::msg::MotorTelemetry>(
        [&](const msp::msg::MotorTelemetry&) {
            const double t = us_dur(Clock::now() - t_start).count();
            std::lock_guard<std::mutex> lock(mtel_mtx);
            mtel_times.push_back(t);
        },
        period
    );

    // Live per-second window counts while the test runs.
    std::cout << "\n  sec | imu_hz | mtel_hz\n"
              << "  ----|--------|--------\n";

    size_t imu_prev = 0, mtel_prev = 0;
    for(int sec = 1; sec <= int(duration_s); ++sec) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        size_t imu_now, mtel_now;
        {
            std::lock_guard<std::mutex> l1(imu_mtx);
            std::lock_guard<std::mutex> l2(mtel_mtx);
            imu_now  = imu_times.size();
            mtel_now = mtel_times.size();
        }
        const size_t imu_delta  = imu_now  - imu_prev;
        const size_t mtel_delta = mtel_now - mtel_prev;
        imu_prev  = imu_now;
        mtel_prev = mtel_now;

        std::cout << "  " << std::setw(3) << sec << " | "
                  << std::fixed << std::setprecision(0)
                  << std::setw(6) << imu_delta  << " | "
                  << std::setw(7) << mtel_delta << "\n";
    }

    client.stop();

    const double elapsed_s = us_dur(Clock::now() - t_start).count() / 1e6;

    // --- Achieved rates ---
    const double imu_hz  = imu_times.size()  / elapsed_s;
    const double mtel_hz = mtel_times.size() / elapsed_s;

    std::cout << "\n=== Achieved rates ===\n"
              << std::fixed << std::setprecision(1)
              << "  IMU:  " << imu_hz  << " Hz  (" << imu_times.size()  << " callbacks)\n"
              << "  MTEL: " << mtel_hz << " Hz  (" << mtel_times.size() << " callbacks)\n";

    // --- Inter-arrival interval stats ---
    auto computeIntervals = [](const std::vector<double>& times) {
        std::vector<double> iv;
        iv.reserve(times.size());
        for(size_t i = 1; i < times.size(); ++i)
            iv.push_back(times[i] - times[i - 1]);
        return iv;
    };

    auto imu_intervals  = computeIntervals(imu_times);
    auto mtel_intervals = computeIntervals(mtel_times);

    const auto imu_iv_stats  = computeStats(imu_intervals);
    const auto mtel_iv_stats = computeStats(mtel_intervals);

    printStats("IMU inter-arrival interval",  imu_iv_stats);
    printStats("MTEL inter-arrival interval", mtel_iv_stats);

    // --- Per-second histogram (post-run, for CSV / detailed view) ---
    const int n_seconds = int(elapsed_s);
    std::cout << "\n=== Per-second rates ===\n"
              << "  sec | imu_hz | mtel_hz\n"
              << "  ----|--------|--------\n";
    for(int sec = 0; sec < n_seconds; ++sec) {
        const double t_lo = sec       * 1e6;
        const double t_hi = (sec + 1) * 1e6;
        const size_t n_imu  = countInWindow(imu_times,  t_lo, t_hi);
        const size_t n_mtel = countInWindow(mtel_times, t_lo, t_hi);
        std::cout << "  " << std::setw(3) << sec << " | "
                  << std::setw(6) << n_imu  << " | "
                  << std::setw(7) << n_mtel << "\n";
    }

    if(write_csv) {
        // Write sorted inter-arrival intervals so the CDF shape is visible.
        const std::string csv_path = "msp_throughput_log.csv";
        std::ofstream f(csv_path);
        if(f) {
            const size_t n = std::max(imu_intervals.size(), mtel_intervals.size());
            f << "index,imu_interval_us,mtel_interval_us\n";
            for(size_t i = 0; i < n; ++i) {
                f << i << ","
                  << (i < imu_intervals.size()  ? imu_intervals[i]  : 0.0) << ","
                  << (i < mtel_intervals.size() ? mtel_intervals[i] : 0.0) << "\n";
            }
            std::cout << "\nCSV written to " << csv_path << "\n";
        } else {
            std::cerr << "Could not write " << csv_path << "\n";
        }
    }

    return 0;
}
