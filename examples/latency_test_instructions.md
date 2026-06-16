# MSP Latency Test

## Building

```bash
cd /home/keikei/indi_experiment/msp

# Configure (Release for accurate timing — Debug adds sanitizers)
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release

# Build just the latency test
cmake --build build --target msp_latency_test
```

## Running

```bash
./build/msp_latency_test <device> <baudrate> <n_samples> [--csv]
```

**Arguments** (all optional, defaults shown):

| Argument | Default | Description |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Serial port of the FC |
| `baudrate` | `115200` | Baud rate |
| `n_samples` | `1000` | Number of timed samples per message type |
| `--csv` | _(off)_ | Write raw timings to `msp_latency_log.csv` |

**Examples:**

```bash
# Defaults (1000 samples, /dev/ttyUSB0, 115200 baud)
./build/msp_latency_test

# Custom port and baud rate
./build/msp_latency_test /dev/ttyACM0 115200

# More samples + save raw data
./build/msp_latency_test /dev/ttyACM0 115200 2000 --csv
```

If the device is not accessible without root: `sudo chmod a+rw /dev/ttyACM0` (or add your user to the `dialout` group).

## What it measures

The test runs three benchmarks sequentially, each preceded by 50 warm-up iterations that are excluded from stats:

1. **`MSP_RAW_IMU`** — single round-trip latency
2. **`MSP_MOTOR_TELEMETRY`** — single round-trip latency
3. **Combined** — both messages back-to-back, simulating one full control-cycle poll

For each it prints `n`, `mean`, `min`, `p50`, `p95`, `p99`, `max`, `stddev` (all in µs), and an estimated max polling rate from the p50. At the end it prints a suggested `imu_latency_offset_ms` config value (p50 combined RTT / 2).
