# MSP Latency and Throughput Tests

Three benchmarks are available, each targeting a different question:

| Test | Binary | Question |
|---|---|---|
| RTT | `msp_rtt_test` | What is the round-trip time per message? (→ measurement age) |
| Throughput | `msp_throughput_test` | What is the maximum sustained callback rate? (→ max polling Hz) |
| Legacy latency | `msp_latency_test` | _(original combined sequential benchmark — kept for reference)_ |

---

## Building

```bash
cd /path/to/msp

# Release build is required for accurate timing — Debug adds sanitizers
cmake -B build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
make -C build msp_rtt_test msp_throughput_test msp_latency_test
```

If the device is not accessible without root: `sudo chmod a+rw /dev/ttyACM0`
(or add your user to the `dialout` group).

---

## `msp_rtt_test` — Individual RTT

### What it measures

The round-trip time (RTT) for `MSP_RAW_IMU` and `MSP_MOTOR_TELEMETRY`
individually. Dividing p50 RTT by 2 gives the best estimate of **measurement
age**: how old a sensor reading is by the time the host receives it.

Requests are issued in an **interleaved loop** (IMU then MotorTelemetry,
alternating) to avoid a stale-ID artifact present in the legacy test. The
artifact occurs because `Client::sendMessage` checks the last received message
ID before blocking — if the same message type was the last one received, it
returns immediately with stale data. Alternating types prevents this.

### Running

```bash
./build/msp_rtt_test [device] [baudrate] [n_samples] [--csv]
```

| Argument | Default | Description |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Serial port of the FC |
| `baudrate` | `115200` | Baud rate |
| `n_samples` | `1000` | Timed samples per message type |
| `--csv` | _(off)_ | Write sorted samples to `msp_rtt_log.csv` |

```bash
# Defaults
./build/msp_rtt_test

# Custom port, with CSV output
./build/msp_rtt_test /dev/ttyACM0 115200 1000 --csv
```

### Output

Per-message stats table (`n`, `mean`, `min`, `p50`, `p95`, `p99`, `max`,
`stddev`, all in µs) followed by measurement age estimates:

```
=== Measurement age estimates (RTT / 2) ===
  imu_latency_offset_ms:  X.XXX
  mtel_latency_offset_ms: X.XXX
```

These values are the recommended `imu_latency_offset_ms` / `mtel_latency_offset_ms`
config entries for the platform node.

---

## `msp_throughput_test` — Async Throughput

### What it measures

The maximum sustained callback rate for both messages running concurrently via
`Client::subscribe()`. The subscription fires requests at `request_hz` —
intentionally faster than the FC can respond — so the achieved callback rate is
bandwidth-limited and directly answers "how fast can the platform node poll?"

Also reports **inter-arrival interval** stats (jitter at the achieved rate) and
a per-second histogram to check whether the rate is stable over time.

### Running

```bash
./build/msp_throughput_test [device] [baudrate] [request_hz] [duration_s] [--csv]
```

| Argument | Default | Description |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Serial port of the FC |
| `baudrate` | `115200` | Baud rate |
| `request_hz` | `500` | Subscription request fire rate (should exceed expected max) |
| `duration_s` | `10` | Test duration in seconds |
| `--csv` | _(off)_ | Write sorted inter-arrival intervals to `msp_throughput_log.csv` |

```bash
# Defaults (500 Hz request rate, 10 s)
./build/msp_throughput_test

# Custom port, longer run
./build/msp_throughput_test /dev/ttyACM0 115200 500 30 --csv
```

### Output

Live per-second callback counts during the run, then:

```
=== Achieved rates ===
  IMU:  XXX.X Hz  (NNNN callbacks)
  MTEL: XXX.X Hz  (NNNN callbacks)
```

Followed by inter-arrival interval stats for each message type and a full
per-second histogram. The **implied rate from p50** in the interval stats is
the best single-number summary of sustained throughput.

### Bandwidth ceiling

At 115200 baud the theoretical maximum for both messages running simultaneously
is approximately **115 Hz each** (based on ~100 bytes per IMU+MotorTelemetry
round trip). Higher baud rates scale this linearly. If your platform node needs
200 Hz, you need at least ~230400 baud, or you need to reduce payload size
(e.g., skip MotorTelemetry on alternate cycles).

---

## `msp_latency_test` — Legacy Sequential Benchmark

Kept for reference. Measures each message type in a separate sequential loop,
then a combined (IMU + MotorTelemetry back-to-back) loop.

> **Note:** The individual IMU and MotorTelemetry RTT values from this test
> are unreliable due to the stale-ID artifact described above. Use
> `msp_rtt_test` for individual RTTs. The **combined** measurement is
> accurate and reflects the true sequential round-trip time (~10–12 ms at
> 115200 baud).

```bash
./build/msp_latency_test [device] [baudrate] [n_samples] [--csv]
```

| Argument | Default | Description |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `115200` | Baud rate |
| `n_samples` | `1000` | Samples per benchmark |
| `--csv` | _(off)_ | Write to `msp_latency_log.csv` |
