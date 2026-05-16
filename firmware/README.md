# Drone Firmware

Custom flight-controller firmware for a micro-class quadcopter on a TI **MSPM0G3507** (Cortex-M0+, MCLK 16 MHz). A from-scratch, stripped-down Betaflight port. v1 scope is intentionally minimal:

- Gyro PID — *not yet ported*
- **DShot300 ESC output** — driver complete, bring-up in progress
- CRSF radio input — *not yet ported*
- **IMU sampling (ICM-42688-P)** — working

Deliberately out of scope for v1: USB, OSD, blackbox logging, GPS.

Built in **Code Composer Studio**. See `.cproject` / `.ccsproject` for the IDE config.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | MSPM0G3507, Cortex-M0+, 16 MHz MCLK |
| ESC | AERO SELFIE 60A 4-in-1, 2S–6S, advertises PWM / DShot / OneShot. Firmware family (BLHeli_S / Bluejay / AM32) unconfirmed. No USB config port. |
| Battery | 2S during bring-up |
| Motors | HGLRC Specter 1202.5, 11000 KV (micro-whoop class). Heavily under-sized for the 60 A ESC. |
| IMU | InvenSense ICM-42688-P on SPI1 |

### Pin map

| Function | Pin | PINCM | Mux | Notes |
|---|---|---|---|---|
| Motor 1 | PB17 | 43 | PF=5 (TIMA1_CCP0) | DShot/PWM |
| Motor 2 | PB18 | 44 | PF=5 (TIMA1_CCP1) | DShot/PWM |
| Motor 3 | PB19 | 45 | PF=4 (TIMG8_CCP1) | DShot/PWM |
| Motor 4 | PB20 | 48 | PF=7 (TIMA0_CCP1) | DShot/PWM |
| IMU CS  | PB6  | 23 | PF=3, driven as GPIO | manual CS |
| IMU PICO| PB8  | 25 | PF=3 (SPI1_PICO) | |
| IMU POCI| PB14 | 31 | PF=3 (SPI1_POCI) | |
| IMU SCLK| PB16 | 33 | PF=3 (SPI1_SCLK) | |

INT1 / INT2 on the IMU are not wired, so the IMU is polled from the scheduler tick — no data-ready IRQ.

---

## File layout

### Drivers

| File | Purpose |
|---|---|
| `dshot.c` / `dshot.h` | **DShot300, 4 motors, DMA-driven timer PWM.** TIMA1 covers M1+M2 (CC0+CC1), TIMG8 covers M3 (CC1), TIMA0 covers M4 (CC1). One DMA channel per motor, all triggered off TIMA1's zero-event through the event fabric. Frame buffer is 17 halfwords (16 bit-CCs + 1 trailing QUIET). |
| `pwm.c` / `pwm.h` | 50 Hz servo PWM fallback driver (1000–2000 µs). Same pin assignment as DShot, no DMA, no event fabric. *Abandoned for bring-up but kept compilable.* |
| `imu.c` / `imu.h` | ICM-42688-P driver on SPI1. Soft-reset → WHO_AM_I=0x47 → enable gyro+accel low-noise → set ODR. Polled 14-byte burst reads at 1 kHz. |
| `delay.c` / `delay.h` | `delay_cycles()` busy-wait. TI BSD-licensed reproduction; the rest of the project is © 2026 Caleb Kemere. |

### Test mains

`dshot_test.c` and `pwm_test.c` both define `main()`. **Exactly one must be excluded from the CCS build at any time** (right-click the file → *Exclude from Build*). Default for current bring-up is to include `dshot_test.c` and exclude `pwm_test.c`.

| File | What it does |
|---|---|
| `dshot_test.c` | Bring-up main: 10 kHz tick, 1 kHz DShot frame, staged motor diagnostic (arm → per-motor beep → per-motor spin ramp → all-spin), plus background IMU poll. Also contains a `PIN_WALK_TEST` macro that bypasses everything and exercises PB17–20 as plain GPIOs with SWD-readable pad readback — a hardware sanity test that needs no scope. |
| `pwm_test.c` | Standalone fallback test using `pwm.c`. Drives 50 Hz servo PWM through arm → per-motor sweep → all-spin. Used to split the "motors won't spin" question into MCU-vs-ESC. Currently abandoned. |

### Build glue (not authored)

| File | Source |
|---|---|
| `startup_mspm0g350x_ticlang.c` | TI SDK vector table / startup |
| `mspm0g3507.cmd` | TI linker command file |
| `targetConfigs/` | CCS XDS target config |
| `.cproject` / `.ccsproject` / `.project` / `.settings/` | CCS project metadata |

---

## DShot driver in one diagram

```
                 +---------+
                 |  TIMA1  |  --pub--> event-fabric ch (EVT_FABRIC_CH)
                 | period  |
                 | 53 ticks|
                 +---------+
                  |       |              +-----+   +-----+   +-----+   +-----+
                  | LACT  | CDACT        | DMA |   | DMA |   | DMA |   | DMA |
                  | (HIGH)| (LOW @ CC)   |  M1 |   |  M2 |   |  M3 |   |  M4 |
                  v       v              +-----+   +-----+   +-----+   +-----+
                  +----+----+               |         |         |         |
                  | CCP0    |   <-----------+         |         |         |
                  | CCP1    |   <---------------------+         |         |
                  +----+----+                                   |         |
        TIMG8 (CC1 only)    <------------------------------------+        |
        TIMA0 (CC1 only)    <---------------------------------------------+
            ^                                                              ^
            |  s_frame[m][0..16]:  16 bit-CCs (BIT0=33, BIT1=13) + QUIET=53
            |  DMA src auto-advances, dst fixed at the timer's CC register
            v
   PB17/18/19/20  --IOMUX alt-func--> ESC inputs   (between frames: GPIO PF=1 driving LOW)
```

- **Bit period**: 53 ticks × 62.5 ns = 3.3125 µs ≈ DShot300's 3.33 µs.
- **BIT1 HIGH**: 39 ticks ≈ 2.44 µs. **BIT0 HIGH**: 19 ticks ≈ 1.19 µs.
- **CCUPD = IMMEDIATELY** (not shadow-on-Z). The DMA arbitration latency is much smaller than the slack to the earliest CDACT (39 ticks for "1"), so the new CC value lands well before it's needed.
- **CCPD = OUTPUT** for the CC channels we use (set in `dshot_init_one_timer`). Without this the compare engine runs internally but the pad stays disconnected — see *Known bugs / fixes* below.
- **Pin parking**: between frames the four motor pins are switched (PINCM) from timer alt-function to GPIO PF=1 driving LOW. `dshot_transmit()` self-parks at the end of every frame so the line ends with a clean falling edge.

---

## Scheduler

`dshot_test.c` runs TIMG0 at 10 kHz. Frame layout per ms:

| Tick offset (× 100 µs) | Action |
|---|---|
| 0 | `diag_stage()` then `dshot_transmit()` — blocks ~56 µs for the frame |
| 1 | `dshot_park()` (redundant safety park) |
| 5 | Set `imu_poll_due = true`; main loop runs `imu_read_sample()` in thread context |
| 2–4, 6–9 | idle |

So DShot frame rate is 1 kHz, IMU sample rate is 1 kHz, and both share a single 10 kHz tick without re-entering the ISR.

---

## SWD-readable diagnostics

All `volatile`, all readable from a CCS Expressions watch without instrumentation.

### From `dshot.c`

| Variable | Healthy value |
|---|---|
| `dshot_frames_started` | climbs at ~1000/s |
| `dshot_frames_skipped` | stays 0 |
| `dshot_align_locked`   | climbs at ~1000/s (every two-stage Z sync) |
| `dshot_align_misses`   | stays 0 (would only fire if TIMA1 stops emitting Z) |
| `dshot_dbg_tima1_ccpd` | `0x3` (CC0 + CC1 OUTPUT) |
| `dshot_dbg_timg8_ccpd` | `0x2` (CC1 OUTPUT) |
| `dshot_dbg_tima0_ccpd` | `0x2` (CC1 OUTPUT) |
| `dshot_dbg_active_cc_m{1..4}` | last CC value written by DMA |
| `dshot_dbg_dmasz_after_tx[]` | all 0 (previous frame complete) |
| `dshot_dbg_dmasa_after_tx[]` | one past end of `s_frame[m]` |
| `dshot_dbg_tima1_ctr` / `_timg8_ctr` / `_tima0_ctr` | non-zero, changing |
| `dshot_dbg_tima1_ctrctl` | `EN` bit set |
| `dshot_dbg_tima1_fpub0` | `EVT_FABRIC_CH` (= 1) |
| `dshot_dbg_dma_fsub0`   | `EVT_FABRIC_CH` (= 1) |
| `dshot_dbg_dmach0_ctl`  | `DMAEN` set |
| `dshot_dbg_dmach0_tctl` | TSEL = external |

### From `dshot_test.c`

| Variable | Meaning |
|---|---|
| `tick_count_10khz` | scheduler tick, climbs by 10000/s |
| `boot_seconds` | seconds since boot |
| `diag_phase` | 0=arm, 1=beep, 2=spin-sweep, 3=all-spin |
| `diag_active_motor` | motor index being exercised, `0xFF` = all/none |
| `pinwalk_*` | results of the `PIN_WALK_TEST` GPIO pad check |

### From `imu.c`

| Variable | Healthy value |
|---|---|
| `imu_ready` | `true` after init |
| `imu_who_am_i_seen` | `0x47` |
| `imu_init_status` | `IMU_STATUS_OK` (3) |
| `imu_sample_count` | climbs at ~1000/s |
| `imu_read_failures` | stays 0 |
| `imu_latest_sample` | live IMU readings |

---

## DShot bring-up sequence

With `dshot_test.c` as the active main, after flash + reset the firmware automatically walks through:

| Window | Phase | What you should see |
|---|---|---|
| 0–3 s | **ARM** | `MOTOR_STOP` on all four. ESC plays its arming tune. |
| 3–5.4 s | **BEEP SWEEP** | DShot `BEEP3` on M1 → M2 → M3 → M4 in 600 ms slots. One audible beep per motor. A silent channel = no signal reaching that ESC input. |
| 5.4–15.4 s | **SPIN SWEEP** | Each motor in turn ramps 48 → 600 (≈ 28 % throttle) over 2 s. Only the active motor should spin. Nails down which PB pin maps to which physical motor. |
| 15.4 s+ | **ALL SPIN** | All four held at 600. |

Interpreting results:

- Beeps on all 4, spins on all 4 → driver fine.
- Beeps on all 4, spins on none → signal+CRC good; problem is ESC config or motors.
- Beeps on some, silent on others → those channels have no signal — wiring or PINCM/PF mismatch.

If nothing happens at all (just the welcome chime), check `dshot_dbg_*_ccpd` first — that's the bug that historically blocked every frame.

---

## Pin-pad self-test (`PIN_WALK_TEST`)

Set `#define PIN_WALK_TEST 1` in `dshot_test.c` to skip DShot entirely and instead exercise each of PB17–PB20 as a plain GPIO. The test drives each pin HIGH and LOW in turn, and **reads the pad voltage back through the input buffer (PINCM.INENA = 1)** — so the result is visible purely over SWD without any probing of the QFN package.

Watch `pinwalk_pass`, `pinwalk_high_rb[0..3]`, `pinwalk_low_rb[0..3]`, `pinwalk_ok[0..3]`. Healthy: `ok[i] == 1` for all four. Caveats:

- `high_rb == 0` → pad can't reach HIGH (cold-joint or net shorted to GND).
- `low_rb == 1` → pad stuck HIGH (net shorted to 3V3).
- An open trace *downstream* of the pad still reads `ok == 1`; the readback only proves the MCU pad itself works.

This test runs in GPIO PF=1 mode and **does not** exercise the timer alt-function path — keep that in mind when reasoning about DShot failures.

---

## Known bugs and recent fixes

### CCPD must be OUTPUT (fixed 2026-05-15)

Every GPTimer has a per-pin direction register `COMMONREGS.CCPD` whose reset value is **0 = INPUT** for every CCPx. Without explicitly setting it to OUTPUT, the timer's compare engine runs but the pad multiplexer's output driver stays disconnected — `CCCTL=COMPARE`, `CCACT=…`, `OCTL=FUNCVAL` and `PINCM=alt-func` all together aren't enough on their own. Symptoms: no waveform ever reaches the ESC, all four motors silent, occasional twitch from noise pickup on a floating line, BLHeli plays its lost-signal chime forever. Fixed in `dshot_init_one_timer()`; verify via `dshot_dbg_*_ccpd`.

### Trailing-HIGH (fixed 2026-05-15)

Original `dshot_transmit()` left the line HIGH for ~43 µs after every frame (the timer's post-bit-0 QUIET period). Some BLHeli_S decoders won't lock on if idle isn't actually idle. `dshot_transmit()` now blocks ~56 µs for DMA completion then parks pins LOW; frames end with a clean falling edge.

### DMA-enable / Z-event race on bit 15 (fixed 2026-05-15)

Old order was `enable DMA → clear RIS.Z → wait for Z`. A Z pulse landing in the ~5-cycle window between enable and clear caused the DMA to silently consume bit 15, then the CPU spin-wait caught what was really bit 14's Z and flipped pins one bit late. Frame CRC fails ~10–20 % of the time, enough to prevent the ESC from locking onto the protocol.

Replaced with a **two-stage Z sync**: spin once to land at a known counter position, then ack + enable DMA inside the post-Z quiet window, then spin again — the second Z is guaranteed to be the same one DMA fires on. CPU and DMA are now synchronized on bit 15. Worst-case spin: ~6.6 µs (2× bit period). The stage-2 timeout path zeroes all four DMASZ to prevent stale-data transfers if TIMA1 ever stops emitting Z.

---

## Build & flash

1. Open the firmware folder in Code Composer Studio (CCS) as an existing project.
2. Ensure exactly one of `dshot_test.c` and `pwm_test.c` is excluded from the build.
3. Build (Project → Build). Flash via XDS-equipped board (see `targetConfigs/`).
4. Attach the SWD debugger and add the relevant `dshot_dbg_*`, `imu_*`, `diag_*` variables to an Expressions watch.

---

## Status (as of 2026-05-15)

- DShot driver: **register-level complete**, CCPD + bit-15 race + trailing-HIGH fixes all in. Awaiting hardware verification.
- IMU driver: **working** (`WHO_AM_I=0x47`, 1 kHz samples, orientation tracking confirmed).
- PWM fallback: compilable but abandoned for bring-up.
- Gyro PID: not started. Will need PT1 / biquad LPF on raw gyro samples first.
- CRSF radio input: not started.
- Mixer: not started.

Largest outstanding risk: no oscilloscope or logic analyzer has been used in any debug so far. Every conclusion above is inferred from register state and SWD readbacks. Acquiring a Saleae-class logic analyzer is the highest-leverage next step.

---

## License

`delay.c` / `delay.h` is a reproduction of standard TI code under TI's BSD-3-Clause license. All other code in this folder is **© 2026 Caleb Kemere, all rights reserved** — see `LICENSE.md` at the repo root (if present) for details.
