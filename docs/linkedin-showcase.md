# nRF52840 Bluetooth Sensor Hub
### Headless IoT Platform with NFC-Assisted Secure Pairing

**Author:** Michael Neumüller · [linkedin.com/in/michael-neumueller1](https://www.linkedin.com/in/michael-neumueller1/)
**Date:** March 2026 · **Source:** [github.com/MikeProStudio/nRF52840-Bluetooth-Sensor-Hub](https://github.com/MikeProStudio/nRF52840-Bluetooth-Sensor-Hub)

---

## Executive Summary

A production-grade embedded firmware for the **Nordic nRF52840** (ARM Cortex-M4F) running on **Zephyr RTOS**, designed as a fully headless multi-sensor BLE beacon. The system solves a core IoT challenge — **secure device onboarding without any physical user interface** — by combining NFC Type 2 Tag emulation with hardware-derived cryptographic pairing, delivering a seamless tap-to-connect experience to any NFC-enabled smartphone.

### Technology Stack

| Layer | Technology |
|---|---|
| **MCU / SoC** | Nordic nRF52840 — ARM Cortex-M4F, 1 MB Flash, 256 KB RAM |
| **RTOS** | Zephyr RTOS (preemptive multithreading, workqueue scheduling) |
| **Connectivity** | Bluetooth 5.0 LE (1M/2M PHY), NFC Type 2 Tag (NDEF) |
| **Security** | BLE Security Level 4 — AES-128 CCM, ECDH key exchange |
| **Sensors** | LSM6DS3TR-C 6-axis IMU (I²C), PDM MEMS Microphone |
| **Power** | LiPo 3.3–4.15 V, BQ25100 charger, adaptive TX power |
| **Interface** | Web Bluetooth API dashboard — zero native app required |
| **Hardware** | Seeed Studio XIAO nRF52840 Sense |

### High-Level System Overview

```
┌──────────────────────────────────────────────────────────────────┐
│                    nRF52840 Sensor Hub                           │
│                                                                  │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌───────────┐  │
│  │ IMU 6-Axis │  │ PDM Micro- │  │  Battery   │  │  RGB LED  │  │
│  │  @52 Hz    │  │ phone @16k │  │ ADC + SMA  │  │ Feedback  │  │
│  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘  └───────────┘  │
│        │               │               │                         │
│  ┌─────▼───────────────▼───────────────▼──────────────────────┐  │
│  │              Zephyr RTOS Kernel (3 Threads)                │  │
│  │     Main Thread  ·  Audio Thread  ·  Sensor Thread         │  │
│  └─────────────────────────┬──────────────────────────────────┘  │
│                            │                                     │
│  ┌─────────────────────────▼──────────────────────────────────┐  │
│  │          Custom BLE GATT Service (6 Characteristics)       │  │
│  │   Accel · Gyro · Audio · TX Power · Battery · Reset        │  │
│  │              ALL require authenticated pairing              │  │
│  └───────────┬────────────────────────────┬───────────────────┘  │
│              │                            │                      │
│  ┌───────────▼──────────┐  ┌──────────────▼───────────────────┐  │
│  │   BLE 5.0 Radio      │  │   NFC T2T Tag (NDEF URI)        │  │
│  │   1M / 2M PHY        │  │   Embedded PIN + Web URL        │  │
│  │   Adaptive TX Power   │  │   Tap-to-Connect Workflow       │  │
│  └───────────┬──────────┘  └──────────────┬───────────────────┘  │
└──────────────┼────────────────────────────┼──────────────────────┘
               │         ◄── AIR ──►        │
┌──────────────▼────────────────────────────▼──────────────────────┐
│                     Smartphone / Browser                         │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │    Web Bluetooth Dashboard — Real-Time Sensor Telemetry    │  │
│  │    Charts · Spectrogram · Power Management · CSV Export    │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

---

## Security Architecture & NFC Workflow

### The Problem: Secure Pairing Without a Screen

Traditional BLE devices display a pairing PIN on a screen or use insecure "Just Works" pairing. A headless sensor node has neither a display nor input buttons — yet must enforce strong authentication to protect sensor data from unauthorized access.

### The Solution: Hardware-Derived PIN via NFC Handoff

This system implements a **three-layer security architecture** that turns NFC into a physical-proximity authentication channel:

```
  DEVICE BOOT                         USER INTERACTION
  ──────────                           ────────────────
  ┌─────────────────────┐
  │ 1. Read FICR HW ID  │    FICR = Factory Information Configuration Register
  │    (unique per chip) │    Burned at manufacturing — immutable, unique
  └─────────┬───────────┘
            │
            ▼
  ┌─────────────────────┐
  │ 2. Derive 6-digit   │    PIN = (DeviceAddress % 900000) + 100000
  │    passkey from FICR │    Deterministic: same PIN every boot, no storage needed
  └─────────┬───────────┘
            │
            ├──────────────────────────────────┐
            ▼                                  ▼
  ┌─────────────────────┐         ┌──────────────────────────────┐
  │ 3a. Register PIN    │         │ 3b. Encode PIN in NFC Tag    │
  │     with BLE SMP    │         │     as NDEF URI:             │
  │     (Passkey Display)│         │     https://.../?pin=XXXXXX  │
  └─────────────────────┘         └───────────────┬──────────────┘
                                                  │
                                                  ▼
                                  ┌──────────────────────────────┐
                                  │ 4. User taps phone on device │
                                  │    → Browser opens Web UI    │
                                  │    → PIN auto-displayed      │
                                  │    → User enters PIN in OS   │
                                  │      pairing dialog          │
                                  └───────────────┬──────────────┘
                                                  │
                                                  ▼
                                  ┌──────────────────────────────┐
                                  │ 5. BLE SMP Passkey Exchange  │
                                  │    → AES-128 CCM encryption  │
                                  │    → Bond stored on both     │
                                  │      sides (NVS Flash)       │
                                  └──────────────────────────────┘
```

### Security Enforcement at GATT Level

Every single characteristic in the custom GATT service enforces `BT_GATT_PERM_READ_AUTHEN` and `BT_GATT_PERM_WRITE_AUTHEN`. This means:

- **No data leaks** — an unauthenticated observer cannot read sensor values
- **No unauthorized control** — the remote bootloader reset requires a bonded connection
- **Encryption is mandatory** — all traffic is AES-128 CCM encrypted after pairing
- **Bonding persists** — credentials survive reboots via Zephyr's NVS/Settings subsystem

### Why NFC Matters for Headless Devices

| Traditional Approach | This Solution |
|---|---|
| Print PIN on label → static, easily shared | PIN derived from hardware → unique per device |
| "Just Works" pairing → zero authentication | Passkey Display (Level 4) → MITM-resistant |
| Requires companion app for onboarding | NFC tap opens standard browser → zero app install |
| PIN visible only during setup wizard | PIN embedded in URL, auto-expires in Web UI (20 s) |

---

## Real-Time System Architecture

### Preemptive Multi-Threading Model

The firmware runs three concurrent threads on Zephyr's preemptive scheduler, plus two asynchronous workqueue tasks for non-blocking power management:

```
Priority
   ▲
   │
 2 │  ┌──────────────────────────────────────────────────────┐
   │  │  AUDIO THREAD                                        │
   │  │  PDM Mic → 512-sample blocks @ 16 kHz                │
   │  │  → RMS (loudness) + ZCR (zero-crossing rate)         │
   │  │  → BLE Notify + RGB LED audio-reactive feedback      │
   │  └──────────────────────────────────────────────────────┘
   │
 3 │  ┌──────────────────────────────────────────────────────┐
   │  │  SENSOR THREAD                                       │
   │  │  LSM6DS3TR-C IMU → Accel + Gyro @ 52 Hz (20 ms)     │
   │  │  → Little-endian serialize → BLE GATT Notify         │
   │  └──────────────────────────────────────────────────────┘
   │
   │  ┌──────────────────────────────────────────────────────┐
   │  │  MAIN THREAD + WORKQUEUES                            │
   │  │  ├─ Battery Work (k_work_delayable)                  │
   │  │  │  ADC burst → SMA(12) filter → SoC estimation      │
   │  │  │  Interval: 1 s (first 30 s) → 10 s (steady)      │
   │  │  ├─ Status Work (1 s polling)                        │
   │  │  │  Charge IC monitoring (BQ25100 /CHG pin)          │
   │  │  │  USB VBUS detection → charge current control      │
   │  │  └─ Adaptive TX Power Logic                          │
   │  │     SoC > 20% OR USB → +8 dBm (high range)          │
   │  │     SoC < 20% battery → −4 dBm (power save)         │
   │  └──────────────────────────────────────────────────────┘
   │
   └──────────────────────────────────────────────────────────►
                                                          Time
```

### Custom BLE GATT Service

A single primary service exposes six characteristics, covering the full sensor pipeline and system control:

| Characteristic | UUID Suffix | Type | Description |
|---|---|---|---|
| Accelerometer | `...def1` | Notify | 3× int32 LE (X, Y, Z) in milli-m/s² |
| Gyroscope | `...def2` | Notify | 3× int32 LE (X, Y, Z) in milli-°/s |
| Audio Level | `...def3` | Notify | uint32 RMS + uint32 ZCR per block |
| TX Power | `...def4` | Read/Notify | int8 current TX power in dBm |
| Battery | `...def5` | Read/Notify | uint16 mV + uint8 SoC% + uint8 status |
| System Reset | `...def6` | Write | Write `0x01` → reboot to UF2 bootloader |

### Intelligent Power Management Pipeline

```
 ┌──────────┐    ┌──────────┐    ┌───────────────┐    ┌──────────────┐
 │ ADC Read │───►│ Burst ×2 │───►│ SMA Filter    │───►│ SoC Estimate │
 │ (P0.14)  │    │ + Average│    │ (12 samples)  │    │ 3.3–4.15 V   │
 └──────────┘    └──────────┘    └───────────────┘    └──────┬───────┘
                                                             │
                      ┌──────────────────────────────────────┘
                      ▼
              ┌───────────────┐         ┌─────────────────────┐
              │ Decision      │────────►│ HCI VS Command      │
              │ SoC < 20%    │  Yes    │ Set TX = −4 dBm     │
              │ & Battery?    │         │ (power save mode)   │
              └───────┬───────┘         └─────────────────────┘
                      │ No / USB
                      ▼
              ┌─────────────────────┐
              │ Set TX = +8 dBm     │
              │ (maximum range)     │
              └─────────────────────┘
```

---

## Web Bluetooth Dashboard & Engineering Highlights

### Browser-Native Sensor Monitoring

The system ships with a fully responsive **Web Bluetooth API** dashboard that requires **zero native app installation**. Users connect directly from Chrome/Edge on any platform:

| Dashboard Feature | Technical Detail |
|---|---|
| **Real-Time Telemetry** | Acceleration & gyroscope charts (Chart.js, 150-point rolling buffer) |
| **Professional Audio Analysis** | 1024-point FFT, A-weighted PSD, live spectrogram (Canvas 2D waterfall) |
| **Power Management View** | Battery voltage, SoC %, charge status, TX power with mW conversion |
| **NFC PIN Integration** | URL parameter `?pin=XXXXXX` auto-displays passkey with 20 s expiry timer |
| **Remote System Control** | One-click reboot to UF2 bootloader for wireless firmware updates |
| **Session Export** | CSV telemetry export for offline analysis |
| **Performance** | Visibility API throttling, 30 FPS render cap, EMA-smoothed audio visuals |

### Architecture Decisions & Engineering Highlights

```
 ┌────────────────────────────────────────────────────────────────┐
 │                    KEY ENGINEERING DECISIONS                    │
 ├────────────────────────────────────────────────────────────────┤
 │                                                                │
 │  ✦ ZERO-UI ONBOARDING                                         │
 │    NFC tag + Web Bluetooth = no display, no app, no manual     │
 │    PIN entry. Tap and go.                                      │
 │                                                                │
 │  ✦ HARDWARE-ROOTED TRUST                                      │
 │    Passkey derived from immutable FICR silicon ID.             │
 │    Deterministic — no RNG, no provisioning step, no storage.   │
 │                                                                │
 │  ✦ DEFENSE IN DEPTH                                            │
 │    NFC = physical proximity gate → BLE SMP = cryptographic     │
 │    gate → GATT AUTHEN = per-characteristic access control.     │
 │                                                                │
 │  ✦ BATTERY-AWARE RADIO                                         │
 │    Dynamic TX power saves energy at low SoC without            │
 │    sacrificing range when power is available.                   │
 │                                                                │
 │  ✦ NON-BLOCKING FIRMWARE                                       │
 │    All I/O is threaded or workqueue-driven. Main loop sleeps   │
 │    at 1 Hz. Zero busy-wait patterns.                           │
 │                                                                │
 │  ✦ BLUETOOTH 5.0 OPTIMIZED                                     │
 │    2M PHY support, extended data length (69 bytes),            │
 │    optimized connection intervals (15–30 ms), up to            │
 │    3 simultaneous connections.                                  │
 │                                                                │
 │  ✦ PRODUCTION PATTERNS                                         │
 │    NVS-backed bonding, SMA-filtered ADC, graceful reconnect,  │
 │    RGB LED startup self-test, remote bootloader access.        │
 │                                                                │
 └────────────────────────────────────────────────────────────────┘
```

### Connect

**LinkedIn:** [linkedin.com/in/michael-neumueller1](https://www.linkedin.com/in/michael-neumueller1/)
**Source Code:** [github.com/MikeProStudio/nRF52840-Bluetooth-Sensor-Hub](https://github.com/MikeProStudio/nRF52840-Bluetooth-Sensor-Hub)

---

*Built with Zephyr RTOS · Nordic nRF Connect SDK · Web Bluetooth API — March 2026*
