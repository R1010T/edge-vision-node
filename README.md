# Edge-AI Vision Node — ESP32-S3

> An on-device person/face detection camera with TLS-secured MQTT telemetry, offline event buffering, and over-the-air firmware updates. No cloud inference. No image upload. Sub-250 ms alert latency.

[![Build](https://github.com/R1010T/edge-vision-node/actions/workflows/ci.yml/badge.svg)](https://github.com/R1010T/edge-vision-node/actions)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.3.2-blue)](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/)
[![Platform](https://img.shields.io/badge/MCU-ESP32--S3-red)](https://www.espressif.com/en/products/socs/esp32-s3)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

<!-- TODO: drop a 3–5 sec GIF of the device detecting a face here -->

---

## What it does

A $10 microcontroller with a camera that:

- 🧠 Runs an **INT8 quantised neural network on-device** (no cloud).
- 📤 Pushes a **structured JSON detection event** over **TLS-encrypted MQTT** within ~250 ms of seeing a face.
- 📶 **Survives network outages**: detections queue to an SD card and replay on reconnect — zero events lost.
- 🚀 **Updates itself over Wi-Fi**: push an MQTT command, device downloads + verifies + reboots into the new firmware. Bad firmware auto-rolls back via the bootloader.
- 🌐 Serves a **live JPEG snapshot** on a local HTTP endpoint for spot-checking.
- 🛡️ Survives **panics, watchdog hangs, and brownouts** with auto-recovery and post-mortem logging.

## Why this exists

I built this to demonstrate production-grade embedded firmware practices on a single board: edge ML inference, secure transport, OTA with rollback, offline resilience, and field hardening — the actual things firmware engineers ship.

---

## Hardware

| Item | Notes |
|---|---|
| **Freenove ESP32-S3-WROOM dev board** | Use the **`USB`** port (right USB-C) — native USB-Serial-JTAG, no driver needed. |
| **ESP32-S3-WROOM-1 N8R8 module** | 8 MB QIO flash + 8 MB **Octal** PSRAM. PSRAM is mandatory (model weights and frame buffers live there). |
| **OV2640 or OV3660 camera** | Plug into the on-board FPC connector. OV3660 (3 MP) auto-detected by the driver. |
| **microSD card** | Any size, FAT-formatted. Slot is on-board. |
| **USB-C cable** | Just a cable. No external programmer needed. |

## Software stack

| Layer | Tech |
|---|---|
| RTOS / SDK | ESP-IDF v5.3.2 · FreeRTOS · CMake + Ninja |
| ML inference | ESP-DL v3.1.2 · `human_face_detect` (INT8 quantised, `espdet_pico_224_224_face`) |
| Camera | `esp32-camera` v2.1.6 over SCCB + parallel DMA |
| Networking | Wi-Fi STA + WPA2-PSK · mbedTLS · esp-mqtt · esp_http_server · esp_https_ota |
| Storage | FATFS over SDMMC (1-bit) · NVS · custom 8 MB partition table with dual-OTA |
| Discovery | mDNS · SNTP |

---

## Architecture

```
       ┌──────────┐    DMA           ┌────────────────────┐         ┌──────────────┐
       │  OV3660  │ ───────────────> │  PSRAM frame buf   │ ──────> │  ESP-DL      │
       │  sensor  │  RGB565 BE       │  3 slots, LATEST   │         │  INT8 model  │
       └──────────┘                  └────────────────────┘         └──────┬───────┘
                                                                            │
                                                                  result    ▼
                              ┌────────────────────┐    online?     ┌──────────────┐
                              │  MQTT-TLS publish  │ ◄───────────── │  router      │
                              │  + retained LWT    │                └────────┬─────┘
                              └────────────────────┘                         │ offline
                                                                             ▼
                                                                ┌─────────────────────┐
                                                                │  FATFS on microSD   │
                                                                │  events.log → replay│
                                                                └─────────────────────┘
```

### MQTT topic design

| Topic | Direction | QoS | Purpose |
|---|---|---|---|
| `edgecam/<id>/telemetry` | device → broker | 1 | Device-state JSON every 5 s |
| `edgecam/<id>/detections` | device → broker | 1 | One JSON event per face (rate-limited 1 Hz) |
| `edgecam/<id>/cmd` | broker → device | 1 | Commands: `ota`, `replay`, `purge`, `reboot` |

`<id>` is the last 3 bytes of the station MAC (e.g. `edgecam/37cd74/...`).

### Flash partition layout (8 MB)

| Label | Type | Offset | Size | Purpose |
|---|---|---|---|---|
| `nvs` | data | 0x9000 | 24 K | Wi-Fi creds, runtime config |
| `otadata` | data | 0xf000 | 8 K | Which OTA slot to boot |
| `phy_init` | data | 0x11000 | 4 K | RF calibration data |
| `ota_0` | app | 0x20000 | 3.5 M | OTA slot A |
| `ota_1` | app | 0x3a0000 | 3.5 M | OTA slot B |
| `storage` | spiffs | 0x720000 | 896 K | Reserved for future SPIFFS use |

The 3.5 MB slot size accommodates the embedded ~3 MB neural network while preserving dual-slot OTA rollback.

---

## Quick start

### 1. Install ESP-IDF v5.3.2

Follow the official guide: <https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32s3/get-started/index.html>

Or use the **ESP-IDF VS Code extension** → `Open ESP-IDF Installation Manager` → Custom → v5.3.2.

### 2. Clone and configure secrets

```bash
git clone https://github.com/R1010T/edge-vision-node.git
cd edge-vision-node
cp main/secrets.h.template main/secrets.h
# edit main/secrets.h with your Wi-Fi + HiveMQ credentials
```

### 3. Build, flash, monitor

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <YOUR_PORT> flash monitor
```

Exit monitor with `Ctrl+]`.

### 4. Watch it work

- **Serial:** every 5 s you'll see a `5s: fps=... mqtt=UP ...` heartbeat.
- **HTTP:** open `http://<board-ip>/` for a live JPEG snapshot.
- **MQTT:** open the HiveMQ Cloud web client, subscribe to `edgecam/+/detections`, point the camera at a face.

---

## Demo: trigger an OTA over MQTT

```
Topic:   edgecam/<id>/cmd
QoS:     1
Payload: ota http://<host>:8000/firmware.bin
```

For HTTPS (production): host the binary on a GitHub Release. The device validates the server certificate against the embedded Mozilla CA bundle.

Other commands:

| Payload | Effect |
|---|---|
| `reboot` | Software reset (boot reason will be `SW_REBOOT` next session) |
| `replay` | Force drain of the SD event buffer |
| `purge` | Wipe the SD buffer without publishing |
| `ota <url>` | Download + verify + reboot into new firmware |

---

## Measured KPIs

| Metric | Value |
|---|---|
| INT8 inference time per frame (QVGA RGB565) | **~185 ms** |
| Confidence on bench-test face | **0.85 – 0.90** |
| Pipeline FPS under full load (Wi-Fi + TLS-MQTT + inference) | **~5 FPS** |
| End-to-end IP → MQTT-TLS CONNECTED | **~4.2 s** |
| TLS handshake | **~940 ms** |
| Detection publish rate cap | **1 Hz** |
| SD-buffered events drained on reconnect | **20 Hz** |
| OTA update v0.5.0 → v0.5.1 over LAN HTTP | **~34 s** for 3.2 MB |
| Post-OTA reboot → MQTT-TLS reconnect | **~3.7 s** |
| Internal heap free, steady state | **~120 KB** |
| PSRAM free, steady state | **~7.5 MB** |
| Heap drift over time | **0** (no leaks observed) |

---

## Project structure

```
edge-vision-node/
├── CMakeLists.txt              # ESP-IDF project entry
├── partitions.csv              # 8 MB layout w/ dual OTA
├── sdkconfig.defaults          # Octal PSRAM, 240 MHz CPU, ...
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml       # esp-dl, mdns, esp32-camera dependencies
│   ├── camera_pins.h           # OV-sensor pin map (Freenove board)
│   ├── secrets.h.template      # Copy → secrets.h, fill credentials
│   ├── inference.h             # C interface to ESP-DL
│   ├── inference.cpp           # C++ wrapper around HumanFaceDetect
│   └── main.c                  # Everything else (see top of file)
├── .github/workflows/
│   └── ci.yml                  # GitHub Actions: idf.py build on every push
├── docs/
│   ├── ARCHITECTURE.md         # Deep dive into each subsystem
│   └── CHEATSHEET.md           # Interview prep + KPIs
└── README.md                   # This file
```

---

## Design highlights (for the reader who is going to ask in an interview)

- **Boot reset-reason captured first.** `esp_reset_reason()` decoded into a human string, logged at boot, embedded in every telemetry packet. Postmortem evidence survives across reboots.
- **Task watchdog on the capture loop.** If a single iteration takes >30 s the device panics. Combined with OTA rollback, this means a bad firmware image cannot brick the device — it bricks itself once, then the bootloader rolls back.
- **Exponential-backoff Wi-Fi reconnect.** 0.5 s → 1 s → 2 s → 4 s → ... → 60 s cap, forever. Initial connect has a 30 s hard cap so the device doesn't block boot indefinitely.
- **Two-topic MQTT design.** Telemetry (broad, low-rate) and detections (narrow, event-driven, rate-limited at the producer). Cloud subscribers can route them to different sinks.
- **Single CA trust root.** Same Mozilla CA bundle validates both MQTT-TLS and HTTPS OTA. No duplicated certificates.
- **SNTP before MQTT.** TLS rejects certs whose validity window doesn't include the current system time. Without time sync, every handshake fails with an obscure `cert_verify_flags` error.
- **Producer/consumer mutex on the SD event log.** Capture loop appends, replay task drains, mutex prevents interleaved file ops.
- **Minimum-heap watermark in telemetry.** Slow leaks become visible on the dashboard hours before they crash the device.

---

## What's next

- Phase 4 model swap to `pedestrian_detect` for industrial use cases
- Phase 5b graceful low-power mode (deep sleep when no motion)
- Phase 6 BLE-based Wi-Fi commissioning (instead of hard-coded credentials)
- Phase 7 Matter (Connected Home over IP) interop layer

---

## License

MIT. See [LICENSE](LICENSE).

Built with ESP-IDF, ESP-DL, esp32-camera, esp-mqtt, mDNS — all open source.
