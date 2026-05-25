# Cheat Sheet

Read this 10 minutes before any interview that mentions this project. Every line is a talking point.

---

## 15-second elevator pitch

> "An ESP32-S3 vision node that runs an INT8 face-detection neural net on-device, ships detection events to the cloud over TLS-encrypted MQTT, updates its own firmware over Wi-Fi with bootloader-level rollback, and buffers events to an SD card during network outages. Built from scratch in ESP-IDF v5.3.2."

If they want more: hit the four pillars — **edge ML, secure transport, OTA with rollback, offline resilience.**

---

## KPI table (memorise the bold ones)

| Metric | Value |
|---|---|
| MCU | ESP32-S3, dual-core Xtensa LX7 @ 240 MHz |
| Flash / PSRAM | 8 MB QIO / **8 MB Octal @ 80 MHz** |
| Sensor | OV3660, 3 MP, RGB565 BE @ 20 MHz XCLK |
| **Inference time / frame** | **185 ms** (espdet_pico_224_224_face INT8) |
| **Pipeline FPS** | **~5** with TLS-MQTT + inference + HTTP server |
| **Bench confidence** | **0.85 – 0.90** on real faces |
| Camera-only FPS (JPEG path, no inference) | 27.77 |
| **End-to-end IP → MQTT-TLS** | **~4.2 s** |
| TLS handshake | ~940 ms |
| SNTP sync from IP | ~2.5 s |
| Detection publish cap | 1 Hz |
| SD replay rate | 20 Hz |
| **OTA download time (3.2 MB over LAN HTTP)** | **~34 s** |
| Post-OTA reboot → MQTT reconnect | ~3.7 s |
| Wi-Fi exp-backoff | 0.5 / 1 / 2 / 4 / 8 / 16 / 32 / 60 s capped, forever |
| TWDT timeout | 30 s on capture loop |
| Internal heap free, steady | ~120 KB |
| PSRAM free, steady | ~7.5 MB |
| Heap drift over hours | 0 (no leaks) |
| Custom partition slot size | 3.5 MB × 2 (OTA A/B) |
| Frame buffer | QVGA RGB565 = 153,600 B × 3 in PSRAM |

---

## Tech stack (one breath)

ESP-IDF v5.3.2 · FreeRTOS · ESP-DL 3.1.2 · esp32-camera · esp-mqtt · mbedTLS · esp_https_ota · esp_http_server · FATFS over SDMMC · mDNS · SNTP · NVS · custom 8 MB partition table.

---

## Likely interview questions + 30-second answers

**Q: Why ESP32-S3 specifically, not ESP32 or ESP32-S2?**
A: S3 adds vector-extension instructions (PIE) and dedicated AI helper instructions that ESP-DL targets. INT8 conv layers run ~10× faster than on the original ESP32. Also dual-core for concurrent Wi-Fi + inference, and Octal PSRAM bandwidth needed for the ~3 MB model.

**Q: Why on-device inference instead of cloud?**
A: Three reasons. Latency: sub-250 ms end-to-end alert vs. seconds for image upload + cloud inference. Privacy: no pixels leave the device. Cost: zero per-detection compute spend, zero bandwidth.

**Q: How does the OTA rollback work?**
A: Dual-slot OTA: `ota_0` and `ota_1` partitions, both bootable. After OTA writes the new image, `otadata` flags it `PENDING_VERIFY`. Device reboots into new slot. After Wi-Fi + MQTT come back up, the app calls `esp_ota_mark_app_valid_cancel_rollback()`. If the new image hangs before that — TWDT panic, reboot — bootloader sees PENDING_VERIFY was never cleared and rolls back to the previous slot.

**Q: Why MQTT and not HTTP for telemetry?**
A: Persistent connection means no per-message TLS handshake. Pub/sub fan-out: one device, many subscribers. Built-in QoS levels. Last-Will gives subscribers instant offline notification. HTTP would re-handshake per request — wasteful and slow.

**Q: Why TLS — is it really needed for telemetry?**
A: Yes. The cmd topic accepts an OTA URL. If an attacker can publish to that topic, they can ship malicious firmware. Wi-Fi password protects the LAN; TLS protects the broker session.

**Q: What if Wi-Fi drops mid-OTA?**
A: `esp_https_ota_perform()` returns `ESP_FAIL`. We call `esp_https_ota_abort()`, which discards the partially-written slot. Current running image is untouched. Next OTA command retries from scratch.

**Q: Why RGB565 and not RGB888?**
A: OV3660 sensor + esp32-camera driver doesn't support direct RGB888 output. RGB565 halves the buffer size (153 KB vs 230 KB at QVGA) anyway, which matters for `fb_count=3`.

**Q: Why fb_count=3?**
A: Two consumers — capture loop and HTTP `/jpg` handler — can hold buffers simultaneously. With `fb_count=2`, the producer (DMA) has zero free buffers and frame capture stalls. Three buffers gives the producer one free slot at all times. Costs 153 KB extra PSRAM.

**Q: Why exponential backoff for Wi-Fi reconnect?**
A: Naïve immediate-retry hammers the AP during outage, drains battery if portable, and floods logs. Exponential backoff (0.5/1/2/4/8/16/32/60 s) gives the network time to heal, lets the radio sleep between attempts. Cap at 60 s so reconnect doesn't take 17 minutes after a long outage.

**Q: Why a mutex on the SD events file?**
A: Two tasks access it — capture loop (append on offline detection) and replay task (read + delete on reconnect). FATFS internal state isn't re-entrant; concurrent open of the same file corrupts the directory entry. The mutex serialises the file ops.

**Q: How do you handle the case where the SD card isn't inserted?**
A: `sd_mount()` returns an error, `s_sd_mounted` stays false. The buffer-write function becomes a no-op when offline. The device works fine in volatile-only mode — just loses events during outages.

**Q: How did you size the partition table?**
A: Started with default 4 MB layout. Embedded model pushed the app past 3 MB so I dropped the factory partition (dual-OTA only), grew slots to 3.5 MB each. SPIFFS reserved for future use. Total fits in 8 MB.

**Q: What does the task watchdog buy you?**
A: Combined with OTA, automatic recovery from bad firmware. A new image that hangs the capture loop → TWDT panic → reboot → bootloader rolls back. Without TWDT, a bad OTA could brick the device requiring physical reflash.

**Q: Why is the reset reason valuable?**
A: It's postmortem evidence that survives the reboot. `TASK_WDT` means our code hung. `PANIC` means we crashed. `BROWNOUT` means the power supply is marginal. Without it you're flying blind on field failures.

**Q: What's the weakest part of your design?**
A: Two things. (1) The model is INT8 quantised at training — I didn't train it, used Espressif's prebuilt. A custom-trained model for the specific application would do better. (2) The detection rate-limit is producer-side only; under attack a misbehaving device could still flood within its 1 Hz cap. Production would add broker-side throttling.

**Q: How would you scale this to a fleet?**
A: MQTT topic structure already supports it: `edgecam/<device_id>/...` is per-device. Add a wildcard-subscribing consumer (Telegraf → InfluxDB for metrics, separate alerting service for detections). For OTA, version-tag the binary URLs and publish to a `fleet/firmware/update` topic with payload `{"version":"x.y.z","url":"...","cohort":"canary"}`.

---

## Resume bullets (drop-in)

> Edge-AI Vision Node (ESP32-S3) — ESP-IDF v5.3.2, FreeRTOS, custom 8 MB dual-OTA partition layout

- Designed and shipped firmware for an on-device person/face detection camera with TLS-secured MQTT telemetry, achieving ~185 ms INT8 inference per QVGA frame at ~5 FPS while concurrently running Wi-Fi, TLS-MQTT, and HTTP server tasks.
- Implemented MQTT-triggered firmware OTA over Wi-Fi with Mozilla CA-bundle server cert validation, dual-slot swap, and bootloader-level rollback; verified end-to-end live upgrade from v0.5.0 to v0.5.1.
- Built offline-tolerant event pipeline: detections buffer to FATFS on microSD when MQTT is down, drain at 20 Hz on reconnect via a producer/consumer mutex pattern. Zero events lost across simulated network drops.
- Hardened for field deployment: 30-s task watchdog on capture loop with auto-panic + rollback, exponential-backoff Wi-Fi reconnect (0.5–60 s) with forever-retry, boot reset-reason decoded and surfaced in cloud telemetry, and minimum-heap watermark tracking to catch slow leaks.
- Integrated ESP-DL `espdet_pico_224_224_face` (INT8, ~3 MB) on the ESP32-S3 vector ISA; diagnosed and fixed IDF v5.3 compatibility (custom `MALLOC_CAP_SIMD` shim), sensor pixel-format mismatch (RGB565 big-endian enum), and partition overflow (resized OTA slots 3 MB → 3.5 MB).
- Two-topic MQTT design (`telemetry`, `detections`) with QoS 1, producer-side rate limiting, retained LWT, and a single Mozilla CA trust root shared between MQTT-TLS and HTTPS OTA.

---

## Demo flow (memorise the order)

1. **Power on** → serial monitor shows boot reason, partition table, PSRAM enumeration, sensor PID, Wi-Fi IP, TLS handshake, MQTT CONNECT.
2. **Show face** → serial `DET pub #N`, HiveMQ web client shows JSON event arrive.
3. **Open browser** → `http://<board-ip>/` → live JPEG snapshot, refreshes every 1 s.
4. **Turn off Wi-Fi (or change password and reflash)** → buffer warnings `DET buffered #N`, telemetry stops.
5. **Restore Wi-Fi** → `spawning replay for N events`, all N appear at HiveMQ in a burst.
6. **MQTT publish `reboot`** → device cleanly reboots, next boot reason `SW_REBOOT`.
7. **MQTT publish `ota <url>`** → 0–100% download in serial, slot swap, reboot, new `fw=...` version logged.

If you only have 60 seconds: do 1, 2, 7. The OTA is the showstopper.

---

## Gotchas that came up during development (be ready to talk about them)

| Symptom | Root cause | Fix |
|---|---|---|
| `MALLOC_CAP_SIMD undefined` | ESP-DL 3.1+ needs IDF v5.4, we're on v5.3.2 | Compile-def shim: `-DMALLOC_CAP_SIMD=MALLOC_CAP_8BIT` |
| `Requested format is not supported` | Tried RGB888 on OV3660 (driver limitation) | Switched to RGB565 BE, matched model's pixel-type enum |
| Inference ran but 0 detections | Wrong pixel-type endianness (LE vs BE) | Grepped `dl_image_define.hpp` for enum, used `RGB565BE` |
| `All app partitions too small for binary` | Embedded model bloated app past 3 MB slot | Resized OTA slots to 3.5 MB, removed unused model data partition |
| `Invalid field value 0xe0000?` | Trailing `?` typo in partitions.csv | Whitespace + character sensitivity; clean rewrite |
| TLS handshake fails post-power-on | System time = 1970, cert NotBefore in future | SNTP sync before MQTT_start |
| FB-SIZE warnings during OTA | Camera HAL falling behind under OTA download load | Benign; recovers when OTA finishes; reflects expected backpressure |
| Console silence after first flash | Wrong USB port (USB UART, not USB OTG) | Use the right USB-C port labeled `USB` |

---

## File locations to mention if asked

- `main/main.c` — capture loop + all subsystem orchestration
- `main/inference.cpp` — C++ wrapper, ~50 lines
- `main/secrets.h` — gitignored credentials
- `partitions.csv` — custom layout
- `sdkconfig.defaults` — Octal PSRAM, 240 MHz, etc.
- `docs/ARCHITECTURE.md` — full deep dive (point them here if they want details)

---

*Print this. Read on the way to the interview.*
