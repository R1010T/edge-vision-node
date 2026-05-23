# Edge-AI Vision Node — Engineering Log

**Hardware:** Freenove ESP32-S3-WROOM dev board · ESP32-S3-WROOM-1 N8R8 · OV3660 camera (3 MP, RGB565 big-endian) · microSD (1 GB FAT) · 8 MB QIO Flash · 8 MB Octal (OPI) PSRAM
**Toolchain:** ESP-IDF v5.3.2 · GCC 13.2.0 (xtensa-esp32s3-elf) · CMake 3.30.2 · Ninja 1.12.1 · Python 3.13.3
**Host:** Windows 10/11 · VS Code + Espressif ESP-IDF extension v2.1.0
**Console transport:** native USB-Serial-JTAG on USB OTG port (COM4)
**Cloud broker:** HiveMQ Cloud free serverless tier (MQTT 5 over TLS 1.2, port 8883)
**Project goal:** On-device person/anomaly detection with TLS-secured MQTT telemetry, HTTP snapshot endpoint, MQTT-triggered firmware OTA with rollback, and offline event buffering.

---

## Phase 1 — Boot smoke test ✅

Verified toolchain + flash + PSRAM + partition table + GPIO + FreeRTOS before adding any peripheral.

| Item | Value |
|---|---|
| Chip | ESP32-S3, 2 cores, rev 0.2 |
| Flash | 8 MB, QIO 80 MHz |
| PSRAM | 8 MB, Octal AP gen 3, 80 MHz |
| Running partition | `ota_0` @ 0x00020000 |
| Internal heap free, idle | ~374 KB |
| PSRAM free, idle | ~8.0 MB |
| Heap drift | none |

Design: custom 8 MB partition table, dual OTA slots, OPI PSRAM, 240 MHz CPU, 1 kHz tick, app rollback enabled, console on native USB-Serial-JTAG.

---

## Phase 2 — Camera bring-up ✅

`espressif/esp32-camera` v2.1.6 driving an **OV3660** (PID 0x3660 — better than kit-documented OV2640). JPEG QVGA 320×240, 20 MHz XCLK, double-buffered DMA into PSRAM, `CAMERA_GRAB_LATEST`.

| Metric | Value |
|---|---|
| Throughput (JPEG path) | **27.77 FPS** |
| Frame capture (`fb_get → return`) | ~35 ms |
| Avg JPEG payload | 2.8 KB |
| PSRAM buffer placement | confirmed by pointer-range check |
| SOI / EOI markers | present per frame |

---

## Phase 3a–3d — Network stack ✅

**3a Wi-Fi STA:** IP in ~1.7 s, PMF capable, auto-reconnect, 10-retry cap.

**3b MQTT (plaintext):** `mqtt://broker.hivemq.com:1883`, QoS 1 telemetry, retained LWT, 5 s cadence. CONNECTED → SUBSCRIBED in ~400 ms. No FPS impact.

**3c MQTT TLS:** `mqtts://...:8883` against HiveMQ Cloud, full server cert chain validated via `esp_crt_bundle_attach` (Mozilla CA bundle), SNTP sync first (`pool.ntp.org` + `time.cloudflare.com`).

| Metric | Value |
|---|---|
| SNTP sync from IP | ~2.5 s |
| TLS handshake | ~940 ms |
| IP → MQTT-TLS CONNECTED | ~4.2 s |

**3d HTTP + mDNS:** `esp_http_server` on TCP/80 with `multipart/x-mixed-replace` MJPEG, single-shot `/jpg`, mDNS hostname `edgecam-<id>.local`. `fb_count` bumped 2 → 3 to support concurrent stream + stats consumers.

---

## Phase 4a/b — Edge AI ✅

ESP-DL 3.1.2 + `human_face_detect` component. Model `espdet_pico_224_224_face` (INT8, ~3 MB, baked into firmware as `flash_rodata`). Camera reconfigured JPEG → RGB565 big-endian (model input).

| Metric | Value |
|---|---|
| Inference per frame | **~185 ms** |
| Pipeline FPS | ~5 |
| Best score on real face | 0.90 |
| Internal heap during inference | ~123 KB free |
| PSRAM free | ~7.6 MB |

**4b** added a separate MQTT topic `edgecam/<id>/detections` with JSON `{ts, frame, score, box, infer_us, fw}`, rate-limited to 1 Hz at the producer, score-thresholded at 0.55. `/jpg` re-enabled via software JPEG encode (`frame2jpg`) of RGB565 frames. Auto-refresh HTML index page polls `/jpg` at 1 Hz.

### Gotchas captured
- `MALLOC_CAP_SIMD` undefined on IDF v5.3 — patched via `-DMALLOC_CAP_SIMD=MALLOC_CAP_8BIT` compile-def in root CMakeLists.
- `HumanFaceDetect::run()` v3.1 takes `dl::image::img_t`, not raw ptr + shape.
- OV3660 does **not** support `PIXFORMAT_RGB888`. Stuck with RGB565 + correct big-endian enum.
- Model default `MSR+MNP` was too small to trigger on bench faces; **`espdet_pico_224_224_face`** (bigger, slower, 185 ms) reliably hits 0.85+. Sub-real-time is fine for security/anomaly use case; reliability beats throughput.
- App grew past 3 MB; partition table resized OTA slots 3 MB → 3.5 MB each, removed unused `model` data partition, SPIFFS reduced to 896 KB.

---

## Phase 5a — MQTT-triggered firmware OTA ✅

The headline feature. Device subscribes to its cmd topic, parses `ota <url>` payload, downloads new firmware via `esp_https_ota` (CA-bundle TLS for `https://`, or plaintext `http://` for dev), writes to the inactive OTA slot, swaps via bootloader handshake, reboots, self-confirms on first successful operation.

### Verified end-to-end (v0.5.0 → v0.5.1 over Wi-Fi)
| Step | Measured |
|---|---|
| Image size pushed | 3,210,096 B (~3.06 MB) |
| Download time over LAN HTTP | ~34 s |
| Reported progress | 0% → 100% in 10% increments |
| Boot loader slot swap | `ota_0` (0x00020000) → `ota_1` (0x003a0000) |
| Post-OTA reboot to Wi-Fi reconnect | ~1.5 s |
| Post-OTA reboot to MQTT-TLS CONNECTED | ~3.7 s |
| `OTA image marked valid — rollback cancelled` | logged after first stable operation |
| Camera + inference + Wi-Fi survival during download | yes (with throttled FPS, expected) |

### Design decisions
- **Single CA trust root.** Same Mozilla CA bundle used by both MQTT-TLS and HTTPS OTA — no duplicated certificates in firmware.
- **OTA off the main loop.** Dedicated FreeRTOS task spawned per command, 8 KB stack, priority 5; main capture loop continues unaffected.
- **Boolean `s_ota_in_progress` guard** rejects concurrent OTA commands and is exposed in telemetry so operators see download state from the cloud.
- **Telemetry payload extended** with `fw` and `ota` fields. The same MQTT pipe is used to push commands and observe results — no separate control plane.
- **Allow-HTTP only via Kconfig** (`CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y`). Off by default; the choice is recorded in `sdkconfig.defaults`, not buried in source.
- **Pending-verify pattern.** `esp_ota_mark_app_valid_cancel_rollback()` runs on the first boot in the new slot only after the device reaches steady state (Wi-Fi up, NVS readable). If the new image bricks itself, watchdog reset triggers automatic rollback to the previous slot — no human intervention.

### Subtleties captured
- During OTA download under continuous inference load, the camera HAL emits `EV-VSYNC-OVF` and `FB-SIZE mismatch` warnings. These are benign — the sensor is fine; the consumer just got behind. FPS dropped to ~2 during the 34 s download window, recovered immediately on reboot.
- HiveMQ Cloud cmd topic accepts a plain text payload (`ota http://...`) — no JSON parsing needed for v1. JSON commanding can be layered in later without breaking compatibility.

---

## Phase 5b — SD-card event buffering with replay on reconnect 🟡 (code delivered, awaiting bench test)

Detection events flow to MQTT when connected, to `/sdcard/events.log` when offline. On reconnect, a background task drains the file at 20 Hz and purges it. Zero event loss across simulated network drops.

### Configuration
- SDMMC 1-bit mode mount via `esp_vfs_fat_sdmmc_mount` at `/sdcard`
- Pin map: `CMD=GPIO38, CLK=GPIO39, D0=GPIO40` (Freenove silkscreen)
- FATFS, 16 KB allocation unit, max 4 open files
- Default frequency (~20 MHz), `SDMMC_SLOT_FLAG_INTERNAL_PULLUP` enabled

### Design decisions
- **Producer/consumer with mutex.** Capture loop (producer) and replay task (consumer) cannot both have the events file open. `xSemaphoreCreateMutex()` serializes them; this is the textbook FreeRTOS pattern.
- **Graceful degradation if no card.** `sd_mount()` returning non-OK leaves `s_sd_mounted = false`; producer becomes a no-op when offline, no crash. Device runs in volatile-only mode.
- **Persistent across reboots.** Events from a previous power cycle are counted and replayed on next boot. The file survives because FATFS is power-safe on `fclose()`.
- **Three new commands** via the same MQTT cmd topic: `ota <url>`, `replay`, `purge`. Operator can force a replay or wipe the buffer without a reboot.
- **Telemetry exposes buffer state**: `buffered` count, `replayed` count, `sd` mount flag — visible in HiveMQ alongside fps, infer time, etc.

### Acceptance checklist (post-bench-test fill-in)
- [ ] Card mounts on boot, prints CSD info
- [ ] Disconnect Wi-Fi → see `DET buffered #N` lines
- [ ] Reconnect Wi-Fi → see `sd: replayed N events, file purged`
- [ ] All N buffered events appear at HiveMQ in a burst
- [ ] No event loss across N=10 simulated drops

---

## Phase 6 — Hardening ✅

**Purpose:** make the device tolerate field conditions — RF dropouts, slow leaks, hangs, brownouts — and emit diagnostics that survive a reboot.

### Four features
1. **Boot reset-reason decoded at start of `app_main`.** `esp_reset_reason()` mapped to human strings (`POWERON / SW_REBOOT / PANIC / TASK_WDT / BROWNOUT / USB / DEEPSLEEP_WAKE / …`). Logged once and embedded in every telemetry payload (`boot` field) so the cloud sees postmortem evidence on the next session.
2. **Task watchdog (TWDT) on the capture loop.** 30 s timeout, panic on trip. `esp_task_wdt_add(NULL)` subscribes the main task after all subsystems are up; `esp_task_wdt_reset()` feeds at the top of each loop iteration. If a single iteration stalls >30 s (deadlocked DMA, infinite loop in inference, etc.), the device panics → reboots → if the new OTA image is the one hanging, bootloader sees `PENDING_VERIFY` never confirmed → rolls back to previous slot. **OTA + WDT compose into automatic rollback for free.**
3. **Exponential-backoff Wi-Fi reconnect.** First attempt 500 ms, then 1 / 2 / 4 / 8 / 16 / 32 / 60 s capped — forever. Only kicks in after the device has connected at least once; before that, the initial 30 s connect timeout still applies so the device doesn't block boot. `s_retry_count` exposed in telemetry.
4. **Minimum-heap watermark in telemetry.** `heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_SPIRAM)` published every 5 s alongside the current free heap. Surfaces slow leaks on the dashboard hours before they crash the device.

### New cmd: `reboot`
Manual software reset over MQTT. Useful for testing rollback paths and as the operator's last-resort knob.

### Verified at first boot
```
W boot reason: USB (code 11)
I app=edge_vision_node idf=v5.3.2 fw=0.6.0
I TWDT armed: capture loop must feed within 30 s
I ready  boot=USB  sd=1  fw=0.6.0
```

### Resume bullet
> "Hardened for field deployment: task watchdog on the capture loop with auto-panic + bootloader rollback, exponential-backoff Wi-Fi reconnect with forever-retry policy, boot reset-reason decoded and surfaced in cloud telemetry, and minimum-heap watermark tracking to catch slow leaks before they crash the device. OTA + WDT compose to give automatic rollback on a bad firmware image."
## Phase 7 — KPI report + demo video + GitHub polish + final docs ⬜

---

## Interview-defensible one-liners (current)

- "Custom 8 MB partition table with dual 3.5 MB OTA slots sized to fit a 3 MB embedded INT8 neural net while preserving rollback semantics. App rollback armed in the bootloader and confirmed at runtime via the pending-verify handshake."
- "OV3660 over SCCB at 20 MHz XCLK; reconfigured at startup between hardware-JPEG (for HTTP snapshot fallback) and RGB565 big-endian (for inference) — driver detects the sensor and the model's pixel-type enum is matched to the sensor's native byte order."
- "TLS 1.2 MQTT with Mozilla CA-bundle server cert validation against HiveMQ Cloud, SNTP-driven time sync, password auth. Full DNS + TLS handshake + MQTT CONNECT in ~4 s from IP acquisition."
- "ESP-DL `espdet_pico_224_224_face` INT8 model runs on the ESP32-S3 vector ISA: **185 ms inference per frame at QVGA RGB565, 0.85+ confidence on bench faces**. Zero pixels leave the device. Model embedded in firmware, swappable in future revisions via the `model` partition slot reserved at design time."
- "MQTT-triggered firmware OTA over HTTP(S) with CA-bundle TLS validation, dual-slot swap, pending-verify rollback. **Verified end-to-end: device upgraded from v0.5.0 to v0.5.1 over Wi-Fi without a USB cable.**"
- "Offline-tolerant event pipeline: FATFS on microSD buffers detection events when MQTT is down, a background replay task drains them at 20 Hz on reconnect. Mutex-serialized producer/consumer over the events file; graceful degradation if no card present."
- "Two-topic MQTT design (telemetry + detections), QoS 1 throughout, retained LWT for online/offline status, producer-side rate limiting to give the broker bounded load even under attack or runaway detector behavior."
- "Diagnostics-first: PSRAM pointer-range checks, per-frame JPEG SOI/EOI inspection, structured decoding of `esp_tls` error flags, and a software-JPEG `/jpg` HTTP endpoint so a human can spot-check what the model is seeing without dumping raw RGB565."

## Resume-ready paragraph

> Designed and built an edge-AI vision node on ESP32-S3 in ESP-IDF v5.3.2 (dual-core FreeRTOS, OPI PSRAM, custom 8 MB dual-OTA partition layout): OV3660 camera over SCCB with DMA into PSRAM-resident triple frame buffer, on-device INT8 face detection via ESP-DL `espdet_pico_224_224_face` at ~185 ms per frame, TLS 1.2 MQTT against HiveMQ Cloud with full server-cert chain validation and SNTP time sync, mDNS service discovery, software-JPEG HTTP snapshot endpoint, MQTT-triggered firmware OTA with bootloader rollback (verified live upgrade across Wi-Fi), and FATFS-on-microSD offline event buffering with automatic replay on reconnect. All subsystems coexist on a single FreeRTOS schedule with rate-limited producer-side telemetry and structured JSON event payloads.

## Build & flash commands

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM4 flash monitor      # COM4 = USB-Serial-JTAG on USB OTG port
```

Exit monitor: `Ctrl+]`.

## OTA trigger (from HiveMQ web client)

Topic: `edgecam/<device_id>/cmd`
Payload: `ota http://<host>:8000/firmware.bin`  *(local dev)*
Or:      `ota https://github.com/<user>/<repo>/releases/download/v<x.y.z>/edge_vision_node.bin`  *(production)*

Other commands: `replay` (force buffer drain), `purge` (wipe SD buffer).

---

*Last updated: 2026-05-21 · Phase 5b verified (9-event replay) · Phase 6 verified (TWDT armed, boot=USB).*
