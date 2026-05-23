# Architecture

Deep-dive reference for engineers reading the source. The README sells the project; this document explains it.

## Table of contents

1. [Boot sequence](#1-boot-sequence)
2. [FreeRTOS task topology](#2-freertos-task-topology)
3. [Memory map](#3-memory-map)
4. [Camera + DMA pipeline](#4-camera--dma-pipeline)
5. [Inference subsystem](#5-inference-subsystem)
6. [Wi-Fi state machine](#6-wi-fi-state-machine)
7. [SNTP + system time](#7-sntp--system-time)
8. [MQTT-TLS session](#8-mqtt-tls-session)
9. [HTTP server](#9-http-server)
10. [SD-card event buffer](#10-sd-card-event-buffer)
11. [OTA state machine](#11-ota-state-machine)
12. [Watchdog + reset reason](#12-watchdog--reset-reason)
13. [Partition table rationale](#13-partition-table-rationale)
14. [Failure modes and recovery](#14-failure-modes-and-recovery)

---

## 1. Boot sequence

```
power on
  └─> 1st-stage bootloader (in ROM, immutable)
        └─> 2nd-stage bootloader (ESP-IDF, in flash @ 0x0)
              ├─> read otadata partition
              │   ├─ blank or invalid     → boot ota_0
              │   └─ valid: pick slot N
              ├─> verify app image SHA256
              ├─> if PENDING_VERIFY: mark NEXT boot to rollback unless confirmed
              └─> jump to app_main()
                    │
                    ├─> capture_reset_reason()
                    ├─> nvs_flash_init()
                    ├─> log_boot_info()
                    ├─> mark_image_valid_if_pending()   # cancel armed rollback
                    ├─> GPIO setup (LED)
                    ├─> sd_mount()                       # non-fatal
                    ├─> camera_bringup()                 # mandatory
                    ├─> inference_init()                 # mandatory
                    ├─> wifi_init_sta()                  # 30 s timeout, non-fatal
                    │     └─> sntp_sync_wait(15)
                    │           └─> mqtt_start()
                    ├─> mdns_start()
                    ├─> httpd_start_server()
                    ├─> wdt_init_and_subscribe()         # arm only after all up
                    └─> capture loop (forever)
```

Ordering matters:
- **`capture_reset_reason()` is first.** The reset-cause register self-clears on read; if any other subsystem queries it first, we lose the signal.
- **NVS before Wi-Fi.** Wi-Fi driver caches some config in NVS.
- **SD mount before camera.** The events file is opened first to count any orphaned events from a previous power cycle.
- **WDT armed last.** Subscribing the main task to TWDT before subsystems are up risks a spurious watchdog reset during slow init (TLS handshake, camera SCCB probe, etc.).

---

## 2. FreeRTOS task topology

| Task | Core | Priority | Stack | Source |
|---|---|---|---|---|
| `main_task` (capture loop) | 0 | 1 | 8 KB (default) | `app_main()` |
| Wi-Fi driver | 0 | 23 | 6.5 KB | esp-wifi (built-in) |
| esp-event default loop | 0 | 20 | 2.3 KB | esp-event |
| LWIP TCP/IP | 0 | 18 | 3 KB | lwip |
| `mqtt_task` | 0 | 5 | 6 KB | esp-mqtt |
| `httpd` | — | 5 | 8 KB | esp-http-server |
| `mdns` | — | 1 | 5 KB | mdns |
| `ota` (transient) | — | 5 | 8 KB | spawned per cmd |
| `replay` (transient) | — | 3 | 4 KB | spawned on (re)connect |
| `wifi_rc` esp_timer | — | high (system) | — | one-shot reconnect |
| Idle tasks | 0+1 | 0 | small | kernel |

The capture loop runs at priority 1 (just above idle) so that all networking and event handling can preempt it — keeps event latency low at the cost of throughput. Acceptable tradeoff: the 5 FPS we get is plenty for a ~1 Hz detection cadence.

Concurrency primitives in use:

- `s_wifi_evt` (EventGroup): synchronises `app_main` → Wi-Fi up
- `s_buffer_mutex` (Mutex): serialises producer (capture loop) and consumer (replay task) on `events.log`
- `s_wifi_reconnect_timer` (esp_timer one-shot): drives the exp-backoff reconnect
- `volatile` flags for cross-task booleans (`s_mqtt_connected`, `s_ota_in_progress`, etc.) — atomic on the Xtensa LX7

---

## 3. Memory map

### Flash (8 MB total)

See [§13](#13-partition-table-rationale).

### RAM at runtime (after all subsystems up)

| Region | Size | Used by |
|---|---|---|
| Internal SRAM (DRAM) | ~512 KB | FreeRTOS stacks, Wi-Fi RX/TX rings, mbedTLS scratch, MQTT buffer, lwIP buffers, file descriptors, .bss/.data |
| Internal IRAM | 16 KB | ISR vectors + IRAM_ATTR code |
| PSRAM (Octal, 80 MHz) | 8 MB | 3× 153 KB camera frame buffers (~460 KB), ESP-DL intermediate tensors (~2.5 MB peak), heap_caps SPIRAM pool (~5 MB free) |
| RTC fast RAM | 8 KB | Reserved (deep-sleep wake stub, etc.) |

PSRAM is mapped to virtual address `0x3C000000`–`0x3E000000` on the ESP32-S3. Frame buffer pointer-range check (`fb_in_psram` boolean) uses this window for cheap validation that DMA landed where we asked.

Internal SRAM is the scarce resource. Wi-Fi alone eats ~80 KB. TLS handshake adds 30–50 KB transient. The MQTT client another ~10 KB. We avoid putting anything large in internal SRAM — large allocations go to PSRAM via `MALLOC_CAP_SPIRAM`.

### Notable allocations

- Camera frame buffer: 320 × 240 × 2 B (RGB565) = **153,600 B**, in PSRAM, ×3 slots.
- ESP-DL `espdet_pico_224_224_face` weights: baked into firmware as `flash_rodata`, accessed via XIP (no RAM cost for weights; only for activations).
- Inference activations: peak ~1.5 MB, in PSRAM.
- MQTT buffer (default): 1 KB tx + 1 KB rx, in internal SRAM.
- HTTP server stack: 8 KB per connection, in internal SRAM.

---

## 4. Camera + DMA pipeline

### Sensor → buffer

```
OV3660 ──parallel 8-bit──> S3 CAM peripheral ──DMA──> PSRAM frame buffer
   ▲                              ▲
   │ SCCB (I²C variant)           │ XCLK 20 MHz from LEDC
   │ register R/W                 │ (drives sensor's internal PLL)
   └──────────────────────────────┘
```

- **XCLK = 20 MHz** delivered from LEDC peripheral on `CAM_PIN_XCLK` (GPIO 15). OV3660 internal PLL multiplies up to 50 MHz SYSCLK; PCLK out to the S3 is ~10 MHz. Lower XCLK (10 MHz) works if signal integrity is poor.
- **Parallel data: 8 lines** D0..D7. Camera component handles the bit-packing into RGB565 (2 bytes per pixel).
- **DMA destination: PSRAM frame buffer**. Allocated once at `esp_camera_init()`; recycled across captures.
- **fb_count = 3**, **grab_mode = LATEST**. Producer (DMA) cycles through the 3 slots. Consumers (capture loop, HTTP) get the most recent completed slot. Old frames are silently discarded.

### Pixel format

OV3660 cannot emit RGB888 directly (driver limitation). We use **RGB565 big-endian** (2 bytes per pixel, R in high nibble of byte 0, G across the byte boundary, B in byte 1). The ESP-DL model accepts RGB565BE via `dl::image::DL_IMAGE_PIX_TYPE_RGB565BE`.

If the model was given the wrong endianness, every pixel would have swapped R/B channels. The detector would silently produce zero detections. We diagnosed this once by `grep`ing the ESP-DL headers for the enum spellings.

### Why fb_count = 3, not 2

With two consumers (capture loop + HTTP `/jpg` handler) calling `esp_camera_fb_get()`, two buffers can be held simultaneously. With fb_count=2, the producer has zero free buffers and the next DMA frame must wait. With fb_count=3, the producer always has at least one free slot. Costs an extra 153 KB in PSRAM.

---

## 5. Inference subsystem

ESP-DL is C++. We expose a plain-C interface to the rest of the project to keep main.c in C.

```
main.c (C) ── inference_run_rgb888() ──► inference.cpp (C++)
                                              │
                                              ├── construct dl::image::img_t
                                              │   from camera_fb_t buffer
                                              │
                                              └── s_detector->run(img)
                                                       │
                                                       ▼
                                                ESP-DL pipeline:
                                                preprocess → quantise → forward →
                                                postprocess → NMS → list<result_t>
```

**Model:** `espdet_pico_224_224_face` (INT8 quantised, ~3 MB). Selected via Kconfig (`CONFIG_DEFAULT_HUMAN_FACE_DETECT_MODEL`).

**Input:** RGB565 BE, any resolution (model rescales internally to 224×224).

**Output:** `std::list<dl::detect::result_t>`. Each entry has:
- `box[4]`: bounding box in source-image pixel coordinates
- `score`: confidence ∈ [0, 1]
- `keypoint`: optional facial landmarks (unused here)

**Latency:** ~185 ms per inference at QVGA. Dominated by INT8 conv layers running on the S3's vector ISA.

**Per-frame overhead in our pipeline:**

| Step | Time |
|---|---|
| `esp_camera_fb_get()` | <1 ms (DMA already done) |
| `inference_run_rgb888()` | ~185 ms |
| `emit_detection()` rate-limit check | <0.1 ms |
| `esp_mqtt_client_publish()` (when triggered) | <10 ms (async, queues internally) |
| `esp_camera_fb_return()` | <0.1 ms |

Net ~5 FPS sustained.

**Why MSR+MNP didn't work for us:** the default tiny model (~50 ms inference) produced zero detections on bench faces. The 224×224 ESPDet Pico is ~3× slower but reliably hits 0.85+ confidence. For a security-camera use case, reliability beats throughput.

---

## 6. Wi-Fi state machine

```
       ┌──────────────────────────────────────────────────────┐
       │                                                      │
       ▼                                                      │
   STA_START ──connect()──> STA_CONNECTED ──> GOT_IP ──> RUN ─┤
       ▲                                                      │
       │                                                      ▼
       │                                              STA_DISCONNECTED
       │                                                      │
       │                                  s_wifi_ever_connected?
       │                                                      │
       │                       NO ◄────────┴────────► YES
       │                       │                       │
       │                  retry_count ≤ 10?    one-shot timer
       │                  ├ yes: connect()      with backoff
       │                  └ no: FAIL_BIT        500/1k/2k/4k/.../60k ms
       │                                              │
       └──────────────────────────────────────────────┘
```

`s_wifi_ever_connected` is a one-way latch. False until the first `GOT_IP`, true forever after. Distinguishes "initial association can fail; proceed offline" from "established session dropped; retry forever".

The exp-backoff `esp_timer` callback runs in the high-priority timer task, calls `esp_wifi_connect()`, returns. `esp_wifi_connect()` is non-blocking; it just signals the Wi-Fi driver, which then runs the association attempt in its own task.

---

## 7. SNTP + system time

```
RTC at power-on:   1970-01-01 00:00:00 UTC
                          │
                          ▼
                 wifi_init_sta()
                          │ (got IP)
                          ▼
                 sntp_sync_wait()
                          │
                  ├──> pool.ntp.org (primary)
                  └──> time.cloudflare.com (backup)
                          │
                          ▼ poll every 500 ms, up to 15 s
                  time(&now) > 1700000000  ← "is it post-2023?"
                          │
                          ▼
                  System time set, MQTT-TLS can start
```

**Why we wait:** TLS server-cert validation compares "now" against the certificate's NotBefore / NotAfter window. With RTC at epoch=0, every TLS handshake fails with `MBEDTLS_ERR_X509_CERT_VERIFY_FAILED` and `cert_verify_flags=0x10` (X509_BADCERT_EXPIRED — counterintuitively, "expired" because the cert's NotBefore is in our future).

**Failure mode:** if SNTP doesn't sync in 15 s, we proceed without MQTT. Camera + HTTP + inference still work. Telemetry resumes on the next reboot if NTP becomes reachable.

---

## 8. MQTT-TLS session

### Handshake (~4.2 s end-to-end from IP)

```
got IP                                          t = 0 ms
  │
  ├── SNTP sync                                t ≈ 2500 ms
  │
  ├── DNS resolve broker.hostname              t ≈ 2600 ms
  │
  ├── TCP connect :8883                        t ≈ 2700 ms
  │
  ├── TLS ClientHello                          t ≈ 2750 ms
  ├── ServerHello + Certificate                t ≈ 2900 ms
  ├── Validate cert against Mozilla CA bundle  t ≈ 2950 ms
  ├── Key exchange + ChangeCipherSpec          t ≈ 3450 ms
  ├── TLS established                          t ≈ 3500 ms
  │
  ├── MQTT CONNECT (with username/password)    t ≈ 3550 ms
  ├── MQTT CONNACK                             t ≈ 3900 ms
  │
  ├── SUBSCRIBE cmd topic                      t ≈ 3950 ms
  ├── SUBACK                                   t ≈ 4150 ms
  │
  └── PUBLISH {"online":true} retained         t ≈ 4200 ms
```

### Topic design rationale

Two topics for the device → broker direction, on purpose:

- `telemetry`: broad, low-rate. ~10 fields, every 5 s. Cloud subscribers usually route this to a time-series database.
- `detections`: narrow, event-driven. ~7 fields, ≤1 Hz, only when something happened. Routed to alerting (Slack, email, etc.).

Mixing them on one topic forces every consumer to filter by message shape. Separating them lets each consumer subscribe only to what they need.

### Last Will and Testament (LWT)

Configured at connect time:
- Topic: `edgecam/<id>/telemetry`
- Payload: `{"online":false}`
- QoS: 1, retained

On ungraceful disconnect (network drop, panic, brownout), the broker publishes this on our behalf. Subscribers see the device go offline within ~90 s (keepalive timeout × 1.5).

On clean connect, we publish a retained `{"online":true}` ourselves. New subscribers see current state immediately on `SUBACK`.

### QoS

Everything is QoS 1 (at-least-once). The broker retries until acked; the client deduplicates by packet ID. We don't use QoS 2 (exactly-once) because the broker still has to ack and the dedup is more expensive — overkill for "a face was seen at t=T".

---

## 9. HTTP server

```
Browser GET / ──> index_handler ──> static HTML with <img src="/jpg">
                                                          │
                                                          ▼
                                       GET /jpg ──> jpg_handler
                                                          │
                                                ┌─────────┴────────┐
                                                ▼                  ▼
                                  esp_camera_fb_get()       (RGB565 frame)
                                                ▼
                                  frame2jpg(fb, quality=80)
                                                ▼
                                  httpd_resp_send(image/jpeg, ...)
                                                ▼
                                  free(jpg_buf)
                                  esp_camera_fb_return(fb)
```

The HTML page polls `/jpg?t=<epoch>` at 1 Hz via setInterval. The query-string parameter is cache-busting; without it Chrome happily reuses the previous image.

`frame2jpg` is the esp32-camera helper that encodes a `camera_fb_t` to JPEG in software. ~50 ms at QVGA quality=80. It allocates the output buffer — caller must `free()` it after sending.

We do NOT serve `multipart/x-mixed-replace` MJPEG here (we did in Phase 3d, before inference was integrated). Reason: with the camera in RGB565 mode and inference running, the additional load of software-JPEG-encoding 5 FPS for every browser client would crater the inference rate. Snapshot-on-demand is the right tradeoff.

---

## 10. SD-card event buffer

### Producer / consumer flow

```
   capture loop                              replay task
        │                                         │
        ▼                                         ▼
   detection?                              MQTT just reconnected
        │                                         │
   mqtt up?                                       │
   ├─ yes ──> publish ()                          │
   └─ no  ──> take mutex                          │
              fopen append                        │
              fwrite line                         │
              fclose                              │
              s_buffered_events++                 │
              release mutex                       │
                                                  │
                                            take mutex
                                            fopen read
                                            for each line:
                                              publish ()
                                              vTaskDelay 50 ms
                                            fclose
                                            if mqtt still up:
                                              remove() file
                                              s_buffered_events = 0
                                            release mutex
                                            task exit
```

### Why a mutex, not a queue

A queue would work in memory but the events file is the actual persistent state. The mutex serialises file ops, not memory ops. FATFS is not internally re-entrant; opening the same file twice from two tasks corrupts directory entries.

### Why 20 Hz replay

Bursting hundreds of buffered events at wire-line rate (~500 publishes/sec) trips HiveMQ Cloud's rate limits and can fill the MQTT TX queue, leading to `MQTT_EVENT_ERROR: outbox full`. 20 Hz (50 ms between publishes) is comfortable.

### Why we don't truncate mid-replay

If MQTT drops while we're in the middle of replaying, we exit the loop but DON'T `remove()` the file. The remaining events live for next reconnect. The events we already published get duplicated — at-least-once is the QoS contract.

### Recovery across reboots

`sd_mount()` counts lines in `events.log` at boot. If nonzero, those events will replay on the next MQTT connect. Power loss in the middle of buffering is fine: FATFS flushes the directory on `fclose()`.

---

## 11. OTA state machine

```
              ┌───────────────────────────┐
              │  current slot = ota_X     │
              │  state = APP_VALID         │
              └───────────────┬───────────┘
                              │
                  MQTT cmd: "ota <url>"
                              │
                              ▼
              ┌───────────────────────────┐
              │  ota_task spawned          │
              │  s_ota_in_progress = true  │
              └───────────────┬───────────┘
                              │
                  esp_https_ota_begin(url)
                       │ TLS handshake
                       │ HTTP GET, parse headers, allocate slot
                              │
                              ▼
              esp_https_ota_perform() loop
                       │ chunked download
                       │ write to inactive slot
                       │ progress logged every 10%
                              │
                              ▼
              esp_https_ota_finish()
                       │ verify SHA256
                       │ otadata: new slot ← PENDING_VERIFY
                              │
                              ▼
                       esp_restart()
                              │
                      ┌───────┴────────┐
                      │ bootloader     │
                      │ reads otadata  │
                      │ picks new slot │
                      │ verifies image │
                      └───────┬────────┘
                              │
                              ▼
                       new app_main()
                              │
                  mark_image_valid_if_pending()
                  (after Wi-Fi + MQTT up and stable)
                              │
                              ▼
              ┌───────────────────────────┐
              │  new state = APP_VALID    │
              │  rollback armed for next  │
              │  boot is CANCELLED        │
              └───────────────────────────┘
```

### Rollback semantics

If the new image hangs the capture loop, TWDT panics → reboot. Bootloader sees the PENDING_VERIFY state was never cleared, picks the OTHER slot, and arms a permanent revert. The bad image can't brick the device.

We confirmed this end-to-end in bench testing: device upgraded from v0.5.0 to v0.5.1 cleanly, with `running partition: ota_1` after reboot.

### Why HTTP-allow is in sdkconfig.defaults

For LAN-only dev work it's convenient to host the binary on `python -m http.server`. The default IDF policy refuses non-HTTPS URLs. We enable HTTP via `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y` in `sdkconfig.defaults` so the choice is reviewable in source rather than hidden in menuconfig.

Production deployment would set `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=n` and serve binaries from GitHub Releases (HTTPS) or S3 with signed URLs.

---

## 12. Watchdog + reset reason

### TWDT (Task Watchdog Timer)

| Property | Value |
|---|---|
| Timeout | 30 seconds |
| Subscribed tasks | capture loop only |
| Trigger | panic + reset |
| Idle tasks watched? | No |

Why 30 s and not 5 s: an OTA download takes ~30 s during which the capture loop may stall on PSRAM contention. A 5 s TWDT would falsely trigger during legitimate OTA. We pay slightly slower recovery for false-positive immunity.

### Reset reason taxonomy

| `esp_reset_reason()` | What it means here |
|---|---|
| `POWERON` | Clean power-on |
| `EXT_PIN` | EN button pressed |
| `SW_REBOOT` | Our code called `esp_restart()` (cmd `reboot`, post-OTA, etc.) |
| `PANIC` | Unhandled CPU exception — bug in firmware |
| `TASK_WDT` | Capture loop stalled — bug or contention issue |
| `INT_WDT` | Interrupt watchdog — worse than TASK_WDT; usually means an ISR took too long |
| `BROWNOUT` | Brownout detector pulled reset — power supply marginal |
| `USB` | USB host commanded a reset (typically during flash) |
| `DEEPSLEEP_WAKE` | Asked for deep sleep, timer fired — N/A in this firmware |

Every reboot publishes its reason in the next telemetry payload. The cloud can see across thousands of devices what their reboot histograms look like and alert on anomalies.

---

## 13. Partition table rationale

```
+----------+------+----------+---------+
| label    | type | offset   | size    |
+----------+------+----------+---------+
| nvs      | data | 0x9000   |   24 KB |  Wi-Fi creds, app config
| otadata  | ota  | 0xf000   |    8 KB |  Which OTA slot is active
| phy_init | phy  | 0x11000  |    4 KB |  RF calibration
| ota_0    | app  | 0x20000  |  3.5 MB |  OTA slot A
| ota_1    | app  | 0x3a0000 |  3.5 MB |  OTA slot B
| storage  | fat  | 0x720000 |  896 KB |  Reserved (future SPIFFS use)
+----------+------+----------+---------+
total: 0x800000 = 8 MB
```

### Why no `factory` partition

With dual-OTA only, the bootloader picks `ota_0` when otadata is blank or invalid. Eliminates ~3 MB of dead weight from a never-updated factory image.

### Why 3.5 MB slots, not 3 MB

The embedded INT8 face-detect model is ~3 MB. App + camera driver + Wi-Fi + MQTT + HTTP + SD + OTA + mDNS + ESP-DL runtime grew the binary to ~3.2 MB. We resized OTA slots from 3 MB to 3.5 MB during Phase 4 to fit, taking the space from the model-only data partition (since the model is now embedded in the app, the dedicated model partition is redundant).

### Why 896 KB storage

Not currently used. Reserved for future SPIFFS use (e.g. signed config files, logs, captured event JPEGs). Padding to align ota_1 + storage to 8 MB.

---

## 14. Failure modes and recovery

| Failure | Detection | Recovery |
|---|---|---|
| Camera disconnect / FPC pop | `esp_camera_fb_get()` returns NULL | 200 ms backoff, retry forever (camera typically self-recovers) |
| Wi-Fi AP outage | `STA_DISCONNECTED` event | Exp-backoff reconnect, capped 60 s, forever |
| MQTT broker outage | `MQTT_EVENT_DISCONNECTED` | esp-mqtt internal reconnect every 5 s; meanwhile events buffer to SD |
| SNTP unreachable | `time() < 1700000000` after 15 s | Skip MQTT for this session; camera + HTTP still work |
| SD card not inserted | `sd_mount()` returns non-OK | Set `s_sd_mounted = false`; buffering becomes no-op |
| SD full | `fwrite()` short write | Log error, continue (future: rotate file) |
| Inference task hangs | TWDT timeout 30 s | Panic + reboot; bootloader rolls back if PENDING_VERIFY |
| TLS cert expiry / replacement | `cert_verify_flags` non-zero | Logged in MQTT_EVENT_ERROR; reboot won't help — needs firmware update with new CA bundle |
| OTA URL unreachable | `esp_https_ota_begin()` non-OK | Log error, exit task, `s_ota_in_progress = false`; device keeps running on current image |
| OTA partial download | `is_complete_data_received() == false` | `esp_https_ota_abort()`; current slot stays active |
| OTA written but bad image | New boot hangs / panics | TWDT/panic → reboot → bootloader sees PENDING_VERIFY, rolls back |
| Heap leak | Telemetry `heap_int_min` trends down over hours | Operator sees trend on dashboard, triggers OTA or reboot |
| Brownout | Brownout detector → reset | `boot reason = BROWNOUT` logged on next boot. If recurring, replace PSU. |

---

*Last updated for fw 0.7.0.*
