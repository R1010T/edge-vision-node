/* ============================================================================
 *  Edge-AI Vision Node — ESP32-S3
 *  Final firmware (Phase 7).
 *
 *  WHAT THIS FIRMWARE DOES
 *  -----------------------
 *  A small camera that thinks. Every captured frame is fed through a quantised
 *  INT8 neural network on-device (no cloud round-trip). When a face is found,
 *  a JSON event is published to an MQTT broker over TLS. Operators can:
 *      * watch a live HTTP snapshot from the device,
 *      * push new firmware to the device over Wi-Fi (OTA),
 *      * survive RF/network outages with zero event loss (SD-card buffering).
 *
 *  RUNTIME DATA FLOW
 *  -----------------
 *       +----------+    DMA       +-----------+         +--------------+
 *       |  OV3660  | -----------> | PSRAM frame buffer | -> | ESP-DL infer |
 *       |  sensor  |  RGB565 BE   |  (3 slots, latest) |   | INT8 model   |
 *       +----------+              +-----------+         +------+-------+
 *                                                              |
 *                                          detection result    v
 *                          +----------------+     online?     +----------+
 *                          |  MQTT (TLS)    | <-------------- |  router  |
 *                          +----------------+                 +-----+----+
 *                                                                   | offline
 *                                                                   v
 *                                                       +-----------------------+
 *                                                       |  FATFS on microSD     |
 *                                                       |  /sdcard/events.log   |
 *                                                       +-----------------------+
 *
 *  CORE / TASK TOPOLOGY
 *  --------------------
 *  FreeRTOS, dual-core, 1 kHz tick, both cores enabled.
 *      main / capture loop  -- CPU0   capture frame, run inference, emit event
 *      Wi-Fi driver task    -- CPU0   handled by IDF, prio 23
 *      MQTT task            -- CPU0   handled by IDF
 *      OTA task             -- spawned per cmd, prio 5, 8K stack
 *      Replay task          -- spawned on reconnect, prio 3, 4K stack
 *      HTTPD task           -- handled by IDF
 *      Idle tasks (×2)      -- not under TWDT
 *
 *  RESOURCE BUDGET (measured, steady state with TLS + inference)
 *  -------------------------------------------------------------
 *      Internal heap free : ~120 KB   (after Wi-Fi, MQTT-TLS, esp-dl, sdmmc)
 *      PSRAM   heap free  : ~7.5 MB   (most consumed by 3× 153 KB frame buffers)
 *      Pipeline FPS       : ~5
 *      Inference time     : ~185 ms / frame   (espdet_pico_224_224_face INT8)
 *      End-to-end alert   : <250 ms target
 *
 *  BUILD & FLASH (Windows + VS Code + ESP-IDF v5.3.2)
 *  --------------------------------------------------
 *      idf.py set-target esp32s3
 *      idf.py build
 *      idf.py -p COM4 flash monitor      (USB-Serial-JTAG on USB OTG port)
 *      Ctrl+] to exit monitor.
 *
 *  REQUIRED HARDWARE
 *  -----------------
 *      Freenove ESP32-S3-WROOM dev board (N8R8 = 8MB flash + 8MB OPI PSRAM)
 *      OV2640 or OV3660 camera module on the on-board FPC connector
 *      microSD card (any size, FAT-formatted)
 *      microUSB or USB-C cable to USB OTG port (silk: "USB", not "USB UART")
 *
 *  ASSOCIATED FILES IN THIS REPO
 *  -----------------------------
 *      partitions.csv    Custom 8 MB layout: 2× 3.5 MB OTA slots + SPIFFS
 *      sdkconfig.defaults Octal PSRAM @ 80 MHz, CPU 240 MHz, 1 kHz tick, ...
 *      camera_pins.h     OV-sensor pin map for the Freenove board
 *      inference.cpp/.h  C++ wrapper around ESP-DL HumanFaceDetect
 *      secrets.h         Wi-Fi + MQTT credentials (gitignored)
 *      idf_component.yml Component-manager dependencies (esp-dl, mdns, etc.)
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

/*-- FreeRTOS ----------------------------------------------------------------
 * FreeRTOS is the small real-time kernel that runs everything on this chip.
 * "Task" = thread of execution. "Queue", "EventGroup", "Semaphore" = the
 * primitives we use to pass data between tasks without races. We never call
 * standard pthreads here; that's all FreeRTOS-flavoured.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

/*-- ESP-IDF system layer ----------------------------------------------------
 * These headers are vendor-supplied (Espressif's "IoT Development Framework"
 * v5.3.2) and wrap the underlying ROM and silicon drivers. ESP_LOG family is
 * the standard logging macro set; everything written with ESP_LOGI/W/E is
 * piped over USB-Serial-JTAG by default.
 */
#include "esp_log.h"
#include "esp_system.h"        /* esp_reset_reason()                       */
#include "esp_task_wdt.h"      /* task watchdog timer                      */
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"     /* heap_caps_get_free_size / minimum_free_size */
#include "esp_ota_ops.h"       /* read running partition, mark valid       */
#include "esp_app_desc.h"      /* esp_app_get_description() version blob   */
#include "esp_timer.h"         /* monotonic microsecond clock + one-shots  */
#include "esp_event.h"         /* esp_event_loop_create_default            */
#include "esp_netif.h"         /* TCP/IP integration layer                 */
#include "esp_wifi.h"
#include "esp_mac.h"           /* read station MAC for device-id           */
#include "esp_sntp.h"          /* network time sync (required for TLS)     */
#include "esp_crt_bundle.h"    /* baked-in Mozilla CA bundle for TLS verify */
#include "esp_http_server.h"   /* HTTP server (snapshot endpoint)          */
#include "esp_http_client.h"
#include "esp_https_ota.h"     /* one-call OTA helper                      */
#include "esp_vfs_fat.h"       /* FATFS mount over SDMMC                   */
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "mqtt_client.h"       /* esp-mqtt                                 */
#include "mdns.h"              /* mDNS (Bonjour-compatible local discovery)*/
#include "driver/gpio.h"
#include "nvs_flash.h"         /* key-value store in NOR flash (creds, etc)*/
#include "esp_camera.h"        /* esp32-camera driver                      */

/*-- Project-local -----------------------------------------------------------*/
#include "camera_pins.h"       /* OV2640/OV3660 pin map for Freenove board */
#include "secrets.h"           /* WIFI_SSID/PASS + MQTT_BROKER_URI/etc.    */
#include "inference.h"         /* C interface to ESP-DL HumanFaceDetect    */

/* ===========================================================================
 *  Tunable constants
 * ==========================================================================*/

#define LED_GPIO        GPIO_NUM_2          /* on-board indicator LED       */
#define TAG             "main"              /* ESP_LOG tag for this TU      */

/* Event group bits exchanged between the Wi-Fi event handler (an ISR-ish
 * callback) and app_main, which blocks on them at boot. Bits live in a
 * single 32-bit word manipulated atomically by the kernel. */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* MQTT detection-publishing rate cap. We don't want to flood the broker
 * (or the SD card buffer) when a face is in frame for ten seconds straight. */
#define DET_PUBLISH_MIN_INTERVAL_MS  1000

/* Below this score, suppress the event entirely. The model still ran (we paid
 * the CPU); we just don't talk about low-confidence hits. */
#define DET_SCORE_THRESHOLD          0.55f

/* SD card mount + pin map (1-bit SDMMC mode — minimum pins, max compatibility).
 * Three pins consumed: CMD, CLK, D0. Other six SDMMC pins (D1..D3, etc.) are
 * left free for future use. Pins picked from the Freenove silkscreen. */
#define SD_MOUNT_POINT  "/sdcard"
#define EVENTS_FILE     "/sdcard/events.log"
#define SD_PIN_CMD      GPIO_NUM_38
#define SD_PIN_CLK      GPIO_NUM_39
#define SD_PIN_D0       GPIO_NUM_40

/* Task watchdog: if the capture loop fails to feed within this window, the
 * kernel panics and reboots the chip. 30 s is a long pause; we set it long
 * because OTA download takes ~30 s under load. Tightening to 5 s would also
 * work but would trigger spurious reboots during OTA. */
#define TWDT_TIMEOUT_S  30

/* ===========================================================================
 *  Module-static state
 *
 *  These are all `static` (file-private) because nothing outside main.c
 *  should touch them. The `volatile` qualifier on the booleans tells the
 *  compiler "this might be modified by an ISR or another task; do not cache
 *  it in a register across reads". Without volatile, the compiler can do
 *  aggressive optimisations that break concurrency.
 * ==========================================================================*/

/* Wi-Fi state ------------------------------------------------------------- */
static EventGroupHandle_t s_wifi_evt;             /* signalled by event handler */
static int                s_retry_count = 0;      /* exp-backoff attempt #     */
static volatile bool      s_wifi_ever_connected = false;  /* first IP latched */
static esp_timer_handle_t s_wifi_reconnect_timer = NULL;  /* one-shot timer    */

/* MQTT state -------------------------------------------------------------- */
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool      s_mqtt_connected = false;

/* Per-device identity. Derived from the station MAC address tail so every
 * board has a unique topic without flashing per-unit configuration. */
static char       s_device_id[16] = {0};         /* "abcdef"                  */
static char       s_topic_tel[64] = {0};         /* edgecam/<id>/telemetry    */
static char       s_topic_cmd[64] = {0};         /* edgecam/<id>/cmd          */
static char       s_topic_det[64] = {0};         /* edgecam/<id>/detections   */
static char       s_lwt_payload[] = "{\"online\":false}";  /* Last-Will       */
static char       s_mdns_host[32] = {0};         /* edgecam-<id>              */

/* Firmware version string. The user bumps this for every release. Surfaced
 * in telemetry so the cloud can confirm an OTA succeeded by reading the
 * version on the next published packet. Avoids "did the update actually
 * land?" guesswork. */
static const char s_fw_ver[] = "0.7.0";

/* HTTP server + bookkeeping --------------------------------------------- */
static httpd_handle_t     s_httpd = NULL;
static int64_t            s_last_det_pub_us = 0;
static volatile uint32_t  s_total_dets_published = 0;
static volatile bool      s_ota_in_progress = false;

/* SD card + buffering ---------------------------------------------------- */
static sdmmc_card_t       *s_sd_card = NULL;
static volatile bool      s_sd_mounted = false;
static SemaphoreHandle_t  s_buffer_mutex = NULL;  /* protects EVENTS_FILE   */
static volatile uint32_t  s_buffered_events = 0;
static volatile uint32_t  s_total_replayed = 0;

/* Boot postmortem -------------------------------------------------------- */
static esp_reset_reason_t s_boot_reason;
static const char        *s_boot_reason_name = "?";

/* ===========================================================================
 *  Camera configuration
 *
 *  The esp32-camera driver wants a single struct telling it which sensor
 *  control pins map to which GPIOs, what pixel format to deliver, and how
 *  many frame buffers to allocate. All values are compile-time constants.
 *
 *  Notable choices:
 *      pixel_format = RGB565        Model wants raw pixels; OV3660 cannot
 *                                   emit RGB888 (driver bug / not supported).
 *      frame_size   = QVGA (320×240) Small enough to fit comfortably in PSRAM,
 *                                   large enough for the model.
 *      fb_count     = 3              Multiple readers (capture loop + HTTP
 *                                   snapshot endpoint) compete for buffers.
 *                                   With only 2 buffers any pair contends on
 *                                   every frame; 3 gives the producer one
 *                                   free slot at all times.
 *      grab_mode    = LATEST         Drop old frames on producer overrun. We
 *                                   want the freshest frame for inference,
 *                                   not a stale one from a queue.
 *      fb_location  = PSRAM          Internal SRAM (~512 KB) is too tight
 *                                   for QVGA RGB565 (153 KB × 3 = 460 KB).
 * ==========================================================================*/
static const camera_config_t s_cam_cfg = {
    .pin_pwdn   = CAM_PIN_PWDN, .pin_reset = CAM_PIN_RESET,
    .pin_xclk   = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD, .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,                  /* 20 MHz, OV's preferred input */
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 0,                          /* unused for RGB565          */
    .fb_count     = 3,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
};

/* ===========================================================================
 *  Boot reset-reason capture
 *
 *  Called as the very first thing in app_main(). esp_reset_reason() reads
 *  the SoC's reset-cause register and returns an enum we can decode for the
 *  log. It's important to do this FIRST, before any subsystem can set its
 *  own reset cause (which it can't — these bits are sticky from before main
 *  ran — but the discipline is good).
 *
 *  Why we care:
 *      POWERON        - clean cold start
 *      EXT_PIN        - someone hit the EN reset button
 *      SW_REBOOT      - we called esp_restart() ourselves (OTA, cmd "reboot")
 *      PANIC          - unhandled exception
 *      TASK_WDT       - capture loop hung (or any TWDT-subscribed task did)
 *      INT_WDT        - interrupt watchdog fired (worse than task WDT)
 *      BROWNOUT       - power supply sagged; brownout detector pulled reset
 *      USB            - a USB host commanded a reset (typically during flash)
 *      DEEPSLEEP_WAKE - we asked for deep sleep and the timer fired
 *
 *  Field meaning: PANIC and any *_WDT mean SOMETHING IS WRONG. They become
 *  the most valuable telemetry the device ever produces.
 * ==========================================================================*/
static void capture_reset_reason(void)
{
    s_boot_reason = esp_reset_reason();
    switch (s_boot_reason) {
    case ESP_RST_POWERON:   s_boot_reason_name = "POWERON";        break;
    case ESP_RST_EXT:       s_boot_reason_name = "EXT_PIN";        break;
    case ESP_RST_SW:        s_boot_reason_name = "SW_REBOOT";      break;
    case ESP_RST_PANIC:     s_boot_reason_name = "PANIC";          break;
    case ESP_RST_INT_WDT:   s_boot_reason_name = "INT_WDT";        break;
    case ESP_RST_TASK_WDT:  s_boot_reason_name = "TASK_WDT";       break;
    case ESP_RST_WDT:       s_boot_reason_name = "OTHER_WDT";      break;
    case ESP_RST_DEEPSLEEP: s_boot_reason_name = "DEEPSLEEP_WAKE"; break;
    case ESP_RST_BROWNOUT:  s_boot_reason_name = "BROWNOUT";       break;
    case ESP_RST_SDIO:      s_boot_reason_name = "SDIO";           break;
    case ESP_RST_USB:       s_boot_reason_name = "USB";            break;
    default:                s_boot_reason_name = "UNKNOWN";        break;
    }
    /* Logged at WARN so it stands out in the firehose. */
    ESP_LOGW(TAG, "boot reason: %s (code %d)", s_boot_reason_name, (int)s_boot_reason);
}

/* ===========================================================================
 *  Task Watchdog Timer (TWDT)
 *
 *  Each subscribed task must call esp_task_wdt_reset() within the configured
 *  timeout, or the SoC panics and reboots. We subscribe the capture loop and
 *  no one else; the idle tasks are intentionally NOT under TWDT here because
 *  on a dual-core S3 the idle task on each core can legitimately sleep for
 *  long periods.
 *
 *  Composability with OTA: if the new firmware (just installed via OTA) hangs
 *  the capture loop, TWDT panics, bootloader sees the PENDING_VERIFY image
 *  never got marked valid, and rolls back to the previous slot. Result: a
 *  bad OTA cannot brick the device.
 * ==========================================================================*/
static void wdt_init_and_subscribe(void)
{
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = TWDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    /* IDF may already have initialised TWDT for its own bookkeeping. Try
     * reconfigure first; if not yet initialised, init. Either is fine. */
    esp_err_t err = esp_task_wdt_reconfigure(&cfg);
    if (err == ESP_ERR_INVALID_STATE) err = esp_task_wdt_init(&cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "TWDT init: %s", esp_err_to_name(err)); return; }

    if (esp_task_wdt_add(NULL) == ESP_OK)
        ESP_LOGI(TAG, "TWDT armed: capture loop must feed within %d s", TWDT_TIMEOUT_S);
    else
        ESP_LOGE(TAG, "TWDT subscribe failed");
}

/* ===========================================================================
 *  Wi-Fi: exponential-backoff reconnect
 *
 *  Naïve reconnect ("just call esp_wifi_connect() in a loop") burns CPU and
 *  drowns the AP in association attempts when the network has actually died.
 *  Industry standard is exponential backoff: 0.5 s, then 1 s, 2 s, 4 s, …
 *  up to some cap (here 60 s), forever.
 *
 *  Implementation note: the Wi-Fi event handler runs in the event-loop task,
 *  which must never block. So we can't sleep there. Instead we arm a one-shot
 *  `esp_timer` which calls esp_wifi_connect() from its own task context.
 *
 *  We distinguish initial connect (limited retries, then proceed offline)
 *  from established-session reconnect (forever-retry). This prevents the
 *  device from sitting at the boot screen for hours just because the AP is
 *  out of range right now.
 * ==========================================================================*/

static void wifi_reconnect_cb(void *arg)
{
    ESP_LOGI(TAG, "wifi: reconnect attempt #%d", s_retry_count);
    esp_wifi_connect();
}

static int wifi_backoff_ms(int attempt)
{
    /* 500 ms × 2^(attempt-1), capped. attempt is 1-based.
     * Sequence:  1=500  2=1000  3=2000  4=4000  5=8000  6=16000  7=32000  >=8=60000 */
    int shift = attempt - 1;
    if (shift < 0) shift = 0;
    if (shift > 7) shift = 7;
    int ms = 500 << shift;
    if (ms > 60000) ms = 60000;
    return ms;
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        /* Driver just powered up the radio; ask it to associate. */
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retry_count++;
        if (!s_wifi_ever_connected) {
            /* Initial connect path. We don't want the device to be stuck
             * here forever — the camera should still run offline. So we
             * cap initial attempts at WIFI_MAX_RETRY and then signal FAIL. */
            if (s_retry_count <= WIFI_MAX_RETRY) {
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "wifi: initial connect failed after %d attempts",
                         WIFI_MAX_RETRY);
                xEventGroupSetBits(s_wifi_evt, WIFI_FAIL_BIT);
            }
        } else {
            /* We've connected before; this is a runtime drop. Retry forever
             * with exponential backoff. */
            int delay_ms = wifi_backoff_ms(s_retry_count);
            ESP_LOGW(TAG, "wifi: link lost, reconnect in %d ms (attempt #%d)",
                     delay_ms, s_retry_count);
            esp_timer_stop(s_wifi_reconnect_timer);
            esp_timer_start_once(s_wifi_reconnect_timer, (uint64_t)delay_ms * 1000);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "wifi: got IP " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_count = 0;
        s_wifi_ever_connected = true;
        xEventGroupSetBits(s_wifi_evt, WIFI_CONNECTED_BIT);
    }
}

/* Brings up the Wi-Fi driver as a station, registers event callbacks, and
 * blocks up to 30 s waiting for IP_EVENT_STA_GOT_IP. Returns ESP_OK if we
 * associated and got DHCP, ESP_FAIL otherwise. */
static esp_err_t wifi_init_sta(void)
{
    s_wifi_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());                  /* TCP/IP stack       */
    ESP_ERROR_CHECK(esp_event_loop_create_default());   /* default event task */
    esp_netif_create_default_wifi_sta();                /* lwIP <-> Wi-Fi STA */

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register one callback for any Wi-Fi event, and one for got-IP. */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    /* Reusable one-shot for the exp-backoff reconnect. */
    esp_timer_create_args_t timer_args = {
        .callback = wifi_reconnect_cb, .name = "wifi_rc",
    };
    esp_timer_create(&timer_args, &s_wifi_reconnect_timer);

    wifi_config_t wc = {
        .sta = {
            .ssid = WIFI_SSID, .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            /* PMF (802.11w): supported but not required. Works on WPA2 and
             * WPA3 routers. Required-mode would break older WPA2-only APs. */
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_evt, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,    /* don't clear bits on exit */
        pdFALSE,    /* wait for ANY of the bits (OR semantics) */
        pdMS_TO_TICKS(30000));
    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

/* ===========================================================================
 *  SNTP — Network time sync
 *
 *  Why this exists: TLS server-cert validation compares "now" against the
 *  certificate's NotBefore / NotAfter timestamps. After power-on the SoC
 *  thinks it's 1970-01-01 (epoch=0); EVERY TLS handshake would fail with a
 *  "certificate not yet valid" error.
 *
 *  We point at pool.ntp.org (anycast, hundreds of servers worldwide) with
 *  Cloudflare time as a backup. Sync is polled, not interrupt-driven —
 *  doesn't need precision better than a few seconds.
 *
 *  Note: ts > 1700000000 means "after Nov 2023" — basically "anything sane".
 *  We use this rather than checking for a non-zero year because RTC defaults
 *  vary between SoC silicon revisions and we don't want to chase that.
 * ==========================================================================*/
static bool sntp_sync_wait(int max_seconds)
{
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.cloudflare.com");
    esp_sntp_init();
    for (int i = 0; i < max_seconds * 2; i++) {
        time_t now = 0; time(&now);
        if (now > 1700000000) {
            ESP_LOGI(TAG, "sntp: synced ts=%lld", (long long)now);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

/* ===========================================================================
 *  SD card — FATFS over SDMMC, single-bit mode
 *
 *  We use 1-bit SDMMC (CMD, CLK, D0) instead of 4-bit because:
 *    (a) it uses fewer pins (frees D1..D3 for future use),
 *    (b) at 20 MHz clock it's plenty fast for ~200 B JSON events,
 *    (c) 1-bit mode is less sensitive to wiring quality (no skew between
 *        data lines), which matters on cheap dev boards.
 *
 *  format_if_mount_failed=false because we'd rather know a brand-new card
 *  wasn't formatted than silently destroy its contents. The user is
 *  expected to FAT-format the card on their PC.
 *
 *  s_buffer_mutex serialises the file between two tasks:
 *      Producer: capture loop appends one line per detection (when offline).
 *      Consumer: replay_task reads the file line-by-line and publishes each.
 *  Without a mutex these two could interleave fopen/fwrite/fclose calls,
 *  corrupting FAT metadata.
 * ==========================================================================*/
static esp_err_t sd_mount(void)
{
    s_buffer_mutex = xSemaphoreCreateMutex();
    if (!s_buffer_mutex) return ESP_ERR_NO_MEM;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;  /* 20 MHz; HIGHSPEED is finicky */

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = SD_PIN_CLK;
    slot.cmd   = SD_PIN_CMD;
    slot.d0    = SD_PIN_D0;
    slot.width = 1;                          /* 1-bit data, simpler routing  */
    slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot,
                                            &mount_config, &s_sd_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sd: mount failed (%s) — buffering disabled",
                 esp_err_to_name(err));
        return err;
    }
    s_sd_mounted = true;
    ESP_LOGI(TAG, "sd: mounted at %s", SD_MOUNT_POINT);

    /* On boot, count any events left over from a previous session that
     * didn't get a chance to drain. They'll replay on next MQTT connect. */
    FILE *f = fopen(EVENTS_FILE, "r");
    if (f) {
        char line[512];
        while (fgets(line, sizeof(line), f)) s_buffered_events++;
        fclose(f);
        if (s_buffered_events)
            ESP_LOGW(TAG, "sd: %lu events from previous session",
                     (unsigned long)s_buffered_events);
    }
    return ESP_OK;
}

static void buffer_event_to_sd(const char *json, int len)
{
    if (!s_sd_mounted) return;
    /* Take the mutex; this might block if the replay task is also working,
     * but FATFS calls are not re-entrant. */
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
    FILE *f = fopen(EVENTS_FILE, "a");
    if (f) {
        fwrite(json, 1, len, f);
        fputc('\n', f);
        fclose(f);
        s_buffered_events++;
    } else {
        ESP_LOGE(TAG, "sd: fopen append failed");
    }
    xSemaphoreGive(s_buffer_mutex);
}

/* Replay task. Created on MQTT (re)connect when s_buffered_events > 0.
 * Reads the file line-by-line, publishes each as a detection event at
 * 20 Hz, then deletes the file and exits. Throttled because publishing
 * thousands of buffered events at the wire-line rate could choke the
 * broker (and trip its rate limits). */
static void replay_task(void *arg)
{
    if (!s_sd_mounted) { vTaskDelete(NULL); return; }
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);

    FILE *f = fopen(EVENTS_FILE, "r");
    if (!f) {
        xSemaphoreGive(s_buffer_mutex);
        vTaskDelete(NULL);
        return;
    }
    char line[512]; int n_replayed = 0;
    while (fgets(line, sizeof(line), f) && s_mqtt_connected) {
        int len = strlen(line);
        /* fgets keeps the newline; strip it before publishing. */
        if (len > 0 && line[len - 1] == '\n') line[--len] = '\0';
        if (len == 0) continue;
        esp_mqtt_client_publish(s_mqtt, s_topic_det, line, len, 1, 0);
        n_replayed++;
        vTaskDelay(pdMS_TO_TICKS(50));  /* 20 Hz cap */
    }
    fclose(f);

    /* Only purge if the whole file made it through. If MQTT dropped mid-
     * replay, keep the file so next reconnect picks up where we left off. */
    if (s_mqtt_connected) {
        remove(EVENTS_FILE);
        s_buffered_events = 0;
        s_total_replayed += n_replayed;
        ESP_LOGW(TAG, "sd: replayed %d events, file purged", n_replayed);
    } else {
        ESP_LOGW(TAG, "sd: mqtt dropped mid-replay, %d sent, file kept", n_replayed);
    }
    xSemaphoreGive(s_buffer_mutex);
    vTaskDelete(NULL);
}

/* ===========================================================================
 *  OTA: download new firmware over HTTP(S), swap slots, reboot
 *
 *  esp_https_ota_perform() is the IDF helper that does the heavy lifting:
 *  TLS handshake, certificate validation via crt_bundle_attach, chunked
 *  download, append to inactive OTA slot, SHA256 verification at the end.
 *
 *  We run it from a dedicated task because the download takes ~30 s under
 *  load — we don't want to block the MQTT event handler that triggered it
 *  (event handlers must return quickly).
 *
 *  Post-OTA, we esp_restart(). The bootloader picks the just-written slot
 *  and marks it PENDING_VERIFY. On boot of the new image, mark_image_valid
 *  _if_pending() flips it to VALID after we've reached steady state. If we
 *  never reach that point (panic, WDT, brownout), the bootloader will roll
 *  back on the NEXT power cycle.
 * ==========================================================================*/
static void ota_task(void *arg)
{
    char *url = (char *)arg;
    s_ota_in_progress = true;
    ESP_LOGW(TAG, "OTA: starting from %s", url);

    esp_http_client_config_t http_cfg = {
        .url               = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
        .timeout_ms        = 15000,
    };
    esp_https_ota_config_t ota_cfg = { .http_config = &http_cfg };

    esp_https_ota_handle_t handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin: %s", esp_err_to_name(err));
        goto cleanup;
    }
    int total = esp_https_ota_get_image_size(handle);
    int last_log = -10;
    while ((err = esp_https_ota_perform(handle)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        int got = esp_https_ota_get_image_len_read(handle);
        if (total > 0) {
            int pct = (got * 100) / total;
            if (pct >= last_log + 10) {
                ESP_LOGI(TAG, "OTA: %d%%", pct);
                last_log = pct;
            }
        }
    }
    if (!esp_https_ota_is_complete_data_received(handle)) {
        esp_https_ota_abort(handle);
        goto cleanup;
    }
    if (esp_https_ota_finish(handle) == ESP_OK) {
        ESP_LOGW(TAG, "OTA: success, reboot in 2 s");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();   /* <— never returns */
    }
cleanup:
    free(url);            /* allocated by handle_cmd, ownership transferred  */
    s_ota_in_progress = false;
    vTaskDelete(NULL);
}

/* ===========================================================================
 *  Command dispatcher
 *
 *  The cmd topic accepts ASCII commands followed by optional args. Tiny
 *  hand-rolled parser; no JSON to keep memory tight and parsing trivial.
 *
 *  Supported:
 *      "ota <url>"   start OTA from URL
 *      "replay"      force drain of the SD buffer
 *      "purge"       wipe the SD buffer without publishing
 *      "reboot"      software reset (boot reason will be SW_REBOOT next boot)
 * ==========================================================================*/
static void handle_cmd(const char *payload, int len)
{
    if (len >= 4 && strncmp(payload, "ota ", 4) == 0) {
        if (s_ota_in_progress) return;
        /* Caller's buffer is not null-terminated; allocate a private copy
         * for the OTA task that owns the URL for the lifetime of the task. */
        char *url = malloc(len - 4 + 1);
        if (!url) return;
        memcpy(url, payload + 4, len - 4);
        url[len - 4] = '\0';
        if (xTaskCreate(ota_task, "ota", 8192, url, 5, NULL) != pdPASS)
            free(url);
    } else if (len >= 6 && strncmp(payload, "replay", 6) == 0) {
        xTaskCreate(replay_task, "replay", 4096, NULL, 3, NULL);
    } else if (len >= 5 && strncmp(payload, "purge", 5) == 0) {
        if (s_sd_mounted) {
            xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
            remove(EVENTS_FILE);
            s_buffered_events = 0;
            xSemaphoreGive(s_buffer_mutex);
            ESP_LOGW(TAG, "cmd: SD buffer purged");
        }
    } else if (len >= 6 && strncmp(payload, "reboot", 6) == 0) {
        ESP_LOGW(TAG, "cmd: reboot requested");
        vTaskDelay(pdMS_TO_TICKS(500));   /* give the log a chance to flush */
        esp_restart();
    }
}

/* ===========================================================================
 *  MQTT-TLS client
 *
 *  esp-mqtt is the IDF helper. It runs in its own task; we just register
 *  an event handler. The session uses:
 *      - Mozilla CA bundle for server cert validation (crt_bundle_attach)
 *      - keepalive 60 s   (broker drops us after 1.5× = 90 s of silence)
 *      - LWT retained     (on dirty disconnect, subscribers learn we're down)
 *      - QoS 1 throughout (at-least-once; survives single network blip)
 * ==========================================================================*/
static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "mqtt-tls: CONNECTED");
        s_mqtt_connected = true;
        esp_mqtt_client_subscribe(s_mqtt, s_topic_cmd, 1);
        /* Retained "online":true on the telemetry topic. Subscribers see
         * device state immediately on subscribe — no waiting for the next
         * publish cycle. */
        esp_mqtt_client_publish(s_mqtt, s_topic_tel, "{\"online\":true}", 0, 1, 1);
        /* Drain anything that piled up while we were offline. */
        if (s_sd_mounted && s_buffered_events > 0) {
            ESP_LOGW(TAG, "spawning replay for %lu events",
                     (unsigned long)s_buffered_events);
            xTaskCreate(replay_task, "replay", 4096, NULL, 3, NULL);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "mqtt cmd in: %.*s = %.*s",
                 e->topic_len, e->topic, e->data_len, e->data);
        handle_cmd(e->data, e->data_len);
        break;

    default:
        break;
    }
}

static void mqtt_start(void)
{
    /* Derive a unique-per-device id from the bottom 3 bytes of the MAC. */
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id), "%02x%02x%02x",
             mac[3], mac[4], mac[5]);
    snprintf(s_topic_tel, sizeof(s_topic_tel), "edgecam/%s/telemetry",  s_device_id);
    snprintf(s_topic_cmd, sizeof(s_topic_cmd), "edgecam/%s/cmd",        s_device_id);
    snprintf(s_topic_det, sizeof(s_topic_det), "edgecam/%s/detections", s_device_id);
    snprintf(s_mdns_host, sizeof(s_mdns_host), "edgecam-%s", s_device_id);
    ESP_LOGI(TAG, "device_id=%s fw=%s", s_device_id, s_fw_ver);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri                    = MQTT_BROKER_URI,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.client_id                 = s_device_id,
        .credentials.username                  = MQTT_USERNAME,
        .credentials.authentication.password   = MQTT_PASSWORD,
        .session.keepalive                     = 60,
        .session.last_will.topic               = s_topic_tel,
        .session.last_will.msg                 = s_lwt_payload,
        .session.last_will.msg_len             = sizeof(s_lwt_payload) - 1,
        .session.last_will.qos                 = 1,
        .session.last_will.retain              = 1,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt);
}

/* Telemetry — periodic device-state JSON, published every 5 s.
 * "Broad" channel: lots of fields, low rate. Cloud subscribers chart this
 * over time to spot trends (leak, FPS drop, retry storms). */
static void mqtt_publish_telemetry(float fps, uint32_t frame_idx,
                                   int64_t avg_infer_us)
{
    if (!s_mqtt_connected) return;
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%llu,\"fw\":\"%s\",\"boot\":\"%s\","
        "\"fps\":%.2f,\"frame\":%lu,\"avg_infer_us\":%lld,"
        "\"dets_pub\":%lu,\"buffered\":%lu,\"replayed\":%lu,\"sd\":%d,"
        "\"heap_int\":%u,\"heap_int_min\":%u,"
        "\"heap_psram\":%u,\"heap_psram_min\":%u,"
        "\"ota\":%d,\"wifi_retries\":%d}",
        (unsigned long long)(esp_timer_get_time()/1000ULL), s_fw_ver, s_boot_reason_name,
        fps, (unsigned long)frame_idx, (long long)avg_infer_us,
        (unsigned long)s_total_dets_published,
        (unsigned long)s_buffered_events,
        (unsigned long)s_total_replayed,
        s_sd_mounted ? 1 : 0,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
        s_ota_in_progress ? 1 : 0,
        s_retry_count);
    if (n > 0 && n < (int)sizeof(buf))
        esp_mqtt_client_publish(s_mqtt, s_topic_tel, buf, n, 1, 0);
}

/* Detection event — narrow channel, rate-limited.
 * Either publishes immediately (online) or appends to SD (offline).
 * Producer-side rate limit (DET_PUBLISH_MIN_INTERVAL_MS) gives the broker
 * bounded load even if the model fires 30× a second. */
static void emit_detection(const inference_result_t *res, uint32_t frame_idx)
{
    if (res->n_detections <= 0) return;
    int64_t now = esp_timer_get_time();
    if (now - s_last_det_pub_us < (int64_t)DET_PUBLISH_MIN_INTERVAL_MS * 1000) return;

    /* Pick the best detection. Multi-face frames produce one event per
     * pulse, not many; cleaner for the consumer. */
    int best_idx = 0; float best_score = res->dets[0].score;
    for (int i = 1; i < res->n_detections; i++)
        if (res->dets[i].score > best_score) {
            best_score = res->dets[i].score; best_idx = i;
        }
    if (best_score < DET_SCORE_THRESHOLD) return;

    time_t wall = 0; time(&wall);
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%lld,\"frame\":%lu,\"score\":%.3f,"
        "\"box\":[%d,%d,%d,%d],\"infer_us\":%lld,\"fw\":\"%s\"}",
        (long long)wall, (unsigned long)frame_idx, best_score,
        res->dets[best_idx].x0, res->dets[best_idx].y0,
        res->dets[best_idx].x1, res->dets[best_idx].y1,
        (long long)res->inference_us, s_fw_ver);
    if (n <= 0 || n >= (int)sizeof(buf)) return;

    if (s_mqtt_connected) {
        esp_mqtt_client_publish(s_mqtt, s_topic_det, buf, n, 1, 0);
        s_total_dets_published++;
        ESP_LOGI(TAG, "DET pub #%lu: score=%.2f",
                 (unsigned long)s_total_dets_published, best_score);
    } else {
        buffer_event_to_sd(buf, n);
        ESP_LOGW(TAG, "DET buffered #%lu (mqtt down): score=%.2f",
                 (unsigned long)s_buffered_events, best_score);
    }
    s_last_det_pub_us = now;
}

/* ===========================================================================
 *  HTTP server — single-shot JPEG snapshot
 *
 *  The camera is in RGB565 mode for the model. To serve a browser-friendly
 *  image we encode RGB565 → JPEG in software via the esp32-camera helper
 *  frame2jpg(). Not cheap (~50 ms per encode at QVGA), but only triggers
 *  when a client actually fetches /jpg, so steady-state cost is zero.
 *
 *  The HTML index page polls /jpg at 1 Hz with a cache-busting query string
 *  so the browser doesn't reuse a stale image.
 * ==========================================================================*/
static const char INDEX_HTML[] =
    "<!doctype html><html><head><meta charset='utf-8'><title>edgecam</title>"
    "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;}"
    "img{max-width:100%;border:1px solid #444;margin-top:1em;}</style>"
    "<script>setInterval(()=>{document.getElementById('s').src='/jpg?t='+Date.now()},1000)</script>"
    "</head><body><h2>edgecam — refresh 1 Hz</h2><img id='s' src='/jpg'/></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t jpg_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return httpd_resp_send_500(req);

    uint8_t *jpg = NULL; size_t jpg_len = 0;
    /* frame2jpg allocates the output buffer; caller MUST free() it. */
    bool ok = frame2jpg(fb, 80, &jpg, &jpg_len);
    esp_camera_fb_return(fb);     /* hand the camera buffer back ASAP        */
    if (!ok || !jpg) return httpd_resp_send_500(req);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, (const char *)jpg, jpg_len);
    free(jpg);
    return r;
}

static void httpd_start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 80;
    cfg.stack_size       = 8192;
    cfg.lru_purge_enable = true;     /* evict oldest idle socket on new conn */
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) return;
    httpd_uri_t u_idx = { .uri = "/",    .method = HTTP_GET, .handler = index_handler };
    httpd_uri_t u_jpg = { .uri = "/jpg", .method = HTTP_GET, .handler = jpg_handler   };
    httpd_register_uri_handler(s_httpd, &u_idx);
    httpd_register_uri_handler(s_httpd, &u_jpg);
}

/* mDNS: advertises this device as "edgecam-<id>.local" on the LAN. Browsers
 * on iOS/macOS can use that name directly; Android can't always. We log the
 * raw IP at boot as the fallback. */
static void mdns_start(void)
{
    if (mdns_init() != ESP_OK) return;
    mdns_hostname_set(s_mdns_host);
    mdns_instance_name_set("Edge-AI Vision Node");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

/* ===========================================================================
 *  Diagnostic helpers
 * ==========================================================================*/

static void log_boot_info(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "app=%s idf=%s fw=%s",
             app->project_name, app->idf_ver, s_fw_ver);
    uint32_t fs = 0; esp_flash_get_size(NULL, &fs);
    ESP_LOGI(TAG, "flash: %lu MB  psram: %u MB",
             (unsigned long)(fs / (1024UL * 1024UL)),
             (unsigned)(esp_psram_get_size() / (1024U * 1024U)));
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "running partition: %s @ 0x%08lx",
             running->label, (unsigned long)running->address);
}

/* If this boot is the first one after a fresh OTA, the bootloader has the
 * image flagged PENDING_VERIFY. Confirming it here cancels the rollback
 * armed for the NEXT boot. We do this after all subsystems are up, so we
 * never confirm a broken image. */
static void mark_image_valid_if_pending(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK
        && state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGW(TAG, "OTA image marked valid");
    }
}

static esp_err_t camera_bringup(void)
{
    esp_err_t err = esp_camera_init(&s_cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init: 0x%x", err);
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s) ESP_LOGI(TAG, "sensor: PID=0x%04x", s->id.PID);
    return ESP_OK;
}

/* ===========================================================================
 *  app_main — entry point. IDF calls this after the bootloader hands over.
 *
 *  Ordering of init matters here:
 *      1. reset reason (must be first; the register self-clears once read)
 *      2. NVS         (Wi-Fi creds, persistent config)
 *      3. SD mount    (event buffer must exist before we accept events)
 *      4. Camera + inference  (model loads ~3 MB into PSRAM)
 *      5. Wi-Fi → SNTP → MQTT → mDNS → HTTPD
 *      6. TWDT subscribe (only after we know everything else works)
 *      7. enter capture loop
 *
 *  If any non-critical init fails (SD, Wi-Fi, etc.) we log and proceed —
 *  the device should remain useful in degraded states.
 * ==========================================================================*/
void app_main(void)
{
    capture_reset_reason();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Either NVS partition is exhausted, or the format upgraded. Erasing
         * is safe at this point — credentials will be re-supplied via OTA
         * or on next provisioning. */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    log_boot_info();
    mark_image_valid_if_pending();

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    sd_mount();    /* non-fatal if it fails                                  */

    if (camera_bringup() != ESP_OK) {
        /* Camera is mandatory; flash the LED fast forever to signal user. */
        while (1) {
            gpio_set_level(LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    if (inference_init() != ESP_OK) vTaskDelay(portMAX_DELAY);

    if (wifi_init_sta() == ESP_OK) {
        if (sntp_sync_wait(15)) mqtt_start();   /* MQTT needs synced time   */
        mdns_start();
        httpd_start_server();
    } else {
        ESP_LOGW(TAG, "running OFFLINE (camera + inference + SD buffer only)");
    }

    wdt_init_and_subscribe();
    ESP_LOGI(TAG, "ready  boot=%s  sd=%d  fw=%s",
             s_boot_reason_name, s_sd_mounted, s_fw_ver);

    /* ---- Capture loop -------------------------------------------------- */
    uint32_t frame_idx = 0;
    int64_t  win_start = esp_timer_get_time();   /* 5-second stats window   */
    uint32_t win_frames = 0;
    int64_t  win_infer_us = 0;
    uint32_t win_hits = 0;

    while (1) {
        /* Feed the watchdog FIRST. If anything below this line hangs, the
         * watchdog catches it within TWDT_TIMEOUT_S. */
        esp_task_wdt_reset();

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            /* Sensor stalled, briefly. Don't busy-loop the CPU; back off
             * and try again. fb_get is internally bounded. */
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* Inference. ~185 ms blocking on the current model. Returns a
         * fixed-size result struct so the C side doesn't deal with C++
         * std::list lifetimes. */
        inference_result_t res = {0};
        inference_run_rgb888(fb->buf, fb->width, fb->height, &res);

        win_frames++;
        win_infer_us += res.inference_us;
        if (res.n_detections > 0) win_hits++;

        emit_detection(&res, frame_idx);

        /* MUST return the buffer or the camera driver runs out and stalls. */
        esp_camera_fb_return(fb);

        /* Once every 5 wall-clock seconds: roll up stats, log, telemetry. */
        int64_t now = esp_timer_get_time();
        if ((now - win_start) >= 5000000) {
            float fps = win_frames * 1000000.0f / (float)(now - win_start);
            int64_t avg_infer = win_frames ? (win_infer_us / win_frames) : 0;
            ESP_LOGI(TAG,
                "5s: fw=%s fps=%.2f infer=%lld us hits=%lu/%lu "
                "pub=%lu buf=%lu int=%u(min=%u) psram=%u(min=%u) "
                "wifi_rc=%d mqtt=%s",
                s_fw_ver, fps, (long long)avg_infer,
                (unsigned long)win_hits, (unsigned long)win_frames,
                (unsigned long)s_total_dets_published,
                (unsigned long)s_buffered_events,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
                s_retry_count,
                s_mqtt_connected ? "UP" : "DOWN");
            mqtt_publish_telemetry(fps, frame_idx, avg_infer);
            win_start = now;
            win_frames = 0; win_infer_us = 0; win_hits = 0;
            /* Toggle the LED so a human in front of the device knows it's
             * alive even without a serial console. */
            gpio_set_level(LED_GPIO, (frame_idx >> 0) & 1);
        }

        frame_idx++;
        /* Yield 1 ms to the scheduler. Without this, the capture loop has
         * no place where lower-priority tasks (HTTP, idle, etc.) can run.
         * 1 ms == 1 FreeRTOS tick at 1 kHz. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
