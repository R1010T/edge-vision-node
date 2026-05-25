/*
 * Edge-AI Vision Node — ESP32-S3
 *
 * On-device face detection with TLS-MQTT telemetry, MQTT-triggered OTA,
 * and SD-buffered offline event replay. See docs/ARCHITECTURE.md for
 * subsystem detail and docs/DEV_LOG.md for measured KPIs.
 *
 * Build & flash:
 *     idf.py set-target esp32s3
 *     idf.py build
 *     idf.py -p COM4 flash monitor
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_sntp.h"
#include "esp_crt_bundle.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "mqtt_client.h"
#include "mdns.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_camera.h"

#include "camera_pins.h"
#include "secrets.h"
#include "inference.h"

#define LED_GPIO        GPIO_NUM_2
#define TAG             "main"

#define WIFI_CONNECTED_BIT           BIT0
#define WIFI_FAIL_BIT                BIT1

#define DET_PUBLISH_MIN_INTERVAL_MS  1000      /* producer-side rate cap     */
#define DET_SCORE_THRESHOLD          0.55f

#define SD_MOUNT_POINT  "/sdcard"
#define EVENTS_FILE     "/sdcard/events.log"
#define SD_PIN_CMD      GPIO_NUM_38
#define SD_PIN_CLK      GPIO_NUM_39
#define SD_PIN_D0       GPIO_NUM_40

#define TWDT_TIMEOUT_S  30                     /* tolerant of OTA stalls     */

/* ---- Wi-Fi state ------------------------------------------------------- */
static EventGroupHandle_t s_wifi_evt;
static int                s_retry_count = 0;
static volatile bool      s_wifi_ever_connected = false;
static esp_timer_handle_t s_wifi_reconnect_timer = NULL;

/* ---- MQTT state -------------------------------------------------------- */
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool      s_mqtt_connected = false;
static char       s_device_id[16] = {0};
static char       s_topic_tel[64] = {0};
static char       s_topic_cmd[64] = {0};
static char       s_topic_det[64] = {0};
static char       s_lwt_payload[] = "{\"online\":false}";
static char       s_mdns_host[32] = {0};
static const char s_fw_ver[] = "0.7.0";

/* ---- HTTP / detection bookkeeping ------------------------------------- */
static httpd_handle_t     s_httpd = NULL;
static int64_t            s_last_det_pub_us = 0;
static volatile uint32_t  s_total_dets_published = 0;
static volatile bool      s_ota_in_progress = false;

/* ---- SD card + event buffer ------------------------------------------- */
static sdmmc_card_t       *s_sd_card = NULL;
static volatile bool      s_sd_mounted = false;
static SemaphoreHandle_t  s_buffer_mutex = NULL;
static volatile uint32_t  s_buffered_events = 0;
static volatile uint32_t  s_total_replayed = 0;

/* ---- boot postmortem -------------------------------------------------- */
static esp_reset_reason_t s_boot_reason;
static const char        *s_boot_reason_name = "?";

/* Camera: RGB565 BE (model native format on OV3660 — sensor cannot emit
 * RGB888). fb_count=3 so the HTTP /jpg handler can hold a buffer while
 * the capture loop is mid-inference. */
static const camera_config_t s_cam_cfg = {
    .pin_pwdn = CAM_PIN_PWDN, .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD, .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 0,
    .fb_count     = 3,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
};

/* ====================================================================== */
/* Boot reason — captured first so postmortem evidence isn't overwritten. */
/* ====================================================================== */

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
    ESP_LOGW(TAG, "boot reason: %s (code %d)", s_boot_reason_name, (int)s_boot_reason);
}

/* ====================================================================== */
/* Task watchdog — armed after all subsystems are up.                     */
/* Composes with OTA: bad firmware that hangs the capture loop triggers   */
/* a TWDT panic and the bootloader rolls back to the previous slot.       */
/* ====================================================================== */

static void wdt_init_and_subscribe(void)
{
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = TWDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_err_t err = esp_task_wdt_reconfigure(&cfg);
    if (err == ESP_ERR_INVALID_STATE) err = esp_task_wdt_init(&cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "TWDT init: %s", esp_err_to_name(err)); return; }
    if (esp_task_wdt_add(NULL) == ESP_OK)
        ESP_LOGI(TAG, "TWDT armed (%d s)", TWDT_TIMEOUT_S);
}

/* ====================================================================== */
/* Wi-Fi with exp-backoff reconnect.                                      */
/* Initial association honors WIFI_MAX_RETRY so boot doesn't block        */
/* forever; runtime reconnect uses 500 ms .. 60 s cap, never gives up.    */
/* ====================================================================== */

static void wifi_reconnect_cb(void *arg)
{
    ESP_LOGI(TAG, "wifi: reconnect attempt #%d", s_retry_count);
    esp_wifi_connect();
}

static int wifi_backoff_ms(int attempt)
{
    int shift = attempt - 1;
    if (shift < 0) shift = 0;
    if (shift > 7) shift = 7;
    int ms = 500 << shift;
    return ms > 60000 ? 60000 : ms;
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retry_count++;
        if (!s_wifi_ever_connected) {
            if (s_retry_count <= WIFI_MAX_RETRY) esp_wifi_connect();
            else                                 xEventGroupSetBits(s_wifi_evt, WIFI_FAIL_BIT);
        } else {
            int delay_ms = wifi_backoff_ms(s_retry_count);
            ESP_LOGW(TAG, "wifi: link lost, reconnect in %d ms (#%d)",
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

static esp_err_t wifi_init_sta(void)
{
    s_wifi_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    esp_timer_create_args_t timer_args = {
        .callback = wifi_reconnect_cb, .name = "wifi_rc",
    };
    esp_timer_create(&timer_args, &s_wifi_reconnect_timer);

    wifi_config_t wc = {
        .sta = {
            .ssid = WIFI_SSID, .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_evt, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

/* ====================================================================== */
/* SNTP — must run before MQTT-TLS or cert validation fails on bad clock. */
/* ====================================================================== */

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

/* ====================================================================== */
/* SD-card event buffer.                                                   */
/* Producer (capture loop) appends; consumer (replay task) drains. Mutex   */
/* serialises file ops because FATFS is not internally re-entrant.         */
/* Mount failure is non-fatal — device runs in volatile-only mode.         */
/* ====================================================================== */

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
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = SD_PIN_CLK;
    slot.cmd   = SD_PIN_CMD;
    slot.d0    = SD_PIN_D0;
    slot.width = 1;
    slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot,
                                            &mount_config, &s_sd_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sd: mount failed (%s)", esp_err_to_name(err));
        return err;
    }
    s_sd_mounted = true;
    ESP_LOGI(TAG, "sd: mounted at %s", SD_MOUNT_POINT);

    /* Count events left over from a prior power cycle; they'll replay on
     * the next successful MQTT connect. */
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

/* Drain at 20 Hz on reconnect. Only purges the file if the entire replay
 * completed without MQTT dropping again — otherwise events stay buffered. */
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
        if (len > 0 && line[len - 1] == '\n') line[--len] = '\0';
        if (len == 0) continue;
        esp_mqtt_client_publish(s_mqtt, s_topic_det, line, len, 1, 0);
        n_replayed++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    fclose(f);

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

/* ====================================================================== */
/* OTA over HTTP(S) — same CA bundle as MQTT-TLS, no duplicated trust.   */
/* ====================================================================== */

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
        esp_restart();
    }
cleanup:
    free(url);
    s_ota_in_progress = false;
    vTaskDelete(NULL);
}

/* Command shell. Tiny hand-rolled parser; no JSON to keep RAM tight. */
static void handle_cmd(const char *payload, int len)
{
    if (len >= 4 && strncmp(payload, "ota ", 4) == 0) {
        if (s_ota_in_progress) return;
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
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}

/* ====================================================================== */
/* MQTT-TLS — Mozilla CA bundle, retained LWT, QoS 1 throughout.          */
/* ====================================================================== */

static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "mqtt-tls: CONNECTED");
        s_mqtt_connected = true;
        esp_mqtt_client_subscribe(s_mqtt, s_topic_cmd, 1);
        esp_mqtt_client_publish(s_mqtt, s_topic_tel, "{\"online\":true}", 0, 1, 1);
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

/* Detection event: publish on best score per frame, rate-limited to 1 Hz.
 * Goes to SD buffer if MQTT is down. */
static void emit_detection(const inference_result_t *res, uint32_t frame_idx)
{
    if (res->n_detections <= 0) return;
    int64_t now = esp_timer_get_time();
    if (now - s_last_det_pub_us < (int64_t)DET_PUBLISH_MIN_INTERVAL_MS * 1000) return;

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
        ESP_LOGW(TAG, "DET buffered #%lu: score=%.2f",
                 (unsigned long)s_buffered_events, best_score);
    }
    s_last_det_pub_us = now;
}

/* ====================================================================== */
/* HTTP — index page + /jpg snapshot via software RGB565→JPEG encode.    */
/* No live stream: frame2jpg + inference would crater FPS.                */
/* ====================================================================== */

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
    bool ok = frame2jpg(fb, 80, &jpg, &jpg_len);
    esp_camera_fb_return(fb);
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
    cfg.server_port = 80;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) return;

    httpd_uri_t u_idx = { .uri = "/",    .method = HTTP_GET, .handler = index_handler };
    httpd_uri_t u_jpg = { .uri = "/jpg", .method = HTTP_GET, .handler = jpg_handler   };
    httpd_register_uri_handler(s_httpd, &u_idx);
    httpd_register_uri_handler(s_httpd, &u_jpg);
}

static void mdns_start(void)
{
    if (mdns_init() != ESP_OK) return;
    mdns_hostname_set(s_mdns_host);
    mdns_instance_name_set("Edge-AI Vision Node");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

/* ====================================================================== */
/* Bring-up helpers                                                        */
/* ====================================================================== */

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

/* Called once subsystems are up. Cancels the pending-verify rollback the
 * bootloader armed on the previous (fresh-OTA) boot. */
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
    if (err != ESP_OK) { ESP_LOGE(TAG, "camera init: 0x%x", err); return err; }
    sensor_t *s = esp_camera_sensor_get();
    if (s) ESP_LOGI(TAG, "sensor: PID=0x%04x", s->id.PID);
    return ESP_OK;
}

/* ====================================================================== */
/* app_main                                                                */
/* ====================================================================== */

void app_main(void)
{
    capture_reset_reason();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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

    sd_mount();

    if (camera_bringup() != ESP_OK) {
        while (1) {
            gpio_set_level(LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    if (inference_init() != ESP_OK) vTaskDelay(portMAX_DELAY);

    if (wifi_init_sta() == ESP_OK) {
        if (sntp_sync_wait(15)) mqtt_start();
        mdns_start();
        httpd_start_server();
    } else {
        ESP_LOGW(TAG, "running offline (camera + inference + SD buffer only)");
    }

    wdt_init_and_subscribe();
    ESP_LOGI(TAG, "ready  boot=%s  sd=%d  fw=%s",
             s_boot_reason_name, s_sd_mounted, s_fw_ver);

    uint32_t frame_idx = 0;
    int64_t  win_start = esp_timer_get_time();
    uint32_t win_frames = 0;
    int64_t  win_infer_us = 0;
    uint32_t win_hits = 0;

    while (1) {
        esp_task_wdt_reset();

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        inference_result_t res = {0};
        inference_run_rgb888(fb->buf, fb->width, fb->height, &res);

        win_frames++;
        win_infer_us += res.inference_us;
        if (res.n_detections > 0) win_hits++;

        emit_detection(&res, frame_idx);
        esp_camera_fb_return(fb);

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
            gpio_set_level(LED_GPIO, (frame_idx >> 0) & 1);
        }
        frame_idx++;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
