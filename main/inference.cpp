/*
 * inference.cpp — C++ wrapper around ESP-DL's HumanFaceDetect model.
 * Exposes a plain-C function the rest of the project (main.c) can call.
 *
 * NOTE on ESP-DL API:
 *   The detector's run() signature has shifted between v3.0, v3.1, and v3.3.
 *   If this file fails to compile, the most likely fixes are:
 *     a) The run() argument list (uint8_t* + shape{} vs dl::image::img_t).
 *     b) The return type (std::list<dl::detect::result_t>& vs std::vector<...>).
 *   Paste the compiler error to the assistant and we'll patch in place.
 */

#include "inference.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "human_face_detect.hpp"

#define TAG "infer"

static HumanFaceDetect *s_detector = nullptr;

extern "C" esp_err_t inference_init(void)
{
    if (s_detector) return ESP_OK;
    s_detector = new HumanFaceDetect();
    if (!s_detector) {
        ESP_LOGE(TAG, "HumanFaceDetect alloc failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "HumanFaceDetect ready");
    return ESP_OK;
}

extern "C" esp_err_t inference_run_rgb888(const uint8_t *rgb888,
                                          int width, int height,
                                          inference_result_t *out)
{
    if (!s_detector || !rgb888 || !out) return ESP_ERR_INVALID_ARG;

    dl::image::img_t img;
    img.data     = const_cast<uint8_t *>(rgb888);
    img.width    = width;
    img.height   = height;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE;

    int64_t t0 = esp_timer_get_time();
    auto &results = s_detector->run(img);
    out->inference_us = esp_timer_get_time() - t0;

    int i = 0;
    for (auto &r : results) {
        if (i >= INFERENCE_MAX_DETS) break;
        out->dets[i].x0    = r.box[0];
        out->dets[i].y0    = r.box[1];
        out->dets[i].x1    = r.box[2];
        out->dets[i].y1    = r.box[3];
        out->dets[i].score = r.score;
        i++;
    }
    out->n_detections = i;
    return ESP_OK;
}