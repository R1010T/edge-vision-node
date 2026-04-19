/*
 * inference.h — C-callable interface for ESP-DL face detection.
 * The implementation (inference.cpp) is C++ because ESP-DL exposes a C++ API.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define INFERENCE_MAX_DETS 8

typedef struct {
    int x0, y0, x1, y1;
    float score;
} inference_box_t;

typedef struct {
    int            n_detections;
    inference_box_t dets[INFERENCE_MAX_DETS];
    int64_t        inference_us;     /* wall-clock duration of run() */
} inference_result_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Allocates the detector. Call once after PSRAM is up. */
esp_err_t inference_init(void);

/* Runs face detection on an RGB888 buffer.
 * width, height: image dimensions in pixels.
 * out: populated with up to INFERENCE_MAX_DETS bounding boxes.
 */
esp_err_t inference_run_rgb888(const uint8_t *rgb888,
                               int width, int height,
                               inference_result_t *out);

#ifdef __cplusplus
}
#endif
