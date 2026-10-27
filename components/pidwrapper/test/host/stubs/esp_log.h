#pragma once
// Host stub for ESP-IDF logging — just routes to stderr so PIDController's
// ESP_LOGI/ESP_LOGE calls compile and are visible during host tests.
#include <cstdio>
#define ESP_LOGE(tag, fmt, ...) std::fprintf(stderr, "E (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) std::fprintf(stderr, "W (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) std::fprintf(stderr, "I (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)
