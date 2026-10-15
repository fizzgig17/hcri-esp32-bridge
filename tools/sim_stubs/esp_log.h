#pragma once
#include <stdio.h>
#define ESP_LOGW(t, ...) do { (void)(t); printf("  [log] " __VA_ARGS__); printf("\n"); } while (0)
#define ESP_LOGI(t, ...) do { (void)(t); printf("  [log] " __VA_ARGS__); printf("\n"); } while (0)
