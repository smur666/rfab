// S169: task-creation helper — stack (+TCB) in PSRAM to keep INTERNAL SRAM
// free for DMA (SDMMC bounce, esp_hosted SDIO, I2S, lwIP). Stock sdkconfig
// has CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=0 (no protected DMA pool) and
// CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=y, so this is the app-side lever.
// RULE: a task whose code touches flash/NVS (Preferences, esp_partition,
// OTA writes) must NOT use this — it asserts when the flash op disables the
// cache. Falls back to an internal-RAM stack if the PSRAM create fails.
#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>
#include <esp_heap_caps.h>
#include <stdio.h>

static inline BaseType_t createTaskPsram(TaskFunction_t fn, const char *name,
                                         uint32_t stack, void *arg,
                                         UBaseType_t prio, TaskHandle_t *h,
                                         BaseType_t core) {
  BaseType_t r = xTaskCreatePinnedToCoreWithCaps(
      fn, name, stack, arg, prio, h, core, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (r == pdPASS) return r;
  printf("task: %s PSRAM stack create failed — internal fallback\n", name);
  return xTaskCreatePinnedToCore(fn, name, stack, arg, prio, h, core);
}
