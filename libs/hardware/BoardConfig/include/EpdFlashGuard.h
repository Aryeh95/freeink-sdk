#pragma once

// Coordinates e-paper refreshes (high-voltage TPS65185 rails, AXI-DMA/LCD scan)
// with internal flash / NVS writes and VCOM updates on the single-core ESP32-S31.
//
// Recursive FreeRTOS mutex so nested callers on the same task succeed. Safe to
// construct before enableEpdFlashGuard() is called or before the scheduler runs.
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace freeink {
namespace detail {

inline SemaphoreHandle_t& epdFlashGuardMutex() {
  static StaticSemaphore_t storage;
  static SemaphoreHandle_t mutex = xSemaphoreCreateRecursiveMutexStatic(&storage);
  return mutex;
}

inline bool& epdFlashGuardEnabled() {
  static bool enabled = false;
  return enabled;
}

}  // namespace detail

inline void enableEpdFlashGuard() {
  detail::epdFlashGuardMutex();
  detail::epdFlashGuardEnabled() = true;
}

struct EpdFlashGuard {
  EpdFlashGuard()
      : active(detail::epdFlashGuardEnabled() &&
               xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    if (active) xSemaphoreTakeRecursive(detail::epdFlashGuardMutex(), portMAX_DELAY);
  }
  ~EpdFlashGuard() {
    if (active) xSemaphoreGiveRecursive(detail::epdFlashGuardMutex());
  }
  EpdFlashGuard(const EpdFlashGuard&) = delete;
  EpdFlashGuard& operator=(const EpdFlashGuard&) = delete;

 private:
  bool active;
};

}  // namespace freeink
