/*----------------------------------------------------------------------------/
  Lovyan GFX - Graphics library for embedded devices.

Original Source:
 https://github.com/lovyan03/LovyanGFX/

Licence:
 [FreeBSD](https://github.com/lovyan03/LovyanGFX/blob/master/license.txt)

Author:
 [lovyan03](https://twitter.com/lovyan03)

Contributors:
 [ciniml](https://github.com/ciniml)
 [mongonta0716](https://github.com/mongonta0716)
 [tobozo](https://github.com/tobozo)
/----------------------------------------------------------------------------*/

#include "Panel_DSI.hpp"

#if SOC_MIPI_DSI_SUPPORTED

#include "../common.hpp"

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_io.h>
#include <esp_heap_caps.h>      // s31a/s31b: heap_caps_malloc + PSRAM stats
#include <esp_log.h>             // s31b: drawbuf alloc diagnostics
#include <esp_timer.h>           // S101 FIX B: DMA2D kick timestamp (inflight gate)
#include "freertos/FreeRTOS.h"  // s31a: binary semaphore
#include "freertos/task.h"
#if __has_include(<esp_idf_version.h>)
 #include <esp_idf_version.h>
#endif

// s31b (SkyTab): draw-buffer alloc state, exposed to the app's SETUP view
// (ui.cpp re-declares the extern "C" hooks; no shared header needed).
static bool s_skytab_drawbuf_ok = false;
static uint32_t s_skytab_psram_free_at_alloc_kb = 0;
// rfab POC: target for the file-end extern "C" hooks.
static lgfx::v1::Panel_DSI *s_rfab_panel = nullptr;
// S101 FIX B: exact DMA2D in-flight tracking for rfab_poc_waitpresent().
// The old gate TOOK the _refresh_done_sem credit that only
// on_frame_buf_complete gives (~24 ms after the copy, one per PRESENT); the
// player consumes one credit per ITERATION (12/s) vs 2-4 presents/s, so one
// drop starved the binary sem -> 100 ms block -> more drops (the
// self-sustaining cascade behind S100's 75% drop rate). Now: flag set at
// kick, cleared by on_color_trans_done (copy done, ~5-9 ms), with a 20 ms
// lost-callback safety timeout (2.2x worst 9 ms copy).
static volatile bool     s_rfab_dma_inflight = false;
static int64_t           s_rfab_dma_kick_us  = 0;
static volatile uint32_t s_rfab_color_done   = 0;

namespace lgfx
{
 inline namespace v1
 {
//----------------------------------------------------------------------------
  bool Panel_DSI::init_panel(void)
  {
    auto bus = getBusDSI();
    if (bus == nullptr) { return false; }

    const bool init_in_command_mode = initInCommandMode();
    if (!init_in_command_mode
     && ESP_OK != esp_lcd_panel_init(_disp_panel_handle)) {
      return false;
    }

    uint8_t madctl_val = 0;
    if (_cfg.rgb_order == false) {
      madctl_val = 1<<3; //LCD_CMD_BGR_BIT;
    }

    uint8_t colmod_val = 0x55;
    if (_write_bits >= 24) {
      colmod_val = 0x77;
    }

    const uint8_t* params;
    for (size_t i = 0; nullptr != (params = getInitParams(i)); ++i)
    {
      size_t len;
      while (0 != (len = params[0])) {
// printf("cmd: %02x, len: %d\n", params[1], len);
        bus->writeParams(params[1], &params[2], len - 1);
        params += len + 1;
      }
      vTaskDelay(pdMS_TO_TICKS(getInitDelay(i)));
    }

    if (init_in_command_mode) {
      bus->writeParams(CMD_MADCTL, &(madctl_val), 1);
      bus->writeParams(CMD_COLMOD, &(colmod_val), 1);
      return (ESP_OK == esp_lcd_panel_init(_disp_panel_handle));
    }
    return true;
  }


  bool Panel_DSI::init_dpi(Bus_DSI* bus)
  {
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = bus->getMipiDsiBus();

    esp_lcd_dpi_panel_config_t dpi_config;
    memset(&dpi_config, 0, sizeof(dpi_config));
    dpi_config.virtual_channel = 0;
    dpi_config.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_config.dpi_clock_freq_mhz = _config_detail.dpi_freq_mhz;
  #if defined (ESP_IDF_VERSION_VAL) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0))
    dpi_config.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi_config.out_color_format = LCD_COLOR_FMT_RGB565;
  #else
    dpi_config.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
  #endif
    dpi_config.num_fbs = 2;
    dpi_config.video_timing.h_size = _cfg.panel_width;
    dpi_config.video_timing.v_size = _cfg.panel_height;
    dpi_config.video_timing.hsync_back_porch  = _config_detail.hsync_back_porch;
    dpi_config.video_timing.hsync_pulse_width = _config_detail.hsync_pulse_width;
    dpi_config.video_timing.hsync_front_porch = _config_detail.hsync_front_porch;
    dpi_config.video_timing.vsync_back_porch  = _config_detail.vsync_back_porch;
    dpi_config.video_timing.vsync_pulse_width = _config_detail.vsync_pulse_width;
    dpi_config.video_timing.vsync_front_porch = _config_detail.vsync_front_porch;
  #if !defined (ESP_IDF_VERSION_VAL) || (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0))
    dpi_config.flags.use_dma2d = true;
  #endif
    auto ret = esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &_disp_panel_handle);
  #if defined (ESP_IDF_VERSION_VAL) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0))
    if (ret == ESP_OK)
    {
      (void)esp_lcd_dpi_panel_enable_dma2d(_disp_panel_handle);
    }
  #endif
    return ret == ESP_OK;
  }

  color_depth_t Panel_DSI::setColorDepth(color_depth_t depth)
  {
    _write_depth = color_depth_t::rgb565_nonswapped;
    _read_depth = color_depth_t::rgb565_nonswapped;
    return color_depth_t::rgb565_nonswapped;
  }

  bool Panel_DSI::init(bool use_reset)
  {
    if (_lines_buffer != nullptr) return false;

    if (!Panel_FrameBufferBase::init(use_reset))
    {
      return false;
    }

    auto bus = getBusDSI();
    if (bus == nullptr) { return false; }
    const size_t reset_delay = getResetDelayBeforeDpi();
    if (reset_delay
     && (!bus->writeParams(CMD_SWRESET, nullptr, 0))) {
      return false;
    }
    if (reset_delay) {
      vTaskDelay(pdMS_TO_TICKS(reset_delay));
    }
    if (init_dpi(bus) && init_panel())
    {
        esp_lcd_dpi_panel_get_frame_buffer(_disp_panel_handle, 1, &(_config_detail.buffer));
    }

    auto ptr = (uint8_t*)_config_detail.buffer;
// printf ("ptr : %08x \n", (uintptr_t)ptr);
    if (ptr == nullptr) { return false; }

    // s31a (SkyTab double buffer): draw into a DEDICATED PSRAM buffer, not
    // the fetched driver fb. The DPI driver ALTERNATES scanout between its
    // two fbs, so a zero-copied fetched fb is the SCANNED (front) fb one
    // frame in two -> residual scanout tear in that frame's draw window.
    // A driver-unknown user buffer makes EVERY present take the draw_bitmap
    // "DMA2D copy to the non-scanned fb + vsync swap" path => the panel
    // never scans the buffer being written => no tear at any frame rate.
    // Cost: +1 fb PSRAM (~1.75 MB; Tab5 has 32 MB) + one ~5-9 ms DMA2D copy
    // per present. Per-op C2M writebacks (Panel_FrameBufferBase::display,
    // _auto_display=true on P4) keep the draw buffer coherent for both the
    // panel read and the DMA2D copy. Alloc failure falls back to the old
    // zero-copy behavior (tears, but works).
    const size_t fb_bytes = (size_t)_cfg.panel_width * _cfg.panel_height
                           * (_write_bits / 8);
    // s31b (SkyTab): diagnostics + retry. First REAL run of this patch (s37:
    // s31a had only edited the non-compiled M5GFX libdeps dir), so log the
    // PSRAM state at alloc time; a silent failure would otherwise be
    // invisible (the SETUP view reads skytab_dsi_drawbuf_ok()).
    s_skytab_psram_free_at_alloc_kb =
        (uint32_t)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
    ESP_LOGI("skytab-dsi",
             "drawbuf: alloc %u B (psram_free=%u KB psram_largest=%u KB)",
             (unsigned)fb_bytes, (unsigned)s_skytab_psram_free_at_alloc_kb,
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) /
                        1024));
    // s31b: heap_caps_malloc — esp_psram_malloc is not declared in this
    // pioarduino toolchain's headers; same SPIRAM pool.
    _draw_buf = heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    for (int i = 0; _draw_buf == nullptr && i < 5; i++) {
      vTaskDelay(pdMS_TO_TICKS(200));  // transient fragmentation at init
      _draw_buf = heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    s_skytab_drawbuf_ok = (_draw_buf != nullptr);
    s_rfab_panel = this;  // rfab POC
    if (_draw_buf) {
      ptr = (uint8_t*)_draw_buf;
      ESP_LOGI("skytab-dsi", "drawbuf: ok ptr=%p (psram_free=%u KB)",
               (void *)_draw_buf,
               (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    } else {
      ESP_LOGI("skytab-dsi",
               "drawbuf: FAIL after retry - zero-copy fallback (tear)");
    }

    // s31a: frame-complete sync (drives waitDisplay(); the app does not
    // need it at 5 fps - the 200 ms frame period >> the copy time).
    if (!_refresh_done_sem) {
      _refresh_done_sem = xSemaphoreCreateBinary();
    }
    if (_refresh_done_sem && _disp_panel_handle) {
      esp_lcd_dpi_panel_event_callbacks_t dpi_cbs = {};
      dpi_cbs.on_frame_buf_complete = on_refresh_done;
      dpi_cbs.on_color_trans_done = on_refresh_done;  // S101 FIX B: copy-done = exact un-kick
      (void)esp_lcd_dpi_panel_register_event_callbacks(_disp_panel_handle,
                                                       &dpi_cbs, this);
    }


//--------------------------------------------------------------------------------
    const auto height = _cfg.panel_height;
// printf ("panel_width  : %d \n", _cfg.panel_width );
// printf ("panel_height : %d \n", _cfg.panel_height);
    size_t la_size = height * sizeof(void*);
    uint8_t** lineArray = (uint8_t**)heap_alloc_dma(la_size);

// printf ("lineArray ptr : %08x \n", (uintptr_t)lineArray);
    if ( nullptr == lineArray ) { return false; }
    memset(lineArray, 0, la_size);

    const size_t line_length = ((_cfg.panel_width * _write_bits >> 3) + 3) & ~3;

    _lines_buffer = lineArray;

    for (int y = 0; y < height; y ++)
    {
      lineArray[y] = ptr;
      ptr = ptr + line_length;
    }

    return true;
  }

  // s31a (SkyTab double buffer): present path. display() is called two
  // ways: (a) automatically per draw op by IPanel::endWrite() with
  // (0,0,0,0) because _auto_display is true on P4 - that stays a cheap
  // C2M dirty-line flush (base display), NO push; (b) once per frame by
  // the app's explicit 4-arg display(0,0,W,H) - w>0/h>0 = present: push
  // the WHOLE draw buffer (see init()).
  void Panel_DSI::display(uint_fast16_t x, uint_fast16_t y, uint_fast16_t w, uint_fast16_t h)
  {
    Panel_FrameBufferBase::display(x, y, w, h);
    if (0 < w && 0 < h && _disp_panel_handle && _draw_buf)
    {
      // S101 FIX B: mark in-flight IMMEDIATELY BEFORE the async DMA2D kick.
      s_rfab_dma_inflight = true;
      s_rfab_dma_kick_us = esp_timer_get_time();
      (void)esp_lcd_panel_draw_bitmap(_disp_panel_handle, 0, 0,
                                      _cfg.panel_width, _cfg.panel_height,
                                      _draw_buf);
    }
  }

  void Panel_DSI::waitDisplay(void)
  {
    if (_refresh_done_sem)
    {
      (void)xSemaphoreTake(_refresh_done_sem, pdMS_TO_TICKS(100));
    }
  }

  bool Panel_DSI::on_refresh_done(esp_lcd_panel_handle_t panel,
                                  esp_lcd_dpi_panel_event_data_t* edata,
                                  void* user_ctx)
  {
    (void)panel;
    (void)edata;
    // S101 FIX B: DMA2D copy is done (on_color_trans_done) or the frame
    // buffer is reusable (on_frame_buf_complete) — either way the present
    // side is un-kicked. Double-clear is a no-op; the sem give is harmless.
    s_rfab_dma_inflight = false;
    s_rfab_color_done++;
    auto self = static_cast<Panel_DSI*>(user_ctx);
    if (!self || !self->_refresh_done_sem) {
      return false;
    }
    BaseType_t high_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(self->_refresh_done_sem, &high_task_woken);
    return high_task_woken == pdTRUE;
  }

//----------------------------------------------------------------------------

  bool Panel_DSI::write_params(uint32_t cmd, const uint8_t* data, size_t length)
  {
    bool res = false;
    auto b = getBusDSI();
    if (b) {
      startWrite();
      res = b->writeParams(cmd, data, length);
      b->flush();
      endWrite();
    }
    return res;
  }

  void Panel_DSI::setInvert(bool invert)
  {
    _invert = invert;
    write_params((invert ^ _cfg.invert) ? CMD_INVON : CMD_INVOFF);
  }

  void Panel_DSI::setSleep(bool flg_sleep)
  {
    write_params(flg_sleep ? CMD_SLPIN : CMD_SLPOUT);
  }

  void Panel_DSI::setPowerSave(bool flg_idle)
  {
    write_params(flg_idle ? CMD_IDMON : CMD_IDMOFF);
  }

//----------------------------------------------------------------------------
}
}

// s31b (SkyTab): diagnostic hooks (read by the app SETUP view, ui.cpp).
extern "C" bool skytab_dsi_drawbuf_ok(void) { return s_skytab_drawbuf_ok; }
extern "C" unsigned int skytab_dsi_psram_free_at_alloc_kb(void) {
  return s_skytab_psram_free_at_alloc_kb;
}


// ---- rfab POC hooks (RFAV-3 player) ------------------------------
// File-scope extern "C" (pioarduino GCC 14 rejects function-scope
// extern "C" — skytab s38).
extern "C" uint16_t *rfab_poc_drawbuf(void) {
  auto p = s_rfab_panel;
  return (p != nullptr) ? (uint16_t *)p->rfabDrawBufRaw() : nullptr;
}
extern "C" bool rfab_poc_drawbuf_ok(void) { return s_skytab_drawbuf_ok; }
extern "C" void rfab_poc_waitpresent(void) {
  auto p = s_rfab_panel;
  if (p == nullptr) return;
  // S101 FIX B: exact in-flight gate (replaces the credit-taking
  // waitDisplay — credit starvation: 12/s consumption vs per-present
  // production self-sustained 100 ms blocks). No DMA2D in flight -> return
  // immediately; else wait for on_color_trans_done, capped by a 100 ms
  // outer timeout and a 20 ms post-kick lost-callback safety (callback
  // never firing degrades to a 20 ms gate, not 100 ms starvation).
  if (!s_rfab_dma_inflight) return;
  const int64_t t0 = esp_timer_get_time();
  while (s_rfab_dma_inflight) {
    if (esp_timer_get_time() - t0 > 100000) break;
    if (esp_timer_get_time() - s_rfab_dma_kick_us > 20000) {
      s_rfab_dma_inflight = false;  // lost-callback safety
      break;
    }
    vTaskDelay(1);
  }
}
extern "C" uint32_t rfab_poc_color_done(void) { return s_rfab_color_done; }

#endif
