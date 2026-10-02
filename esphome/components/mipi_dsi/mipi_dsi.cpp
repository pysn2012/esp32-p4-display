#ifdef USE_ESP32_VARIANT_ESP32P4
#include <utility>
#include "mipi_dsi.h"
#include "esphome/core/helpers.h"

namespace esphome::mipi_dsi {

// Maximum bytes to log for init commands (truncated if larger)
static constexpr size_t MIPI_DSI_MAX_CMD_LOG_BYTES = 64;

static bool notify_refresh_ready(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
  SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(user_ctx);
  BaseType_t need_yield = pdFALSE;
  xSemaphoreGiveFromISR(sem, &need_yield);
  return (need_yield == pdTRUE);
}

void MipiDsi::smark_failed(const LogString *message, esp_err_t err) {
  ESP_LOGE(TAG, "%s: %s", LOG_STR_ARG(message), esp_err_to_name(err));
  this->mark_failed(message);
}

void MipiDsi::setup() {
  ESP_LOGCONFIG(TAG, "Running Setup");

  if (!this->enable_pins_.empty()) {
    for (auto *pin : this->enable_pins_) {
      pin->setup();
      pin->digital_write(true);
    }
    delay(10);
  }

  esp_lcd_dsi_bus_config_t bus_config = {
      .bus_id = 0,  // index from 0, specify the DSI host to use
      .num_data_lanes =
          this->lanes_,  // Number of data lanes to use, can't set a value that exceeds the chip's capability
      // phy_clk_src left at 0 to enable runtime auto-select (upstream PR #18984).
      // ESP32-P4 rev 3.0+ silicon rejects MIPI_DSI_PHY_CLK_SRC_DEFAULT (= PLL_F20M)
      // and aborts; leaving it 0 lets IDF pick the revision-correct source.
      .lane_bit_rate_mbps = this->lane_bit_rate_,  // Bit rate of the data lanes, in Mbps
  };
  auto err = esp_lcd_new_dsi_bus(&bus_config, &this->bus_handle_);
  if (err != ESP_OK) {
    this->smark_failed(LOG_STR("lcd_new_dsi_bus failed"), err);
    return;
  }
  esp_lcd_dbi_io_config_t dbi_config = {
      .virtual_channel = 0,
      .lcd_cmd_bits = 8,    // according to the LCD spec
      .lcd_param_bits = 8,  // according to the LCD spec
  };
  err = esp_lcd_new_panel_io_dbi(this->bus_handle_, &dbi_config, &this->io_handle_);
  if (err != ESP_OK) {
    this->smark_failed(LOG_STR("new_panel_io_dbi failed"), err);
    return;
  }
  // clang-format off
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  auto color_format = LCD_COLOR_FMT_RGB565;
  if (this->color_depth_ == display::COLOR_BITNESS_888) {
    color_format = LCD_COLOR_FMT_RGB888;
  }
  esp_lcd_dpi_panel_config_t dpi_config = {.virtual_channel = 0,
                                           .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
                                           .dpi_clock_freq_mhz = this->pclk_frequency_,
                                           .in_color_format = color_format,
#else
  auto pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
  if (this->color_depth_ == display::COLOR_BITNESS_888) {
    pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888;
  }
  esp_lcd_dpi_panel_config_t dpi_config = {.virtual_channel = 0,
                                           .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
                                           .dpi_clock_freq_mhz = this->pclk_frequency_,
                                           .pixel_format = pixel_format,
#endif
                                           .num_fbs = 1,  // number of frame buffers to allocate
                                           .video_timing =
                                               {
                                                   .h_size = this->width_,
                                                   .v_size = this->height_,
                                                   .hsync_pulse_width = this->hsync_pulse_width_,
                                                   .hsync_back_porch = this->hsync_back_porch_,
                                                   .hsync_front_porch = this->hsync_front_porch_,
                                                   .vsync_pulse_width = this->vsync_pulse_width_,
                                                   .vsync_back_porch = this->vsync_back_porch_,
                                                   .vsync_front_porch = this->vsync_front_porch_,
                                               },
                                           .flags = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
                                               .use_dma2d = true,
#endif
                                           }};
  // clang-format on
  err = esp_lcd_new_panel_dpi(this->bus_handle_, &dpi_config, &this->handle_);
  if (err != ESP_OK) {
    this->smark_failed(LOG_STR("esp_lcd_new_panel_dpi failed"), err);
    return;
  }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  err = esp_lcd_dpi_panel_enable_dma2d(this->handle_);
  if (err != ESP_OK) {
    this->smark_failed(LOG_STR("esp_lcd_dpi_panel_enable_dma2d failed"), err);
    return;
  }
#endif
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(10);
    this->reset_pin_->digital_write(false);
    // RESETX low pulse must be >= 10ms: on warm reset the panel is still powered
    // and this pulse is its only reset source (cold boot has internal POR).
    // Timing aligned with Espressif esp_lcd_st7102_mipi.c panel_st7102_reset().
    delay(20);
    this->reset_pin_->digital_write(true);
    // ★★★ 2026-09-30 修复"上电有图 / 按 RESET 无图"：
    //   面板复位释放后，必须等它规定的 settle 时间（ST7102 要求 ≥120ms）过去，
    //   才能下发任何 DCS 命令。这里原来是 delay(10) —— 太短。
    //   冷启动时上电爬坡 + 各种初始化已经耗掉大部分时间，所以侥幸能工作；
    //   而热复位（按 RESET 键）时面板刚被上面这只脉冲复位、内部还在 settle，
    //   紧接着就灌入 0x99 解锁等命令 → 命令被拒 → 背光亮但屏无图形。
    //   这正是"上电正常、复位黑屏"的直接原因（与 DSI 带宽/PHY 无关）。
    delay(120);
  } else {
    esp_lcd_panel_io_tx_param(this->io_handle_, SW_RESET_CMD, nullptr, 0);
    // A software reset needs the same settling time before any further command.
    delay(120);
  }
  // need to know when the display is ready for SLPOUT command - will be 120ms after reset
  auto when = millis() + 120;
  err = esp_lcd_panel_init(this->handle_);
  if (err != ESP_OK) {
    this->smark_failed(LOG_STR("esp_lcd_init failed"), err);
    return;
  }
  size_t index = 0;
  auto &vec = this->init_sequence_;
  while (index != vec.size()) {
    if (vec.size() - index < 2) {
      this->mark_failed(LOG_STR("Malformed init sequence"));
      return;
    }
    uint8_t cmd = vec[index++];
    uint8_t x = vec[index++];
    if (x == DELAY_FLAG) {
      ESP_LOGD(TAG, "Delay %dms", cmd);
      delay(cmd);
    } else {
      uint8_t num_args = x & 0x7F;
      if (vec.size() - index < num_args) {
        this->mark_failed(LOG_STR("Malformed init sequence"));
        return;
      }
      if (cmd == SLEEP_OUT) {
        // are we ready, boots?
        int duration = when - millis();
        if (duration > 0) {
          delay(duration);
        }
      }
      const auto *ptr = vec.data() + index;
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE
      char hex_buf[format_hex_pretty_size(MIPI_DSI_MAX_CMD_LOG_BYTES)];
#endif
      ESP_LOGVV(TAG, "Command %02X, length %d, byte(s) %s", cmd, num_args,
                format_hex_pretty_to(hex_buf, ptr, num_args, '.'));
      err = esp_lcd_panel_io_tx_param(this->io_handle_, cmd, ptr, num_args);
      if (err != ESP_OK) {
        this->smark_failed(LOG_STR("lcd_panel_io_tx_param failed"), err);
        return;
      }
      index += num_args;
      if (cmd == SLEEP_OUT)
        delay(10);
    }
  }
  this->io_lock_ = xSemaphoreCreateBinary();
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = notify_refresh_ready,
  };

  err = (esp_lcd_dpi_panel_register_event_callbacks(this->handle_, &cbs, this->io_lock_));
  if (err != ESP_OK) {
    this->smark_failed(LOG_STR("Failed to register callbacks"), err);
    return;
  }

  ESP_LOGCONFIG(TAG, "MIPI DSI setup complete");
}

void MipiDsi::update() {
  if (this->auto_clear_enabled_) {
    this->clear();
  }
  if (this->show_test_card_) {
    this->test_card();
  } else if (this->page_ != nullptr) {
    this->page_->get_writer()(*this);
  } else if (this->writer_.has_value()) {
    (*this->writer_)(*this);
  } else {
    this->stop_poller();
  }
  if (this->buffer_ == nullptr || this->x_low_ > this->x_high_ || this->y_low_ > this->y_high_)
    return;
  ESP_LOGV(TAG, "x_low %d, y_low %d, x_high %d, y_high %d", this->x_low_, this->y_low_, this->x_high_, this->y_high_);
  int w = this->x_high_ - this->x_low_ + 1;
  int h = this->y_high_ - this->y_low_ + 1;
  this->write_to_display_(this->x_low_, this->y_low_, w, h, this->buffer_, this->x_low_, this->y_low_,
                          this->width_ - w - this->x_low_);
  // invalidate watermarks
  this->x_low_ = this->width_;
  this->y_low_ = this->height_;
  this->x_high_ = 0;
  this->y_high_ = 0;
}

void MipiDsi::draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr, display::ColorOrder order,
                             display::ColorBitness bitness, bool big_endian, int x_offset, int y_offset, int x_pad) {
  if (w <= 0 || h <= 0)
    return;
  // if color mapping is required, pass the buck.
  // note that endianness is not considered here - it is assumed to match!
  if (bitness != this->color_depth_) {
    display::Display::draw_pixels_at(x_start, y_start, w, h, ptr, order, bitness, big_endian, x_offset, y_offset,
                                     x_pad);
    return;
  }
  this->write_to_display_(x_start, y_start, w, h, ptr, x_offset, y_offset, x_pad);
}

void MipiDsi::write_to_display_(int x_start, int y_start, int w, int h, const uint8_t *ptr, int x_offset, int y_offset,
                                int x_pad) {
  esp_err_t err = ESP_OK;
  auto bytes_per_pixel = 3 - this->color_depth_;
  auto stride = (x_offset + w + x_pad) * bytes_per_pixel;
  ptr += y_offset * stride + x_offset * bytes_per_pixel;  // skip to the first pixel
  // x_ and y_offset are offsets into the source buffer, unrelated to our own offsets into the display.
  if (x_offset == 0 && x_pad == 0) {
    err = esp_lcd_panel_draw_bitmap(this->handle_, x_start, y_start, x_start + w, y_start + h, ptr);
    xSemaphoreTake(this->io_lock_, portMAX_DELAY);

  } else {
    // draw line by line
    for (int y = 0; y != h; y++) {
      err = esp_lcd_panel_draw_bitmap(this->handle_, x_start, y + y_start, x_start + w, y + y_start + 1, ptr);
      if (err != ESP_OK)
        break;
      ptr += stride;  // next line
      xSemaphoreTake(this->io_lock_, portMAX_DELAY);
    }
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "lcd_lcd_panel_draw_bitmap failed: %s", esp_err_to_name(err));
  }
}

bool MipiDsi::check_buffer_() {
  if (this->is_failed())
    return false;
  if (this->buffer_ != nullptr)
    return true;
  // this is dependent on the enum values.
  auto bytes_per_pixel = 3 - this->color_depth_;
  RAMAllocator<uint8_t> allocator;
  this->buffer_ = allocator.allocate(this->height_ * this->width_ * bytes_per_pixel);
  if (this->buffer_ == nullptr) {
    this->mark_failed(LOG_STR("Could not allocate buffer for display!"));
    return false;
  }
  return true;
}

void MipiDsi::draw_pixel_at(int x, int y, Color color) {
  if (!this->get_clipping().inside(x, y))
    return;

  switch (this->rotation_) {
    case display::DISPLAY_ROTATION_0_DEGREES:
      break;
    case display::DISPLAY_ROTATION_90_DEGREES:
      std::swap(x, y);
      x = this->width_ - x - 1;
      break;
    case display::DISPLAY_ROTATION_180_DEGREES:
      x = this->width_ - x - 1;
      y = this->height_ - y - 1;
      break;
    case display::DISPLAY_ROTATION_270_DEGREES:
      std::swap(x, y);
      y = this->height_ - y - 1;
      break;
  }
  if (x >= this->get_width_internal() || x < 0 || y >= this->get_height_internal() || y < 0) {
    return;
  }
  if (!this->check_buffer_())
    return;
  // ★★★ 2026-10-02 R5 面板相位补偿 —— 关键修正：必须按「整帧线性」取模，不能按「行内」取模
  //   【现象（用户实烧两轮，逐轮确认）】
  //     a) 只有 X_SHIFT 之前：整幅画面沿用户视角的**竖直**方向偏 10px，且**回绕** ——
  //        屏幕最上沿那 10px 露出的其实是画面**最底部**的内容（蓝/白角块下沿 = 1/4 块）。
  //     b) 用「行内取模」补偿 10px 之后：正片全对齐了，**只剩最顶上那 10px 条带整体偏左 1px**。
  //   【唯一自洽的模型（由 a、b 两条反推）】
  //     面板读这一帧时，**起点比我们早 10 个像素，而且是整帧线性的**（不是"每行各偏 10"）。
  //     即：面板位置 p（p = 行 r × width_ + 列 c）显示的是缓冲区**线性下标 (p − 10)**。
  //     · 对 c ≥ 10 的绝大多数像素：等价于"行内右移 10" ⇒ 行内取模补偿 10 刚好抵消 ✔
  //     · 对**每行最前面 10 列**（c ≤ 9）：p − 10 会跨到**上一行的最后 10 列**，
  //       而行内取模只搬了列、没搬行 ⇒ 这 10 列拿到的内容差**整整一行**。
  //       ★ 用户视角：这 10 列正是"最顶上那条 10px"，差一行 = **水平方向差 1px**
  //         ⇒ 完全对得上"只有补正到顶部的那 10 偏左了"。
  //     · 反过来也解释 (a)：当时没补偿，(r, c≤9) 拿到的是上一行末 10 列的原始内容，
  //       而那 10 列写的正是画面**最底部** 10 行的内容 ⇒ "底部卷到顶部" ✔
  //   【正确做法】把补偿作用在**整帧线性下标**上 —— 搬列的同时把"跨行"也搬对：
  //       pos = (y × width_ + x − 10) mod (width_ × height_)
  //     这样每行最前 10 列会从上一行的末 10 列取到**正确**内容，1px 台阶消失。
  //   【修完怎么判】屏幕最上面那条 10px 条带的内容必须与它下面**完全对齐**。
  //     最直观的验法见同目录 full_test.yaml：display lambda 里画了"顶部 + 中部"两把刻度尺，
  //     两把尺子的竖边应严丝合缝；差 1px 都会露出台阶。
  //   【为什么只能软件修】实测 hsync_back_porch 40→30 重烧，画面**完全不动**；
  //     porch 确实进了 esp_lcd_dpi_panel_config_t.video_timing（见本文件 84 行），
  //     且整帧走 write_to_display_ 的对齐整幅快路径（x_offset==0 && x_pad==0）。
  //   【调法】X_SHIFT = 0 关闭；与实测不符时改符号（−10 → +10，即把下面那个减法换成加法）。
  //     若哪天要退回旧的"行内取模"，把那两行换成：
  //       x = (x - X_SHIFT + width_) % width_;   pos = y * width_ + x;
  //   【覆盖面】display.cpp 的 filled_rectangle / horizontal_line / line / circle /
  //     filled_circle / rectangle 都最终经 draw_pixel_at ⇒ 只改这一处即全局生效。
  //     两个已核例外（本补偿**不**覆盖）：
  //       ① MipiDsi::fill() 被本文件重写为直接 std::fill_n —— 整屏同色，平移不影响观感。
  //       ② draw_pixels_at()（贴图/位图路径）绕过 draw_pixel_at ⇒ M4 显示图片时要另行补偿。
  static constexpr int X_SHIFT = 10;
  const size_t frame_px = static_cast<size_t>(this->width_) * static_cast<size_t>(this->height_);
  size_t pos = (static_cast<size_t>(y) * static_cast<size_t>(this->width_)) + static_cast<size_t>(x);
  pos = (pos + frame_px - static_cast<size_t>(X_SHIFT)) % frame_px;
  switch (this->color_depth_) {
    case display::COLOR_BITNESS_565: {
      auto *ptr_16 = reinterpret_cast<uint16_t *>(this->buffer_);
      uint8_t hi_byte = static_cast<uint8_t>(color.r & 0xF8) | (color.g >> 5);
      uint8_t lo_byte = static_cast<uint8_t>((color.g & 0x1C) << 3) | (color.b >> 3);
      uint16_t new_color = lo_byte | (hi_byte << 8);  // little endian
      if (ptr_16[pos] == new_color)
        return;
      ptr_16[pos] = new_color;
      break;
    }
    case display::COLOR_BITNESS_888:
      if (this->color_mode_ == display::COLOR_ORDER_BGR) {
        this->buffer_[pos * 3] = color.b;
        this->buffer_[pos * 3 + 1] = color.g;
        this->buffer_[pos * 3 + 2] = color.r;
      } else {
        this->buffer_[pos * 3] = color.r;
        this->buffer_[pos * 3 + 1] = color.g;
        this->buffer_[pos * 3 + 2] = color.b;
      }
      break;
    case display::COLOR_BITNESS_332:
      break;
  }
  // low and high watermark may speed up drawing from buffer
  if (x < this->x_low_)
    this->x_low_ = x;
  if (y < this->y_low_)
    this->y_low_ = y;
  if (x > this->x_high_)
    this->x_high_ = x;
  if (y > this->y_high_)
    this->y_high_ = y;
}
void MipiDsi::fill(Color color) {
  if (!this->check_buffer_())
    return;

  // If clipping is active, fall back to base implementation
  if (this->get_clipping().is_set()) {
    Display::fill(color);
    return;
  }

  switch (this->color_depth_) {
    case display::COLOR_BITNESS_565: {
      auto *ptr_16 = reinterpret_cast<uint16_t *>(this->buffer_);
      uint8_t hi_byte = static_cast<uint8_t>(color.r & 0xF8) | (color.g >> 5);
      uint8_t lo_byte = static_cast<uint8_t>((color.g & 0x1C) << 3) | (color.b >> 3);
      uint16_t new_color = lo_byte | (hi_byte << 8);  // little endian
      std::fill_n(ptr_16, this->width_ * this->height_, new_color);
      break;
    }

    case display::COLOR_BITNESS_888:
      if (this->color_mode_ == display::COLOR_ORDER_BGR) {
        for (size_t i = 0; i != this->width_ * this->height_; i++) {
          this->buffer_[i * 3 + 0] = color.b;
          this->buffer_[i * 3 + 1] = color.g;
          this->buffer_[i * 3 + 2] = color.r;
        }
      } else {
        for (size_t i = 0; i != this->width_ * this->height_; i++) {
          this->buffer_[i * 3 + 0] = color.r;
          this->buffer_[i * 3 + 1] = color.g;
          this->buffer_[i * 3 + 2] = color.b;
        }
      }

    default:
      break;
  }
}

int MipiDsi::get_width() {
  switch (this->rotation_) {
    case display::DISPLAY_ROTATION_90_DEGREES:
    case display::DISPLAY_ROTATION_270_DEGREES:
      return this->get_height_internal();
    case display::DISPLAY_ROTATION_0_DEGREES:
    case display::DISPLAY_ROTATION_180_DEGREES:
    default:
      return this->get_width_internal();
  }
}

int MipiDsi::get_height() {
  switch (this->rotation_) {
    case display::DISPLAY_ROTATION_0_DEGREES:
    case display::DISPLAY_ROTATION_180_DEGREES:
      return this->get_height_internal();
    case display::DISPLAY_ROTATION_90_DEGREES:
    case display::DISPLAY_ROTATION_270_DEGREES:
    default:
      return this->get_width_internal();
  }
}

static const uint8_t PIXEL_MODES[] = {0, 16, 18, 24};

void MipiDsi::dump_config() {
  ESP_LOGCONFIG(TAG,
                "MIPI_DSI RGB LCD"
                "\n  Model: %s"
                "\n  Width: %u"
                "\n  Height: %u"
                "\n  Rotation: %d degrees"
                "\n  DSI Lanes: %u"
                "\n  Lane Bit Rate: %.0fMbps"
                "\n  HSync Pulse Width: %u"
                "\n  HSync Back Porch: %u"
                "\n  HSync Front Porch: %u"
                "\n  VSync Pulse Width: %u"
                "\n  VSync Back Porch: %u"
                "\n  VSync Front Porch: %u"
                "\n  Buffer Color Depth: %d bit"
                "\n  Display Pixel Mode: %d bit"
                "\n  Invert Colors: %s"
                "\n  Pixel Clock: %.1fMHz",
                this->model_, this->width_, this->height_, this->rotation_, this->lanes_, this->lane_bit_rate_,
                this->hsync_pulse_width_, this->hsync_back_porch_, this->hsync_front_porch_, this->vsync_pulse_width_,
                this->vsync_back_porch_, this->vsync_front_porch_, (3 - this->color_depth_) * 8, this->pixel_mode_,
                YESNO(this->invert_colors_), this->pclk_frequency_);
  LOG_PIN("  Reset Pin ", this->reset_pin_);
}
}  // namespace esphome::mipi_dsi
#endif  // USE_ESP32_VARIANT_ESP32P4
