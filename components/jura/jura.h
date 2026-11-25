#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace esphome {
namespace jura {

static const char *const TAG = "jura";
static const char *const DUMP_TAG = "jura.dump";

// Non-blocking Jura UART protocol.
//
// Every ASCII character is sent as 4 UART bytes (2 payload bits each, in bit 2 and bit 5),
// followed by a pause before the next character. Responses use the same encoding and end with CRLF.
// Instead of blocking the main loop with delay(), commands are queued and processed step by step
// in loop(): one character per step on TX, and all available bytes per step on RX.
class Jura : public PollingComponent, public uart::UARTDevice {
 public:
  using ResponseCallback = std::function<void(const std::string &)>;

  // Timing as in the original f50-poc.ino sketch and github.com/opajonk/jura-f50-esphome, which runs
  // on the Impressa F50 with ESPHome 2026.10: 1 ms between the 4 UART bytes of a character,
  // 8 ms after the 4th byte before the next character.
  static constexpr uint32_t INTRA_BYTE_US = 1000;
  static constexpr uint32_t INTER_CHAR_US = 8000;
  // Max. time to wait for the CRLF of a response after the last character was sent.
  static constexpr uint32_t RESPONSE_TIMEOUT_MS = 2000;
  // Pause between the end of one transaction and the start of the next one.
  static constexpr uint32_t INTER_COMMAND_MS = 20;
  // Quiet period after a timeout in which late replies are discarded (see loop(), State::IDLE).
  static constexpr uint32_t LATE_REPLY_DRAIN_MS = 2000;
  // Guard against a runaway RX stream (longest known response, RT:0000, is ~67 characters).
  static constexpr size_t MAX_RESPONSE_LEN = 256;
  static constexpr size_t MAX_QUEUE_LEN = 8;
  // A gap this long inside a 4-byte group means bytes were lost: drop the partial group (resync).
  static constexpr uint32_t RX_FRAME_GAP_US = 20000;
  // Consecutive failed power polls before the power state is reported as unknown.
  static constexpr uint8_t POWER_MAX_FAILURES = 5;
  // After set_power(): poll RR:44 this often (front of the queue) until the machine confirms the new state,
  // and show the requested state in power_switch_state() for at most POWER_CONFIRM_MS.
  static constexpr uint32_t POWER_FAST_POLL_MS = 500;
  static constexpr uint32_t POWER_CONFIRM_MS = 10000;

  void set_model(const std::string &m) { model_ = m; }

  void register_metric_sensor(const std::string &key, sensor::Sensor *s) { numeric_[key] = s; }
  void register_text_sensor(const std::string &key, text_sensor::TextSensor *t) { text_[key] = t; }
  // Power state from RAM byte 0x44 (RR:44): 01 while on (heating, rinsing, ready), 00 while off.
  // Verified on the Impressa F50 (ty:EPSILON MASK 11), see github.com/opajonk/jura-f50-esphome.
  void set_power_sensor(binary_sensor::BinarySensor *s) { power_sensor_ = s; }
  void set_power_interval(uint32_t ms) { power_interval_ms_ = ms; }
  // IC: flags observed on the Impressa F50 (ty:EPSI XL M V01.04): byte A bit 5 = tap (steam/hot water)
  // open, byte A bit 1 = 0 in parts of maintenance programmes (e.g. heating and rinse prompt at the end of
  // descaling), 1 when ready and while off.
  // It stayed 1 while heating after power on, so it is not a heating flag (see set_heated_sensor()).
  void set_tap_open_sensor(binary_sensor::BinarySensor *s) { tap_open_sensor_ = s; }
  void set_ready_sensor(binary_sensor::BinarySensor *s) { ready_sensor_ = s; }
  // Maintenance flags in RAM byte 0x10 (RR:10), observed on the Impressa F50 (ty:EPSI XL M V01.04):
  // bit 6 = descaling needed (cleared by the descaling programme), bit 7 = cleaning needed.
  void set_descale_needed_sensor(binary_sensor::BinarySensor *s) { descale_needed_sensor_ = s; }
  void set_clean_needed_sensor(binary_sensor::BinarySensor *s) { clean_needed_sensor_ = s; }
  void set_maintenance_interval(uint32_t ms) { maintenance_interval_ms_ = ms; }
  // Startup rinse prompt ("Pflege druecken" after power on): RAM byte 0x30 bit 7, i.e. RR:30 word & 0x8000,
  // on the Impressa F50 (ty:EPSI XL M V01.04). Confirmed three times (dumps 2026-11-24/25 and live in HA):
  // set while the prompt is shown, 0 after rinsing and in all other recorded states. NOT set for the rinse
  // prompt at the end of the descaling programme. Reported only while on; sets the status "Rinse Needed"
  // and clears "ready".
  void set_startup_rinse_sensor(binary_sensor::BinarySensor *s) { startup_rinse_sensor_ = s; }
  // Heated (operating temperature reached): RAM byte 0x04 bit 2, i.e. RR:04 word & 0x0400 (the same bit as
  // RR:03 & 0x0004 in github.com/SkyFox277/esphome-jura). On the Impressa F50 (ty:EPSI XL M V01.04): 0 while
  // heating after power on and during descaling, 1 at the rinse prompt and when ready. It stays set for some
  // minutes after power off while the machine is warm, so it is reported only while on. While it is 0 the
  // status is "Heating" and "ready" is off.
  void set_heated_sensor(binary_sensor::BinarySensor *s) { heated_sensor_ = s; }

  // Machine clock from RAM bytes 0xBC/0xBD (RR:BC): binary hours and minutes. Read-only.
  // Verified on the Impressa F50 (ty:EPSILON MASK 11), see github.com/opajonk/jura-f50-esphome.
  void set_clock_sensor(text_sensor::TextSensor *s) { clock_sensor_ = s; }
  void set_clock_interval(uint32_t ms) { clock_interval_ms_ = ms; }

  // Diagnostic: read the whole RAM window RR:00..RR:FE (2 bytes per read) plus TY: and IC:, and log
  // every reply as "DUMP <cmd>=<reply>" (tag jura.dump, level INFO). Read-only commands only.
  // Reads are chained (one at a time), so normal polling keeps working in between. ~20 s total.
  void start_ram_dump() {
    if (dump_active_) {
      ESP_LOGW(DUMP_TAG, "Dump already running");
      return;
    }
    dump_active_ = true;
    dump_addr_ = 0;
    dump_retry_ = 0;
    ESP_LOGI(DUMP_TAG, "DUMP start uptime=%u", (unsigned) (millis() / 1000));
    enqueue_("TY:", [this](const std::string &r) {
      ESP_LOGI(DUMP_TAG, "DUMP TY:=%s", r.empty() ? "????" : r.c_str());
      enqueue_("IC:", [this](const std::string &r) {
        ESP_LOGI(DUMP_TAG, "DUMP IC:=%s", r.empty() ? "????" : r.c_str());
        dump_next_();
      });
    });
  }

  // Diagnostic: poll a RAM word (RR:xx) regularly, publish it as a number and log every change as
  // "WATCH RR:xx=yyyy" (tag jura.dump). Used to find which bytes track power, rinse prompt etc.
  void add_watch(uint8_t addr, sensor::Sensor *s) { watches_.push_back({addr, s, -1}); }
  void set_watch_interval(uint32_t ms) { watch_interval_ms_ = ms; }

  // Poll the power state now, e.g. right after AN:01 / AN:02, for quick feedback.
  void request_power_update() {
    if (power_sensor_ != nullptr)
      poll_power_();
  }

  // Switch the machine on (AN:01) or off (AN:02), for a template switch:
  //   switch:
  //     - platform: template
  //       lambda: return id(jura_main).power_switch_state();
  //       turn_on_action:  - lambda: id(jura_main).set_power(true);
  //       turn_off_action: - lambda: id(jura_main).set_power(false);
  // power_switch_state() reports the requested state at once (no bouncing back in the UI). RR:44 is then
  // polled every POWER_FAST_POLL_MS until the machine confirms it; after POWER_CONFIRM_MS without
  // confirmation the switch shows the real power state again.
  void set_power(bool on) {
    power_target_ = on;
    power_target_active_ = true;
    power_target_since_ms_ = millis();
    send_command(on ? "AN:01" : "AN:02", [this](const std::string &) { fast_power_poll_(); });
  }
  optional<bool> power_switch_state() const {
    if (power_target_active_)
      return power_target_;
    if (power_sensor_ == nullptr || !power_known_)
      return {};
    return power_on_;
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override {
    if (status_interval_ms_ > 0) {
      set_interval("status", status_interval_ms_, [this]() { poll_ic_(); });
      poll_ic_();
    }
    if (power_sensor_ != nullptr) {
      set_interval("power", power_interval_ms_, [this]() { poll_power_(); });
      poll_power_();
    }
    if (clock_sensor_ != nullptr) {
      set_interval("clock", clock_interval_ms_, [this]() { poll_clock_(); });
      poll_clock_();
    }
    if (!watches_.empty())
      set_interval("watch", watch_interval_ms_, [this]() { poll_watch_(); });
    if (descale_needed_sensor_ != nullptr || clean_needed_sensor_ != nullptr) {
      set_interval("maintenance", maintenance_interval_ms_, [this]() { poll_maintenance_(); });
      poll_maintenance_();
    }
    if (startup_rinse_sensor_ != nullptr) {
      set_interval("rinse", status_interval_ms_ > 0 ? status_interval_ms_ : 5000, [this]() { poll_rinse_(); });
      poll_rinse_();
    }
    if (heated_sensor_ != nullptr) {
      set_interval("heat", status_interval_ms_ > 0 ? status_interval_ms_ : 5000, [this]() { poll_heat_(); });
      poll_heat_();
    }
  }

  // Queue a command for the machine. The optional callback receives the response
  // (without CRLF), or an empty string on timeout.
  void send_command(const std::string &cmd, ResponseCallback cb = nullptr) {
    // User commands (buttons) go before queued polling commands.
    enqueue_(cmd, std::move(cb), /*front=*/true);
  }

  // Kept for compatibility with existing YAML lambdas. Since the protocol no longer blocks,
  // the command is queued and an empty string is returned immediately.
  // Use send_command(cmd, callback) to get the response.
  std::string cmd2jura(const std::string &cmd) {
    send_command(cmd);
    return "";
  }

  // Poll the IC: status flags on their own interval instead of together with the RT: counters.
  // IC: is short (~0.1 s on the bus), RT:0000 is the longest reply (~0.4 s).
  void set_status_interval(uint32_t ms) { status_interval_ms_ = ms; }
  // Bits of IC: byte A to leave out of the "IC Bits" diagnostic sensor and its log line, e.g. 0x40 for
  // a bit that toggles all the time (F50: byte A bit 6, meaning unknown). Status sensors are unaffected.
  void set_ic_bits_ignore_mask(uint8_t mask) { ic_bits_ignore_mask_ = mask; }

  void update() override {
    poll_rt_();
    if (status_interval_ms_ == 0)
      poll_ic_();
  }

  void loop() override {
    const uint32_t now_ms = millis();
    switch (state_) {
      case State::IDLE: {
        // After a timeout, discard everything that still arrives for a while: RR: replies carry no address
        // ("rr:0121"), so a late reply would otherwise be taken as the reply to the next RR: command.
        if (drain_until_ms_ != 0) {
          uint8_t junk;
          while (available() && read_byte(&junk))
            drained_bytes_++;
          if ((int32_t) (now_ms - drain_until_ms_) < 0)
            return;
          if (drained_bytes_ > 0)
            ESP_LOGD(TAG, "Discarded %u late bytes after timeout", (unsigned) drained_bytes_);
          drain_until_ms_ = 0;
          drained_bytes_ = 0;
        }
        if (queue_.empty() || (now_ms - last_done_ms_) < INTER_COMMAND_MS)
          return;
        current_ = std::move(queue_.front());
        queue_.pop_front();
        // Drop stale input before starting a new transaction.
        uint8_t dummy;
        while (available())
          read_byte(&dummy);
        tx_buf_ = current_.cmd + "\r\n";
        tx_pos_ = 0;
        tx_frame_idx_ = 4;
        rx_buf_.clear();
        rx_raw_ = 0;
        rx_shift_ = 0;
        high_freq_.start();
        state_ = State::SENDING;
        tx_step_();
        break;
      }
      case State::SENDING: {
        if ((micros() - last_tx_us_) < tx_wait_us_)
          return;
        if (!tx_step_()) {
          rx_start_ms_ = now_ms;
          state_ = State::RECEIVING;
        }
        break;
      }
      case State::RECEIVING: {
        if (rx_shift_ > 0 && (micros() - rx_last_us_) > RX_FRAME_GAP_US) {
          ESP_LOGW(TAG, "Dropping incomplete frame (%u/4 bytes) in response to '%s'", (unsigned) (rx_shift_ / 2),
                   current_.cmd.c_str());
          rx_raw_ = 0;
          rx_shift_ = 0;
        }
        uint8_t raw;
        while (available() && read_byte(&raw)) {
          rx_last_us_ = micros();
          if (decode_rx_byte_(raw)) {
            if (reply_matches_(current_.cmd, rx_buf_)) {
              finish_(true);
              return;
            }
            // Typically a late reply to a command that already timed out: drop it, keep waiting.
            ESP_LOGD(TAG, "Discarding stray line '%s' while waiting for reply to '%s'", rx_buf_.c_str(),
                     current_.cmd.c_str());
            rx_buf_.clear();
          }
        }
        if ((now_ms - rx_start_ms_) > RESPONSE_TIMEOUT_MS) {
          ESP_LOGW(TAG, "Timeout waiting for response to '%s' (%u chars received)", current_.cmd.c_str(),
                   (unsigned) rx_buf_.size());
          finish_(false);
        }
        break;
      }
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Jura:");
    ESP_LOGCONFIG(TAG, "  Model: %s", model_.c_str());
    LOG_UPDATE_INTERVAL(this);
    if (power_sensor_ != nullptr) {
      LOG_BINARY_SENSOR("  ", "Power", power_sensor_);
      ESP_LOGCONFIG(TAG, "  Power interval: %u ms", (unsigned) power_interval_ms_);
    }
    if (clock_sensor_ != nullptr) {
      LOG_TEXT_SENSOR("  ", "Clock", clock_sensor_);
      ESP_LOGCONFIG(TAG, "  Clock interval: %u ms", (unsigned) clock_interval_ms_);
    }
  }

 protected:
  enum class State : uint8_t { IDLE, SENDING, RECEIVING };

  struct Command {
    std::string cmd;
    ResponseCallback cb;
  };

  void enqueue_(const std::string &cmd, ResponseCallback cb, bool front = false) {
    if (queue_.size() >= MAX_QUEUE_LEN) {
      ESP_LOGW(TAG, "Command queue full, dropping '%s'", cmd.c_str());
      if (cb)
        cb("");
      return;
    }
    Command c{cmd, std::move(cb)};
    if (front)
      queue_.push_front(std::move(c));
    else
      queue_.push_back(std::move(c));
  }

  // Send the next UART byte. Returns false once the whole command (incl. CRLF) has been sent
  // and the pause after the last character has elapsed.
  bool tx_step_() {
    if (tx_frame_idx_ >= 4) {
      if (tx_pos_ >= tx_buf_.size())
        return false;
      const uint8_t src = static_cast<uint8_t>(tx_buf_[tx_pos_++]);
      for (int i = 0; i < 4; i++) {
        uint8_t raw = 0xFF;
        const int s = i * 2;
        if (!((src >> s) & 1))
          raw &= ~(1 << 2);
        if (!((src >> (s + 1)) & 1))
          raw &= ~(1 << 5);
        tx_frame_[i] = raw;
      }
      tx_frame_idx_ = 0;
    }
    write_byte(tx_frame_[tx_frame_idx_++]);
    last_tx_us_ = micros();
    tx_wait_us_ = (tx_frame_idx_ >= 4) ? INTER_CHAR_US : INTRA_BYTE_US;
    return true;
  }

  // A reply belongs to a command if it starts with the command's 2-letter prefix in lower case
  // ("IC:" -> "ic:", "RR:44" -> "rr:"), or with "ok:" for commands that only acknowledge (AN:, FA:, DA:).
  static bool reply_matches_(const std::string &cmd, const std::string &reply) {
    if (reply.size() < 3 || reply[2] != ':')
      return false;
    if (reply.compare(0, 3, "ok:") == 0)
      return true;
    return cmd.size() >= 2 && reply[0] == (char) tolower((unsigned char) cmd[0]) &&
           reply[1] == (char) tolower((unsigned char) cmd[1]);
  }

  // Decode one raw UART byte. Returns true once a complete line (CRLF) was received.
  bool decode_rx_byte_(uint8_t raw) {
    if ((raw >> 2) & 1)
      rx_raw_ |= (1 << rx_shift_);
    if ((raw >> 5) & 1)
      rx_raw_ |= (1 << (rx_shift_ + 1));
    rx_shift_ += 2;
    if (rx_shift_ < 8)
      return false;

    rx_buf_.push_back(static_cast<char>(rx_raw_));
    rx_raw_ = 0;
    rx_shift_ = 0;

    const size_t n = rx_buf_.size();
    if (n >= 2 && rx_buf_[n - 2] == '\r' && rx_buf_[n - 1] == '\n') {
      rx_buf_.resize(n - 2);
      return true;
    }
    if (n > MAX_RESPONSE_LEN) {
      ESP_LOGW(TAG, "Response to '%s' exceeds %u chars without CRLF, discarding", current_.cmd.c_str(),
               (unsigned) MAX_RESPONSE_LEN);
      rx_buf_.clear();
    }
    return false;
  }

  void finish_(bool ok) {
    high_freq_.stop();
    state_ = State::IDLE;
    last_done_ms_ = millis();
    if (!ok) {
      drain_until_ms_ = last_done_ms_ + LATE_REPLY_DRAIN_MS;
      if (drain_until_ms_ == 0)
        drain_until_ms_ = 1;
    }
    Command done = std::move(current_);
    current_ = Command{};
    ESP_LOGV(TAG, "'%s' -> '%s'", done.cmd.c_str(), ok ? rx_buf_.c_str() : "<timeout>");
    if (done.cb)
      done.cb(ok ? rx_buf_ : std::string());
  }

  // ---------- response handlers ----------

  // A valid reply starts with the lower-case command prefix (e.g. "ic:") followed by hex digits only.
  // Garbled replies happen on this bus; publishing them would make the status sensors flap.
  static bool is_valid_reply_(const std::string &r, const char *prefix, size_t min_len) {
    if (r.size() < min_len || r.compare(0, 3, prefix) != 0)
      return false;
    for (size_t i = 3; i < r.size(); i++) {
      if (!isxdigit(static_cast<unsigned char>(r[i])))
        return false;
    }
    return true;
  }

  // fast = true: after set_power(), at the front of the queue and independent of the regular poll.
  void poll_power_(bool fast = false) {
    bool &pending = fast ? power_fast_pending_ : power_pending_;
    if (pending)
      return;
    pending = true;
    enqueue_(
        "RR:44",
        [this, fast](const std::string &r) {
          (fast ? power_fast_pending_ : power_pending_) = false;
          handle_power_(r);
        },
        /*front=*/fast);
  }

  void handle_power_(const std::string &r) {
    if (r.size() == 7 && is_valid_reply_(r, "rr:", 7)) {
      power_failures_ = 0;
      const bool on = (r[3] != '0' || r[4] != '0');
      const bool changed = !power_known_ || on != power_on_;
      power_on_ = on;
      power_known_ = true;
      if (power_target_active_ && power_on_ == power_target_) {
        power_target_active_ = false;
        cancel_timeout("power_fast");
        ESP_LOGI(TAG, "Power %s confirmed after %u ms", power_on_ ? "on" : "off",
                 (unsigned) (millis() - power_target_since_ms_));
      }
      power_sensor_->publish_state(power_on_);
      if (changed)
        on_power_changed_();
      return;
    }
    ESP_LOGW(TAG, "Unexpected RR:44 response '%s'", r.c_str());
    if (power_failures_ < POWER_MAX_FAILURES && ++power_failures_ == POWER_MAX_FAILURES) {
      power_known_ = false;
      power_sensor_->invalidate_state();
    }
  }

  // The sensors that depend on power would otherwise follow only with their own polls (up to the status
  // interval). Republish them from the last values at once (all off when switched off), then read
  // RR:30, RR:04 and IC: again right away, in this order, so "ready" and the status use fresh values.
  void on_power_changed_() {
    if (heated_sensor_ != nullptr && heat_known_)
      heated_sensor_->publish_state(at_temp_ && power_on_);
    if (startup_rinse_sensor_ != nullptr && rinse_known_)
      startup_rinse_sensor_->publish_state(rinse_needed_ && power_on_);
    if (startup_rinse_sensor_ != nullptr)
      poll_rinse_();
    if (heated_sensor_ != nullptr)
      poll_heat_();
    poll_ic_();
  }

  // Poll RR:44 every POWER_FAST_POLL_MS until handle_power_() sees the requested state or time runs out.
  void fast_power_poll_() {
    if (!power_target_active_)
      return;
    if ((millis() - power_target_since_ms_) > POWER_CONFIRM_MS) {
      power_target_active_ = false;
      ESP_LOGW(TAG, "Machine did not confirm power %s within %u ms", power_target_ ? "on" : "off",
               (unsigned) POWER_CONFIRM_MS);
      return;
    }
    if (power_sensor_ != nullptr)
      poll_power_(/*fast=*/true);
    set_timeout("power_fast", POWER_FAST_POLL_MS, [this]() { fast_power_poll_(); });
  }

  struct Watch {
    uint8_t addr;
    sensor::Sensor *sensor;
    int32_t last;
  };

  void poll_watch_() {
    if (watch_pending_ || dump_active_)
      return;
    watch_pending_ = true;
    watch_next_(0);
  }

  void watch_next_(size_t i) {
    if (i >= watches_.size()) {
      watch_pending_ = false;
      return;
    }
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "RR:%02X", (unsigned) watches_[i].addr);
    const std::string c(cmd);
    enqueue_(c, [this, c, i](const std::string &r) {
      if (r.size() == 7 && is_valid_reply_(r, "rr:", 7)) {
        const int32_t v = static_cast<int32_t>(strtoul(r.substr(3, 4).c_str(), nullptr, 16));
        Watch &w = watches_[i];
        if (v != w.last) {
          ESP_LOGI(DUMP_TAG, "WATCH %s=%s", c.c_str(), r.c_str() + 3);
          w.last = v;
          w.sensor->publish_state(v);
        }
      }
      watch_next_(i + 1);
    });
  }

  void dump_next_() {
    if (dump_addr_ > 0xFE) {
      ESP_LOGI(DUMP_TAG, "DUMP done");
      dump_active_ = false;
      return;
    }
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "RR:%02X", (unsigned) dump_addr_);
    const std::string c(cmd);
    enqueue_(c, [this, c](const std::string &r) {
      const bool ok = r.size() == 7 && is_valid_reply_(r, "rr:", 7);
      // Retry a missing/garbled read up to DUMP_MAX_RETRIES times, so a dump has no gaps that could hide a flag.
      if (!ok && dump_retry_ < DUMP_MAX_RETRIES) {
        dump_retry_++;
        dump_next_();
        return;
      }
      ESP_LOGI(DUMP_TAG, "DUMP %s=%s", c.c_str(), ok ? r.c_str() + 3 : "????");
      dump_retry_ = 0;
      dump_addr_ += 2;
      dump_next_();
    });
  }

  // Skip a poll while the previous one of the same kind is still queued (e.g. machine not answering).
  void poll_rt_() {
    if (rt_pending_)
      return;
    rt_pending_ = true;
    enqueue_("RT:0000", [this](const std::string &r) {
      rt_pending_ = false;
      handle_rt_(r);
    });
  }

  void poll_ic_() {
    if (ic_pending_)
      return;
    ic_pending_ = true;
    enqueue_("IC:", [this](const std::string &r) {
      ic_pending_ = false;
      handle_ic_(r);
    });
  }

  void poll_rinse_() {
    if (rinse_pending_)
      return;
    rinse_pending_ = true;
    enqueue_("RR:30", [this](const std::string &r) {
      rinse_pending_ = false;
      if (r.size() != 7 || !is_valid_reply_(r, "rr:", 7)) {
        ESP_LOGW(TAG, "Unexpected RR:30 response '%s'", r.c_str());
        return;  // keep the last valid state
      }
      const uint8_t b30 = static_cast<uint8_t>(strtoul(r.substr(3, 2).c_str(), nullptr, 16));
      rinse_needed_ = (b30 >> 7) & 1;
      rinse_known_ = true;
      const bool known_off = power_sensor_ != nullptr && power_known_ && !power_on_;
      startup_rinse_sensor_->publish_state(rinse_needed_ && !known_off);
    });
  }

  void poll_heat_() {
    if (heat_pending_)
      return;
    heat_pending_ = true;
    enqueue_("RR:04", [this](const std::string &r) {
      heat_pending_ = false;
      if (r.size() != 7 || !is_valid_reply_(r, "rr:", 7)) {
        ESP_LOGW(TAG, "Unexpected RR:04 response '%s'", r.c_str());
        return;  // keep the last valid state
      }
      const uint8_t b04 = static_cast<uint8_t>(strtoul(r.substr(3, 2).c_str(), nullptr, 16));
      at_temp_ = (b04 >> 2) & 1;
      heat_known_ = true;
      const bool known_off = power_sensor_ != nullptr && power_known_ && !power_on_;
      heated_sensor_->publish_state(at_temp_ && !known_off);
    });
  }

  void poll_maintenance_() {
    if (maintenance_pending_)
      return;
    maintenance_pending_ = true;
    enqueue_("RR:10", [this](const std::string &r) {
      maintenance_pending_ = false;
      if (r.size() != 7 || !is_valid_reply_(r, "rr:", 7)) {
        ESP_LOGW(TAG, "Unexpected RR:10 response '%s'", r.c_str());
        return;  // keep the last valid state
      }
      const uint8_t flags = static_cast<uint8_t>(strtoul(r.substr(3, 2).c_str(), nullptr, 16));
      if (descale_needed_sensor_ != nullptr)
        descale_needed_sensor_->publish_state((flags >> 6) & 1);
      if (clean_needed_sensor_ != nullptr)
        clean_needed_sensor_->publish_state((flags >> 7) & 1);
    });
  }

  void poll_clock_() {
    if (clock_pending_)
      return;
    clock_pending_ = true;
    enqueue_("RR:BC", [this](const std::string &r) {
      clock_pending_ = false;
      if (r.size() != 7 || !is_valid_reply_(r, "rr:", 7)) {
        ESP_LOGW(TAG, "Unexpected RR:BC response '%s'", r.c_str());
        return;  // keep the last valid time
      }
      const unsigned word = static_cast<unsigned>(strtoul(r.substr(3, 4).c_str(), nullptr, 16));
      const unsigned hours = word >> 8, minutes = word & 0xFF;
      if (hours > 23 || minutes > 59) {
        ESP_LOGW(TAG, "Implausible clock value '%s'", r.c_str());
        return;
      }
      // An unset clock and midnight both read as zero.
      char buf[8];
      if (word == 0)
        snprintf(buf, sizeof(buf), "--:--");
      else
        snprintf(buf, sizeof(buf), "%02u:%02u", hours, minutes);
      if (!clock_sensor_->has_state() || clock_sensor_->state != buf)
        clock_sensor_->publish_state(buf);
    });
  }

  void handle_rt_(const std::string &result) {
    if (!is_valid_reply_(result, "rt:", 64)) {
      ESP_LOGW(TAG, "Unexpected RT:0000 response len=%d", (int) result.size());
      return;
    }
    // Parse all counters available (4 hex chars per field starting at pos=3)
    std::vector<long> current = parse_all_counters_(result);
    for (int n = 1; n <= 16; n++)
      publish_number("counter_" + std::to_string(n), get_counter_n_(current, n));
    publish_counter_changes_(current);
  }

  void handle_ic_(const std::string &ic) {
    if (!is_valid_reply_(ic, "ic:", 7)) {
      ESP_LOGW(TAG, "Unexpected IC response '%s'", ic.c_str());
      return;
    }
    const uint8_t a = static_cast<uint8_t>(strtol(ic.substr(3, 2).c_str(), nullptr, 16));
    const uint8_t b = static_cast<uint8_t>(strtol(ic.substr(5, 2).c_str(), nullptr, 16));

    publish_ic_bits_if_changed_(static_cast<uint8_t>(a & ~ic_bits_ignore_mask_), b);

    // Tray (A bit 4) and water tank (A bit 3, 0 = missing/empty) are owner-verified on the Impressa F50.
    // left_ready / right_busy come from the F7 mapping and are not verified on the F50.
    const bool tray_present = (a >> 4) & 1;
    const bool left_ready = (a >> 2) & 1;
    const bool tank_ok = (a >> 3) & 1;
    const bool right_busy = (b >> 6) & 1;

    const bool tap_open = (a >> 5) & 1;
    // A bit 1 is 0 while a programme runs and also 1 while off, so "ready" additionally requires power on.
    const bool ic_idle = (a >> 1) & 1;
    const bool known_off = power_sensor_ != nullptr && power_known_ && !power_on_;
    const bool rinse = startup_rinse_sensor_ != nullptr && rinse_known_ && rinse_needed_;
    // With the heated sensor (RR:04), "ready" also needs the operating temperature; until the first
    // valid RR:04 reply it is treated as not reached.
    const bool heating = heated_sensor_ != nullptr && heat_known_ && !at_temp_;
    const bool temp_ok = heated_sensor_ == nullptr || (heat_known_ && at_temp_);
    if (tap_open_sensor_ != nullptr)
      tap_open_sensor_->publish_state(tap_open);
    if (ready_sensor_ != nullptr)
      ready_sensor_->publish_state(ic_idle && temp_ok && !known_off && !rinse);

    std::string machine_status = "Ready";
    if (model_ == "F50") {
      // Impressa F50: highest priority last.
      if (!ic_idle)
        machine_status = "Not Ready";  // programme running (or heating, without the heated sensor)
      if (rinse)
        machine_status = "Rinse Needed";  // startup rinse prompt
      if (heating)
        machine_status = "Heating";  // the rinse prompt is already set while heating, the display shows heating
      if (tap_open)
        machine_status = "Tap Open";
      if (!tank_ok)
        machine_status = "Fill Tank";
      if (!tray_present)
        machine_status = "Tray Missing";
    } else {
      if (!tray_present)
        machine_status = "Tray Missing";
      if (!tank_ok)
        machine_status = "Fill Tank";
      if (right_busy)
        machine_status = "Busy (Milk Drink)";
      if (!left_ready)
        machine_status = "Busy (Coffee Drink)";
    }
    // The IC: flags say nothing about power; with a power sensor, report an off machine as such.
    if (power_sensor_ != nullptr && power_known_ && !power_on_)
      machine_status = "Off";

    publish_text("tray_status", tray_present ? "Present" : "Missing");
    publish_text("water_tank_status", tank_ok ? "OK" : "Fill Tank");
    publish_text("machine_status", machine_status);
  }

  // ---------- publishing helpers ----------

  void publish_number(const std::string &key, float value) {
    auto it = numeric_.find(key);
    if (it != numeric_.end() && it->second)
      it->second->publish_state(value);
  }
  void publish_text(const std::string &key, const std::string &value) {
    auto it = text_.find(key);
    if (it == text_.end() || !it->second)
      return;
    // Only publish changes, so Home Assistant does not record identical states every poll.
    if (it->second->has_state() && it->second->state == value)
      return;
    it->second->publish_state(value);
  }

  // Return counter_n (1-based) from vector or -1 if missing
  long get_counter_n_(const std::vector<long> &v, int n) const {
    const size_t idx = (n >= 1) ? (size_t) (n - 1) : (size_t) -1;
    if (idx < v.size())
      return v[idx];
    return -1;
  }

  // Parse all 4-hex counters from RT payload starting at pos=3
  std::vector<long> parse_all_counters_(const std::string &rt) const {
    std::vector<long> out;
    for (size_t pos = 3; pos + 4 <= rt.size(); pos += 4)
      out.push_back(strtol(rt.substr(pos, 4).c_str(), nullptr, 16));
    return out;
  }

  void publish_counter_changes_(const std::vector<long> &current) {
    if (!last_counters_initialized_) {
      last_counters_ = current;
      last_counters_initialized_ = true;
      return;
    }

    std::string msg;
    bool any = false;
    const size_t max_n = std::max(last_counters_.size(), current.size());
    char buf[48];

    for (size_t i = 0; i < max_n; ++i) {
      long prev = (i < last_counters_.size()) ? last_counters_[i] : -1;
      long now = (i < current.size()) ? current[i] : -1;
      if (prev != now) {
        if (any)
          msg += ", ";
        snprintf(buf, sizeof(buf), "counter_%u %ld→%ld", (unsigned) (i + 1), prev, now);
        msg += buf;
        any = true;
      }
    }

    if (any) {
      publish_text("counters_changed", msg);
      ESP_LOGD(TAG, "Changed: %s", msg.c_str());
    }

    last_counters_ = current;
  }

  static inline void byte_to_bits(uint8_t v, char out[9]) {
    for (int i = 7; i >= 0; --i)
      out[7 - i] = (v & (1u << i)) ? '1' : '0';
    out[8] = '\0';
  }

  void publish_ic_bits_if_changed_(uint8_t a, uint8_t b) {
    if (!ic_bits_initialized_ || a != last_ic_a_ || b != last_ic_b_) {
      char abits[9], bbits[9], buf[32];
      byte_to_bits(a, abits);
      byte_to_bits(b, bbits);
      snprintf(buf, sizeof(buf), "A=%s B=%s", abits, bbits);
      publish_text("ic_bits", std::string(buf));

      last_ic_a_ = a;
      last_ic_b_ = b;
      ic_bits_initialized_ = true;

      ESP_LOGD(TAG, "IC bits changed: %s", buf);
    }
  }

  std::string model_{"UNKNOWN"};

  std::map<std::string, sensor::Sensor *> numeric_;
  std::map<std::string, text_sensor::TextSensor *> text_;

  // protocol state
  State state_{State::IDLE};
  std::deque<Command> queue_;
  Command current_;
  std::string tx_buf_;
  size_t tx_pos_{0};
  std::string rx_buf_;
  uint8_t rx_raw_{0};
  uint8_t rx_shift_{0};
  uint8_t tx_frame_[4]{};
  uint8_t tx_frame_idx_{4};
  uint32_t last_tx_us_{0};
  uint32_t tx_wait_us_{0};
  uint32_t rx_last_us_{0};
  uint32_t rx_start_ms_{0};
  uint32_t last_done_ms_{0};
  uint32_t drain_until_ms_{0};
  uint32_t drained_bytes_{0};
  bool rt_pending_{false};
  bool ic_pending_{false};
  uint32_t status_interval_ms_{0};
  uint8_t ic_bits_ignore_mask_{0};
  HighFrequencyLoopRequester high_freq_;

  binary_sensor::BinarySensor *power_sensor_{nullptr};
  uint32_t power_interval_ms_{10000};
  bool power_pending_{false};
  bool power_on_{false};
  bool power_known_{false};
  uint8_t power_failures_{0};
  bool power_fast_pending_{false};
  bool power_target_{false};
  bool power_target_active_{false};
  uint32_t power_target_since_ms_{0};

  binary_sensor::BinarySensor *tap_open_sensor_{nullptr};
  binary_sensor::BinarySensor *ready_sensor_{nullptr};
  binary_sensor::BinarySensor *descale_needed_sensor_{nullptr};
  binary_sensor::BinarySensor *clean_needed_sensor_{nullptr};
  binary_sensor::BinarySensor *startup_rinse_sensor_{nullptr};
  bool rinse_pending_{false};
  bool rinse_needed_{false};
  bool rinse_known_{false};
  binary_sensor::BinarySensor *heated_sensor_{nullptr};
  bool heat_pending_{false};
  bool at_temp_{false};
  bool heat_known_{false};
  uint32_t maintenance_interval_ms_{60000};
  bool maintenance_pending_{false};

  text_sensor::TextSensor *clock_sensor_{nullptr};
  uint32_t clock_interval_ms_{30000};
  bool clock_pending_{false};

  static constexpr uint8_t DUMP_MAX_RETRIES = 2;
  bool dump_active_{false};
  uint16_t dump_addr_{0};
  uint8_t dump_retry_{0};

  std::vector<Watch> watches_;
  uint32_t watch_interval_ms_{5000};
  bool watch_pending_{false};

  std::vector<long> last_counters_;
  bool last_counters_initialized_{false};

  uint8_t last_ic_a_{0};
  uint8_t last_ic_b_{0};
  bool ic_bits_initialized_{false};
};

}  // namespace jura
}  // namespace esphome
