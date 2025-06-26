/*
 *
 * Copyright 2021-2025 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#pragma once

#include "srsran/ofh/timing/ofh_ota_symbol_boundary_notifier_manager.h"
#include "srsran/srslog/logger.h"
#include "srsran/support/executors/task_executor.h"
#include <atomic>

namespace srsran {

/// A GPS clock implementation.
struct gps_clock {
  using duration   = std::chrono::nanoseconds;
  using rep        = duration::rep;
  using period     = duration::period;
  using time_point = std::chrono::time_point<gps_clock>;
  /// Difference between Unix seconds to GPS seconds.
  /// GPS epoch: 1980.1.6 00:00:00 (UTC); Unix time epoch: 1970:1.1 00:00:00 UTC
  /// Last leap second added in 31st Dec 2016 by IERS.
  /// 1970:1.1 - 1980.1.6: 3657 days
  /// 3657*24*3600=315 964 800 seconds (Unix seconds value at 1980.1.6 00:00:00 (UTC))
  /// There are 18 leap seconds inserted after 1980.1.6 00:00:00 (UTC), which means GPS is 18 seconds larger.
  static constexpr uint64_t UNIX_TO_GPS_SECONDS_OFFSET = 315964800ULL - 18ULL;

  /// Offset for converting from UTC to GPS time including Alpha and Beta parameters.
  static constexpr std::chrono::nanoseconds gps_offset = std::chrono::seconds(UNIX_TO_GPS_SECONDS_OFFSET);

  static time_point now()
  {
    ::timespec ts;
    ::clock_gettime(CLOCK_REALTIME, &ts);

    time_point now(std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec));

    return now - gps_offset;
  }
};

class dvb_slot_symbol_point
{
  public:
    dvb_slot_symbol_point(const dvb_slot_symbol_point& symbol_point) :
      nof_symbols(symbol_point.get_nof_symbols()),  count_val(symbol_point.to_uint()) { }
    /// Takes a numerology, total count value and number of symbols.
    dvb_slot_symbol_point(uint16_t count_, uint8_t nof_symbols_) :
      nof_symbols(nof_symbols_), count_val(count_) {}
    /// Slot point.
    uint8_t get_frame() const
    {
      return count_val / nof_symbols;
    }
    uint16_t to_uint() const { return count_val; }
    uint8_t get_symbol_index() const
    {
      return count_val % nof_symbols;
    }
    uint8_t get_nof_symbols() const
    {
      return nof_symbols;
    }

    dvb_slot_symbol_point& operator+=(int jump)
    {
      count_val += jump;
      return *this;
    }

  private:
    uint8_t nof_symbols;
    uint16_t count_val;
};

class dvb_symbol_boundary_notifier
{
public:
  virtual ~dvb_symbol_boundary_notifier() = default;

  /// \brief Notifies a new DVB symbol boundary event.
  ///
  /// Notifies that the beginning of a new OTA symbol has started.
  ///
  /// \param[in] symbol_point Current slot and symbol point.
  virtual void on_new_symbol(dvb_slot_symbol_point symbol_point) = 0;
};

class dvb_symbol_boundary_notifier_manager
{
  public:
  /// Default destructor.
  virtual ~dvb_symbol_boundary_notifier_manager() = default;

  /// Subscribes the given notifiers to listen to OTA symbol boundary events.
  virtual void subscribe(span<dvb_symbol_boundary_notifier*> notifiers) = 0;
};

/// Realtime worker that generates OTA symbol notifications.
class dvb_tx_sim_timing_notifier : public dvb_symbol_boundary_notifier_manager
{
  enum class worker_status { running, stop_requested, stopped };

  srslog::basic_logger&                           logger;
  std::vector<dvb_symbol_boundary_notifier*> ota_notifiers;
  task_executor&                                  executor;
  const unsigned                                  frame_peroid;
  const unsigned                                  nof_symbols_per_slot;
  const unsigned                                  nof_slots_per_system_frame;
  const std::chrono::duration<double, std::nano>  symbol_duration;
  const std::chrono::nanoseconds                  sleep_time;
  dvb_slot_symbol_point                           slot_symbol_point;
public:
  gps_clock::time_point                           start_tp;
  unsigned                                        previous_symb_index = 0;
  std::atomic<worker_status>                      status{worker_status::running};

  dvb_tx_sim_timing_notifier(srslog::basic_logger& logger_, task_executor& executor_, const unsigned frame_period_);

  /// Starts operation of the timing notifier.
  void start(void);

  /// Stops operation of the timing notifier.
  void stop();

  unsigned get_current_frame() { return slot_symbol_point.get_frame(); }
  const dvb_slot_symbol_point& get_symbol_point() { return slot_symbol_point; }

  /// See interface for documentation.
  void subscribe(span<dvb_symbol_boundary_notifier*> notifiers) override;

private:
  /// Main timing loop.
  void timing_loop();

  /// Polls the system time checking for the start of a new OTA symbol.
  void poll();

  /// Notifies the given slot symbol point through the registered notifiers.
  void notify_slot_symbol_point(dvb_slot_symbol_point slot);
};

} // namespace srsran
