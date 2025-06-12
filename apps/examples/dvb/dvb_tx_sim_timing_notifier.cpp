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

#include "dvb_tx_sim_timing_notifier.h"
#include "srsran/ran/cyclic_prefix.h"
#include <future>
#include <thread>

using namespace srsran;
using namespace ofh;

/// Calculates the fractional part inside a second from the given time point.
static std::chrono::nanoseconds calculate_ns_fraction_from(gps_clock::time_point tp)
{
  auto tp_sec = std::chrono::time_point_cast<std::chrono::seconds>(tp);
  return tp - tp_sec;
}

/// Returns the symbol index inside a second.
static unsigned get_symbol_index(const unsigned frame_duration,
                                 std::chrono::nanoseconds                 fractional_ns,
                                 std::chrono::duration<double, std::nano> symbol_duration)
{
  // Perform operation with enough precision to avoid rounding errors when the amount of fractional nanoseconds is big.

  return (fractional_ns % frame_duration) / symbol_duration;
}

/// Returns current system slot.
static inline uint64_t get_current_system_slot(gps_clock::time_point    tp,
                                               std::chrono::nanoseconds slot_duration,
                                               unsigned                 nof_slots_per_system_frame)
{
  // Get the time since the epoch.
  auto time_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return (time_since_epoch / slot_duration) % nof_slots_per_system_frame;
}

/// Returns the difference between cur and prev taking into account a potential wrap arounds of the values.
static unsigned circular_distance(unsigned cur, unsigned prev, unsigned size)
{
  return (cur >= prev) ? (cur - prev) : (size + cur - prev);
}

dvb_tx_sim_timing_notifier::dvb_tx_sim_timing_notifier(srslog::basic_logger&  logger_,
                                                         srsran::task_executor& executor_,
                                                         unsigned int frame_period_) :
  logger(logger_),
  executor(executor_),
  frame_peroid(frame_period_),
  nof_symbols_per_slot(16),
  nof_slots_per_system_frame(1),
  symbol_duration(frame_period_/nof_symbols_per_slot),
  sleep_time(std::chrono::duration_cast<std::chrono::nanoseconds>(symbol_duration) / 16),
  slot_symbol_point(0,16)
{
  // The GPS time epoch starts on 1980.1.6 so make sure that the system time is set after this date.
  // For simplicity reasons, only allow dates after 1981.
  ::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  ::tm     utc_time;
  ::gmtime_r(&tt, &utc_time);
  int current_year = utc_time.tm_year + 1900;
  report_error_if_not(current_year >= 1981,
                      "The Open FrontHaul standard uses GPS time for synchronization. Make sure that system time is "
                      "set after the year 1981 since the GPS time epoch starts on 1980.1.6");
  logger.info("frame_period = {} symbol_duration = {}", frame_peroid, symbol_duration.count());
}

void dvb_tx_sim_timing_notifier::start(void)
{
  logger.info("Starting the realtime timing notifier");

  std::promise<void> p;
  std::future<void>  fut = p.get_future();

  if (!executor.defer([this, &p]() {
        status.store(worker_status::running, std::memory_order_relaxed);
        // Signal start() caller thread that the operation is complete.
        p.set_value();

        // waitting for second boundary
        auto ns_fraction    = calculate_ns_fraction_from(gps_clock::now());

        while(ns_fraction > symbol_duration) {
          std::this_thread::sleep_for(sleep_time);
          ns_fraction = calculate_ns_fraction_from(gps_clock::now());
        }
        logger.info("the first symbol comming");
        start_tp = std::chrono::time_point_cast<std::chrono::seconds>(gps_clock::now());
        previous_symb_index = 0;
        notify_slot_symbol_point(slot_symbol_point);
        timing_loop();
      })) {
    report_fatal_error("Unable to start the realtime timing worker");
  }

  // Block waiting for timing executor to start.
  fut.wait();

  logger.info("Started the realtime timing worker");
}

void dvb_tx_sim_timing_notifier::stop()
{
  logger.info("Requesting stop of the realtime timing worker");
  status.store(worker_status::stop_requested, std::memory_order_relaxed);

  // Wait for the timing thread to stop - this line also introduces a happens-before relationship with the clear on
  // ota_notifier.
  while (status.load(std::memory_order_acquire) != worker_status::stopped) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Clear the subscribed notifiers.
  ota_notifiers.clear();

  logger.info("Stopped the realtime timing worker");
}

void dvb_tx_sim_timing_notifier::timing_loop()
{
  while (SRSRAN_LIKELY(status.load(std::memory_order_relaxed) == worker_status::running)) {
    poll();
  }

  // Acquire/Release semantics - ota_notifiers is cleared and destructed by a different thread.
  status.store(worker_status::stopped, std::memory_order_release);
}

void dvb_tx_sim_timing_notifier::poll()
{
  auto now         = gps_clock::now() - start_tp;
  unsigned current_symbol_index = get_symbol_index(frame_peroid, now, symbol_duration);
  unsigned delta                = circular_distance(current_symbol_index, previous_symb_index, 16);
  previous_symb_index           = current_symbol_index;

  // Are we still in the same symbol as before?
  if (delta == 0) {
    std::this_thread::sleep_for(sleep_time);
    return;
  }

  // Check if we have missed more than one symbol.
  if (SRSRAN_UNLIKELY(delta > 1)) {
    logger.info("DVB tx sim timming worker woke up late, skipped '{}' symbols", delta);
  }
  if (SRSRAN_UNLIKELY(delta >= nof_symbols_per_slot)) {
    logger.warning("DVB tx sim timing worker woke up late, sleep time has been '{}us', or equivalently, '{}' symbols",
                   std::chrono::duration_cast<std::chrono::microseconds>(delta * symbol_duration).count(),
                   delta);
  }


  slot_symbol_point +=  delta;
  notify_slot_symbol_point(slot_symbol_point);
}

void dvb_tx_sim_timing_notifier::notify_slot_symbol_point(dvb_slot_symbol_point slot)
{
  for (auto* notifier : ota_notifiers) {
    notifier->on_new_symbol(slot);
  }
}

void dvb_tx_sim_timing_notifier::subscribe(span<dvb_symbol_boundary_notifier*> notifiers)
{
  // The defer() call in start() synchronizes the contents of ota_notifiers with the worker thread.
  ota_notifiers.assign(notifiers.begin(), notifiers.end());
}
