/*
 *
 * Copyright 2021-2024 Software Radio Systems Limited
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

 #include "dvb_frame_pool.h"
 #include "dvb_tx_sim_timing_notifier.h"
 #include "srsran/support/error_handling.h"

 namespace srsran {

 /// Scoped helper class that retrieves an Ethernet frame buffer from a buffer pool and marks it as ready to send upon
 /// destruction.
 class scoped_frame_buffer
 {
   ether::dvb_frame_pool&          frame_pool;
   const dvb_slot_symbol_point  symbol_point;
   const span<ether::frame_buffer> frames;

 public:
   /// On construction, acquire Ethernet frame buffers for the given slot, symbol and Open Fronthaul type.
   scoped_frame_buffer(ether::dvb_frame_pool& frame_pool_,
                       dvb_slot_symbol_point      symbol_point_) :
     frame_pool(frame_pool_), symbol_point(symbol_point_), frames(frame_pool.get_frame_buffers(symbol_point_))
   {
   }

   /// Returns the frame retrieved from the pool.
   ether::frame_buffer& get_next_frame()
   {
     for (auto& frame : frames) {
       if (frame.empty()) {
         return frame;
       }
     }

     srsran_terminate("No empty Ethernet frame available in frame '{}' symbol '{}'\n",
                      symbol_point.get_frame(),
                      symbol_point.get_symbol_index());
   }

   bool empty() const { return frames.empty(); }

   /// Destructor marks the acquired buffers as ready to be sent.
   ~scoped_frame_buffer()
   {
     if (!frames.empty()) {
       frame_pool.push_frame_buffers(symbol_point, frames);
     }
   }
 };

 } // namespace srsran