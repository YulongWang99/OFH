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

#include "srsran/adt/span.h"
#include "srsran/ofh/ethernet/ethernet_mac_address.h"
#include "srsran/ran/bs_channel_bandwidth.h"
#include "srsran/ru/ru_ofh_configuration.h"

namespace srsran {

/// Helper function to convert array of port indexes to string.
inline std::string port_ids_to_str(span<unsigned> ports)
{
  fmt::memory_buffer str_buffer;
  fmt::format_to(str_buffer, "{");
  for (unsigned i = 0, e = ports.size(); i != e; ++i) {
    fmt::format_to(str_buffer, "{}{}", ports[i], (i == (e - 1)) ? "}" : ", ");
  }
  return to_string(str_buffer);
}

/// Helper function to parse list of ports provided as a string.
inline std::vector<unsigned> parse_port_id(const std::string& port_id_str)
{
  std::vector<unsigned> port_ids;
  size_t                start_pos = port_id_str.find('{');
  size_t                end_pos   = port_id_str.find('}');
  if (start_pos == std::string::npos || end_pos == std::string::npos) {
    return port_ids;
  }
  std::string       ports_comma_separated = port_id_str.substr(start_pos + 1, end_pos - 1);
  std::stringstream ss(ports_comma_separated);
  int               port;
  while (ss >> port) {
    port_ids.push_back(port);
    if (ss.peek() == ',' || ss.peek() == ' ') {
      ss.ignore();
    }
  }
  return port_ids;
}

/// Parses the string containing Ethernet MAC address.
inline bool parse_mac_address(const std::string& mac_str, ether::mac_address& mac)
{
  std::array<unsigned, 6> data       = {};
  int                     bytes_read = std::sscanf(
      mac_str.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
  if (bytes_read != ether::ETH_ADDR_LEN) {
    fmt::print("Invalid MAC address provided: {}\n", mac_str);
    return false;
  }

  std::copy(data.begin(), data.end(), mac.begin());

  return true;
}

inline std::string generate_time_format()
{
  // 获取当前时间点
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);

  // 转换为本地时间
  std::tm local_tm = *std::localtime(&time_t_now);

  // 获取毫秒部分
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()
  ) % 1000;

  // 格式化为字符串
  std::ostringstream oss;
  oss << "data_"
      << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
      << "_" << std::setfill('0') << std::setw(3) << ms.count()
      << ".log";

  return oss.str();
}

} // namespace srsran
