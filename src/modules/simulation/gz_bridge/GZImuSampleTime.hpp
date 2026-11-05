/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <limits>
#include <gz/msgs/header.pb.h>

// The optional IMU relay preserves sensor sample time while transport callbacks
// may arrive in batches within a single simulator clock update.
class GZImuSampleTime
{
public:

	bool update(const gz::msgs::Header &header, uint64_t receive_time, uint64_t &sample_time)
	{
		sample_time = receive_time;
		bool relayed = false;

		for (const auto &entry : header.data()) {
			if (entry.key() == "rocket_flight_vibration" && entry.value_size() > 0 && entry.value(0) == "1") {
				relayed = true;
				break;
			}
		}

		// Preserve the original bridge behavior and history for native IMUs.
		if (!relayed) {
			return true;
		}

		const auto &stamp = header.stamp();

		if (!header.has_stamp() || stamp.sec() < 0 || stamp.nsec() < 0 || stamp.nsec() >= 1000000000) {
			return false;
		}

		const uint64_t subseconds = static_cast<uint64_t>(stamp.nsec()) / 1000;
		const uint64_t seconds = static_cast<uint64_t>(stamp.sec());

		if (seconds > (std::numeric_limits<uint64_t>::max() - subseconds) / 1000000) {
			return false;
		}

		const uint64_t source_time = seconds * 1000000 + subseconds;

		if (source_time == 0) {
			return false;
		}

		// Gazebo world resets can rewind its clock; this state is only used by
		// marked IMUs, and never changes native IMU timestamp handling.
		if (receive_time < _previous_receive_time) {
			_previous_sample_time = 0;
		}

		_previous_receive_time = receive_time;

		if (source_time <= _previous_sample_time) {
			return false;
		}

		_previous_sample_time = source_time;
		sample_time = source_time;
		return true;
	}

private:
	uint64_t _previous_sample_time{0};
	uint64_t _previous_receive_time{0};
};
