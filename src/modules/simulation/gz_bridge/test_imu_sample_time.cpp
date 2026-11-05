#include "GZImuSampleTime.hpp"

#include <cassert>
#include <cstdint>
#include <limits>

static gz::msgs::Header header(int64_t seconds, int32_t nanoseconds, bool marked = true)
{
	gz::msgs::Header value;
	value.mutable_stamp()->set_sec(seconds);
	value.mutable_stamp()->set_nsec(nanoseconds);

	if (marked) {
		auto *entry = value.add_data();
		entry->set_key("rocket_flight_vibration");
		entry->add_value("1");
	}

	return value;
}

int main()
{
	GZImuSampleTime time;
	uint64_t sample = 0;
	// Native IMUs keep callback time, including when source stamps are bad.
	assert(time.update(header(-1, -1, false), 10000, sample) && sample == 10000);
	assert(time.update(header(20, 0, false), 10000, sample) && sample == 10000);
	// Batched callbacks can share receive time but must keep distinct samples.
	assert(time.update(header(1, 1000000), 1000000, sample) && sample == 1001000);
	assert(time.update(header(1, 2000000), 1000000, sample) && sample == 1002000);
	assert(!time.update(header(1, 2000000), 1000000, sample));
	assert(!time.update(header(1, 1000000), 1000000, sample));
	assert(!time.update(header(0, 0), 1000000, sample));
	assert(!time.update(header(-1, 0), 1000000, sample));
	assert(!time.update(header(1, -1), 1000000, sample));
	assert(!time.update(header(1, 1000000000), 1000000, sample));
	assert(!time.update(header(std::numeric_limits<int64_t>::max(), 0), 1000000, sample));
	assert(time.update(header(1, 3000000), 1000000, sample) && sample == 1003000);
	// A simulation clock reset starts a new sample history.
	assert(time.update(header(0, 1000000), 1000, sample) && sample == 1000);
	assert(time.update(header(0, 2000000), 1000, sample) && sample == 2000);
	// Native traffic neither consumes nor clears the relay history.
	assert(time.update(header(0, 0, false), 0, sample) && sample == 0);
	assert(!time.update(header(0, 2000000), 1000, sample));
	// Wrong marker value and missing marker payload preserve native behavior.
	auto wrong = header(0, 0);
	wrong.mutable_data(0)->set_value(0, "0");
	assert(time.update(wrong, 42, sample) && sample == 42);
	wrong.mutable_data(0)->clear_value();
	assert(time.update(wrong, 43, sample) && sample == 43);
	return 0;
}
