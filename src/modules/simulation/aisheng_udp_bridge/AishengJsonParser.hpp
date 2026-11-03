/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/
#pragma once

#include <uORB/topics/dyt_aisheng_telemetry.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace aisheng_udp
{
constexpr size_t kUdpBufferSize = 4096;

enum class ValueType { U8, U16, U32, U64, Gate, I32, Float };

struct Field {
	const char *name;
	size_t offset;
	ValueType type;
	unsigned count;
};

#define AISHENG_FIELD(name, type) {#name, offsetof(dyt_aisheng_telemetry_s, name), ValueType::type, 1}
#define AISHENG_ARRAY(name, type) {#name, offsetof(dyt_aisheng_telemetry_s, name), ValueType::type, 4}
static constexpr Field kFields[] {
	AISHENG_FIELD(timestamp, U64),
	AISHENG_FIELD(self_check, U8),
	AISHENG_FIELD(work_status, U8),
	AISHENG_FIELD(track_status, U8),
	AISHENG_FIELD(pitch_angle, Float),
	AISHENG_FIELD(yaw_angle, Float),
	AISHENG_FIELD(gate_x, Gate),
	AISHENG_FIELD(gate_y, Gate),
	AISHENG_FIELD(gyro_pitch_rate, Float),
	AISHENG_FIELD(gyro_yaw_rate, Float),
	AISHENG_FIELD(pitch_misalignment, Float),
	AISHENG_FIELD(yaw_misalignment, Float),
	AISHENG_FIELD(tracking_confidence, Float),
	AISHENG_FIELD(pitch_los_rate, Float),
	AISHENG_FIELD(yaw_los_rate, Float),
	AISHENG_FIELD(status, U8),
	AISHENG_FIELD(autonomous, U8),
	AISHENG_ARRAY(target_type, U8),
	AISHENG_ARRAY(target_confidence, Float),
	AISHENG_ARRAY(target_lon, I32),
	AISHENG_ARRAY(target_lat, I32),
	AISHENG_ARRAY(target_alt, Float),
	AISHENG_FIELD(status2, U8),
	AISHENG_FIELD(tracking_gate_w, U16),
	AISHENG_FIELD(tracking_gate_h, U16),
	AISHENG_FIELD(pitch_attitude, Float),
	AISHENG_FIELD(yaw_attitude, Float),
	AISHENG_FIELD(roll_attitude, Float),
	AISHENG_FIELD(tracking_target_type, U8),
	AISHENG_FIELD(laser_range, Float),
	AISHENG_FIELD(laser_update, U8),
	AISHENG_FIELD(aircraft_pitch, Float),
	AISHENG_FIELD(aircraft_roll, Float),
	AISHENG_FIELD(aircraft_yaw, Float),
	AISHENG_FIELD(ip_segment, U8),
	AISHENG_FIELD(ip_node, U8),
	AISHENG_FIELD(device_timestamp, U32),
};
#undef AISHENG_FIELD
#undef AISHENG_ARRAY
constexpr unsigned kFieldCount = sizeof(kFields) / sizeof(kFields[0]);
static_assert(kFieldCount == 37, "ASD 遥测需要除 LOS 向量外的全部 37 个字段");

class JsonParser
{
public:
	JsonParser(const char *data, size_t length) : _cursor(data), _end(data + length) {}

	bool parse(dyt_aisheng_telemetry_s &message)
	{
		skip_whitespace();

		if (!consume('{')) { return false; }

		uint64_t seen = 0;

		while (_cursor < _end) {
			skip_whitespace();
			char key[48] {};

			if (!parse_string(key, sizeof(key))) { return false; }

			skip_whitespace();

			if (!consume(':')) { return false; }

			skip_whitespace();
			const int index = field_index(key);

			if (index >= 0) {
				const uint64_t bit = uint64_t{1} << index;

				if ((seen & bit) != 0 || !parse_field(kFields[index], message)) { return false; }

				seen |= bit;

			} else if (!skip_value(0)) {
				return false;
			}

			skip_whitespace();

			if (consume('}')) {
				skip_whitespace();
				return _cursor == _end && seen == ((uint64_t{1} << kFieldCount) - 1)
				       &&message.track_status <= 1 && message.autonomous <= 1;
			}

			if (!consume(',')) { return false; }
		}

		return false;
	}

private:
	static bool is_digit(char value) { return value >= '0' && value <= '9'; }

	void skip_whitespace()
	{
		while (_cursor < _end && (*_cursor == ' ' || *_cursor == '\t' || *_cursor == '\r' || *_cursor == '\n')) {
			++_cursor;
		}
	}

	bool consume(char value)
	{
		if (_cursor >= _end || *_cursor != value) { return false; }

		++_cursor;
		return true;
	}

	bool consume_literal(const char *literal)
	{
		const size_t length = strlen(literal);

		if (static_cast<size_t>(_end - _cursor) < length || memcmp(_cursor, literal, length) != 0) { return false; }

		_cursor += length;
		return true;
	}

	bool parse_string(char *output, size_t capacity)
	{
		if (!consume('"')) { return false; }

		size_t length = 0;
		bool overflow = false;

		while (_cursor < _end) {
			char value = *_cursor++;

			if (value == '"') {
				if (output != nullptr) { output[overflow ? 0 : length] = '\0'; }

				return true;
			}

			if (static_cast<unsigned char>(value) < 0x20) { return false; }

			if (value == '\\') {
				if (_cursor == _end) { return false; }

				value = *_cursor++;

				switch (value) {
				case '"': case '\\': case '/': break;

				case 'b': value = '\b'; break;

				case 'f': value = '\f'; break;

				case 'n': value = '\n'; break;

				case 'r': value = '\r'; break;

				case 't': value = '\t'; break;

				case 'u': {
						unsigned code = 0;

						for (unsigned i = 0; i < 4; ++i) {
							if (_cursor == _end) { return false; }

							const char digit = *_cursor++;
							unsigned number;

							if (is_digit(digit)) { number = digit - '0'; }

							else if (digit >= 'a' && digit <= 'f') { number = digit - 'a' + 10; }

							else if (digit >= 'A' && digit <= 'F') { number = digit - 'A' + 10; }

							else { return false; }

							code = (code << 4) | number;
						}

						// 字段名只使用 ASCII；其他 Unicode 文本属于未知字段。
						value = code > 0 && code < 128 ? static_cast<char>(code) : '?';
						break;
					}

				default: return false;
				}
			}

			if (output != nullptr) {
				if (length + 1 < capacity) { output[length++] = value; }

				else { overflow = true; }
			}
		}

		return false;
	}

	bool parse_integer(uint64_t limit, uint64_t &result)
	{
		if (_cursor == _end || !is_digit(*_cursor)) { return false; }

		if (*_cursor == '0' && _cursor + 1 < _end && is_digit(_cursor[1])) { return false; }

		result = 0;

		while (_cursor < _end && is_digit(*_cursor)) {
			const unsigned digit = *_cursor++ - '0';

			if (result > limit / 10 || (result == limit / 10 && digit > limit % 10)) { return false; }

			result = result * 10 + digit;
		}

		return _cursor == _end || (*_cursor != '.' && *_cursor != 'e' && *_cursor != 'E');
	}

	bool parse_number(double &result)
	{
		const char *start = _cursor;
		(void)consume('-');

		if (_cursor == _end) { return false; }

		if (*_cursor == '0') {
			++_cursor;

			if (_cursor < _end && is_digit(*_cursor)) { return false; }

		} else if (*_cursor >= '1' && *_cursor <= '9') {
			while (_cursor < _end && is_digit(*_cursor)) { ++_cursor; }

		} else {
			return false;
		}

		if (consume('.')) {
			if (_cursor == _end || !is_digit(*_cursor)) { return false; }

			while (_cursor < _end && is_digit(*_cursor)) { ++_cursor; }
		}

		if (_cursor < _end && (*_cursor == 'e' || *_cursor == 'E')) {
			++_cursor;

			if (_cursor < _end && (*_cursor == '+' || *_cursor == '-')) { ++_cursor; }

			if (_cursor == _end || !is_digit(*_cursor)) { return false; }

			while (_cursor < _end && is_digit(*_cursor)) { ++_cursor; }
		}

		// 转换函数只接收带结尾的数字，避免越过数据报边界读取。
		const size_t length = static_cast<size_t>(_cursor - start);
		char token[96] {};

		if (length >= sizeof(token)) { return false; }

		memcpy(token, start, length);
		char *end = nullptr;
		result = strtod(token, &end);
		return end == token + length && std::isfinite(result);
	}

	bool parse_value(ValueType type, void *output)
	{
		if (type == ValueType::Float) {
			double number = 0.;

			if (!parse_number(number) || fabs(number) > static_cast<double>(std::numeric_limits<float>::max())) { return false; }

			*static_cast<float *>(output) = static_cast<float>(number);
			return true;
		}

		const bool negative = type == ValueType::I32 && consume('-');
		uint64_t limit;

		switch (type) {
		case ValueType::U8: limit = UINT8_MAX; break;

		case ValueType::U16: limit = UINT16_MAX; break;

		case ValueType::U32: limit = UINT32_MAX; break;

		case ValueType::U64: limit = UINT64_MAX; break;

		case ValueType::Gate: limit = INT16_MAX; break;

		case ValueType::I32: limit = negative ? uint64_t{INT32_MAX} + 1 : INT32_MAX; break;

		default: return false;
		}

		uint64_t number = 0;

		if (!parse_integer(limit, number)) { return false; }

		switch (type) {
		case ValueType::U8: *static_cast<uint8_t *>(output) = number; break;

		case ValueType::U16: *static_cast<uint16_t *>(output) = number; break;

		case ValueType::U32: *static_cast<uint32_t *>(output) = number; break;

		case ValueType::U64: *static_cast<uint64_t *>(output) = number; break;

		case ValueType::Gate: *static_cast<int16_t *>(output) = number; break;

		case ValueType::I32:
			*static_cast<int32_t *>(output) = negative ? -static_cast<int64_t>(number) : static_cast<int64_t>(number);
			break;

		default: return false;
		}

		return true;
	}

	bool parse_field(const Field &field, dyt_aisheng_telemetry_s &message)
	{
		char *output = reinterpret_cast<char *>(&message) + field.offset;

		if (field.count == 1) { return parse_value(field.type, output); }

		if (!consume('[')) { return false; }

		const size_t stride = field.type == ValueType::U8 ? sizeof(uint8_t) : sizeof(uint32_t);

		for (unsigned i = 0; i < field.count; ++i) {
			skip_whitespace();

			if (!parse_value(field.type, output + stride * i)) { return false; }

			skip_whitespace();

			if (i + 1 < field.count && !consume(',')) { return false; }
		}

		return consume(']');
	}

	bool skip_value(unsigned depth)
	{
		if (depth > 8 || _cursor == _end) { return false; }

		if (*_cursor == '"') { return parse_string(nullptr, 0); }

		if (*_cursor == '{' || *_cursor == '[') {
			const bool object = *_cursor++ == '{';
			const char closing = object ? '}' : ']';
			skip_whitespace();

			if (consume(closing)) { return true; }

			while (_cursor < _end) {
				if (object) {
					if (!parse_string(nullptr, 0)) { return false; }

					skip_whitespace();

					if (!consume(':')) { return false; }

					skip_whitespace();
				}

				if (!skip_value(depth + 1)) { return false; }

				skip_whitespace();

				if (consume(closing)) { return true; }

				if (!consume(',')) { return false; }

				skip_whitespace();
			}

			return false;
		}

		if (consume_literal("true") || consume_literal("false") || consume_literal("null")) { return true; }

		double unused;
		return parse_number(unused);
	}

	static int field_index(const char *key)
	{
		for (unsigned i = 0; i < kFieldCount; ++i) {
			if (strcmp(key, kFields[i].name) == 0) { return i; }
		}

		return -1;
	}

	const char *_cursor;
	const char *_end;
};
}
