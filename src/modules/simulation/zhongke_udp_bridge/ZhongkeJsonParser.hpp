/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/
#pragma once

#include <uORB/topics/dyt_zhongke_telemetry.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace zhongke_udp
{
constexpr size_t kUdpBufferSize = 4096;
constexpr unsigned kMaxFields = 32;
constexpr size_t kKeyCapacity = 64;
constexpr unsigned kMaxNesting = 8;

enum class ValueType { U8, U64, Float };

struct Field {
	const char *name;
	size_t offset;
	ValueType type;
};

#define ZHONGKE_FIELD(name, type) {#name, offsetof(dyt_zhongke_telemetry_s, name), ValueType::type}
static constexpr Field kFields[] {
	ZHONGKE_FIELD(timestamp, U64),
	ZHONGKE_FIELD(status1, U8),
	ZHONGKE_FIELD(status2, U8),
	ZHONGKE_FIELD(status3, U8),
	ZHONGKE_FIELD(zoom, Float),
	ZHONGKE_FIELD(offset_x, Float),
	ZHONGKE_FIELD(offset_y, Float),
	ZHONGKE_FIELD(frame_roll, Float),
	ZHONGKE_FIELD(frame_pitch, Float),
	ZHONGKE_FIELD(frame_yaw, Float),
	ZHONGKE_FIELD(roll_rate, Float),
	ZHONGKE_FIELD(pitch_rate, Float),
	ZHONGKE_FIELD(yaw_rate, Float),
	ZHONGKE_FIELD(laser_range, Float),
	ZHONGKE_FIELD(self_check, U8),
};
#undef ZHONGKE_FIELD
constexpr unsigned kFieldCount = sizeof(kFields) / sizeof(kFields[0]);
static_assert(kFieldCount == 15, "中科遥测需要 timestamp 和全部 14 个测量字段");

class JsonParser
{
public:
	JsonParser(const char *data, size_t length) : _cursor(data), _end(data + length), _length(length) {}

	bool parse(dyt_zhongke_telemetry_s &message)
	{
		if (_length == 0 || _length >= kUdpBufferSize) { return false; }

		skip_whitespace();

		if (!consume('{')) { return false; }

		uint32_t seen = 0;
		char keys[kMaxFields][kKeyCapacity] {};
		size_t key_lengths[kMaxFields] {};
		unsigned key_count = 0;

		while (_cursor < _end) {
			if (key_count == kMaxFields) { return false; }

			skip_whitespace();
			char *key = keys[key_count];
			size_t &key_length = key_lengths[key_count];

			if (!parse_string(key, kKeyCapacity, &key_length)) { return false; }

			for (unsigned i = 0; i < key_count; ++i) {
				if (key_lengths[i] == key_length && memcmp(keys[i], key, key_length) == 0) { return false; }
			}

			++key_count;
			skip_whitespace();

			if (!consume(':')) { return false; }

			skip_whitespace();
			const int index = field_index(key, key_length);

			if (index >= 0) {
				const uint32_t bit = uint32_t{1} << index;
				char *output = reinterpret_cast<char *>(&message) + kFields[index].offset;

				if ((seen & bit) != 0 || !parse_value(kFields[index].type, output)) { return false; }

				seen |= bit;

			} else if (!skip_value(0)) {
				return false;
			}

			skip_whitespace();

			if (consume('}')) {
				skip_whitespace();
				return _cursor == _end && seen == ((uint32_t{1} << kFieldCount) - 1);
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
		if (_cursor == _end || *_cursor != value) { return false; }

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

	bool parse_hex_code(uint32_t &code)
	{
		code = 0;

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

		return true;
	}

	bool parse_unicode_escape(uint32_t &code)
	{
		if (!parse_hex_code(code)) { return false; }

		if (code >= 0xD800 && code <= 0xDBFF) {
			uint32_t low = 0;

			if (!consume('\\') || !consume('u') || !parse_hex_code(low) || low < 0xDC00 || low > 0xDFFF) { return false; }

			code = 0x10000 + ((code - 0xD800) << 10) + low - 0xDC00;

		} else if (code >= 0xDC00 && code <= 0xDFFF) {
			return false;
		}

		return true;
	}

	bool parse_utf8(unsigned char first, uint32_t &code)
	{
		unsigned remaining;
		uint32_t minimum;

		if (first >= 0xC2 && first <= 0xDF) {
			code = first & 0x1F;
			remaining = 1;
			minimum = 0x80;

		} else if (first >= 0xE0 && first <= 0xEF) {
			code = first & 0x0F;
			remaining = 2;
			minimum = 0x800;

		} else if (first >= 0xF0 && first <= 0xF4) {
			code = first & 0x07;
			remaining = 3;
			minimum = 0x10000;

		} else {
			return false;
		}

		for (unsigned i = 0; i < remaining; ++i) {
			if (_cursor == _end) { return false; }

			const unsigned char byte = static_cast<unsigned char>(*_cursor++);

			if ((byte & 0xC0) != 0x80) { return false; }

			code = (code << 6) | (byte & 0x3F);
		}

		return code >= minimum && code <= 0x10FFFF && !(code >= 0xD800 && code <= 0xDFFF);
	}

	static bool append_code(uint32_t code, char *output, size_t capacity, size_t &length)
	{
		const unsigned bytes = code < 0x80 ? 1 : code < 0x800 ? 2 : code < 0x10000 ? 3 : 4;

		if (output != nullptr) {
			if (length + bytes >= capacity) { return false; }

			if (bytes == 1) {
				output[length] = static_cast<char>(code);

			} else {
				const unsigned prefix = bytes == 2 ? 0xC0 : bytes == 3 ? 0xE0 : 0xF0;
				output[length] = static_cast<char>(prefix | (code >> (6 * (bytes - 1))));

				for (unsigned i = 1; i < bytes; ++i) {
					output[length + i] = static_cast<char>(0x80 | ((code >> (6 * (bytes - 1 - i))) & 0x3F));
				}
			}
		}

		length += bytes;
		return true;
	}

	bool parse_string(char *output, size_t capacity, size_t *output_length = nullptr)
	{
		if (!consume('"')) { return false; }

		size_t length = 0;

		while (_cursor < _end) {
			const unsigned char value = static_cast<unsigned char>(*_cursor++);

			if (value == '"') {
				if (output != nullptr) { output[length] = '\0'; }

				if (output_length != nullptr) { *output_length = length; }

				return true;
			}

			if (value < 0x20) { return false; }

			uint32_t code = value;

			if (value == '\\') {
				if (_cursor == _end) { return false; }

				const char escape = *_cursor++;

				switch (escape) {
				case '"': case '\\': case '/': code = escape; break;

				case 'b': code = '\b'; break;

				case 'f': code = '\f'; break;

				case 'n': code = '\n'; break;

				case 'r': code = '\r'; break;

				case 't': code = '\t'; break;

				case 'u':
					if (!parse_unicode_escape(code)) { return false; }

					break;

				default: return false;
				}

			} else if (value >= 0x80 && !parse_utf8(value, code)) {
				return false;
			}

			if (!append_code(code, output, capacity, length)) { return false; }
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

		// 只转换本数据报内的数字，不允许 strtod 越过输入边界。
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

		uint64_t number = 0;
		const uint64_t limit = type == ValueType::U8 ? UINT8_MAX : UINT64_MAX;

		if (!parse_integer(limit, number)) { return false; }

		if (type == ValueType::U8) { *static_cast<uint8_t *>(output) = number; }

		else { *static_cast<uint64_t *>(output) = number; }

		return true;
	}

	bool skip_value(unsigned depth)
	{
		if (_cursor == _end) { return false; }

		if (*_cursor == '"') { return parse_string(nullptr, 0); }

		if (*_cursor == '{' || *_cursor == '[') {
			if (depth >= kMaxNesting) { return false; }

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

	static int field_index(const char *key, size_t length)
	{
		for (unsigned i = 0; i < kFieldCount; ++i) {
			if (strlen(kFields[i].name) == length && memcmp(key, kFields[i].name, length) == 0) { return i; }
		}

		return -1;
	}

	const char *_cursor;
	const char *_end;
	size_t _length;
};
}
