/**
 * \file test_EV3UartProtocolParserSensorSide_MultipleMessages.cpp
 *
 * More unit tests for functionality contained in
 * EV3UartProtocolParserSensorSide.cpp
 *
 * The tests in this file verify that:
 * - The parser defined in the source file is able to:
 *   - Parse multiple messages, one after another.
 *   - Parse messages prepended with invalid bytes.
 *
 * \copyright Shenghao Yang, 2018
 * 
 * See LICENSE for details
 */

#include <EV3UartProtocolParserSensorSide.hpp>
#include <EV3UartGenerator.hpp>
#include "catch.hpp"
#include <vector>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cstring>
#include <array>
#include <tuple>

using namespace EV3UartProtocolParserSensorSide;
using namespace EV3UartGenerator;

static uint8_t* add_invalid_bytes(uint8_t* target, uint32_t count) {
	for (uint32_t i = 0; i < count ; i++) {
		*(target++) = static_cast<uint8_t>(Magics::DATA::DATA_BASE);
	}
	return target;
}

TEST_CASE("Parser is able to parse multiple messages, one after another",
		  "[Parser]") {
	Parser p { };
	std::array<uint8_t, Framing::BUFFER_MIN * 6> message { };
	std::array<uint8_t, Framing::BUFFER_MIN * 6>::iterator write_target {
		message.begin()
	};

	write_target += Framing::frame_cmd_write_message(write_target,
			reinterpret_cast<const uint8_t*>("Goodbye"),
			std::strlen("Goodbye"));
	write_target += Framing::frame_sys_message(write_target, Magics::SYS::ACK);
	write_target += Framing::frame_sys_message(write_target, Magics::SYS::NACK);
	write_target += Framing::frame_cmd_select_message(write_target, 0x00);
	write_target += Framing::frame_cmd_select_message(write_target, 0x01);

	std::array<ParseResult, 6> expected_results {
		ParseResult::RECEIVED_CMD_WRITE,
		ParseResult::RECEIVED_SYS_ACK,
		ParseResult::RECEIVED_SYS_NACK,
		ParseResult::RECEIVED_CMD_SELECT,
		ParseResult::RECEIVED_CMD_SELECT
	};

	uint32_t message_count = 0;
	for (auto data = message.begin(); data != write_target; data++) {
		ParseResult res;
		if ((res = p.update(*data).res) != ParseResult::INSUFFICIENT_DATA) {
			REQUIRE(expected_results[message_count++] == res);
		}
	}
}

TEST_CASE("Parser is able to parse messages prepended with invalid bytes",
		  "[Parser]") {
	Parser p { };
		std::array<uint8_t, Framing::BUFFER_MIN * 6 + (6 * 5)> message { };
		decltype(message)::iterator write_target {
			message.begin()
		};

		write_target = add_invalid_bytes(write_target, 5);
		write_target += Framing::frame_cmd_write_message(write_target,
				reinterpret_cast<const uint8_t*>("Goodbye"),
				std::strlen("Goodbye"));
		write_target = add_invalid_bytes(write_target, 5);
		write_target += Framing::frame_sys_message(write_target, Magics::SYS::ACK);
		write_target = add_invalid_bytes(write_target, 5);
		write_target += Framing::frame_sys_message(write_target, Magics::SYS::NACK);
		write_target = add_invalid_bytes(write_target, 5);
		write_target += Framing::frame_cmd_select_message(write_target, 0x00);
		write_target = add_invalid_bytes(write_target, 5);
		write_target += Framing::frame_cmd_select_message(write_target, 0x01);
		write_target = add_invalid_bytes(write_target, 5);

		std::array<ParseResult, 6> expected_results {
			ParseResult::RECEIVED_CMD_WRITE,
			ParseResult::RECEIVED_SYS_ACK,
			ParseResult::RECEIVED_SYS_NACK,
			ParseResult::RECEIVED_CMD_SELECT,
			ParseResult::RECEIVED_CMD_SELECT
		};

		uint32_t message_count = 0;
		for (auto data = message.begin(); data != write_target; data++) {
			ParseResult res { p.update(*data).res };
			if ((res != ParseResult::INSUFFICIENT_DATA)
			    && (res != ParseResult::RECEIVED_INVALID_HEADER)) {
				REQUIRE(expected_results[message_count++] == res);
			}
		}
}

