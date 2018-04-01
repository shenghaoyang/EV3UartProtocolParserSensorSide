/**
 * \file test_EV3UartProtocolParserSensorSide.cpp
 *
 * Unit tests for functionality contained in EV3UartProtocolParserSensorSide.cpp
 *
 * The tests in this file verify that:
 * - The main utility functions in the source file work as intended.
 * - The parser defined in the source file:
 * 	 - Is able to parse single bytes correctly
 * 	 - Is able to parse CMD_WRITE messages correctly
 * 	 - Is able to parse CMD_SELECT messages correctly
 * 	 - Is ready to parse a new message after parsing any type of message
 * 	 - Resets when Parser::reset_state() is called
 * 	 - Provides the right memory area when Parser::data() is called
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

using namespace EV3UartGenerator;
using namespace EV3UartProtocolParserSensorSide;

/**
 * Tests that the parser is ready to process another message.
 *
 * How this test operates is as follows:
 * - If the parser is ready to process another message:
 * - The next byte passed to the Parser::update() function should be treated
 *   as a header byte
 * - Therefore, if we pass in an invalid header byte, we expect the parser to
 *   return a ParseResult::res contained in ParserReturn of value
 *   ParseResult::RECEIVED_INVALID_HEADER
 *
 * @param p parser to test
 * @retval true parser ready to process another message
 * @retval false parser is not ready to process another message
 */
static bool test_parser_ready_to_process_another_message(Parser& p) {
	// Update parser with invalid header byte
	ParserReturn rtn = p.update(static_cast<uint8_t>(Magics::CMD::CMD_BASE)
			 	 	 	 	    | static_cast<uint8_t>(Magics::CMD::TYPE));
	// Require that parser treated that byte as the header byte
	return rtn.res == ParseResult::RECEIVED_INVALID_HEADER;
}

TEST_CASE("two_pow() returns correct results", "[two_pow()]") {
	std::array<uint8_t, 8> ref_results {
		1, 2, 4, 8, 16, 32, 64, 128
	};
	for (uint8_t i = 0; i < ref_results.size(); i++) {
		REQUIRE(two_pow(i) == ref_results[i]);
	}
}

TEST_CASE("next_state() returns correct results", "[next_state()]") {
	SECTION("next_state() increments state variable if state variable"
			" is STATE_END") {
		for (uint8_t i = static_cast<uint8_t>(State::STATE_START);
			 i < static_cast<uint8_t>(State::STATE_END); i++) {
			REQUIRE(next_state(static_cast<State>(i)) ==
					static_cast<State>(i + 1));
		}
	}
	SECTION("next_state() wraps state variable back to STATE_START if "
			" state variable is STATE_END") {
		REQUIRE(next_state(State::STATE_END) == State::STATE_START);
	}
}

TEST_CASE("Parser returns correct values for single header bytes and"
		  " single byte messages. Parser is ready to process new messages "
		  "after processing complete single byte messages", "[Parser]"
		  " [Header] [SYS_ACK] [SYS_NACK]") {
	std::vector<uint8_t> valid_bytes {
		// Initialize valid header bytes with SYS_ACK, SYS_NACK, and CMD_SELECT
		(static_cast<uint8_t>(Magics::SYS::SYS_BASE)
		 | static_cast<uint8_t>(Magics::SYS::ACK)),
		(static_cast<uint8_t>(Magics::SYS::SYS_BASE)
		 | static_cast<uint8_t>(Magics::SYS::NACK)),
		(static_cast<uint8_t>(Magics::CMD::CMD_BASE)
		 | static_cast<uint8_t>(Magics::CMD::SELECT))
	};
	// Add CMD_WRITE header bytes with all possible valid length codes
	// to set of valid header bytes.
	// Total set of valid header bytes include SYS_ACK, SYS_NACK, CMD_SELECT,
	// as well as all possible header bytes representing CMD_WRITE (because
	// this header includes the length code, which varies for different
	// messages)
	for (uint8_t length_code = 0; length_code < 6; length_code++) {
		valid_bytes.push_back(static_cast<uint8_t>(Magics::CMD::CMD_BASE)
							  | static_cast<uint8_t>(Magics::CMD::WRITE)
							  | (length_code << 0x03));
	}
	for (uint16_t i = 0; i < 0x100; i++) {
		Parser p { };
		ParserReturn rtn = p.update(i);

		if (std::find(valid_bytes.begin(), valid_bytes.end(), i)
			== valid_bytes.end()) {
			// Header byte is not a valid header byte
			REQUIRE(rtn.res == ParseResult::RECEIVED_INVALID_HEADER);
		} else {
			// Header byte is a valid one
			if (i == valid_bytes[0]) {
				// Header byte is a SYS_ACK byte
				REQUIRE(rtn.res == ParseResult::RECEIVED_SYS_ACK);
				REQUIRE(test_parser_ready_to_process_another_message(p));
			} else if (i == valid_bytes[1]) {
				// Header byte is a SYS_NACK byte
				REQUIRE(rtn.res == ParseResult::RECEIVED_SYS_NACK);
				REQUIRE(test_parser_ready_to_process_another_message(p));
			} else {
				// Header byte is a CMD_SELECT byte or CMD_WRITE byte
				REQUIRE(rtn.res == ParseResult::INSUFFICIENT_DATA);
			}
		}
		// For all cases, the hdr field must contain the header
		REQUIRE(rtn.hdr == i);
	}
}

TEST_CASE("Parser returns correct results for CMD_SELECT messages and "
		  "is ready to parse new messages after parsing a CMD_SELECT "
		  "message", "[Parser]"
		  " [CMD_SELECT]") {
	SECTION("Parser returns correct results for valid CMD_SELECT messages") {
		for (uint8_t mode = 0; mode < 0x08; mode++) {
			std::array<uint8_t, Framing::BUFFER_MIN> buffer;
			const int8_t frame_size { Framing::frame_cmd_select_message(
					buffer.data(), mode) };

			Parser p { };
			for (uint8_t i = 0; i < frame_size; i++) {
				ParserReturn rtn = p.update(buffer[i]);
				if ((i + 1) != frame_size) {
					// We have not reached the end of frame, we expect
					// INSUFFICIENT_DATA
					REQUIRE(rtn.res == ParseResult::INSUFFICIENT_DATA);
				} else {
					// End of frame parsed, we expect recognition of this
					REQUIRE(rtn.res == ParseResult::RECEIVED_CMD_SELECT);
					// Length must match payload length
					REQUIRE(rtn.len == 0x01);
					// Data stored must match sent payload
					REQUIRE(*(p.data()) == buffer[1]);
					// Must be ready to parse new message
					REQUIRE(test_parser_ready_to_process_another_message(p));
				}
				// In all situations, we expect the header to be valid
				REQUIRE(rtn.hdr == buffer[0]);
			}
		}
	}
	SECTION("Parser returns correct results for invalid CMD_SELECT messages"
			" with invalid FCS") {
		for (uint8_t mode = 0; mode < 0x08; mode++) {
			std::array<uint8_t, Framing::BUFFER_MIN> buffer;
			const int8_t frame_size { Framing::frame_cmd_select_message(
					buffer.data(), mode) };

			// Purposely damage FCS
			buffer[frame_size - 1] = (buffer[frame_size - 1] + 0x01);

			Parser p { };
			for (uint8_t i = 0; i < frame_size; i++) {
				ParserReturn rtn = p.update(buffer[i]);
				if ((i + 1) != frame_size) {
					// We have not reached the end of frame, we expect
					// INSUFFICIENT_DATA
					REQUIRE(rtn.res == ParseResult::INSUFFICIENT_DATA);
				} else {
					// End of frame parsed, we expect recognition of this
					REQUIRE(rtn.res == ParseResult::RECEIVED_CMD_INVALID_FCS);
					// Length must match payload length
					REQUIRE(rtn.len == 0x01);
					// Data stored must match sent payload
					REQUIRE(*(p.data()) == buffer[1]);
					// Must be ready to parse new message
					REQUIRE(test_parser_ready_to_process_another_message(p));
				}
				// In all situations, we expect the header to be valid
				REQUIRE(rtn.hdr == buffer[0]);
			}
		}
	}
}

TEST_CASE("Parser returns correct results for CMD_WRITE messages and"
		  "is ready to parse a new message after a CMD_WRITE message",
		  "[Parser]"
		  " [CMD_WRITE]") {
	SECTION("Parser returns correct results for valid CMD_WRITE messages"
			" with valid FCS") {
		for (uint8_t payload_length = 1; payload_length <= 0x20;
				payload_length++) {
			// Generate payload, simply 0.... payload_length
			uint8_t payload[payload_length];
			std::iota(payload, payload + payload_length, 0x00);
			// Generate output frame
			std::array<uint8_t, Framing::BUFFER_MIN> buffer;
			const int8_t frame_size { Framing::frame_cmd_write_message(
					buffer.data(), payload, payload_length) };

			Parser p { };
			for (uint8_t i = 0; i < frame_size; i++) {
				ParserReturn rtn = p.update(buffer[i]);
				if ((i + 1) != frame_size) {
					// We have not reached the end of frame, we expect
					// INSUFFICIENT_DATA
					REQUIRE(rtn.res == ParseResult::INSUFFICIENT_DATA);
				} else {
					// End of frame parsed, we expect recognition of this
					REQUIRE(rtn.res == ParseResult::RECEIVED_CMD_WRITE);
					// Length must match payload length, rounded up to nearest
					// power of two if not power of two
					REQUIRE(rtn.len == two_pow(Framing::log2(payload_length)));
					// Data stored must match sent payload
					REQUIRE(std::equal(payload, payload + payload_length,
							p.data()) == true);
					// Parser must be ready to parse a new message
					REQUIRE(test_parser_ready_to_process_another_message(p));
				}
				// In all situations, we expect the header to be valid
				REQUIRE(rtn.hdr == buffer[0]);
			}
		}
	}

	SECTION("Parser returns correct results for invalid CMD_WRITE messages"
			" with invalid FCS") {
		for (uint8_t payload_length = 1; payload_length <= 0x20;
				payload_length++) {
			// Generate payload, simply 0.... payload_length
			uint8_t payload[payload_length];
			std::iota(payload, payload + payload_length, 0x00);
			// Generate output frame
			std::array<uint8_t, Framing::BUFFER_MIN> buffer;
			const int8_t frame_size { Framing::frame_cmd_write_message(
					buffer.data(), payload, payload_length) };
			// Purposely corrupt FCS
			buffer.data()[frame_size - 1] =
					(buffer.data()[frame_size - 1] + 0x01);

			Parser p { };
			for (uint8_t i = 0; i < frame_size; i++) {
				ParserReturn rtn = p.update(buffer[i]);
				if ((i + 1) != frame_size) {
					// We have not reached the end of frame, we expect
					// INSUFFICIENT_DATA
					REQUIRE(rtn.res == ParseResult::INSUFFICIENT_DATA);
				} else {
					// End of frame parsed, we expect recognition of this
					REQUIRE(rtn.res == ParseResult::RECEIVED_CMD_INVALID_FCS);
					// Length must match payload length, rounded up to nearest
					// power of two if not power of two
					REQUIRE(rtn.len == two_pow(Framing::log2(payload_length)));
					// Data stored must match sent payload
					REQUIRE(std::equal(payload, payload + payload_length,
							p.data()) == true);
					// Parser must be ready to parse a new message
					REQUIRE(test_parser_ready_to_process_another_message(p));
				}
				// In all situations, we expect the header to be valid
				REQUIRE(rtn.hdr == buffer[0]);
			}
		}
	}
}

TEST_CASE("Parser::reset_state() correctly resets the state of the parser",
		  "[Parser] [reset_state]") {
	// The only time when the parser needs to have its state reset is when
	// it's waiting for more data to come in from the other endpoint
	// feed the parser an incomplete message:
	std::array<uint8_t, Framing::BUFFER_MIN> message;
	const uint8_t message_size {
		Framing::frame_cmd_write_message(message.data(),
		reinterpret_cast<const uint8_t*>("Hello world!"),
		std::strlen("Hello world!"))
	};

	Parser p { };
	std::for_each(message.begin(), message.begin() + (message_size / 2),
			[&p](const uint8_t b) { p.update(b); });

	// Verify that the parser is still waiting for a message:
	REQUIRE(p.update(0x00).res == ParseResult::INSUFFICIENT_DATA);
	// Reset the parser
	p.reset_state();
	// Feed in data and verify that the parser treats the new data byte as a
	// header byte, which means that the parser has indeed reset
	REQUIRE(test_parser_ready_to_process_another_message(p));
}
