/**
 * \file EV3UartProtocolParserSensorSide.cpp
 *
 * Definitions for the EV3 UART sensor protocol parsers
 *
 * \copyright Shenghao Yang, 2018
 * 
 * See LICENSE for details
 */

#include <EV3UartProtocolParserSensorSide.hpp>
#include <stdint.h>

namespace EV3UartProtocolParserSensorSide {

using namespace EV3UartGenerator;

uint8_t Parser::payload_length(const uint8_t hdr) {
	return two_pow((hdr >> 0x03) & 0x07);
}

Parser::HeaderInformation Parser::analyze_header(const uint8_t hdr) {
	// Check type first
	HeaderInformation info {
		false, (hdr & 0xc7), 0x00
	};

	const uint8_t payload_len_code { (hdr >> 0x03) & 0x07 };

	switch (info.header_sanitized) {
	case (static_cast<uint8_t>(Magics::SYS::SYS_BASE)
		  | static_cast<uint8_t>(Magics::SYS::ACK)):
		if (!payload_len_code)
			info.header_valid = true;
		break;

	case (static_cast<uint8_t>(Magics::SYS::SYS_BASE)
		  | static_cast<uint8_t>(Magics::SYS::NACK)):
		if (!payload_len_code)
			info.header_valid = true;
		break;

	case (static_cast<uint8_t>(Magics::CMD::CMD_BASE)
		  | static_cast<uint8_t>(Magics::CMD::SELECT)):
		if (payload_len_code == 0) {
			info.header_valid = true;
			info.payload_length = payload_length(hdr);
		}
		break;

	case (static_cast<uint8_t>(Magics::CMD::CMD_BASE)
		  | static_cast<uint8_t>(Magics::CMD::WRITE)):
		if (payload_len_code < 6) {
			info.header_valid = true;
			info.payload_length = payload_length(hdr);
		}
		break;

	default:
		break;
	}

	return info;
}

Parser::Parser() {

}

ParserReturn Parser::update(uint8_t input) {
	ParserReturn rtn { };

	switch (current_state) {
	case State::WAIT_HEADER:
		{
			HeaderInformation info { analyze_header(input) };
			buffer[0] = input;
			if (info.header_valid) {
				if (info.payload_length > 0) { // NOT SYS - is a CMD header
					// CMD - setup byte counters
					message_payload_length = info.payload_length;
					message_pending_bytes = (info.payload_length + 0x01); // + 1 FCS
					rtn.res = ParseResult::INSUFFICIENT_DATA;
					// Advance state
					current_state = next_state(current_state);
				} else { // SYS - translate and return
					switch (info.header_sanitized & 0x07) { // Mask out irrelevant bits
					case static_cast<uint8_t>(Magics::SYS::NACK):
						rtn.res = ParseResult::RECEIVED_SYS_NACK;
						break;
					case static_cast<uint8_t>(Magics::SYS::ACK):
						rtn.res = ParseResult::RECEIVED_SYS_ACK;
						break;
					}
					// No state advance
				}
			} else {
				rtn.res = ParseResult::RECEIVED_INVALID_HEADER;
			}
		}
		break;
	case State::WAIT_CHECKSUM:
		const uint8_t write_index { ((message_payload_length + 0x01)
								    - message_pending_bytes) + 0x01 };
		buffer[write_index] = input;
		message_pending_bytes -= 0x01;

		if (message_pending_bytes) {
			rtn.res = ParseResult::INSUFFICIENT_DATA;
		} else {
			rtn.len = message_payload_length; // len must be valid for RECEIVED_CMD_*
			if (Framing::checksum(buffer, message_payload_length + 1) // + 1 for header
			 	!= buffer[write_index]) {     // Checksum Error
				rtn.res = ParseResult::RECEIVED_CMD_INVALID_FCS;
			} else {					      // Checksum OK
				switch (buffer[0] & 0x07) {   // Mask out irrelevant bits
				case static_cast<uint8_t>(Magics::CMD::SELECT):
					rtn.res = ParseResult::RECEIVED_CMD_SELECT;
					break;
				case static_cast<uint8_t>(Magics::CMD::WRITE):
					rtn.res = ParseResult::RECEIVED_CMD_WRITE;
					break;
				}
			}
			current_state = next_state(current_state); // Increment state
		}
		break;
	}

	rtn.hdr = buffer[0];
	return rtn;
}

uint8_t* Parser::data() {
	return (buffer + 1);
}

const uint8_t* Parser::data() const {
	return (buffer + 1);
}

void Parser::reset_state() {
	current_state = State::STATE_START;
}
}


