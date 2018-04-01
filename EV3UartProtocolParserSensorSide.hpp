/**
 * \file EV3UartProtocolParserSensorSide.hpp
 *
 * Main header file for the sensor-side EV3 UART sensor protocol
 * parser
 *
 * \copyright Shenghao Yang, 2018
 * 
 * See LICENSE for details
 */

/**
 * \mainpage
 *
 * This is a C++ parser library for the EV3 UART protocol, meant to decode
 * messages that may arrive at the sensor side of the connection.
 *
 * Usage
 * -----
 * - Add the following folders to your include paths:
 * 	 - \c EV3UartGenerator/
 * 	 - \c / (root folder of this library)
 * - Add the source files in the following folders to the list of built files:
 *   - \c EV3UartGenerator/
 *   - \c / (root folder of this library)
 * - All operations must be done non-recursively
 *
 * The EV3UartProtocolParserSensorSide::Parser accepts data,
 * byte-by-byte from any UART backend that
 * supplies data that is sent from the EV3 to the sensor.
 * \code{.cpp}
 * uint8_t data = uart_get_byte();
 * Parser p { };
 * p.update(data);
 * \endcode
 *
 * The parse result is available from the
 * EV3UartProtocolParserSensorSide::Parser::update() function as a
 * ParserReturn structure:
 * \code{.cpp}
 * uint8_t data = uart_get_byte();
 * Parser p { };
 * ParserReturn r = p.update(data);
 * \endcode
 *
 * The various fields in the
 * EV3UartProtocolParserSensorSide::ParserReturn structure offers more information
 * on what was parsed. EV3UartProtocolParserSensorSide::Parser::data()
 * obtains the payload of the message sent
 * from the EV3, if the header byte from the EV3 does not represent the
 * entirety of the message (i.e. CMD_WRITE and CMD_SELECT messages)
 * \code{.cpp}
 * Parser p { };
 * uint8_t* data = p.data();
 * \endcode
 *
 * If there is a need to reset the state of the parser, i.e. the connection
 * between the EV3 and the sensor has been reset, the function
 * EV3UartProtocolParserSensorSide::Parser::reset_state() can be called.
 * The next byte parsed will be treated as a header byte.
 * \code {.cpp}
 * Parser p { };
 * p.reset_state();
 * \endcode
 *
 * For more information, see EV3UartProtocolParserSensorSide
 *
 * Tests
 * -----
 *
 * This library uses the Catch2 testing framework:
 * https://github.com/catchorg/Catch2
 *
 * Tests are located under the \c test/ subfolder.
 *
 * To build and run the tests,
 * - Add the \c test/ folder recursively to your build path
 * - Run the compiled executable.
 *
 * Licensed under the MIT license.
 *
 * See LICENSE for more details.
 *
 */

#ifndef EV3UARTPROTOCOLPARSERSENSORSIDE_HPP_
#define EV3UARTPROTOCOLPARSERSENSORSIDE_HPP_

#include <magics.hpp>
#include <framing.hpp>

namespace EV3UartProtocolParserSensorSide {

/**
 * Length of internally allocated buffer to store messages incoming from the
 * EV3
 */
constexpr uint8_t BUFFER_LEN { EV3UartGenerator::Framing::BUFFER_MIN };

/**
 * Raises two to the power of \c val
 *
 * @param val what to raise two to the power of, in the range [0, 7]
 * @return two raised to the power of \c val
 *
 * @warning undefined for values of \c val greater than \c 7.
 */
constexpr uint8_t two_pow(uint8_t val) {
	return (0x01 << val);
}

/**
 * Enumeration listing the states the parser state machine can be in
 */
enum class State : uint8_t {
	STATE_START = 0,	///< Starting value for State values
	WAIT_HEADER = 0,	///< Parser is waiting for the header byte
	WAIT_CHECKSUM = 1,	///< Parser is waiting for the checksum byte
	STATE_END = 1,		///< End value for State values
};

/**
 * Increments the variable representing the current parser state machine
 * state to the next state, wrapping around if necessary.
 *
 * @param st current state of the state machine
 * @return next state of the state machine, wrapping around
 * to State::STATE_START if necessary.
 */
constexpr State next_state(State st) {
	if ((static_cast<uint8_t>(st))
			!= (static_cast<uint8_t>(State::STATE_END)))
		return static_cast<State>(static_cast<uint8_t>(st) + 1);
	else
		return State::STATE_START;
}

/**
 * Enumeration listing the possible results from parsing an additional byte
 * of data coming from the EV3, returned by Parser
 */
enum class ParseResult : uint8_t {
	/**
	 * The parser does not currently have sufficient data to return a
	 * significant parsing result. More data should be passed to the
	 * \ref Parser::update() function.
	 */
	INSUFFICIENT_DATA,
	/**
	 * The parser received an invalid header byte, a byte that is not the
	 * header for any message that the EV3 can send to a sensor.
	 *
	 * The reasons for this could be:
	 * - Invalid header byte type (not CMD or SYS)
	 * - Invalid header byte sub-type (not [SELECT, WRITE] for CMD types or
	 *   not [ACK, NACK] for SYS types)
	 * - Invalid payload length in header byte (not [0] for SYS types, or
	 *   not [1] for SELECT sub-type, or not [1, 32] for WRITE sub-type)
	 */
	RECEIVED_INVALID_HEADER,
	/**
	 * Parser received a SYS ACK message
	 */
	RECEIVED_SYS_ACK,
	/**
	 * Parser received a SYS ACK message
	 */
    RECEIVED_SYS_NACK,
	/**
	 * Parser received a CMD SELECT message with good FCS
	 */
	RECEIVED_CMD_SELECT,
	/**
	 * Parser received a CMD WRITE message with good FCS
	 */
	RECEIVED_CMD_WRITE,
	/**
	 * Parser received a CMD message with invalid FCS
	 */
	RECEIVED_CMD_INVALID_FCS,
};

/**
 * Structure returned by the Parser::update() function.
 *
 * The ParserReturn::hdr values are only valid in these situations:
 * res                       |hdr
 * --------------------------|---
 * INSUFFICIENT_DATA		 | Valid
 * RECEIVED_INVALID_HEADER	 | Valid
 * RECEIVED_SYS_ACK			 | Valid
 * RECEIVED_SYS_NACK		 | Valid
 * RECEIVED_CMD_SELECT		 | Valid
 * RECEIVED_CMD_WRITE		 | Valid
 * RECEIVED_CMD_INVALID_FCS	 | Valid
 *
 * The ParserReturn::len values can be interpreted this way:
 * res                       |len
 * --------------------------|---
 * INSUFFICIENT_DATA		 | No meaning
 * RECEIVED_INVALID_HEADER	 | No meaning
 * RECEIVED_SYS_ACK			 | No meaning
 * RECEIVED_SYS_NACK		 | No meaning
 * RECEIVED_CMD_SELECT		 | Length of the SELECT message's payload (1 byte)
 * RECEIVED_CMD_WRITE		 | Length of the WRITE message's payload
 * RECEIVED_CMD_INVALID_FCS	 | Length of the payload with invalid FCS
 */
struct ParserReturn {
	ParseResult res; ///< Result of parsing
	uint8_t hdr;	 ///< Header of the parsed message
	uint8_t len; 	 ///< Payload length of the parsed message
};

/**
 * Parser for parsing EV3 UART sensor protocol messages that come from the
 * EV3
 */
class Parser {
private:
	/**
	 * Internal buffer used to buffer information from the EV3.
	 *
	 * \c buffer[0] stores the header byte received from the EV3
	 * \c buffer[1] stores the first byte of the payload from the EV3
	 * \c buffer[payload_length + 1] stores the FCS byte received from the EV3
	 */
	uint8_t buffer[BUFFER_LEN];
	uint8_t message_payload_length = 0;
	uint8_t message_pending_bytes = 0;
	State current_state = State::STATE_START;

	/**
	 * Obtain the payload length from a valid message header byte
	 * @param hdr message header byte
	 * @return payload length of a particular message, in bytes.
	 */
	uint8_t payload_length(const uint8_t hdr);

	/**
	 * Structure containing header information from \ref analyze_header()
	 */
	struct HeaderInformation {
		/**
		 * \c true if header is valid, \c false otherwise.
		 * If the header byte is not valid,
		 * refer to \ref ParseResult::RECEIVED_INVALID_HEADER for the reasons
		 * why that may be so.
		 */
		bool header_valid;
		/**
		 * Sanitized header byte, with length information removed.
		 * Only valid if \c header_valid is \c true
		 */
		uint8_t header_sanitized;
		/**
		 * Payload length of the message, in bytes
		 * Only valid if \c header_valid is \c true
		 */
		uint8_t payload_length;
	};
	/**
	 * Analyze an EV3 message header
	 *
	 * @param hdr message header byte
	 * @return \ref HeaderInformation structure containing information
	 * about the header
	 */
	HeaderInformation analyze_header(const uint8_t hdr);
public:

	// We use the default constructor, because we don't really need to do
	// anything

	Parser();
	Parser(const Parser&) = delete;

	/**
	 * Update the parser with one byte of information from the EV3
	 *
	 * @param input byte of information from the EV3
	 * @return \ref ParserReturn structure containing parsing information
	 */
	ParserReturn update(uint8_t input);

	/**
	 * Obtain a pointer to the data received from the EV3 by the parser.
	 *
	 * Memory addresses up to, (but not including)
	 * \c BUFFER_LEN \c -1 bytes away from this pointer are guaranteed to be
	 * accessible.
	 *
	 * @pre update() returned a ParserReturn structure that
	 * has ParserReturn::res set to any value except
	 * ParseResult::INSUFFICIENT_DATA. If this precondition is not met,
	 * data can still be viewed through this pointer, but the area pointed to
	 * by this pointer <b> SHOULD NOT </b> be modified.
	 *
	 * @return pointer to the data received from the EV3
	 * res                       | Pointer points to
	 * --------------------------|------------------
	 * RECEIVED_INVALID_HEADER	 | Meaningless data
	 * RECEIVED_SYS_ACK			 | Meaningless data
	 * RECEIVED_SYS_NACK		 | Meaningless data
	 * RECEIVED_CMD_SELECT		 | SELECT message's payload
	 * RECEIVED_CMD_WRITE		 | WRITE message's payload
	 * RECEIVED_CMD_INVALID_FCS	 | Payload of message with invalid FCS
	 *
	 * \warning The block of memory pointed to by the returned pointer may be
	 * modified by update() on the next call to that function.
	 * Ownership of this area of memory is NOT TRANSFERRED to the user's
	 * application. If the payload needs to be reused <b> after another call
	 * to update() </b>, it needs to be copied to another
	 * buffer.
	 */
	uint8_t* data();

	/**
	 * Provides the same functionality as the similarly named function,
	 * except that it returns a pointer to a memory region that cannot
	 * be modified.
	 *
	 * @sa update()
	 * @return pointer to the data received from the EV3
	 */
	const uint8_t* data() const;

	/**
	 * Reset the state of the parser, so that the next byte input into the
	 * parser will be treated as a <b> header byte </b> candidate.
	 */
	void reset_state();
};
}




#endif /* EV3UARTPROTOCOLPARSERSENSORSIDE_HPP_ */
