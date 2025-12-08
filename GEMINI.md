# GEMINI.md - Project Overview

This document provides a brief overview of key files within the project, as requested.

## `Src/examples/shared_data/shared_functions.h`

This header file declares a collection of utility functions and macros primarily used for interaction with Decawave DW3000 Ultra-Wideband (UWB) transceivers. Its purpose is to provide common functionalities shared across various examples, such as:

- **Frame Manipulation**: Functions like `frame_is_poll_for_me`, `get_src_addr`, `set_dst_addr` are used for parsing and constructing UWB frames, handling source and destination addresses, and identifying message types (poll, response, final).
- **Timestamp Handling**: Functions like `get_tx_timestamp_u64`, `get_rx_timestamp_u64`, `final_msg_set_ts`, and `resp_msg_get_ts` are crucial for accurately reading and setting timestamps, which are fundamental to time-of-flight (ToF) calculations in ranging applications.
- **Power Boost Calculation**: Includes macros and a function `calculate_power_boost` to determine allowed power boosts based on frame duration, optimizing transmit power for different scenarios.
- **System Status and Delays**: Provides functions like `check_for_status_errors`, `get_rx_delay_time_txpreamble`, `set_delayed_rx_time`, and `waitforsysstatus` for managing the DW3000's system status, handling delays, and configuring receiver timeouts.

Essentially, `shared_functions.h` centralizes common low-level UWB communication and device management functionalities, promoting code reusability and consistency across different UWB examples.

## `Src/examples/ex_05a_ds_twr_init/ds_twr_initiator.c`

This file implements the **initiator** side of a Double-Sided Two-Way Ranging (DS TWR) distance measurement exchange using the Decawave DW3000 UWB transceiver.

Its primary responsibilities include:

- **Ranging Initiation**: It sends "poll" frames to start the ranging process with a responder.
- **Message Exchange**: It manages the sequence of messages: sending polls, receiving responses, and sending final messages.
- **Timestamp Collection**: Records various timestamps (TX time of poll, RX time of response, TX time of final message) at the initiator's end. These timestamps are critical for calculating time-of-flight.
- **Distance Calculation**: After receiving the necessary timestamps from the responder (embedded in the final message), it calculates the time-of-flight and subsequently the distance to the responder.
- **Configuration**: Configures the DW3000 device with specific UWB parameters (channel, preamble length, data rate, etc.) for optimal ranging performance.
- **Error Handling**: Includes callbacks for handling various events such as successful reception (`rx_ok_cb`), receive timeouts (`rx_to_cb`), and receive errors (`rx_err_cb`).

This example demonstrates how an UWB device can act as an initiator to actively measure distances to other UWB devices.

## `Src/examples/ex_05b_ds_twr_resp/ds_twr_responder.c`

This file implements the **responder** side of a Double-Sided Two-Way Ranging (DS TWR) distance measurement exchange. It is designed to work in conjunction with the `ds_twr_initiator.c` example.

Its key functions are:

- **Listening for Polls**: The responder continuously listens for "poll" messages from an initiator.
- **Responding to Polls**: Upon receiving a poll message, it records the RX timestamp of the poll and sends a "response" message with its own TX timestamp.
- **Processing Final Messages**: It then waits for a "final" message from the initiator. This final message contains timestamps from the initiator's perspective (poll TX, response RX, final TX).
- **Time-of-Flight Calculation**: Using its own recorded timestamps (poll RX, response TX, final RX) and the initiator's timestamps from the final message, the responder calculates the time-of-flight and estimates the distance to the initiator.
- **"INFO RECEIVER" Functionality**: This file also appears to implement functionality to receive and process "info" messages, suggesting it can act as a general data receiver in addition to its ranging role.
- **Configuration**: Similar to the initiator, it configures the DW3000 device with appropriate UWB parameters for accurate and reliable communication.

This example showcases how an UWB device can passively participate in a ranging exchange by responding to initiator requests and performing its own distance calculations.

## `Src/example_selection.h`

This header file serves as a central configuration point for selecting which example application is built and run.

- **Conditional Compilation**: It utilizes `#define` preprocessor directives to enable (uncomment) a single example at a time. Only the `#define` directive that is uncommented will have its corresponding code compiled into the final executable.
- **Example Listing**: The file lists numerous available examples, covering various UWB functionalities such as:
    - Reading Device ID (`TEST_READING_DEV_ID`)
    - Simple Transmit/Receive (`TEST_SIMPLE_TX`, `TEST_SIMPLE_RX`)
    - Two-Way Ranging (DS TWR, SS TWR in various modes)
    - Continuous Wave/Frame Transmission
    - Advanced features like AES, SPI CRC, GPIO control, and power adjustments.

By modifying this file, developers can easily switch between different example applications to test specific features or functionalities of the DW3000 UWB transceiver.
