#include "port.hpp"
#include "debug_options.hpp"
#include "structs.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <boost/system/error_code.hpp>
#include <chrono>
#include <cstring>
#include <spdlog/fmt/bin_to_hex.h>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <thread>
#include <toml++/toml.hpp>

// Check if DebugOptions is defined, if not provide default implementation
extern DebugOptions options;

SerialPort::SerialPort(std::string path) : port{ctx}, timer{ctx}, raw_cbuffer{kRecvMsgCount} {
    spdlog::info("Initializing serial port");
    last_recv = std::chrono::steady_clock::now();

    if (options.port.initialization)
        spdlog::info("Reading config from {}", path);

    try {
        toml::table config = toml::parse_file(path);

        // Read configuration from file
        cfg.baudrate  = config["baudrate"].value_or(460800);
        cfg.databit   = config["databit"].value_or(8);
        cfg.stopbit   = config["stopbit"].value_or(1);
        cfg.parity    = config["parity"].value_or(0);
        cfg.endbyte   = config["endbyte"].value_or(0xAA);
        cfg.startbyte = config["startbyte"].value_or(0x3A);

        auto ports = config["ports"];
        if (const auto *arr = ports.as_array()) {
            for (const auto &port : *arr)
                this->port_options.push_back(port.as_string()->get());
        } else {
            // Use the default port list if none provided in config
            this->port_options = kAlternativePorts;
        }

        // Use the first port in the list
        if (!this->port_options.empty()) {
            this->port_name = this->port_options[0];
        } else {
            spdlog::error("No serial ports defined in configuration");
            throw std::runtime_error("No serial ports defined");
        }

    } catch (const std::exception &e) {
        spdlog::error("Error in initializing port: {}", e.what());
        throw;
    }

    if (options.port.initialization)
        spdlog::info("Configuration loaded, setting options");

    this->open_port();
}

void SerialPort::open_port() {
    using namespace boost::asio;

    try {
        if (options.port.initialization)
            spdlog::info("Attempting to open port {} with io service", this->port_name);

        // Set options for port
        this->port.open(this->port_name);

        // Configure port parameters
        this->port.set_option(serial_port_base::baud_rate(this->cfg.baudrate));
        this->port.set_option(serial_port_base::character_size(this->cfg.databit));

        // Configure stop bits
        this->port.set_option(serial_port_base::stop_bits(
            this->cfg.stopbit == 1 ? serial_port_base::stop_bits::one : serial_port_base::stop_bits::two
        ));

        // Configure parity
        this->port.set_option(serial_port_base::parity(
            this->cfg.parity == 0   ? serial_port_base::parity::none
            : this->cfg.parity == 1 ? serial_port_base::parity::odd
                                    : serial_port_base::parity::even
        ));

        // Disable flow control
        this->port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        if (options.port.initialization) {
            spdlog::info("Serial port parameters:");
            spdlog::info("  Baudrate: {}", this->cfg.baudrate);
            spdlog::info("  Data bits: {}", this->cfg.databit);
            spdlog::info("  Stop bits: {}", this->cfg.stopbit);
            spdlog::info("  Parity: {}", this->cfg.parity);
            spdlog::info("  Start byte: 0x{:02X}", static_cast<int>(this->cfg.startbyte));
            spdlog::info("  End byte: 0x{:02X}", static_cast<int>(this->cfg.endbyte));
        }

        spdlog::info("Port options successfully set");
        this->port_ok = true;
    } catch (const boost::system::system_error &e) {
        spdlog::error("Failed to open port {}: {}", this->port_name, e.what());
        this->port_ok = false;
    }
}

void SerialPort::close_port() {
    if (this->port.is_open()) {
        boost::system::error_code ec;
        ec = this->port.cancel(ec);
        if (ec) {
            spdlog::error("Error cancelling port operations: {}", ec.message());
        }

        ec = this->port.close(ec);
        if (ec) {
            spdlog::error("Error closing port: {}", ec.message());
        }
        this->timer.cancel();

        spdlog::info("Serial port closed");
        this->port_ok = false;
    }
}

bool SerialPort::send_message(const VisionPLCSendMsg &payload) {
    if (!this->port_ok || !this->port.is_open()) {
        spdlog::error("Port is not ready for sending");
        return false;
    }

    // Create a local copy of the payload and update the flag
    VisionPLCSendMsg msg = payload;
    this->updated        = 1 - this->updated; // Toggle between 0 and 1
    msg.flag_updated     = this->updated;

    // Copy to send buffer
    std::memcpy(this->send_buffer.data(), &msg, kSendMsgSize);

    // Debug output if enabled
    if (options.port.inspect_data) {
        spdlog::info(
            "Sending data: {:X:n}", spdlog::to_hex(this->send_buffer.data(), this->send_buffer.data() + kSendMsgSize)
        );
    }

    try {
        // Write the data to the serial port
        size_t bytes_written
            = boost::asio::write(this->port, boost::asio::buffer(this->send_buffer.data(), kSendMsgSize));
        return bytes_written == kSendMsgSize;
    } catch (const boost::system::system_error &e) {
        spdlog::error("Failed to send message: {}", e.what());
        return false;
    }
}

void SerialPort::test_send() {
    VisionPLCSendMsg msg;
    std::array<u8, kSendMsgSize> send_buffer;
    while (true) {
        for (auto i : send_buffer)
            i = 89;
        std::memcpy(&msg, send_buffer.data(), kSendMsgSize);
        msg.flag_updated = 1 - this->updated; // Toggle between 0 and 1

        send_message(msg);
    }
}

void SerialPort::read_raw() {
    RawMessage raw;
    spdlog::info("(thread) Reading raw data from port...");

    while (true) {
        raw.fill(0);
        if (!this->port_ok || !this->port.is_open()) {
            spdlog::error("Port is not ready for reading");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        try {
            // Read data from the port
            size_t bytes_read = boost::asio::read(this->port, boost::asio::buffer(raw.data(), kRecvMsgSize));

            if (bytes_read > 0) {
                // Debug output if enabled
                if (options.port.inspect_data) {
                    spdlog::info("Received {} bytes", bytes_read);
                    spdlog::info("Raw data: {:X:n}", spdlog::to_hex(raw.data(), raw.data() + bytes_read));
                }

                this->raw_cbuffer.push(raw);
                this->last_recv = std::chrono::steady_clock::now();
            }
        } catch (const boost::system::system_error &e) {
            spdlog::error("Error reading from port: {}", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}

bool SerialPort::verify(const RawMessage &msg) {
    // Verify that the message follows the protocol format
    if (msg[0] != cfg.startbyte || msg[kRecvMsgSize - 1] != cfg.endbyte) {
        return false;
    }

    // Additional validation can be added here

    return true;
}

void SerialPort::process_raw() {
    VisionPLCRecvMsg payload;
    RawMessage raw;
    bool success = false;

    spdlog::info("(thread) Processing raw data...");

    while (true) {
        // Get the next raw message from the buffer
        std::tie(raw, success) = this->raw_cbuffer.pop();

        if (!success) {
            // No data available yet
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Process the raw message
        for (size_t i = 0; i < raw.size();) {
            size_t j = i + kRecvMsgSize;
            if (j > raw.size()) {
                break; // Not enough data
            }

            // Extract the message if it passes verification
            if (verify(raw)) {
                std::memcpy(&payload, raw.data() + i, kRecvMsgSize);
                this->recv_cbuffer.push(payload);
                i = j;
            } else {
                // Move one byte forward and continue scanning
                i++;
            }
        }
    }
}

void SerialPort::check_reconnect() {
    using namespace std::chrono;
    auto last_check = steady_clock::now();

    while (true) {
        if (last_recv == steady_clock::time_point()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue; // No messages received yet
        }

        auto now      = steady_clock::now();
        auto duration = duration_cast<milliseconds>(now - last_recv);

        if (duration.count() < kTimeout) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue; // Connection is still active
        }

        spdlog::error("Connection lost, trying to reconnect...");

        size_t idx = 0;
        while (true) {
            // Wait at least 1 second between reconnection attempts
            while (1000 > duration_cast<milliseconds>(steady_clock::now() - last_check).count()) {
                std::this_thread::yield();
            }

            last_check = steady_clock::now();

            // Try the next port in the list
            this->port_name = this->port_options[idx];
            idx             = (idx + 1) % this->port_options.size();

            spdlog::info("Attempting to connect to {}", this->port_name);

            this->close_port();
            this->open_port();

            if (this->port_ok) {
                spdlog::info("Successfully connected to {}", this->port_name);
                break;
            }
        }
    }
}