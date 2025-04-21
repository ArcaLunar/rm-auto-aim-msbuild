#include "port.hpp"
#include "debug_options.hpp"
#include "structs.hpp"
#include "toml++/impl/parser.hpp"
#include "toml++/impl/table.hpp"
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <chrono>
#include <cstring>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>

extern DebugOptions options;

SerialPort::SerialPort(std::string path) : port{ctx}, timer{ctx}, raw_cbuffer{kRecvMsgCount} {
    spdlog::info("initializing serial port");
    last_recv = std::chrono::steady_clock::now();

    if (options.port.initialization)
        spdlog::info("reading config from {}", path);

    try {
        toml::table config = toml::parse_file(path);

        // * read config from file
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
        }
        this->port_name = this->port_options[0];

    } catch (std::runtime_error &e) {
        spdlog::error("Error in initializing port: {}", e.what());
        throw e;
    }

    if (options.port.initialization)
        spdlog::info("configuration loaded, setting options");

    this->open_port();
}

void SerialPort::open_port() {
    using namespace boost::asio;

    if (options.port.initialization)
        spdlog::info("port name {} initialized with io service", this->port_name);

    //~ Set options for port
    this->port.open(this->port_name);
    //~ baud rate
    this->port.set_option(serial_port_base::baud_rate(this->cfg.baudrate));

    //~ data bit
    this->port.set_option(serial_port_base::character_size(this->cfg.databit));

    //~ stop bit
    this->port.set_option(serial_port_base::stop_bits(
        this->cfg.stopbit == 1 ? serial_port_base::stop_bits::one : serial_port_base::stop_bits::two
    ));
    //~ parity
    this->port.set_option(serial_port_base::parity(
        this->cfg.parity == 0   ? serial_port_base::parity::none
        : this->cfg.parity == 1 ? serial_port_base::parity::odd
                                : serial_port_base::parity::even
    ));
    this->port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

    if (options.port.initialization) {
        spdlog::info("baudrate: {}", this->cfg.baudrate);
        spdlog::info("databit: {}", this->cfg.databit);
        spdlog::info("stopbit: {}", this->cfg.stopbit);
        spdlog::info("parity: {}", this->cfg.parity);
        spdlog::info("endbyte: {:2X}", (int)this->cfg.endbyte);
        spdlog::info("startbyte: {:2X}", (int)this->cfg.startbyte);
    }

    spdlog::info("port options successfully set");
    this->port_ok = true;
}

void SerialPort::close_port() {
    if (this->port.is_open()) {
        this->port.cancel();
        this->port.close();
        this->timer.cancel();

        spdlog::info("serial port closed");
    }
}

void SerialPort::read_raw() {
    RawMessage raw;
    spdlog::info("(thread) reading raw data from port ...");
    while (true) {
        raw.fill(0);
        if (!this->port.is_open()) {
            spdlog::error("port is not ready");
            continue;
        }

        // read from port
        auto size_data = boost::asio::read(this->port, boost::asio::buffer(raw.data(), kRecvMsgSize));

        if (size_data > 0) {
            // debug
            if (options.port.inspect_data) {
                spdlog::info("reveiced {} bytes", size_data);
                std::stringstream ss;
                for (int i = 0; i < size_data; ++i)
                    ss << "[" << i << "]=" << std::hex << std::setw(2) << std::setfill('0') << (int)raw[i] << " ";
                spdlog::info("raw data: {}", ss.str());
            }

            this->raw_cbuffer.push(raw);
            this->last_recv = std::chrono::steady_clock::now();
        }
    }
}

void SerialPort::process_raw() {
    VisionPLCRecvMsg payload;
    RawMessage raw;
    bool verdict = false;
    raw.fill(0);
    spdlog::info("(thread) processing raw data ...");
    while (true) {
        std::tie(raw, verdict) = this->raw_cbuffer.pop();
        if (!verdict) {
            spdlog::warn("no data yet");
            continue;
        }

        for (size_t i = 0; i < raw.size();) {
            size_t j = i + kRecvMsgSize;
            if (j > raw.size()) {
                spdlog::warn("not enough data");
                break;
            }

            // * Verify
            if (j - i != sizeof(VisionPLCRecvMsg)) {
                spdlog::warn("data size mismatch");
                i++;
                continue;
            } else if (raw.size() < sizeof(VisionPLCRecvMsg)) {
                spdlog::warn("data size mismatch");
                i++;
                continue;
            } else if (i > j or j > raw.size()) {
                spdlog::warn("invalid data range");
                i++;
                continue;
            } else if (raw[i] != this->cfg.startbyte or raw[j - 1] != this->cfg.endbyte) {
                spdlog::warn("protocol check failed!");
                i++;
                continue;
            }
            std::memcpy(&payload, raw.data() + i, kRecvMsgSize);
            recv_cbuffer.push(payload);
            i = j;
        }
    }
}

void SerialPort::check_reconnect() {
    using namespace std::chrono;
    auto last_check = steady_clock::now();

    while (true) {
        if (last_recv == steady_clock::time_point())
            continue; // just received a message

        auto now      = steady_clock::now();
        auto duration = duration_cast<milliseconds>(now - last_recv);

        if (duration.count() <= kTimeout)
            continue; // did not exceed time out

        spdlog::error("port timeout, trying to reconnect ...");

        size_t idx = 0; // index for next port
        while (true) {
            while (1000 > duration_cast<milliseconds>(steady_clock::now() - last_check).count())
                std::this_thread::yield();

            last_check = steady_clock::now();

            this->port_name = this->port_options[idx];
            idx             = (idx + 1) % this->port_options.size();

            this->port_ok = false;
            this->close_port();

            this->open_port();
            if (this->port_ok) {
                spdlog::info("successfully connected to port {}", this->port_name);
                this->port_ok = true;
                break;
            }
        }
    }
}