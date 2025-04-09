#include "port.hpp"
#include "debug_options.hpp"
#include "toml++/impl/parser.hpp"
#include "toml++/impl/table.hpp"
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <chrono>
#include <spdlog/spdlog.h>
#include <stdexcept>

extern DebugOptions options;

SerialPort::SerialPort(std::string path) {
    spdlog::info("initializing serial port");
    last_recv = std::chrono::system_clock::now();

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

    this->init_port();
}

void SerialPort::init_port() {
    using namespace boost::asio;
    this->port = std::make_unique<serial_port>(this->port_io, this->port_name);
    if (options.port.initialization)
        spdlog::info("port name {} initialized with io service", this->port_name);

    //~ Set options for port
    //~ baud rate
    this->port->set_option(serial_port_base::baud_rate(this->cfg.baudrate));

    //~ data bit
    this->port->set_option(serial_port_base::character_size(this->cfg.databit));

    //~ stop bit
    this->port->set_option(serial_port_base::stop_bits(
        this->cfg.stopbit == 1 ? serial_port_base::stop_bits::one : serial_port_base::stop_bits::two
    ));
    //~ parity
    this->port->set_option(serial_port_base::parity(
        this->cfg.parity == 0   ? serial_port_base::parity::none
        : this->cfg.parity == 1 ? serial_port_base::parity::odd
                                : serial_port_base::parity::even
    ));
    this->port->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

    if (options.port.initialization)
        spdlog::info("port options successfully set");
}