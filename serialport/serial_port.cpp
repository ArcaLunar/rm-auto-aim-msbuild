#include "serial_port.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <spdlog/spdlog.h>

#include <toml++/toml.hpp>

SerialPort::SerialPort(const std::string &config_path) {
    try {
        spdlog::info("serial_port: reading config from {}", config_path);

        toml::table T = toml::parse_file(config_path);
        [&]() { // retrieve candidate port names
            auto alt_ports = T["alternative_ports"];
            if (const auto *arr = alt_ports.as_array()) {
                for (const auto &port : *arr) this->cfg_.alternative_ports.push_back(port.as_string()->get());
            }
            cfg_.port_name = cfg_.alternative_ports[0];
        }();
    } catch (std::exception &err) {
        spdlog::error("serial_port: error reading config: {}, using fallback", err.what());
    }
}

bool SerialPort::initialize_port() {
    try {
        this->port_ = std::make_unique<boost::asio::serial_port>(this->io_service_, this->cfg_.port_name);
        this->__set_options();
        return true;
    } catch (std::exception &err) {
        spdlog::error("port init error: {}", err.what());
        return false;
    }
}

void SerialPort::__set_options() {
    using namespace boost::asio;
    // set baud rate
    this->port_->set_option(serial_port_base::baud_rate(this->cfg_.baud_rate));
    // set data bits
    this->port_->set_option(serial_port_base::character_size(this->cfg_.data_bits));
    // set stop bits
    this->port_->set_option(serial_port_base::stop_bits(
        this->cfg_.stop_bits == 1 ? serial_port_base::stop_bits::one : serial_port_base::stop_bits::two
    ));
    // set parity
    this->port_->set_option(serial_port_base::parity(
        this->cfg_.parity == 0   ? serial_port_base::parity::none
        : this->cfg_.parity == 1 ? serial_port_base::parity::odd
                                 : serial_port_base::parity::even
    ));
    // set flow control
    this->port_->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
}