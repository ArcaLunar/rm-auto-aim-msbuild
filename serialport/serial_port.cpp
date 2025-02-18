#include "serial_port.hpp"

#include <chrono>
#include <cstring>
#include <exception>
#include <memory>
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

SerialPort::SerialPort(const std::string &config_path) : port_index_{0}, updated_{0}, port_ok{false} {
    try {
        spdlog::info("serial_port: reading config from {}", config_path);
        toml::table T = toml::parse_file(config_path);

        [&]() { // retrieve candidate port names
            auto alt_ports = T["alternative_ports"];
            if (const auto *arr = alt_ports.as_array()) {
                for (const auto &port : *arr) this->alt_ports_.push_back(port.as_string()->get());
            }
            cfg_.port_name = alt_ports_[0];
        }();
    } catch (std::exception &err) {
        spdlog::error("serial_port: error reading config: {}, using fallback", err.what());
    }
}

void SerialPort::initialize_port() {
    try {
        this->port_ = std::make_unique<boost::asio::serial_port>(this->io_service_, this->cfg_.port_name);
        spdlog::info("port {} opened successfully", this->cfg_.port_name);
        this->__set_options();
        this->port_ok = true;
        spdlog::info("port options set successfully");
    } catch (std::exception &err) {
        spdlog::error("port init error: {}", err.what());
        exit(-1);
    }
}

void SerialPort::close_port() {
    if (this->port_->is_open()) {
        this->port_->close();
        spdlog::info("port closed");
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

bool SerialPort::send_data(const VisionPLCSendMsg &msg) {
    VisionPLCSendMsg msg_to_send  = msg;
    updated_                      = 1 - updated_; // set the updated flag
    msg_to_send.flag_have_updated = updated_;
    spdlog::info("sending data");
    memset(send_frame_buffer_, 0, kSendBufSize);
    memcpy(send_frame_buffer_, &msg_to_send, kSendBufSize); // copy the data to the buffer

    // send buffer to the port (with error handling)
    try {
        auto size_of_data_sent
            = boost::asio::write(*this->port_, boost::asio::buffer(send_frame_buffer_, kSendBufSize));
        return size_of_data_sent == kSendBufSize;
    } catch (const std::exception &err) {
        spdlog::error("serial_port.send_data() error: {}", err.what());
        return false;
    }
}