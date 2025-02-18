#include "serial_port.hpp"
#include "structs.hpp"

#include <boost/asio/basic_datagram_socket.hpp>
#include <chrono>
#include <cstring>
#include <exception>
#include <memory>
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

SerialPort::SerialPort(const std::string &config_path)
    : port_index_{0},
      port_ok{false},
      updated_{0},
      last_recv_(std::chrono::steady_clock::now()),
      recv_buffer_(kRecvMsgCount) {
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

void SerialPort::read_raw_data_from_port() {
    SerialPort::RecvMsgBuffer data;
    try {
        while (true) {
            data.fill(0);
            if (!this->port_ok) continue;

            auto size_of_data_read = boost::asio::read(*this->port_, boost::asio::buffer(data.data(), data.size()));
            if (size_of_data_read == kRecvMsgSize) this->recv_buffer_.push(data);
            else spdlog::warn("serial_port.read_raw_data_from_port() warning: read size mismatch, ignoring");
        }
    } catch (const std::exception &err) {
        spdlog::error("serial_port.read_raw_data_from_port() error: {}", err.what());
        exit(-1);
    }
}

void SerialPort::process_raw_data_from_buffer() {
    VisionPLCRecvMsg data;
    SerialPort::RecvMsgBuffer buffer;
    buffer.fill(0);

    while (true) {
        auto data_opt = this->recv_buffer_.pop();
        if (!data_opt.has_value()) continue; // wait for data

        for (size_t i = 0, n = buffer.size(); i < n;) { // Process raw data to msg data structure
            size_t j = i + kRecvMsgSize;
            if (j > n) break; // range: [i, j)

            //* frame verification
            if (!this->__verify_frame<VisionPLCRecvMsg>(buffer, i, j)) {
                // invalid frame, skip
                ++i;
                continue;
            }
            // valid frame, copy to data recv buffer
            std::memcpy(&data, buffer.data() + i, kRecvMsgSize);
            data_recv_buffer_.push(data);
            i = j;
        }
    }
}

template <typename MsgProtocol>
bool SerialPort::__verify_frame(const SerialPort::RecvMsgBuffer &buffer, size_t begin, size_t end) {
    if (end - begin != sizeof(MsgProtocol)) return false;                // 下标标记的 buffer 大小不对
    else if (buffer.size() < sizeof(MsgProtocol)) return false;          // buffer 太小
    else if (begin < 0 || end < 0) return false;                         // 下标为负数
    else if (end > buffer.size() || begin > buffer.size()) return false; // 下标超出范围
    else if (buffer[begin] != kProtocolRecvHead || buffer[end - 1] != kProtocolTail) return false; // 首尾不对
    else return true;
}
