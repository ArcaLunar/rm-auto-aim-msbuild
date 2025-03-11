#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "serial_port.hpp"
#include "config.hpp"
#include "structs.hpp"

#include <boost/asio/basic_datagram_socket.hpp>
#include <chrono>
#include <cstring>
#include <exception>
#include <memory>
#include <spdlog/spdlog.h>
#include <sstream>
#include <thread>
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
            std::stringstream ss;
            auto alt_ports = T["alternative_ports"];
            if (const auto *arr = alt_ports.as_array()) {
                for (const auto &port : *arr)
                    this->alt_ports_.push_back(port.as_string()->get());
                for (auto &port : this->alt_ports_)
                    ss << port << " ";
            }
            cfg_.port_name = alt_ports_[0];

            if constexpr (SerialPortDebug)
                spdlog::info("serial_port: alternative ports = {}", ss.str());
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
    if constexpr (SerialPortDebug)
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
            data.fill(1);
            if (!this->port_ok)
                continue;

            auto size_of_data_read = boost::asio::read(*this->port_, boost::asio::buffer(data.data(), data.size()));
            if (size_of_data_read == kRecvMsgSize)
                this->recv_buffer_.push(data);
            else if constexpr (SerialPortDebug)
                spdlog::warn("serial_port.read_raw_data_from_port() warning: read size mismatch, ignoring");
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
        if (!data_opt.has_value())
            continue; // wait for data

        if constexpr (SerialPortDebug)
            spdlog::info("data received from port buffer");

        buffer = data_opt.value();
        for (size_t i = 0, n = buffer.size(); i < n;) { // Process raw data to msg data structure
            size_t j = i + kRecvMsgSize;
            if (j > n)
                break; // range: [i, j)

            //* frame verification
            if (!this->__verify_frame<VisionPLCRecvMsg>(buffer, i, j)) {
                // invalid frame, skip
                spdlog::warn("invalid frame, skip");
                ++i;
                continue;
            }
            // valid frame, copy to data recv buffer
            std::memcpy(&data, buffer.data() + i, kRecvMsgSize);
            data_recv_buffer_.push(data);
            i = j;
            if constexpr (SerialPortDebug)
                spdlog::info("data processed from bits to float");
        }
    }
}

template <typename MsgProtocol>
bool SerialPort::__verify_frame(const SerialPort::RecvMsgBuffer &buffer, size_t begin, size_t end) {
    if constexpr (SerialPortDebug)
        spdlog::info(
            "buffer[0]={:x}, buffer[1]={:x}, buffer[2]={:x}, buffer[3]={:x} buffer[4]={:x} buffer[5]={:x} "
            "buffer[6]={:x} "
            "buffer[7]={:x} "
            "buffer[8]={:x} buffer[9]={:x} buffer[10]={:x} buffer[11]={:x} buffer[12]={:x} buffer[13]={:x} "
            "buffer[14]={:x} "
            "buffer[15]={:x} "
            "buffer[16]={:x}",
            buffer[0],
            buffer[1],
            buffer[2],
            buffer[3],
            buffer[4],
            buffer[5],
            buffer[6],
            buffer[7],
            buffer[8],
            buffer[9],
            buffer[10],
            buffer[11],
            buffer[12],
            buffer[13],
            buffer[14],
            buffer[15],
            buffer[16]
        );

    if (end - begin != sizeof(MsgProtocol)) {
        if constexpr (SerialPortDebug)
            spdlog::error("invalid range size: begin={} end={}", begin, end);
        return false;
    } // 下标标记的 buffer 大小不对
    else if (buffer.size() < sizeof(MsgProtocol)) {
        if constexpr (SerialPortDebug)
            spdlog::error("buffer size {} too small", buffer.size());
        return false;
    } // buffer 太小
    else if (begin < 0 || end < 0) {
        if constexpr (SerialPortDebug)
            spdlog::error("invalid range: begin={} end={}", begin, end);
        return false;
    } // 下标为负数
    else if (end > buffer.size() || begin > buffer.size()) {
        if constexpr (SerialPortDebug)
            spdlog::error("invalid range: begin={} end={} buffer_size={}", begin, end, buffer.size());
        return false;
    } // 下标超出范围
    else if (buffer[begin] != kProtocolRecvHead || buffer[end - 1] != kProtocolTail) {
        if constexpr (SerialPortDebug)
            spdlog::error("invalid head or tail: begin={:x} end={:x}", buffer[begin], buffer[end - 1]);
        return false;
    } // 首尾不对
    else
        return true;
}

void SerialPort::check_port_and_auto_reconnect() {
    using namespace std::chrono;
    auto last_check = high_resolution_clock::now();

    while (true) {
        if (last_recv_ == steady_clock::time_point())
            continue;

        auto now      = steady_clock::now();
        auto duration = duration_cast<milliseconds>(now - last_recv_);

        if (duration.count() < kTimeout)
            continue; // 未超时
        spdlog::error("serial_port error: port timeout, reconnecting");

        while (true) {
            // test at 1Hz
            while (1000 > duration_cast<milliseconds>(high_resolution_clock::now() - last_check).count())
                std::this_thread::yield();

            last_check = high_resolution_clock::now();

            std::string next_port = alt_ports_[port_index_];               // 先重试当前端口
            port_index_           = (port_index_ + 1) % alt_ports_.size(); // 选择下一个端口
            port_ok               = false;                                 // 重置端口状态
            close_port();
            cfg_.port_name = next_port;

            try {
                initialize_port();
                if (!port_ok)
                    throw std::runtime_error(cfg_.port_name + " port not ok");
                spdlog::info("port reconnected successfully");
                break;
            } catch (const std::exception &err) {
                spdlog::error("port reconnect error: {}", err.what());
            }
        }
    }
}

std::optional<VisionPLCRecvMsg> SerialPort::get_data() { return data_recv_buffer_.pop(); }