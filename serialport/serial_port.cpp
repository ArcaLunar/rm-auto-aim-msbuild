#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "serial_port.hpp"
#include "config.hpp"
#include "structs.hpp"

#include <boost/asio/basic_datagram_socket.hpp>
#include <chrono>
#include <cstring>
#include <exception>
#include <memory>
#include <spdlog/sinks/stdout_color_sinks.h>
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
        this->log_ = spdlog::stdout_color_mt("serial_port");
        this->log_->set_level(spdlog::level::trace);
        this->log_->set_pattern("[%H:%M:%S, +%4oms] [%15s:%3# in %!] [%^%l%$] %v");

        SPDLOG_LOGGER_INFO(this->log_, "serial_port: reading config from {}", config_path);
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
                SPDLOG_LOGGER_INFO(this->log_, "serial_port: alternative ports = {}", ss.str());
        }();
    } catch (std::exception &err) {
        SPDLOG_LOGGER_ERROR(this->log_, "serial_port: error reading config: {}, using fallback", err.what());
    }
}

// SerialPort::~SerialPort() {
//     SPDLOG_LOGGER_INFO(this->log_, "serial_port: exiting......");
//     close_port();
// }

void SerialPort::initialize_port() {
    try {
        this->port_ = std::make_unique<boost::asio::serial_port>(this->io_service_, this->cfg_.port_name);
        SPDLOG_LOGGER_INFO(this->log_, "port {} opened successfully", this->cfg_.port_name);
        this->__set_options();
        this->port_ok = true;
        SPDLOG_LOGGER_INFO(this->log_, "port options set successfully");
    } catch (std::exception &err) {
        SPDLOG_LOGGER_ERROR(this->log_, "port init error: {}", err.what());
        exit(-1);
    }
}

void SerialPort::close_port() {
    if (this->port_->is_open()) {
        this->port_->close();
        SPDLOG_LOGGER_INFO(this->log_, "port closed");
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
        SPDLOG_LOGGER_INFO(this->log_, "sending data");
    memset(send_frame_buffer_, 0, kSendBufSize);
    memcpy(send_frame_buffer_, &msg_to_send, kSendBufSize); // copy the data to the buffer

    // send buffer to the port (with error handling)
    try {
        auto size_of_data_sent
            = boost::asio::write(*this->port_, boost::asio::buffer(send_frame_buffer_, kSendBufSize));
        return size_of_data_sent == kSendBufSize;
    } catch (const std::exception &err) {
        SPDLOG_LOGGER_ERROR(this->log_, "serial_port.send_data() error: {}", err.what());
        return false;
    }
}

void SerialPort::read_raw_data_from_port() {
    RawMessage<kRecvMsgSize> tmp_data;

    try {
        while (true) {
            tmp_data.data.fill(1);
            tmp_data.timestamp = std::chrono::system_clock::now();
            if (!this->port_ok)
                continue;

            auto size_of_data_read
                = boost::asio::read(*this->port_, boost::asio::buffer(tmp_data.data.data(), tmp_data.data.size()));

            if (size_of_data_read == kRecvMsgSize)
                this->recv_buffer_.push(tmp_data);
            else if constexpr (SerialPortDebug)
                SPDLOG_LOGGER_WARN(
                    this->log_, "serial_port.read_raw_data_from_port() warning: read size mismatch, ignoring"
                );
        }
    } catch (const std::exception &err) {
        SPDLOG_LOGGER_ERROR(this->log_, "serial_port.read_raw_data_from_port() error: {}", err.what());
        exit(-1);
    }
}

void SerialPort::process_raw_data_from_buffer() {
    // VisionPLCRecvMsg data;
    SerialPort::RecvMsgBuffer buffer;
    StampedRecvMsg stamped_data;
    buffer.data.fill(0);

    while (true) {
        auto data_opt = this->recv_buffer_.pop();
        if (!data_opt.has_value())
            continue; // wait for data

        if constexpr (SerialPortDebug)
            SPDLOG_LOGGER_INFO(this->log_, "data received from port buffer");

        buffer = data_opt.value();
        for (size_t i = 0, n = buffer.data.size(); i < n;) { // Process raw data to msg data structure
            size_t j = i + kRecvMsgSize;
            if (j > n)
                break; // range: [i, j)

            //* frame verification
            if (!this->__verify_frame<VisionPLCRecvMsg>(buffer, i, j)) {
                // invalid frame, skip
                SPDLOG_LOGGER_WARN(this->log_, "invalid frame, skip");
                ++i;
                continue;
            }
            // valid frame, copy to data recv buffer
            std::memcpy(&stamped_data.msg, buffer.data.data() + i, kRecvMsgSize);
            data_recv_buffer_.push(stamped_data);
            i = j;
            if constexpr (SerialPortDebug)
                SPDLOG_LOGGER_INFO(this->log_, "data processed from bits to float");
        }
    }
}

template <typename MsgProtocol>
bool SerialPort::__verify_frame(const SerialPort::RecvMsgBuffer &buffer, size_t begin, size_t end) {
    if constexpr (SerialPortDebug)
        SPDLOG_LOGGER_INFO(
            this->log_,
            "buffer[0]={:x}, buffer[1]={:x}, buffer[2]={:x}, buffer[3]={:x} buffer[4]={:x} buffer[5]={:x} "
            "buffer[6]={:x} "
            "buffer[7]={:x} "
            "buffer[8]={:x} buffer[9]={:x} buffer[10]={:x} buffer[11]={:x} buffer[12]={:x} buffer[13]={:x} "
            "buffer[14]={:x} "
            "buffer[15]={:x} "
            "buffer[16]={:x}",
            buffer.data[0],
            buffer.data[1],
            buffer.data[2],
            buffer.data[3],
            buffer.data[4],
            buffer.data[5],
            buffer.data[6],
            buffer.data[7],
            buffer.data[8],
            buffer.data[9],
            buffer.data[10],
            buffer.data[11],
            buffer.data[12],
            buffer.data[13],
            buffer.data[14],
            buffer.data[15],
            buffer.data[16]
        );

    if (end - begin != sizeof(MsgProtocol)) {
        if constexpr (SerialPortDebug)
            SPDLOG_LOGGER_ERROR(this->log_, "invalid range size: begin={} end={}", begin, end);
        return false;
    } // 下标标记的 buffer 大小不对
    else if (buffer.data.size() < sizeof(MsgProtocol)) {
        if constexpr (SerialPortDebug)
            SPDLOG_LOGGER_ERROR(this->log_, "buffer size {} too small", buffer.data.size());
        return false;
    } // buffer 太小
    else if (begin < 0 || end < 0) {
        if constexpr (SerialPortDebug)
            SPDLOG_LOGGER_ERROR(this->log_, "invalid range: begin={} end={}", begin, end);
        return false;
    } // 下标为负数
    else if (end > buffer.data.size() || begin > buffer.data.size()) {
        if constexpr (SerialPortDebug)
            SPDLOG_LOGGER_ERROR(
                this->log_, "invalid range: begin={} end={} buffer_size={}", begin, end, buffer.data.size()
            );
        return false;
    } // 下标超出范围
    else if (buffer.data[begin] != kProtocolRecvHead || buffer.data[end - 1] != kProtocolTail) {
        if constexpr (SerialPortDebug)
            SPDLOG_LOGGER_ERROR(
                this->log_, "invalid head or tail: begin={:x} end={:x}", buffer.data[begin], buffer.data[end - 1]
            );
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
        SPDLOG_LOGGER_ERROR(this->log_, "serial_port error: port timeout, reconnecting");

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
                SPDLOG_LOGGER_INFO(this->log_, "port reconnected successfully");
                break;
            } catch (const std::exception &err) {
                SPDLOG_LOGGER_ERROR(this->log_, "port reconnect error: {}", err.what());
            }
        }
    }
}

std::optional<StampedRecvMsg> SerialPort::get_data() { return data_recv_buffer_.pop(); }