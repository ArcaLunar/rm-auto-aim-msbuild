#ifndef __PORT_HPP__
#define __PORT_HPP__

#include "boost/asio.hpp"
#include "structs.hpp"
#include <array>
#include <boost/asio/io_service.hpp>
#include <memory>

class SerialPort {
    static constexpr size_t kSendBufferSize = sizeof(VisionPLCSendMsg);
    static constexpr size_t kRecvBufferSize = sizeof(VisionPLCRecvMsg);
    static constexpr size_t kRecvMsgCount   = 20; // maximum of 20 cached messages

    using RecvMsgBuffer = std::array<VisionPLCRecvMsg, kRecvMsgCount>;
    using RawMessage    = std::array<u8, kRecvBufferSize>;

  public:
    SerialPort(std::string path = "../config/port.toml");

    void init_port();
    void read_raw();

  private:
    boost::asio::io_service port_io;
    std::unique_ptr<boost::asio::serial_port> port;

    size_t port_index{0};
    std::string port_name;
    std::vector<std::string> port_options; // alternative port options

    bool port_ok;

  protected:
    void _set_options();
};

#endif