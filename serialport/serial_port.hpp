#ifndef __SERIAL_PORT_HPP__
#define __SERIAL_PORT_HPP__

#include "circular_buffer.hpp"
#include "structs.hpp"
#include <boost/asio.hpp>

class SerialPort {
  public:
    SerialPort(const std::string &config_path);
    ~SerialPort();
    bool initialize_port();

  protected:
    boost::asio::io_service io_service_;
    std::unique_ptr<boost::asio::serial_port> port_;
    SerialPortConfiguration cfg_;

  private:
    void __set_options();
};

#endif // __SERIAL_PORT_HPP__