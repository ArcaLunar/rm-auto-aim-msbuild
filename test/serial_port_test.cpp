#include <boost/asio.hpp>
#include <exception>
#include <iostream>
#include <spdlog/spdlog.h>

int main() {
    try {
        using namespace boost::asio;
        io_service io;

        serial_port port(io, "/dev/pts/2");
        port.set_option(serial_port_base::baud_rate(9600));
        port.set_option(serial_port_base::character_size(8));
        port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        port.set_option(serial_port_base::parity(serial_port_base::parity::none));
        port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        // 写入数据
        std::string message = "Hello from Boost.Asio!\n";
        write(port, buffer(message.c_str(), message.size()));
        std::cout << "Sent: " << message;

        // 读取响应（同步读取）
        char response[128];
        size_t len = read(port, buffer(response), transfer_at_least(1));
        std::cout << "Received: " << std::string(response, len) << std::endl;

    } catch (const std::exception &err) {
        spdlog::critical("error in auto_aim.main(): {}", err.what());
        exit(-1);
    }
}