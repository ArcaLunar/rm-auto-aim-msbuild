#include "port.hpp"
#include "structs.hpp"
#include <memory>
#include <thread>

int main() {
    std::shared_ptr<SerialPort<SentryVisionRecvMsg>> port
        = std::make_shared<SerialPort<SentryVisionRecvMsg>>("../config/port.toml");

    std::thread port_reader([&]{ port->read_raw();});
    std::thread port_processor([&]{ port->process_raw();});
    std::thread port_checker([&]{ port->check_reconnect();});
    // std::thread port_sender(&SerialPort::test_send, &port);
    port_reader.join();
    port_processor.join();
    port_checker.join();
    // port_sender.join();

    return 0;
}