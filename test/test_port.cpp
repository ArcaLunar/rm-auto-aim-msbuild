#include "port.hpp"
#include <thread>

int main() {
    SerialPort port("../../config/port.toml");

    std::thread port_reader(&SerialPort::read_raw, &port);
    std::thread port_processor(&SerialPort::process_raw, &port);
    std::thread port_checker(&SerialPort::check_reconnect, &port);
    port_reader.join();
    port_processor.join();
    port_checker.join();

    return 0;
}