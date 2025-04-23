#include "port.hpp"
#include "structs.hpp"
#include <thread>

int main() {
    SerialPort<SentryVisionRecvMsg> port("../config/port.toml");

    std::thread port_reader(&SerialPort<SentryVisionRecvMsg>::read_raw, &port);
    std::thread port_processor(&SerialPort<SentryVisionRecvMsg>::process_raw, &port);
    std::thread port_checker(&SerialPort<SentryVisionRecvMsg>::check_reconnect, &port);
    // std::thread port_sender(&SerialPort::test_send, &port);
    port_reader.join();
    port_processor.join();
    port_checker.join();
    // port_sender.join();

    return 0;
}