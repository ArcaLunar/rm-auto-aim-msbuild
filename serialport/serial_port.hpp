#ifndef __SERIAL_PORT_HPP__
#define __SERIAL_PORT_HPP__

#include "circular_buffer.hpp"
#include "structs.hpp"
#include <boost/asio.hpp>
#include <chrono>

constexpr size_t kSendBufSize = sizeof(VisionPLCSendMsg);
constexpr size_t kRecvMsgSize = sizeof(VisionPLCRecvMsg);
constexpr size_t kRecvBufSize = kRecvMsgSize * 10;

class SerialPort {
  public:
    /**
     * @brief Construct a new SerialPort object, and read the configuration from the specified path.
     *
     * @param config_path path to the configuration file.
     */
    SerialPort(const std::string &config_path);

    /**
     * @brief Destroy the Serial Port object. close port on exit
     *
     */
    ~SerialPort();

    /**
     * @brief Try to open the serial port and set the options.
     * @details This will cause a force exit if the port cannot be opened.
     *
     */
    void initialize_port();

    /**
     * @brief Close the serial port.
     *
     */
    void close_port();

    /**
     * @brief 向串口发送打包好的数据
     */
    bool send_data(const VisionPLCSendMsg &msg);

    /**
     * @brief 从串口读取原始数据，并放入缓冲区
     * @remark make it `thread`
     */
    void read_raw_data_from_port();

    /**
     * @brief 从缓冲区处理原始数据
     * @remark make it `thread`
     */
    void process_raw_data_from_buffer();

    /**
     * @brief 检查串口是否正常，如果不正常则自动换串口重连
     * @remark make it `thread`
     */
    void check_port_and_auto_reconnect();

  protected:
    boost::asio::io_service io_service_;             // io_service
    std::unique_ptr<boost::asio::serial_port> port_; // 串口
    size_t port_index_{0};                           // 端口索引
    std::vector<std::string> alt_ports_;             // 备选端口

    SerialPortConfiguration cfg_; // 串口配置
    bool port_ok;                 // 串口是否正常

    uint8_t updated_;                                 // 更新位，用于判断是否是新数据
    std::chrono::steady_clock::time_point last_recv_; // 上次更新时间

    uint8_t send_frame_buffer_[kSendBufSize]; // 发送缓冲区

  private:
    void __set_options();
};

#endif // __SERIAL_PORT_HPP__