#include "work_queue.hpp"
#include <spdlog/spdlog.h>
#include <unistd.h>

int main() {
    WorkQueue<int, 1024> wq;
    volatile std::atomic<int> goods = 0;
    wq.set_producer(
        [&]() {
            spdlog::info("producing: {}", goods);
            usleep(10000);
            return goods++;
        },
        3
    );
    wq.set_consumer([](int work) { spdlog::info("work: {}", work); }, 5);
    wq.start();

    sleep(1);
    wq.stop();

    return 0;
}