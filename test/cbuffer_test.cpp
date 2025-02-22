#include "circular_buffer.hpp"
#include <iostream>

int main() {
    CircularBuffer<int> buffer(10);

    std::cout << buffer.size() << '\n';

    for (int i = 0; i < 10; ++i) { buffer.push(i); }

    std::cout << buffer.size() << '\n';

    for (int i = 0; i < 10; ++i) {
        auto val = buffer.pop();
        if (val.has_value()) { std::cout << val.value() << '\n'; }
    }
}