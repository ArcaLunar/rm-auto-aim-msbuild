#include <iostream>
#include <toml++/toml.hpp>

int main() {
    toml::table T;
    try {
        T = toml::parse_file("../config/transform.toml");
        std::cout << T << std::endl;
    } catch (const toml::parse_error &err) {
        std::cerr << "Failed for " << err << std::endl;
        return 1;
    }

    return 0;
}