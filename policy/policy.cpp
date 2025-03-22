#include "policy.hpp"
#include "structs.hpp"
#include <algorithm>
#include <spdlog/spdlog.h>

AutoAim::Labels SelectingPolicy::select_and_release(const std::vector<AnnotatedArmorInfo> &armors) {
    using namespace std::ranges;
    if (armors.empty())
        return AutoAim::Labels::None;

    // 1. 找上一次打过的车
    auto target = find_if(armors, [&](const AnnotatedArmorInfo &armor) { return armor.result == previous_.result; });
    if (target != armors.end()) {
        spdlog::info("found historical armor");
        return target->result;
    }

    // 2.
    return AutoAim::Labels::None;
}