#include "policy.hpp"
#include "structs.hpp"
#include <algorithm>
#include <spdlog/spdlog.h>

void SelectingPolicy::select_and_release(const std::vector<AnnotatedArmorInfo> &armors) {
    using namespace std::ranges;
    if (armors.empty())
        return;

    // 1. 找上一次打过的车
    auto target = find_if(armors, [&](const AnnotatedArmorInfo &armor) { return armor.result == previous_.result; });
    if (target != armors.end()) {
        spdlog::info("found historical armor");
        grant_fire_permission(target->result);
        return;
    }

    // 2.
}