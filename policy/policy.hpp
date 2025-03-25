/**
 * @file policy.hpp
 * @author arca
 * @brief Given a set of armors, filter and select the armor to fire.
 * @version 0.1
 * @date 2025-02-06
 */

#ifndef __POLICY_HPP__
#define __POLICY_HPP__

#include "structs.hpp"

#include <vector>

class SelectingPolicy {
  public:
    /**
     * @brief 根据当前帧的装甲板信息，选择一个装甲板作为目标
     *
     * @param armors
     */
    AutoAim::Labels select(const std::vector<Armor3d> &armors);

  protected:
    Armor3d previous_;
};

#endif // __POLICY_HPP__