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

class PolicySelector {
  public:
    void relabel(std::vector<AutoAim::Armor> &armor);
    void select();

  protected:
    AutoAim::Armor previous_;
};

#endif // __POLICY_HPP__