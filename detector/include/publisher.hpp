#ifndef __PUBLISHER_HPP__
#define __PUBLISHER_HPP__

#include "detector.hpp"
#include "pnp_solver.hpp"
#include "structs.hpp"

#include <memory>

namespace AutoAim {

/**
 * @brief 将识别出来的装甲板（三维）发送出去「到 Job Queue 中」
 *
 */
class Publisher {
    std::unique_ptr<Detector> detector_;

  public:
};

} // namespace AutoAim

#endif // __PUBLISHER_HPP__