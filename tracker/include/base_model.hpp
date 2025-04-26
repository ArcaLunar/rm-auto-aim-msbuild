#ifndef __TRACKER_DYNAMIC_MODEL_HPP__
#define __TRACKER_DYNAMIC_MODEL_HPP__

#include <Eigen/Dense>

class ObservationModel {
  public:
    virtual Eigen::MatrixXd operator()(const Eigen::VectorXd &state);
};

class ExactTransitionModel {
  public:
    virtual Eigen::MatrixXd operator()(const Eigen::VectorXd &state);
};

#endif