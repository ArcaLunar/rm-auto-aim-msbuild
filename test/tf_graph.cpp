#include "transform.hpp"
#include <iostream>

int main() {
    Transform::CoordinateManager cm;
    Eigen::Matrix4d a = Eigen::Matrix4d::Random(), b = Eigen::Matrix4d::Random(), c = Eigen::Matrix4d::Random();
    cm.register_tf("world", "camera", a);
    cm.register_tf("camera", "base_link", b);
    cm.register_tf("base_link", "laser", c);

    Eigen::Matrix4d tf = cm.extract_tf_matrix("world", "laser");
    assert(tf == a * b * c);
    assert(cm.extract_tf_matrix("camera", "world") == a.inverse());
    assert(cm.extract_tf_matrix("base_link", "camera") == b.inverse());
    assert(cm.extract_tf_matrix("laser", "base_link") == c.inverse());
    assert(cm.extract_tf_matrix("laser", "world") == c.inverse() * b.inverse() * a.inverse());
    std::cout << tf << std::endl;

    return 0;
}