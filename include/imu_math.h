//
// Created by huchao on 9/26/18.
//

#ifndef PROJECT_IMU_MATH_H
#define PROJECT_IMU_MATH_H
#include <Eigen/Dense>

namespace imu_loca
{
    Eigen::Matrix3d getDissymmetyMatrix(const Eigen::Vector3d& vector);
}

#endif //PROJECT_IMU_MATH_H
