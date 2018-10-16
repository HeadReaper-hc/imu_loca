//
// Created by huchao on 9/26/18.
//

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <Eigen/Dense>

#define OmegaIe 0.00007292115
#define delta_t 0.01              //sampling period  100hz
#define PI 3.1415926
#define Gravity  9.8         //gravity acc
#define Gravity_vec Eigen::Vector3d(0,0,Gravity)

#endif //PROJECT_CONFIG_H
