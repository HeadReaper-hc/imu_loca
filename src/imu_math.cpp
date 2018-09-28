//
// Created by huchao on 9/26/18.
//

#include "imu_math.h"

namespace imu_loca
{
    Eigen::Matrix3d getDissymmetyMatrix(const Eigen::Vector3d& vector)
    {
        Eigen::Matrix3d dissymmety;
        dissymmety << 0 , -vector(2) , vector(1) ,
                      vector(2) , 0 , -vector(0) ,
                      -vector(1) , vector(0) , 0 ;
        return dissymmety;
    }

}