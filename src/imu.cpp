//
// Created by huchao on 9/26/18.
//

#include "imu.h"
#include <math.h>
#include "config.h"
#include <Eigen/Dense>
#include "imu_math.h"

namespace imu_loca
{
    imu::imu(Pose _inertiaCoord):imuAbstract(_inertiaCoord)
    {
       bodyCoord.velocity = Eigen::Vector3d(0,0,0);
       bodyCoord.rota = Eigen::Matrix3d::Identity();
       bodyCoord.trans = Eigen::Vector3d(0,0,0);
    };

    imu::~imu(){};

    //rotation increment is obtained from the gyroscope data(only used in fixed-axis rotation
    //Use geographic coordinates as inertial coordinates
     Eigen::Matrix3d imu::bodyRotationUpdate(const Eigen::Vector3d& omega)
    {
        Eigen::Vector3d delta_theta = omega * delta_t ;
        double delta_theta_norm = delta_theta.norm();
        return Eigen::Matrix3d::Identity()+sin(delta_theta_norm)/delta_theta_norm*getDissymmetyMatrix(delta_theta)+
        (1.0-cos(delta_theta_norm))/delta_theta_norm/delta_theta_norm*getDissymmetyMatrix(delta_theta)*
         getDissymmetyMatrix(delta_theta);

    }

    // Quaterniond increment is obtained from the gyroscope data(only used in fixed-axis rotation (x y z w)
    //Use geographic coordinates as inertial coordinates
     Eigen::Quaterniond imu::bodyQuaterniondUpdate(const Eigen::Vector3d& omega)
     {
         return Eigen::Quaterniond ( bodyRotationUpdate(omega) );
     }

     Eigen::Vector3d imu::bodyVelocityUpdate(const Eigen::Vector3d& acc)
     {
         return bodyCoord.rota*acc*delta_t - Gravity_vec*delta_t ;
     }

     Eigen::Vector3d imu::bodyPositionUpdate(const Eigen::Vector3d& acc)
     {
         return bodyCoord.velocity*delta_t + 0.5*bodyCoord.rota*acc*delta_t*delta_t - 0.5 * Gravity_vec *
                delta_t*delta_t ;
     }

     void imu::bodyPoseUpdate(const Eigen::Vector3d& acc , const Eigen::Vector3d& omega)
     {
         bodyCoord.trans += bodyPositionUpdate( acc );
         bodyCoord.velocity += bodyVelocityUpdate( acc );
         bodyCoord.rota = bodyCoord.rota*bodyRotationUpdate(omega);
     }


}

