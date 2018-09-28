//
// Created by huchao on 9/26/18.
//

#ifndef IMU_H_
#define IMU_H_
#include <Eigen/Dense>
#include "imu_abstract.h"


namespace imu_loca
{
    class imu : public imuAbstract
    {
        public:



            imu(Pose _inertiaCoord);
            ~imu();

            virtual Eigen::Matrix3d bodyRotationUpdate(const Eigen::Vector3d& omega)  ;

            virtual Eigen::Quaterniond bodyQuaterniondUpdate(const Eigen::Vector3d& omega);

            virtual Eigen::Vector3d bodyVelocityUpdate(const Eigen::Vector3d& acc) ;

            virtual Eigen::Vector3d bodyPositionUpdate(const Eigen::Vector3d& acc) ;

            virtual void bodyPoseUpdate(const Eigen::Vector3d& acc , const Eigen::Vector3d& omega) ;


    };


}
#endif //PROJECT_IMU_H
