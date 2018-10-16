//
// Created by huchao on 9/26/18.
//

#ifndef IMU_H_
#define IMU_H_
#include <Eigen/Dense>
#include "imu_abstract.h"
#include <vector>


namespace imu_loca
{
    class imu : public imuAbstract
    {
        public:


			imu(){};
            imu(Pose _inertiaCoord);
            ~imu();

            virtual Eigen::Matrix3d bodyRotationUpdate(const Eigen::Vector3d& omega)  ;

            virtual Eigen::Quaterniond bodyQuaterniondUpdate(const Eigen::Vector3d& omega);

            virtual Eigen::Vector3d bodyVelocityUpdate(const Eigen::Vector3d& acc) ;

            virtual Eigen::Vector3d bodyPositionUpdate(const Eigen::Vector3d& acc) ;

            virtual void bodyPoseUpdate(const Eigen::Vector3d& acc , const Eigen::Vector3d& omega) ;

            virtual Eigen::Matrix3d bodyEquivalentRotationVectorUpdate(const Eigen::Vector3d& omega) ;



        private:

        std::vector<Eigen::Vector3d> omega_2;
        bool omega_time = true;


    };


}
#endif //PROJECT_IMU_H
