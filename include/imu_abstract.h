/* author: hu chao
** email : 11734026@zju.edu.cn
** date  : 2018.9.26
*/

#ifndef IMU_ABSTRACT_H_
#define IMU_ABSTRACT_H_
#include <Eigen/Dense>
#include <math.h>
#include "config.h"


namespace imu_loca {


    typedef struct pose {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d trans;   //define translation vector
        Eigen::Matrix3d rota; //define rotation matrix
        Eigen::Vector3d velocity; //define body velocity
    } Pose;


    class imuAbstract {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		imuAbstract(){};

        imuAbstract(Pose _inertiaCoord){inertiaCoord=_inertiaCoord;};

        virtual ~imuAbstract(){};

        inline Eigen::Matrix3d coord_inertial2earth(double time)
        {
            Eigen::Matrix3d t;
            t<< cos(OmegaIe*time) , sin(OmegaIe*time) , 0 ,
                    -sin(OmegaIe*time) , cos(OmegaIe*time) , 0 ,
                    0 , 0 , 1;
            return t;
        }

        inline Eigen::Matrix3d coord_earth2inertial(double time)
        {
            return coord_inertial2earth(time).transpose();
        }

        inline Eigen::Matrix3d coord_inertial2geographic(double time)
        {
            Eigen::Matrix3d t;
            double landa_=l-l0-OmegaIe*time;
            t << -sin(landa_) , cos(landa_) , 0 ,
                  -cos(landa_)*sin(L) , -sin(landa_)*sin(L) , cos(L) ,
                  cos(landa_)*cos(L) , sin(landa_)*cos(L) , sin(L) ;
            return t;
        }

        inline Eigen::Matrix3d coord_geographic2inertial(double time)
        {
            return coord_inertial2geographic(time).transpose();
        }

        inline Eigen::Matrix3d coord_earth2geographic(double time)
        {
            Eigen::Matrix3d t;
            double landa=l-l0;
            t <<  -sin(landa) , cos(landa) , 0 ,
                  -cos(landa)*sin(L) , -sin(landa)*sin(L) , cos(L) ,
                  cos(landa)*cos(L) , sin(landa)*cos(L) , sin(L) ;
            return t;
        }

        inline Eigen::Matrix3d coord_geographic2earth(double time)
        {
            return coord_geographic2earth(time).transpose();
        }


        inline Eigen::Matrix3d geographic2body()
        {
            return bodyCoord.rota.transpose()*geographicCoord.rota;
        }

        inline Eigen::Matrix3d body2geographic()
        {
            return geographicCoord.rota.transpose()*bodyCoord.rota;
        }

        virtual Eigen::Matrix3d bodyRotationUpdate(const Eigen::Vector3d& omega) =0;

        virtual Eigen::Quaterniond bodyQuaterniondUpdate(const Eigen::Vector3d& omega) =0;

        virtual Eigen::Matrix3d bodyEquivalentRotationVectorUpdate(const Eigen::Vector3d& omega) =0;

        virtual Eigen::Vector3d bodyVelocityUpdate(const Eigen::Vector3d& acc) =0;

        virtual Eigen::Vector3d bodyPositionUpdate(const Eigen::Vector3d& acc) =0;

        virtual void bodyPoseUpdate(const Eigen::Vector3d& acc , const Eigen::Vector3d& omega) =0;

        //Euler angles are calculated in three steps,useless
        inline Eigen::Vector3d getEulerAngle(const Eigen::Matrix3d& rotationBody2Geog)
        {
             Eigen::Vector3d eulerAngle;
             eulerAngle(0) = asin(rotationBody2Geog(2,1));

             if(rotationBody2Geog(2,2)>0)
                 eulerAngle(1) = atan2(-rotationBody2Geog(2,0),rotationBody2Geog(2,2));
             else if(atan2(-rotationBody2Geog(2,0),rotationBody2Geog(2,2))>0)
                 eulerAngle(1) = atan2(-rotationBody2Geog(2,0),rotationBody2Geog(2,2))>0 + PI ;
             else
                 eulerAngle(1) = atan2(-rotationBody2Geog(2,0),rotationBody2Geog(2,2))>0 - PI ;

             if(rotationBody2Geog(1,1)<0)
                 eulerAngle(2) = atan2(-rotationBody2Geog(0,1),rotationBody2Geog(1,1)) + PI ;
             else if(atan2(-rotationBody2Geog(0,1),rotationBody2Geog(1,1))>0)
                 eulerAngle(2) = atan2(-rotationBody2Geog(0,1),rotationBody2Geog(1,1));
             else
                 eulerAngle(2) = atan2(-rotationBody2Geog(0,1),rotationBody2Geog(1,1)) + PI*2 ;

             return eulerAngle;
        }

        //Euler angles are calculated in one steps theta_x,theta_y,theta_zm
        inline Eigen::Vector3d getEulerAngleDirect2(const Eigen::Matrix3d& rotationBody2Geog)
        {
            Eigen::Vector3d eulerAngle;

            eulerAngle(0) = atan2(rotationBody2Geog(2,1), rotationBody2Geog(2,2));
            eulerAngle(1) = atan2(-rotationBody2Geog(2,0),sqrt(rotationBody2Geog(2,1)*rotationBody2Geog(2,1)+rotationBody2Geog(2,2)*rotationBody2Geog(2,2)));
            eulerAngle(2) = atan2(rotationBody2Geog(1,0),rotationBody2Geog(0,0));
            return eulerAngle;
        }

        //Euler angles are calculated in one steps theta_z,theta_y,theta_x
        inline Eigen::Vector3d getEulerAngleDirect(const Eigen::Matrix3d& rotationBody2Geog)
        {
            return rotationBody2Geog.eulerAngles ( 2, 1, 0 );
        }

        inline Eigen::Vector3d getEUlerAngleNum(const Eigen::Matrix3d& rotationBody2Geog)
        {
            return  getEulerAngle(rotationBody2Geog)/PI*180;
        }

        inline Pose getbodypose() const
        {
            return bodyCoord;
        }

    protected:

        Pose inertiaCoord;     //define inerita coordinate
        Pose earthCoord;       //define earth  coordinate
        Pose geographicCoord;  //define geographic coordinate
        Pose earthCoreCoord;   //define earth core coordinate
        Pose bodyCoord;        //define bodY coordinate
        double l;              //book(page17)  longitude
        double l0;             //book(page17)
        double L;              //latitude

    };
}

#endif
