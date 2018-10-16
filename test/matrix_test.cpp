
#include <Eigen/Dense>
#include "imu.h"
#include "imu_math.h"
#include "imu_abstract.h"
#include <iostream>
using namespace imu_loca;
using namespace std;

int main(int argc , char** argv)
{
	Pose a;
    a.trans = Eigen::Vector3d(0,0,0);
    a.velocity = Eigen::Vector3d(0,0,0);
	a.rota = Eigen::Matrix3d::Identity();

   // Eigen::Matrix3d b;
	Eigen::Vector3d omega(0,0,1);
	imuAbstract* p = new imu(a);
  // cout<<p->getEulerAngle(b)<<endl;
  // cout<<p->getEulerAngleDirect(b)<<endl;
  // cout<<p->getEulerAngleDirect2(b)<<endl;

	cout<<p->bodyRotationUpdate(omega)<<endl;
    cout<<p->bodyQuaterniondUpdate(omega).coeffs()<<endl;
    cout<<p->bodyEquivalentRotationVectorUpdate(omega)<<endl;

	while(1);

}
	
