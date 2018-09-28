
#include <Eigen/Dense>
#include "imu.h"
#include "imu_math.h"
#include "imu_abstract.h"
#include <iostream>
using namespace imu_loca;
using namespace std;

int main(int argc , char** argv)
{
	Pose inertia;
	imuAbstract* p = new imu(inertia);
	
	Eigen::Vector3d omega = Eigen::Vector3d(0,0,0.05);
	Eigen::Vector3d acc = Eigen::Vector3d(0.1,0,9.8);
	for(int i=0 ; i<11 ; i++)
		p->bodyPoseUpdate(acc,omega);
	cout<<p->getbodypose().velocity<<endl;
	cout<<p->getbodypose().rota<<endl;
	cout<<p->getbodypose().trans<<endl;

}
	
