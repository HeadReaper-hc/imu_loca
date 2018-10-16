#include "imuDataRead.h"
#include <iostream>
#include "imu.h"
#include "imu_abstract.h"
#include <Eigen/Dense>
#include <semaphore.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>

using namespace imu_loca;
using namespace std;

imuDataRead::Data t={0,{0,0,0},{0,0,0}};
char buff[255];
sem_t binSem;

//imu data read thread func
void* imuReadFunc(void* arg)
{
	imuDataRead imu("/dev/ttyACM0", buff, t);
	while(1)
	{
		sem_wait(&binSem);
		imu.readData(buff);
		imu.changeStr2Data(t, buff);
		sem_post(&binSem);
		usleep(1000);
	}
	pthread_exit((void*)1);
}


//robot pose update thread func
void* imuUpdate(void* arg)
{
	
	imuAbstract* p = new imu();
	//Eigen::Vector3d acc,omega;
	Eigen::Map<Eigen::MatrixXd>  acc(t.acc,3,1);
	Eigen::Map<Eigen::MatrixXd>  omega(t.gyo,3,1);
	while(1)
	{
		sem_wait(&binSem);
		cout<<"acc:"<<acc<<endl;
		cout<<"gyo:"<<omega<<endl;     
		p->bodyPoseUpdate(acc*10,omega/180*M_PI);
		cout<<p->getbodypose().velocity<<endl;
		cout<<p->getbodypose().rota<<endl;
		cout<<p->getbodypose().trans<<endl;
		sem_post(&binSem);
		usleep(1000);
	}
	delete p;
	pthread_exit((void*)2);
}
	

int main(int argc, char* argv[])
{
	int res = 0;
	int res1 =0;
	int res2 =0;
	pthread_t imuRead;
	pthread_t imuUpd;

     // Initialize semaphore
     res = sem_init(&binSem, 0, 1);
    if (res) {
         printf("Semaphore initialization failed!!\n");
         exit(EXIT_FAILURE);
     }

	 res1= pthread_create( &imuRead, NULL,imuReadFunc, NULL);
	 res2= pthread_create( &imuUpd, NULL, imuUpdate, NULL);

	 if( res1 || res2)
	 {
		printf("create pthread failed\n");
	    return 0;
	 }
	 pthread_exit(0);
}


