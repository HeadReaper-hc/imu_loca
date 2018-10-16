//
// Created by huchao on 10/11/18.
//

#ifndef IMU_IMUDATAREAD_H
#define IMU_IMUDATAREAD_H

#define BAUDRATE B115200

#include <string>

class imuDataRead {
public:
    struct data{
        double time;
        double acc[3];
        double gyo[3];
    };
    typedef struct data Data;

public:
    imuDataRead(const std::string& port, char* buff, imuDataRead::Data &t);
    ~imuDataRead();

private:
    std::string _port;
    int fd;
    bool init();
public:
    void readData(char* buff);
    int changeStr2Data(imuDataRead::Data &t, char *buff);

};




#endif //IMU_IMUDATAREAD_H
