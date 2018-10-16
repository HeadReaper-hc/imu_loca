#include <iostream>
#include "imuDataRead.h"

imuDataRead::Data t={0,{0,0,0},{0,0,0}};
int main() {
    char buff[255];
    imuDataRead imu("/dev/ttyACM0", buff, t);
    std::cout << "Hello, World!" << std::endl;
    return 0;
}


//#include <string.h>
//#include <stdio.h>
//int main(){
//    char a[30] = "string(1)";
//    char b[] = "string(2),stre";
//    printf("before strcpy() :%s\n", a);
//    printf("after strcpy() :%s\n", strcpy(a, b));
//}

