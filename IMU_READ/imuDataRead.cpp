//
// Created by huchao on 10/11/18.
//

#include "imuDataRead.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <time.h>

imuDataRead::imuDataRead(const std::string &port, char *buff, imuDataRead::Data &t)
                        :_port(port){
    if(init() == true)
    {
        printf("------init success------\n");
       
        //while(1) {
            //strcpy(last_buff, buff);
           // printf("last_buff is %s\n", last_buff);
            //readData(buff);
            //printf("buff is %s\n", buff);
            //changeStr2Data(t, buff);
        //}
    }
    else{
        printf("------init failed------\n");
    	close(fd);
		exit(0);
	}
}

imuDataRead::~imuDataRead() {}

bool imuDataRead::init() {
    fd = open(_port.c_str(), O_RDWR | O_NOCTTY );
    if (fd <0) {perror(_port.c_str()); exit(-1); }

    struct termios oldtio,newtio;

    tcgetattr(fd,&oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /*
      BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
      CRTSCTS : output hardware flow control (only used if the cable has
                all necessary lines. See sect. 7 of Serial-HOWTO)
      CS8     : 8n1 (8bit,no parity,1 stopbit)
      CLOCAL  : local connection, no modem contol
      CREAD   : enable receiving characters
    */
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    /*
      IGNPAR  : ignore bytes with parity errors
      ICRNL   : map CR to NL (otherwise a CR input on the other computer
                will not terminate input)
      otherwise make device raw (no other input processing)
    */
    newtio.c_iflag = IGNPAR | ICRNL;

    /*
     Raw output.
    */
    newtio.c_oflag = 0;

    /*
      ICANON  : enable canonical input
      disable all echo functionality, and don't send signals to calling program
    */
    newtio.c_lflag = ICANON;

    /*
      initialize all control characters
      default values can be found in /usr/include/termios.h, and are given
      in the comments, but we don't need them here
    */
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    /*
      now clean the modem line and activate the settings for the port
    */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    return true;
}

void imuDataRead::readData(char *buff) {
    int res = read(fd,buff,255);
//    if(buff[0] == '\n')
//        printf("buff[0]:%c\n",buff[0]);
    buff[res]='\0';             /* set end of string, so we can printf */
   // printf("buff is %s\n", buff);
}

int imuDataRead::changeStr2Data(imuDataRead::Data &t, char *buff){

    if( buff[0] == '\n')
       readData(buff);                        //represent failed

    const char * split = ",";
    char *p;
    int i=0;
    std::stringstream ss;
    p = strtok(buff,split);

    while(p!=NULL){
       // printf("p : %s\n", p);
        ss.clear();
        ss << p;
        if(i == 0)
        {
            ss >> t.time;
            //printf("time is :%f",time);
        }
        else if( i <= 3)
        {
            ss >> t.acc[i-1];
            //printf("acc is :%f",t.acc[i-1]);
        }
        else {
            ss >> t.gyo[i - 4];
            //printf("gyo is :%f",t.gyo[i-4]);
        }


        i = i+1;

        p = strtok(NULL,split);

    }
    printf("time: %f, acc: %f, %f, %f, gyo: %f, %f, %f;\n", t.time, t.acc[0], t.acc[1],
           t.acc[2], t.gyo[0], t.gyo[1], t.gyo[2]);

    return 0;
}
