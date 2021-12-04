#define FIFO_CLEAR  0x1
#define BUFF_LEN 20
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>


void main(){
    int fd,num;
    char rd_ch[BUFF_LEN];
    fd_set rfds,wfds; //file descriptions  (read and write)

    fd = open("/dev/globalmem",O_RDONLY |O_NONBLOCK); //the function return a file description.
    if(fd!= -1){
        if(ioctl(fd,FIFO_CLEAR,0)<0){ //set the mm to zero.
            printf("ioctl command failed.\n");
        }
        while(1){
            FD_ZERO(&rfds); //set rfds to zero
            FD_ZERO(&wfds); //set wfds to zero
            FD_SET(fd,&rfds); //add fd to rfds
            FD_SET(fd,&wfds); //add fd to wfds
            select(fd+1,&rfds,&wfds,NULL,NULL); //refer to select function.
            if(FD_ISSET(fd,&rfds)){
                printf("Poll monitor: can be read.\n");
            }
            if(FD_ISSET(fd,&wfds)){
                printf("Poll monitor: can be writter.\n");
            }
        }
    }else{
        printf("Device open failure.\n");
    }
}