// C library
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include "sys/time.h"
#include "time.h"
#include <pthread.h>
#include <stdbool.h>
#include <fcntl.h>

// thermal camera
#include "small_thermal/guideusbcamera.h"

// ros2 
#include "rclcpp/rclcpp.hpp"


#define WIDTH 384
#define HEIGHT 288

int main(void) {

    guide_usb_setloglevel(LOG_TEST);
    int ret = guide_usb_initial("/dev/video0");
    if(ret < 0)
    {
        printf("Initial fail:%d \n",ret);
        return -1;
    }
    
    return 0;
}