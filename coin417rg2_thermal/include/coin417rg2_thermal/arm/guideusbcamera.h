#ifndef GUIDEUSBCAMERA_H
#define GUIDEUSBCAMERA_H

/*************************************************************************/
typedef enum
{
    CLOSE            = 0,   //close log
    LOG_FATALEER     = 1,
    LOG_ERROR        = 3,
    LOG_WARN         = 7,
    LOG_INFO         = 15,
    LOG_TEST         = 31
}guide_usb_log_level_e;
/*************************************************************************/

typedef enum
{
    YUV = 0,
    YUV_PARAM = 1,
    Y16 = 2,
    Y16_PARAM = 3,
    Y16_YUV = 4,
    Y16_PARAM_YUV = 5,
    X16 = 6,
    X16_PARAM = 7,
    TMP = 8,
    TMP_PARAM = 9,
    TMP_YUV = 10,
    TMP_PARAM_YUV = 11
}guide_usb_video_mode_e;

typedef enum
{
    DEVICE_CONNECT_OK = 1,                 //连接正常
    DEVICE_DISCONNECT_OK = -1,             //断开连接
}guide_usb_device_status_e;

typedef struct
{
    int width;                              //图像宽度
    int height;                             //图像高度
    guide_usb_video_mode_e video_mode;      //视频模式
}guide_usb_device_info_t;

typedef struct
{
    int frame_width;                        //图像宽度
    int frame_height;                       //图像高度
    unsigned char* frame_rgb_data;          //rgb数据
    int frame_rgb_data_length;              //rgb数据长度
    short* frame_src_data;                  //原始数据，x16/y16
    int frame_src_data_length;              //原始数据长度
    short* frame_yuv_data;                  //yuv数据
    int frame_yuv_data_length;              //yuv数据长度
    short* paramLine;                       //参数行
    int paramLine_length;                   //参数行长度
}guide_usb_frame_data_t;


typedef int ( *OnDeviceConnectStatusCB)(guide_usb_device_status_e deviceStatus);
typedef int ( *OnFrameDataReceivedCB)(guide_usb_frame_data_t *pVideoData);

int guide_usb_initial(const char* deviceName);//初始化
int guide_usb_openstream(guide_usb_device_info_t* deviceInfo,OnFrameDataReceivedCB frameRecvCB,OnDeviceConnectStatusCB connectStatusCB);//连接设备
int guide_usb_setpalette(int paletteIndex);  //color  在Y16模式下,接口可以设置伪彩
int guide_usb_closestream();//断开设备
int guide_usb_exit(); //退出
int guide_usb_setloglevel(int level);

#endif // GUIDEUSBCAMERA_H
