extern "C"
{
#include "guideusbcamera.h"
#include "guidemt.h"
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
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <libudev.h>
}
#include <iostream>
#include <string>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

/* ----------------------------------- ROS ---------------------------------- */
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"   // for pixel
#include "std_msgs/msg/float32.hpp"             // for temperature
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <std_msgs/msg/string.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include "thermal_msgs/msg/ThermalAlert.hpp"



using namespace std::chrono_literals;

#define WIDTH 384
#define HEIGHT 288
const int SIZE = 384 * 288; // = 110592
const int CENTER_X = 192; 
const int CENTER_Y = 144;


/* --------------------------------- 熱像儀info -------------------------------- */
const std::string vendorId = "04b4";
const std::string productId = "f9f9";
const std::string deviceDescription = "Cypress Semiconductor Corp. GuideCamera";


int frameCallBack(guide_usb_frame_data_t *pVideoData);
int connectStatusCallBack(guide_usb_device_status_e deviceStatus);
cv::Mat convertYUV422ToBGR(const short *yuv422Data);
cv::Mat convertRGBToMat(const unsigned char *rgbData);
cv::Mat convertY16ToGray(const short *y16Data);
std::string Find_Thermal_Device();
bool comparePaths(const std::string &path1, const std::string &path2);


struct FrameData
{
    int frame_width;               // 圖像寬度
    int frame_height;              // 圖像高度
    unsigned char *frame_rgb_data; // RGB 數據
    int frame_rgb_data_length;     // RGB 數據長度
    short *frame_src_data;         // 原始數據 Y16
    int frame_src_data_length;     // 原始數據長度
    short *frame_yuv_data;         // YUV422 數據
    int frame_yuv_data_length;     // YUV422 數據長度
    short *paramLine;              // 參數行
    int paramLine_length;          // 參數行長度

    // 構造函式
    FrameData() : frame_width(0), frame_height(0),
                  frame_rgb_data(nullptr), frame_rgb_data_length(0),
                  frame_src_data(nullptr), frame_src_data_length(0),
                  frame_yuv_data(nullptr), frame_yuv_data_length(0),
                  paramLine(nullptr), paramLine_length(0) {}

    // 解構函式
    ~FrameData()
    {
        // 釋放記憶體
        if (frame_rgb_data != nullptr)
        {
            delete[] frame_rgb_data;
            frame_rgb_data = nullptr;
        }
        if (frame_src_data != nullptr)
        {
            delete[] frame_src_data;
            frame_src_data = nullptr;
        }
        if (frame_yuv_data != nullptr)
        {
            delete[] frame_yuv_data;
            frame_yuv_data = nullptr;
        }
        if (paramLine != nullptr)
        {
            delete[] paramLine;
            paramLine = nullptr;
        }
    }
};

struct ThermalOutputData
{
    unsigned char *paramline;
    float *pTemper;
    short *pGray;
    unsigned char *pRgb;

    // Constructor
    ThermalOutputData() : paramline(nullptr), pTemper(nullptr), pGray(nullptr), pRgb(nullptr) {}

    // Destructor
    ~ThermalOutputData()
    {
        if (paramline != nullptr)
        {
            delete[] paramline;
            paramline = nullptr;
        }
        if (pTemper != nullptr)
        {
            delete[] pTemper;
            pTemper = nullptr;
        }
        if (pGray != nullptr)
        {
            delete[] pGray;
            pGray = nullptr;
        }
        if (pRgb != nullptr)
        {
            delete[] pRgb;
            pRgb = nullptr;
        }
    }
};


FrameData frameData;                                  // 存放熱像儀的輸出資料
ThermalOutputData thermalOutputData;                  // 存放熱像儀計算出的數據
guide_measure_external_param_t *measureExternalParam; // 熱像儀參數設定



/* -------------------------------- ros node -------------------------------- */
class ThermalCameraNode : public rclcpp::Node {
public:
    ThermalCameraNode() : Node("thermal_camera_node") {
        // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);
    
        pixel_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/coin417rg2_thermal/hot_spot_temperature_pos", 10);    // temperature position [x, y]
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/coin417rg2_thermal/thermal_image", 10);                  // thermal rgb img
        temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>("/coin417rg2_thermal/hot_spot_temperature", 10);       // hot spot temperature
        hot_spot_pub_ = this->create_publisher<thermal_msgs::msg::ThermalAlert>("/coin417rg2_thermal/hot_spot", 10);   // x, y of pixel and temperature
        timer_ = this->create_wall_timer(1ms, std::bind(&ThermalCameraNode::publishThermalData, this));
    }

private:
    // callback funtion
    void publishThermalData() {

        if ((frameData.paramLine != NULL) && (thermalOutputData.paramline != NULL)) {


            guide_measure_convertgray2temper(1, 1, frameData.frame_src_data, thermalOutputData.paramline, SIZE, measureExternalParam, thermalOutputData.pTemper);

            max_temperature_ = thermalOutputData.pTemper[0];
            for (int i = 0; i < SIZE; i++) {
                if (thermalOutputData.pTemper[i] > max_temperature_) {
                    max_temperature_ = thermalOutputData.pTemper[i];
                    maxIndex = i;
                }
            }

            /* --------------------------------- 以左上角為原點 -------------------------------- */
            x_pixel_ = (maxIndex)%WIDTH +1;
            y_pixel_ = (maxIndex)/WIDTH +1;


            /* -------------------------------- 轉換成中心當原點 -------------------------------- */
            x_pixel_ = x_pixel_ - CENTER_X;
            y_pixel_ = y_pixel_ - CENTER_Y;


            // Publish pixel values
            auto pixel_msg = std::make_unique<std_msgs::msg::Int32MultiArray>();
            pixel_msg->data = {x_pixel_, y_pixel_};
            pixel_pub_->publish(std::move(pixel_msg));
            RCLCPP_INFO(this->get_logger(), "Published pixel values: [%d, %d]", x_pixel_, y_pixel_);


            // Publish temperature
            auto temperature_msg = std::make_unique<std_msgs::msg::Float32>();
            temperature_msg->data = max_temperature_;
            temperature_pub_->publish(std::move(temperature_msg));
            RCLCPP_INFO(this->get_logger(), "Published max temperature: %f", max_temperature_);
            
            // Publish hot spot
            auto hot_spot_msg = std::make_unique<thermal_msgs::msg::ThermalAlert>();
            hot_spot_msg->x = x_pixel_;
            hot_spot_msg->y = y_pixel_;
            hot_spot_msg->temperature = max_temperature_;
            hot_spot_pub_->publish(std::move(hot_spot_msg));
            RCLCPP_INFO(this->get_logger(), "Published hot spot: [%d, %d, %f]", x_pixel_, y_pixel_, max_temperature_);

            if (frameData.frame_yuv_data != NULL){
                cv::Mat YUV_Image = convertYUV422ToBGR(frameData.frame_yuv_data);
                image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", YUV_Image).toImageMsg();

                // Publish the image message
                image_pub_->publish(*image_msg.get());
                // RCLCPP_INFO(this->get_logger(), "Thermal image is published");
            }
        }
    }
    
    sensor_msgs::msg::Image::SharedPtr image_msg;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pixel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
    rclcpp::Publisher<thermal_msgs::msg::ThermalAlert>::SharedPtr hot_spot_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int x_pixel_ = 0;
    int y_pixel_ = 0;
    int maxIndex = 0;
    float max_temperature_ = 0.0;
};




int main(int argc, char *argv[]) {

    int maxIndex = 0;
    float maxTemperature = 0.0;

    int ret=0;
    guide_usb_setloglevel(LOG_INFO);

    std::string device_path = Find_Thermal_Device();

    if (!device_path.empty()){
        std::cout << "Found Device Path: " << device_path << std::endl;
        ret = guide_usb_initial(device_path.c_str());

        if(ret < 0)
        {   
            std::cerr << "Initial fail:" << ret << std::endl;
            return -1;
        }
    
        guide_usb_setpalette(5);


        /* ---------------------------------- 分配空間 ---------------------------------- */
        frameData.frame_src_data = (short *)malloc(WIDTH * HEIGHT * sizeof(short)); // y16dta
        frameData.frame_yuv_data = (short *)malloc(WIDTH * HEIGHT * sizeof(short)); // yuvdata
        frameData.paramLine = (short *)malloc(WIDTH);                               // 參數行

        thermalOutputData.paramline = (unsigned char *)malloc(WIDTH * 2);            // 參數行
        thermalOutputData.pTemper = (float *)malloc(sizeof(float) * WIDTH * HEIGHT); // 溫度
        thermalOutputData.pRgb = (unsigned char *)malloc(WIDTH * HEIGHT * 3);        // 偽彩

        measureExternalParam = (guide_measure_external_param_t *)malloc(sizeof(guide_measure_external_param_t));
        measureExternalParam->emiss = 98;
        // measureExternalParam->distance = 50;
        measureExternalParam->relHum = 60;
        measureExternalParam->atmosphericTemper = 230;
        measureExternalParam->reflectedTemper = 230;
        measureExternalParam->modifyK = 100;
        measureExternalParam->modifyB = 0;

        guide_usb_device_info_t* deviceInfo = (guide_usb_device_info_t*)malloc(sizeof (guide_usb_device_info_t));
        deviceInfo->width = WIDTH;
        deviceInfo->height = HEIGHT;
        deviceInfo->video_mode = Y16_PARAM_YUV;

    
        ret = guide_usb_openstream(deviceInfo, (OnFrameDataReceivedCB)frameCallBack, (OnDeviceConnectStatusCB)connectStatusCallBack);
        if(ret < 0)
        {
            std::cerr << "Open fail!" << ret << std::endl;

            delete deviceInfo;
            return ret;
        }


        rclcpp::init(argc, argv);
        auto node = std::make_shared<ThermalCameraNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();

        ret = guide_usb_closestream();
        std::cout << "close usb return" << ret << std::endl;

        ret = guide_usb_exit();
        std::cout << "exit return" << ret << std::endl; 

        delete deviceInfo;
        return ret;
    }
    return ret;
}


int connectStatusCallBack(guide_usb_device_status_e deviceStatus)
{
    if(deviceStatus == DEVICE_CONNECT_OK)
    {
        std::cout << "VideoStream is Staring..." << std::endl;
    }
    else
    {
        std::cout << "VideoStream is Closing..." << std::endl;
    }
}



int frameCallBack(guide_usb_frame_data_t *pVideoData)
{
    // check the data is exist, if exit copy data to framdata
    if (pVideoData->frame_src_data != NULL)
    {
        memcpy(frameData.frame_src_data, pVideoData->frame_src_data, pVideoData->frame_src_data_length * sizeof(short));
    }

    if (pVideoData->frame_yuv_data != NULL)
    {
        memcpy(frameData.frame_yuv_data, pVideoData->frame_yuv_data, pVideoData->frame_yuv_data_length * sizeof(short));
    }

    if (pVideoData->paramLine != NULL)
    {
        memcpy(frameData.paramLine, pVideoData->paramLine, pVideoData->paramLine_length);
        memcpy(thermalOutputData.paramline, pVideoData->paramLine, pVideoData->paramLine_length);
    }

    frameData.frame_width = pVideoData->frame_width;
    frameData.frame_height = pVideoData->frame_height;
    frameData.frame_src_data_length = pVideoData->frame_src_data_length;
    frameData.frame_yuv_data_length = pVideoData->frame_yuv_data_length;
    frameData.paramLine_length = pVideoData->paramLine_length;
}


cv::Mat convertYUV422ToBGR(const short *yuv422Data)
{
    cv::Mat yuvImage(HEIGHT, WIDTH, CV_8UC2, (void *)yuv422Data);
    cv::Mat bgrImage;
    cv::cvtColor(yuvImage, bgrImage, cv::COLOR_YUV2BGR_YUYV); // 使用OpenCV函數將YUV轉換為BGR
    return bgrImage;
}

cv::Mat convertRGBToMat(const unsigned char *rgbData)
{
    cv::Mat frameMat(HEIGHT, WIDTH, CV_8UC3, const_cast<unsigned char *>(rgbData));
    return frameMat.clone(); // 返回複製的影像以避免內存問題
}

cv::Mat convertY16ToGray(const short *y16Data)
{
    cv::Mat frameMat(HEIGHT, WIDTH, CV_16UC1, const_cast<short *>(y16Data));
    // cv::Mat grayImage(HEIGHT, WIDTH, CV_16UC1, (void*)y16Data);
    // Convert to 8-bit grayscale image
    cv::Mat gray8Bit;
    frameMat.convertTo(gray8Bit, CV_8U); // Scale to 0-255
    return gray8Bit;
}


std::string Find_Thermal_Device() {

    struct udev *udev = udev_new();
    if (!udev) {
        std::cerr << "Failed to initialize udev." << std::endl;
        return "";
    }

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    if (!enumerate) {
        std::cerr << "Failed to create udev enumerator." << std::endl;
        udev_unref(udev);
        return "";
    }

    // 添加匹配规则：使用设备的厂商 ID、产品 ID 和描述信息
    udev_enumerate_add_match_property(enumerate, "ID_VENDOR_ID", vendorId.c_str());
    udev_enumerate_add_match_property(enumerate, "ID_MODEL_ID", productId.c_str());
    udev_enumerate_add_match_property(enumerate, "ID_MODEL", deviceDescription.c_str());
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *entry;

    std::string maxVideoPath = "";

    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        struct udev_device *device = udev_device_new_from_syspath(udev, path);

        const char *deviceName = udev_device_get_sysname(device);
        if (deviceName) {
            const char *devicePath = udev_device_get_devnode(device);
            if (devicePath && std::string(deviceName).find("video") != std::string::npos) {
                std::cout << "Device Name: " << deviceName << std::endl;
                std::cout << "Device Path: " << devicePath << std::endl;
                if (maxVideoPath.empty() || comparePaths(devicePath, maxVideoPath)) {
                    maxVideoPath = devicePath;
                }
            }
        }
        udev_device_unref(device);
    }

    if (!maxVideoPath.empty()) {
        udev_enumerate_unref(enumerate);
        udev_unref(udev);
        return maxVideoPath;
    }

    std::cout << "Device: Cypress Semiconductor Corp. GuideCamera was not found :(" << std::endl;

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return "";
}


bool comparePaths(const std::string& path1, const std::string& path2) {
    size_t pos1 = path1.find_last_of("/");
    size_t pos2 = path2.find_last_of("/");
    if (pos1 == std::string::npos || pos2 == std::string::npos) {
        return false;
    }
    std::string path1Suffix = path1.substr(pos1 + 1);
    std::string path2Suffix = path2.substr(pos2 + 1);
    size_t numPos1 = path1Suffix.find("video");
    size_t numPos2 = path2Suffix.find("video");
    if (numPos1 == std::string::npos || numPos2 == std::string::npos) {
        return false;
    }
    int num1 = std::stoi(path1Suffix.substr(numPos1 + 5));
    int num2 = std::stoi(path2Suffix.substr(numPos2 + 5));
    return num1 < num2;
}