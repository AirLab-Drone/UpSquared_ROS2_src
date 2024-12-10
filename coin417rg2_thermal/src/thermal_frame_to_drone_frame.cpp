#include <rclcpp/rclcpp.hpp>
#include <thermal_msgs/msg/thermal_alert.hpp>
#include "sensor_msgs/msg/range.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std;
class ThermalFrameToDroneFrame : public rclcpp::Node
{
public:
    ThermalFrameToDroneFrame() : Node("thermal_frame_to_drone_frame")
    {
        this->create_subscription<thermal_msgs::msg::ThermalAlert>(
            "/coin417rg2_thermal/hot_spot", 10, std::bind(&ThermalFrameToDroneFrame::hotSpotPoseCallback, this, std::placeholders::_1));
        this->create_subscription<sensor_msgs::msg::Range>(
            "/mavros/rangefinder_pub", 10, std::bind(&ThermalFrameToDroneFrame::rangefinderCallback, this, std::placeholders::_1));
        hot_spot_drone_frame_pub_ = this->create_publisher<thermal_msgs::msg::ThermalAlert>("/coin417rg2_thermal/hot_spot_drone_frame", 10);
        meter_per_pixel = compute_meter_per_pixel();
    }

private:
    int x_pixel = 0;
    int y_pixel = 0;
    float temperature = 0.0;
    float rangefinder_alt = 0.0;
    float meter_per_pixel = 0.0;
    rclcpp::Publisher<thermal_msgs::msg::ThermalAlert>::SharedPtr hot_spot_drone_frame_pub_;
    void hotSpotPoseCallback(const thermal_msgs::msg::ThermalAlert msg)
    {
        x_pixel = msg.x;
        y_pixel = msg.y;
        temperature = msg.temperature;
        thermal_msgs::msg::ThermalAlert hot_spot_drone_frame;
        hot_spot_drone_frame.x = x_pixel * meter_per_pixel;
        hot_spot_drone_frame.y = y_pixel * meter_per_pixel;
        hot_spot_drone_frame.temperature = temperature;
        cout << hot_spot_drone_frame.x << " " << hot_spot_drone_frame.y << " " << hot_spot_drone_frame.temperature << endl;
        hot_spot_drone_frame_pub_->publish(hot_spot_drone_frame);
    }
    void rangefinderCallback(const sensor_msgs::msg::Range msg)
    {
        rangefinder_alt = msg.range;
    }
    YAML::Node get_yaml_config(const std::string &package_name, const std::string &config_file_name)
    {
        // 獲取包的共享目錄
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        std::string config_file_path = package_share_directory + "/config/" + config_file_name;

        // 檢查文件是否存在
        std::ifstream file(config_file_path);
        if (!file.good())
        {
            std::cerr << "Cannot load Aruco markers config file: " << config_file_path << std::endl;
            return YAML::Node();
        }

        // 加載配置文件
        YAML::Node config = YAML::LoadFile(config_file_path);
        return config;
        // try
        // {
        //     YAML::Node config = YAML::LoadFile(config_file_path);
        //     return config;
        // }
        // catch (const YAML::Exception &e)
        // {
        //     std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        //     return YAML::Node();
        // }
    }
    float compute_meter_per_pixel()
    {
        YAML::Node config = get_yaml_config("coin417rg2_thermal", "thermal_camera.yaml");

        float fov = config["thermal_camera_params"]["fov"].as<float>();
        float image_width = config["thermal_camera_params"]["width"].as<float>();
        return 2 * rangefinder_alt * tan(fov / 2) / image_width;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermalFrameToDroneFrame>());
    rclcpp::shutdown();
    return 0;
}