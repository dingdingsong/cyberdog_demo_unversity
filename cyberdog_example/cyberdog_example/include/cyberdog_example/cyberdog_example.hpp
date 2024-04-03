#ifndef CYBERDOG_EXAMPLE_HPP_
#define CYBERDOG_EXAMPLE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "motion_utils/motion_utils.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/srv/camera_service.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace cyberdog_example
{
class CyberdogExample final : public rclcpp::Node
{
    using AudioMsg = protocol::msg::AudioPlayExtend;
    using AudioSrv = protocol::srv::AudioTextPlay;
    using MotionCmdSrv = protocol::srv::MotionResultCmd;
    using MotionStatusMsg = protocol::msg::MotionStatus;
    using CameraSrv = protocol::srv::CameraService;
    using CameraMsg = sensor_msgs::msg::Image;
public:
    explicit CyberdogExample(const std::string & node_name);
    ~CyberdogExample();

private:

    // 请求语音和运动服务的回调函数
    void DemoSrv(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr respose);
    
    // 控制ai相机开关服务的回调函数
    void ControlAiCameraSrv(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr esponse);
    
    // 动作id订阅者回调函数
    void GetCurrentMotionID(const MotionStatusMsg::SharedPtr msg);

    // ai相机订阅者回调函数
    void AiCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    int motion_id_;                 // 动作id
    uint16_t image_width_ {4208};   // 图像宽
    uint16_t image_height_ {3120};  // 图像高
    uint16_t image_fps_ {10};       // 图像频率

    // 定义回调组
    rclcpp::CallbackGroup::SharedPtr demo_Mutually_callback_group_;  // 不可重入回调组
    rclcpp::CallbackGroup::SharedPtr demo_Reen_callback_group_;      // 可重入回调组

    // 定义语音发布者
    rclcpp::Publisher<AudioMsg>::SharedPtr audio_play_pub_ {nullptr};

    // 定义动作id订阅者
    rclcpp::Subscription<MotionStatusMsg>::SharedPtr motion_status_sub_ {nullptr};
    // 定义请求动作执行的client
    rclcpp::Client<MotionCmdSrv>::SharedPtr motion_ressult_client_ {nullptr};

    // 服务端的服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr demo_srv_ {nullptr};

    //请求AI相机打开或关闭的client
    rclcpp::Client<CameraSrv>::SharedPtr ai_camera_client_ {nullptr};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_ai_camera_srv_ {nullptr};
    rclcpp::Subscription<CameraMsg>::SharedPtr ai_camera_img_sub_ {nullptr};  // AI相机图像

};  // class CyberdogExample
}  // namespace cyberdog_example

#endif  // CYBERDOG_EXAMPLE_HPP_