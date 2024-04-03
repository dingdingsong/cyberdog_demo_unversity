#include "cyberdog_example/cyberdog_example.hpp"

namespace cyberdog_example
{
CyberdogExample::CyberdogExample(const std::string & node_name)
:Node(node_name)
{
    // ROS2官方日志打印
    // RCLCPP_INFO(get_logger(), "CyberdogExample node init");

    // 铁蛋项目封装了ROS2的官方日志打印，使用INFO、DEBUG、ERROR等可直接打印日志
    INFO("CyberdogExample node init");

    // 初始化回调组
    this->demo_Mutually_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);  // 不可重入回调组
    this->demo_Reen_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);  // 可重入回调组
    
    // 创建服务
    this->demo_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "demo_srv",
        std::bind(&CyberdogExample::DemoSrv, this,
        std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, this->demo_Mutually_callback_group_);

    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = demo_Reen_callback_group_;
    // 创建音频播放publisher
    this->audio_play_pub_ = this->create_publisher<protocol::msg::AudioPlayExtend>(
        "speech_play_extend",
        rclcpp::SystemDefaultsQoS(), pub_options);
    
    // 创建动作id订阅者. GetCurrentMotionID是回调函数
    this->motion_status_sub_ =
        this->create_subscription<protocol::msg::MotionStatus>(
        "motion_status", 10,
        std::bind(&CyberdogExample::GetCurrentMotionID, this, std::placeholders::_1));

    // 创建请求动作执行的client
    this->motion_ressult_client_ =
        this->create_client<protocol::srv::MotionResultCmd>("motion_result_cmd");
    
    // AI相机client
    this->ai_camera_client_ = 
        this->create_client<protocol::srv::CameraService>("camera_service");
    
    this->control_ai_camera_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "control_ai_camera",
        std::bind(&CyberdogExample::ControlAiCameraSrv, this,
        std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, this->demo_Mutually_callback_group_);
    
    // AI相机订阅者
    this->ai_camera_img_sub_ = 
        this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10,
        std::bind(&CyberdogExample::AiCameraCallback, this, std::placeholders::_1));
}

CyberdogExample::~CyberdogExample()
{
    INFO("CyberdogExample node shutdown");
}

/**
 * @brief 订阅AI相机图像的回调函数
*/
void CyberdogExample::AiCameraCallback(
     const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 进入回调后，就可以按照需求使用/image的图像数据了
    INFO(
        "image:{step = %d, height = %d, width = %d, data_size = %d}",
        static_cast<int>(msg->step),
        static_cast<int>(msg->height),
        static_cast<int>(msg->width),
        static_cast<int>(msg->data.size()));
}

/**
 * @brief 获取当前动作ID(示例ROS订阅者回调函数写法)
 * @param msg 当前动作ID消息
*/
void CyberdogExample::GetCurrentMotionID(
    const protocol::msg::MotionStatus::SharedPtr msg)
{
    INFO("Get current motion id: %d", msg->motion_id);
}

/**
 * @brief 示例服务回调函数
 * @param request 请求(ROS官方服务接口写法)
 * @param respose 响应
*/
void CyberdogExample::DemoSrv(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr respose)
{
    // 播放在线语音消息
    protocol::msg::AudioPlayExtend text_msg;
    text_msg.module_name = "cyberdog_example";
    text_msg.is_online = true;
    text_msg.text = "测试测试测试";
    audio_play_pub_->publish(text_msg);

    // 请求动作的服务或指令
    std::chrono::seconds timeout(15);
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    protocol::srv::MotionResultCmd::Response rsp; 
    req->motion_id = 101;   
    req->cmd_source = 1;
    auto future_result = motion_ressult_client_->async_send_request(req);
    std::future_status status = future_result.wait_for(timeout);
    if (status == std::future_status::ready) {
      DEBUG("success to call callMotionServoCmd services.");
      INFO("success to call callMotionServoCmd services.");
    } else {
      INFO("Failed to call callMotionServoCmd services.");
      rsp.code = -1;
      return;
    }
    rsp.motion_id = future_result.get()->motion_id;
    rsp.result = future_result.get()->result;
    rsp.code = future_result.get()->code;
    if (rsp.code != 0) {
      ERROR("call motion_result_cmd service result code:%d", rsp.code);
    }
}

/**
 * @brief 请求打开/关闭AI相机的服务的回调函数。
 * @param request---服务的请求参数
 * @param response ---服务回应的参数
*/
void CyberdogExample::ControlAiCameraSrv(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    const std_srvs::srv::SetBool::Response::SharedPtr response)
{

    // 播放在线语音消息
    protocol::msg::AudioPlayExtend text_msg;
    text_msg.module_name = "cyberdog_example";
    text_msg.is_online = true;

    // 参数赋值可参考https://miroboticslab.github.io/blogs/#/cn/developer_guide
    // 开发者手册--视觉服务--AI相机的开启、关闭相机的ros指令中的参数
    int command = -1;
    std::string camera_args = "";
    if(request->data){
        // 打开AI相机
        command = 9;
        this->image_width_  = 2560;
        this->image_height_ = 1920;
        this->image_fps_ = 10;
        text_msg.text = "打开AI相机";
    }else{
        // 关闭AI相机
        command = 10;
        text_msg.text = "关闭AI相机";
    }
    audio_play_pub_->publish(text_msg);

    /**设置请求ai相机服务的参数**/
    auto request_ai_camera_client_ = 
        std::make_shared<protocol::srv::CameraService::Request>();
    request_ai_camera_client_->command = command;
    request_ai_camera_client_->width   = this->image_width_;
    request_ai_camera_client_->height  = this->image_height_;
    request_ai_camera_client_->fps     = this->image_fps_;
    request_ai_camera_client_->args    = camera_args;

    /**请求ai相机服务**/
      auto result = this->ai_camera_client_->async_send_request(request_ai_camera_client_);
      std::future_status status = result.wait_for(std::chrono::seconds(5));
      if (status != std::future_status::ready) {
        WARN("Request camera module timedout or deferred.");
        response->success = false;
        response->message = "Request camera module timedout or deferred";
      }
      auto response_ptr = result.get();
      if (response_ptr->result == CameraSrv::Response::RESULT_SUCCESS) {
        INFO(
          "Control camera module succeeded, %d, <%s>", static_cast<int>(response_ptr->result),
          response_ptr->msg.c_str());
        response->success = true;
      } else {
        WARN(
          "Control camera module failed, %d, <%s>", static_cast<int>(response_ptr->result),
          response_ptr->msg.c_str());

        response->success = false;
        response->message = "Control camera module failed";
      }
}

}  // namespace cyberdog_example
