
/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp> // 1.0.0

// 1.0.0




//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();
    
    tracking_state_pub_ =
    this->create_publisher<std_msgs::msg::Int32>("/orb_slam3/tracking_state", 10);

    tracking_ok_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("/orb_slam3/tracking_ok", 10);


    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    // VERSION 1.0.0
    
    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    img_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_group_;

    rclcpp::SubscriptionOptions img_opts;
    img_opts.callback_group = img_group_;
    
    auto qos_img = rclcpp::SensorDataQoS().keep_last(10);
  //  auto qos_imu = rclcpp::SensorDataQoS().keep_last(1000);
    auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(2000)).reliable();



    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
    subImuMsgName = "/mono_py_driver/imu";


    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 2);

    //* subscrbite to the image messages coming from the Python driver node
   //1.0.0 subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, qos_img, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));
    
   //1.0.0 subImu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuMsgName, qos_imu, std::bind(&MonocularMode::Imu_callback, this, _1));
   
   subImu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
         subImuMsgName, qos_imu,
         std::bind(&MonocularMode::Imu_callback, this, _1),
        imu_opts);

subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, qos_img,
        std::bind(&MonocularMode::Img_callback, this, _1),
        img_opts);

    
    //* PUBLISHER PER ROS2 POSE
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/pose", 10);


    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

MonocularMode::~MonocularMode()
{
    if (pAgent) {
        pAgent->Shutdown();
        delete pAgent;
        pAgent = nullptr;
    }
}

void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    if (slam_initialized_.exchange(true)) return;

    experimentConfig = msg.data;
    initializeVSLAM(experimentConfig);

    auto ack = std_msgs::msg::String();
    ack.data = "ACK";
    configAck_publisher_->publish(ack);

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized, ACK sent. Ready to receive images.");
}


//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    // BEFORE IT WAS THIS NOW I ADD THE LINE UNDER sensorType = ORB_SLAM3::System::MONOCULAR; 
    sensorType = ORB_SLAM3::System::IMU_MONOCULAR;

    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}


void MonocularMode::Imu_callback(const sensor_msgs::msg::Imu& msg)
{
    const double t = static_cast<double>(msg.header.stamp.sec) +
                     static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    const float wx = static_cast<float>(msg.angular_velocity.x);
    const float wy = static_cast<float>(msg.angular_velocity.y);
    const float wz = static_cast<float>(msg.angular_velocity.z);

    const float ax = static_cast<float>(msg.linear_acceleration.x);
    const float ay = static_cast<float>(msg.linear_acceleration.y);
    const float az = static_cast<float>(msg.linear_acceleration.z);

    ORB_SLAM3::IMU::Point p(ax, ay, az, wx, wy, wz, t);

    std::lock_guard<std::mutex> lock(mImuBuf_);

    // ENFORCE MONOTONIC IMU: drop out-of-order / duplicate stamps
    if (!imuBuf_.empty() && t <= imuBuf_.back().t) {
        return;
    }

    imuBuf_.push_back(p);

    // keep buffer reasonable: last 10 seconds
    while (!imuBuf_.empty() && (t - imuBuf_.front().t) > 10.0) {
        imuBuf_.pop_front();
    }
    
    static double last_t = -1.0;
    if (last_t > 0.0) {
        double dt = t - last_t;
        if (dt > 0.02) { // >20ms: a 250Hz dovrebbe essere ~0.004s
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "IMU GAP dt=%.4f s", dt);
       }
    }
    last_t = t;

}


bool MonocularMode::peekImuMeasurements(double t0, double t1,
                                        std::vector<ORB_SLAM3::IMU::Point>& out)
{
    out.clear();
    std::lock_guard<std::mutex> lock(mImuBuf_);
    if (imuBuf_.empty()) return false;

    // se il primo sample è già > t1, non hai nulla per questo frame
    if (imuBuf_.back().t < t1) return false;  // da > a < if (imuBuf_.front().t >t1) return false;
    
    if (imuBuf_.front().t > t1) return false;


    for (const auto& p : imuBuf_) {
        if (p.t < t0) continue;
        if (p.t > t1) break;
        out.push_back(p);
    }
    return !out.empty();
}

void MonocularMode::discardImuUpTo(double t)
{
    std::lock_guard<std::mutex> lock(mImuBuf_);
    while (!imuBuf_.empty() && imuBuf_.front().t <= t) {
        imuBuf_.pop_front();
    }
}


void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    if (!pAgent) {
        return;  // not initialized yet
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    
    
    const int state = pAgent->GetTrackingState();

std_msgs::msg::Int32 st_msg;
st_msg.data = state;
tracking_state_pub_->publish(st_msg);

std_msgs::msg::Bool ok_msg;
ok_msg.data = (state == 2);  // Tracking::OK == 2 nel tuo Tracking.h
tracking_ok_pub_->publish(ok_msg);

    
    
    
    

    const double t1 =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    if (lastImageTime_ >= 0.0 && t1 <= lastImageTime_) {
        RCLCPP_WARN(this->get_logger(),
            "Non-monotonic image timestamp (t1=%.9f <= last=%.9f), skipping frame",
            t1, lastImageTime_);
        return;
    }
    
    

    // --------- guard against big gaps (timing issues) ----------
// policy:
//  - dt > gap_reset : reset active map (BUT keep recent IMU data)
//  - dt > gap_skip  : don't reset, just resync time reference (avoid huge preintegration)
//  - cooldown after reset to avoid reset storms
constexpr double gap_skip  = 0.25;  // soft threshold (skip/resync)
constexpr double gap_reset = 0.80;  // hard threshold (real reset)
constexpr double imu_keep  = 2.0;   // seconds of IMU history to keep

static double last_reset_t = -1.0;

if (lastImageTime_ > 0.0) {
    const double dt = t1 - lastImageTime_;

    // cooldown: if we reset very recently, don't reset again
    if (last_reset_t > 0.0 && (t1 - last_reset_t) < 1.0) {
        if (dt > gap_skip) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "In reset cooldown (%.2fs). Large dt=%.3fs -> resync only", (t1 - last_reset_t), dt);
            lastImageTime_ = t1;
            discardImuUpTo(t1 - imu_keep);
            return;
        }
    }

    // If dt is big but not crazy: resync only (no reset)
    if (dt > gap_skip && dt <= gap_reset) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Medium image gap dt=%.3fs -> resync time, keep IMU (no reset)", dt);

        lastImageTime_ = t1;                 // avoid integrating over a huge dt
        discardImuUpTo(t1 - imu_keep);       // keep last imu_keep seconds
        return;
    }

    // Hard gap: reset active map (but DO NOT clear IMU)
    if (dt > gap_reset) {
        RCLCPP_WARN(this->get_logger(),
            "Large image gap dt=%.3fs -> ResetActiveMap (keeping last %.1fs IMU)", dt, imu_keep);

        try {
            pAgent->ResetActiveMap();
        } catch (...) {
            // avoid throwing from callback
        }

        // Keep recent IMU; do NOT clear everything
        discardImuUpTo(t1 - imu_keep);

        lastImageTime_ = -1.0;   // force "first frame" behavior after reset
        last_reset_t = t1;
        return;
    }
}

    
    

    // For first frame, use an approximate dt (30Hz -> 0.0333)
    const double approx_dt = 1.0 / 30.0;
    const double t0 = (lastImageTime_ < 0.0) ? (t1 - approx_dt) : lastImageTime_;

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    if (!peekImuMeasurements(t0, t1, vImuMeas)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "IMU not ready/covering [%.6f, %.6f] yet (buffer empty or last_imu < t1), skipping frame",
            t0, t1
        );
        return;
    }

    // With IMU at 250Hz and images at 30Hz, we expect ~8 samples per frame.
    // Require at least a few.
    if (vImuMeas.size() < 8) { // da 5 a 8
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Too few IMU samples (%zu) for [%.6f, %.6f], skipping frame",
            vImuMeas.size(), t0, t1
        );
        return;
    }

    Sophus::SE3f Tcw;
    try {
        Tcw = pAgent->TrackMonocular(cv_ptr->image, t1, vImuMeas);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "ORB-SLAM3 exception: %s", e.what());
        // IMPORTANT: do NOT discard IMU, do NOT update lastImageTime_
        return;
    }

    // SUCCESS -> commit
    lastImageTime_ = t1;
    discardImuUpTo(t1);

    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q(Twc.rotationMatrix());

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg.header.stamp;
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_publisher_->publish(pose_msg);
}
