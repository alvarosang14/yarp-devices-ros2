// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAPTIC_DEVICE_SERVER_ROS2_HPP__
#define __HAPTIC_DEVICE_SERVER_ROS2_HPP__

#include <string>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IHapticDevice.h>
#include <Ros2Spinner.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <yarp_control_msgs/srv/set_feedback_mode.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "HapticDevice_nws_ros2_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup HapricDeviceRos2
 *
 * @brief Contains HapricDeviceRos2.
 */

class HapticDevice_nws_ros2 : public yarp::dev::DeviceDriver,
                                    public yarp::dev::WrapperSingle,
                                    public yarp::os::PeriodicThread,
                                    public HapticDevice_nws_ros2_ParamsParser
{
public:
    HapticDevice_nws_ros2();
    ~HapticDevice_nws_ros2();

    // Implementation in DeviceDriverImpl.cpp
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // Implementation in IWrapperImpl.cpp
    bool attach(yarp::dev::PolyDriver * poly) override;
    bool detach() override;

    // Implementation in PeriodicThread.cpp
    bool threadInit() override;
    void threadRelease() override;
    void run() override;

private:
    bool publisherConfigureRosHandlers();
    bool subscriberConfigureRosHandlers();
    bool servicesConfigureRosHandlers();

    void _feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void setForceModeCallback(
        const std::shared_ptr<yarp_control_msgs::srv::SetFeedbackMode::Request> request,
        std::shared_ptr<yarp_control_msgs::srv::SetFeedbackMode::Response> response);

    void destroyRosHandlers();

    rclcpp::Node::SharedPtr m_node;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_stat;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr m_buttons;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr m_force;
    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr m_transform;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_feedback;

    rclcpp::Service<yarp_control_msgs::srv::SetFeedbackMode>::SharedPtr m_setForceModeService;

    Ros2Spinner* m_spinner;

    yarp::dev::IHapticDevice *iHapticDevice;

};

#endif // __HAPTIC_DEVICE_SERVER_ROS2_HPP__
