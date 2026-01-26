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
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "HapticDevice_nws_ros2_ParamsParser.h"

#define DEFAULT_THREAD_PERIOD 0.02 //s

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
    bool configureRosHandlers();
    void destroyRosHandlers();

    std::string m_name = {"haptic_device_ros2"};

    rclcpp::Node::SharedPtr m_node;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_stat;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr m_buttons;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr m_force;
    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr m_transform;

    Ros2Spinner* m_spinner;

    yarp::dev::IHapticDevice *iHapticDevice;

};

#endif // __HAPTIC_DEVICE_SERVER_ROS2_HPP__
