// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HapticDevice_nws_ros2.h"

#include <cmath> // std::modf

#include <algorithm> // std::transform
#include <vector>
#include <kdl/frames.hpp>
#include <yarp/os/LogStream.h>

#define HAPTICDEVICE_WRAPPER_DEFAULT_PERIOD     0.02          // [s]

namespace {
    YARP_LOG_COMPONENT(HapticDevice_nws_ros2ParamsCOMPONENT, "yarp.device.HapticDevice_nws_ros2")
}

// -----------------------------------------------------------------------------

bool HapticDevice_nws_ros2::configureRosHandlers()
{
    const auto prefix = "/" + m_name; // In nws: "map2D_nws_ros2"
    m_stat = m_node->create_publisher<geometry_msgs::msg::Pose>(prefix + "/state/pose", 10);

    return true;
}

// -----------------------------------------------------------------------------

void HapticDevice_nws_ros2::destroyRosHandlers()
{
    m_stat.reset();
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::attach(yarp::dev::PolyDriver * poly)
{
    if (poly == nullptr)
    {
        yCError(HapticDevice_nws_ros2ParamsCOMPONENT) << "attach() received nullptr";
        return false;
    }

    if (!poly->isValid())
    {
        yCError(HapticDevice_nws_ros2ParamsCOMPONENT) << "attach() received invalid PolyDriver";
        return false;
    }

    // yarp::dev::IHapticDevice *device;
    if (!poly->view(iHapticDevice))
    {
        yCError(HapticDevice_nws_ros2ParamsCOMPONENT) << "attach() failed to obtain iHapticDevice interface";
        return false;
    }

    if (!configureRosHandlers())
    {
        yCError(HapticDevice_nws_ros2ParamsCOMPONENT) << "Failed to configure ROS handlers";
        destroyRosHandlers(); // cleanup
        return false;
    }

    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::detach()
{
    yarp::os::PeriodicThread::stop();
    destroyRosHandlers();
    iHapticDevice = nullptr;
    return true;
}

// ------------------- DeviceDriver Related ------------------------------------

bool HapticDevice_nws_ros2::open(yarp::os::Searchable & config)
{
    yarp::os::PeriodicThread::setPeriod(HAPTICDEVICE_WRAPPER_DEFAULT_PERIOD);

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // ROS2 initialization
    m_node = std::make_shared<rclcpp::Node>(m_name);
    m_spinner = new Ros2Spinner(m_node);

    return m_spinner->start();
}

// -----------------------------------------------------------------------------

bool HapticDevice_nws_ros2::close()
{
    bool ret = true;

    if (m_spinner)
    {
        ret = m_spinner->stop();
        delete m_spinner;
        m_spinner = nullptr;
    }

    return ret;
}

// -----------------------------------------------------------------------------
void HapticDevice_nws_ros2::run()
{
    if (iHapticDevice != nullptr)
    {
        yarp::sig::Vector pos, rpy;
        iHapticDevice->getPosition(pos);
        iHapticDevice->getOrientation(rpy);

        geometry_msgs::msg::Pose msg;
        msg.position.x = pos[0];
        msg.position.y = pos[1];
        msg.position.z = pos[2];

        const auto ori = KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]);
        ori.GetQuaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

        m_stat->publish(msg);
    }
}