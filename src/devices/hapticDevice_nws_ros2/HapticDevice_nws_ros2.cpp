// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HapticDevice_nws_ros2.h"

#include <cmath> // std::modf

#include <algorithm> // std::transform
#include <vector>
#include <kdl/frames.hpp>
#include <yarp/os/LogStream.h>

YARP_LOG_COMPONENT(HAPTICDEVICE_NWS_ROS2, "yarp.devices.HapticDevice_nws_ros2")

HapticDevice_nws_ros2::HapticDevice_nws_ros2() : yarp::os::PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}

HapticDevice_nws_ros2::~HapticDevice_nws_ros2()
{
    iHapticDevice = nullptr;
}

// -----------------------------------------------------------------------------

bool HapticDevice_nws_ros2::configureRosHandlers()
{
    const auto prefix = "/" + m_node_name;

    m_stat = m_node->create_publisher<geometry_msgs::msg::Pose>(
        prefix + "/state/pose", 10);

    m_buttons = m_node->create_publisher<std_msgs::msg::Int32MultiArray>(
        prefix + "/state/buttons", 10);

    m_force = m_node->create_publisher<geometry_msgs::msg::Wrench>(
        prefix + "/state/force_feedback", 10);

    m_transform = m_node->create_publisher<geometry_msgs::msg::Transform>(
        prefix + "/state/transform", 10);

    return true;
}

// -----------------------------------------------------------------------------

void HapticDevice_nws_ros2::destroyRosHandlers()
{
    m_stat.reset();
    m_buttons.reset();
    m_force.reset();
    m_transform.reset();
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::attach(yarp::dev::PolyDriver * poly)
{
    if (poly == nullptr)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "attach() received nullptr";
        return false;
    }

    if (!poly->isValid())
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "attach() received invalid PolyDriver";
        return false;
    }

    // yarp::dev::IHapticDevice *device;
    if (!poly->view(iHapticDevice))
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "attach() failed to obtain iHapticDevice interface";
        return false;
    }

    if (!configureRosHandlers())
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to configure ROS handlers";
        destroyRosHandlers(); // cleanup
        return false;
    }

    yarp::os::PeriodicThread::setPeriod(m_period);
    return yarp::os::PeriodicThread::start();
}

bool HapticDevice_nws_ros2::threadInit()
{
    return true;
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    destroyRosHandlers();
    iHapticDevice = nullptr;
    return true;
}

// ------------------- DeviceDriver Related ------------------------------------

bool HapticDevice_nws_ros2::open(yarp::os::Searchable & config)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    parseParams(config);

    // ROS2 initialization
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);

    m_node = std::make_shared<rclcpp::Node>(m_node_name);
    m_spinner = new Ros2Spinner(m_node);

    return m_spinner->start();
}

void HapticDevice_nws_ros2::threadRelease()
{
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

    detach();

    return ret;
}

// -----------------------------------------------------------------------------
void HapticDevice_nws_ros2::run()
{
    if (iHapticDevice != nullptr)
    {
        // Pose
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

        // Buttons
        std_msgs::msg::Int32MultiArray btn_msg;
        yarp::sig::Vector buttons;
        iHapticDevice->getButtons(buttons);
        for (size_t i = 0; i < buttons.size(); ++i) {
            btn_msg.data.push_back(static_cast<int>(buttons[i]));
        }
        m_buttons->publish(btn_msg);

        // Force Feedback
        yarp::sig::Vector force;
        iHapticDevice->getMaxFeedback(force);
        geometry_msgs::msg::Wrench force_msg;
        force_msg.force.x = force[0];
        force_msg.force.y = force[1];
        force_msg.force.z = force[2];
        m_force->publish(force_msg);

        // Transform
        yarp::sig::Matrix trans;
        iHapticDevice->getTransformation(trans);
        geometry_msgs::msg::Transform trans_msg;
        trans_msg.translation.x = trans(0, 3);
        trans_msg.translation.y = trans(1, 3);
        trans_msg.translation.z = trans(2, 3);
        // ... extraer rotación de la matriz
        m_transform->publish(trans_msg);
    }
}