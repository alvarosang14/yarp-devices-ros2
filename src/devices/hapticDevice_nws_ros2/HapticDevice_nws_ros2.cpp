// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HapticDevice_nws_ros2.h"

#include <cmath> // std::modf

#include <algorithm> // std::transform
#include <vector>
#include <kdl/frames.hpp>
#include <yarp/os/LogStream.h>

YARP_LOG_COMPONENT(HAPTICDEVICE_NWS_ROS2, "yarp.devices.HapticDevice_nws_ros2")

HapticDevice_nws_ros2::HapticDevice_nws_ros2() : yarp::os::PeriodicThread(m_period)
{
}

HapticDevice_nws_ros2::~HapticDevice_nws_ros2()
{
    iHapticDevice = nullptr;
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::publisherConfigureRosHandlers()
{
    const auto prefix = "/" + m_topic_name;

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

bool HapticDevice_nws_ros2::subscriberConfigureRosHandlers()
{
    const auto prefix = "/" + m_topic_name;

    m_feedback = m_node->create_subscription<sensor_msgs::msg::JointState>(
        prefix + "/feedback", 10,
        std::bind(&HapticDevice_nws_ros2::_feedbackCallback, this, std::placeholders::_1));

    return true;
}

bool HapticDevice_nws_ros2::servicesConfigureRosHandlers()
{
    const auto prefix = "/" + m_topic_name;

    m_setForceModeService = m_node->create_service<std_srvs::srv::SetBool>(
        prefix + "/set_force_mode",
        std::bind(&HapticDevice_nws_ros2::setForceModeCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    return true;
}

// -----------------------------------------------------------------------------

void HapticDevice_nws_ros2::destroyRosHandlers()
{
    // Publishers
    m_stat.reset();
    m_buttons.reset();
    m_force.reset();
    m_transform.reset();

    // Subscribers
    m_feedback.reset();

    // Services
    m_setForceModeService.reset();
}

// -----------------------------------------------------------------------------
void HapticDevice_nws_ros2::_feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (iHapticDevice != nullptr)
    {
        yarp::sig::Vector force(3, 0.0);
        if (msg->effort.size() >= 3)
        {
            force[0] = msg->effort[0];;
            force[1] = msg->effort[1];;
            force[2] = msg->effort[2];;
        }
        iHapticDevice->setFeedback(force);
    }
    else
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available in feedback callback";
    }
}

void HapticDevice_nws_ros2::setForceModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!iHapticDevice)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available";
        response->success = false;
        response->message = "IHapticDevice interface not available";
        return;
    }

    bool result = request->data ?
                  iHapticDevice->setCartesianForceMode() :
                  iHapticDevice->setJointTorqueMode();

    response->message = request->data ?
                          "Cartesian Force mode enabled" :
                          "Joint Torque mode enabled";

    if (result)
    {
        yCInfo(HAPTICDEVICE_NWS_ROS2) << response->message;
        response->success = true;
    }
    else
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to set haptic mode";
        response->success = false;
        response->message = "Failed to set haptic mode";
    }
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

    if (!publisherConfigureRosHandlers() || !subscriberConfigureRosHandlers() || !servicesConfigureRosHandlers())
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