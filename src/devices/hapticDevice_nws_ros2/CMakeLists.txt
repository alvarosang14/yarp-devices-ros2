# ------------------ TODO: CAMBIAR -------------
# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(hapticDevice_nws_ros2
  CATEGORY device
  TYPE HapticDevice_nws_ros2
  INCLUDE HapticDevice_nws_ros2.h
  DEFAULT ON
)

if(NOT SKIP_hapticDevice_nws_ros2)
  yarp_add_plugin(yarp_hapticDevice_nws_ros2)
  generateDeviceParamsParser(HapticDevice_nws_ros2 hapticDevice_nws_ros2)

  target_sources(yarp_hapticDevice_nws_ros2
    PRIVATE
      HapticDevice_nws_ros2.cpp
      HapticDevice_nws_ros2.h
      HapticDevice_nws_ros2_ParamsParser.h
      HapticDevice_nws_ros2_ParamsParser.cpp
  )
  target_sources(yarp_hapticDevice_nws_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)

  target_include_directories(yarp_hapticDevice_nws_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)
  find_package(orocos_kdl REQUIRED)
  target_link_libraries(yarp_hapticDevice_nws_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
      orocos-kdl
      Ros2Utils
      yarp_control_msgs::yarp_control_msgs__rosidl_typesupport_cpp
  )
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
    YARP_os
    YARP_sig
    YARP_dev
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    Ros2Utils
  )

  yarp_install(
    TARGETS yarp_hapticDevice_nws_ros2
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_hapticDevice_nws_ros2 PROPERTY FOLDER "Plugins/Device/NWS")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()
endif()
