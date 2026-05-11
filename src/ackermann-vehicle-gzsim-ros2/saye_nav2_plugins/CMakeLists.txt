cmake_minimum_required(VERSION 3.14)
project(saye_nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ── 依赖 ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

# ── OSQP 预编译库 (复用 my_pnc_core 中已构建的) ──────────────────
set(OSQP_LIB_DIR     "$ENV{HOME}/my_project/my_pnc_core/build/_deps/osqp-build/out")
set(OSQP_INCLUDE_DIR "$ENV{HOME}/my_project/my_pnc_core/build/_deps/osqp-src/include")

if(NOT EXISTS "${OSQP_LIB_DIR}/libosqp.a")
    message(FATAL_ERROR "OSQP lib not found at ${OSQP_LIB_DIR}/libosqp.a. "
            "Build my_pnc_core first to download OSQP.")
endif()

# ── 共享库: 包含所有 4 个插件 ────────────────────────────────────
add_library(saye_nav2_plugins SHARED
    src/hybrid_astar_planner.cpp
    src/mpc_controller.cpp
    src/pure_pursuit_controller.cpp
    src/stanley_controller.cpp
)

target_include_directories(saye_nav2_plugins PRIVATE
    include
    ${OSQP_INCLUDE_DIR}
    /opt/ros/humble/include/ompl-1.7
)

target_link_libraries(saye_nav2_plugins
    ${OSQP_LIB_DIR}/libosqp.a
    Eigen3::Eigen
    ompl::ompl
)

ament_target_dependencies(saye_nav2_plugins
    nav2_core nav2_costmap_2d nav2_util pluginlib rclcpp rclcpp_lifecycle
    tf2_ros tf2_geometry_msgs nav_msgs geometry_msgs
)

# ── 安装 ─────────────────────────────────────────────────────────
install(TARGETS saye_nav2_plugins
    ARCHIVE DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin
)

install(PROGRAMS scripts/odom_to_tf.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

install(FILES plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

# 让 pluginlib 能找到我们的插件
ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(saye_nav2_plugins)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()
