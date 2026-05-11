/**
 * @file    mpc_controller.hpp
 * @brief   MPC 模型预测控制器 (Nav2 Controller 插件, 含 OSQP)
 */
#pragma once

#include "saye_nav2_plugins/common.hpp"

#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>

#include <osqp.h>

#include <memory>
#include <string>
#include <vector>

namespace saye_nav2_plugins {

class MPCController : public nav2_core::Controller {
public:
    MPCController();
    ~MPCController() override;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void cleanup() override;
    void activate() override {}
    void deactivate() override {}

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

    void setPlan(const nav_msgs::msg::Path& path) override { global_plan_ = path; target_idx_ = 0; }
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:
    void initOSQP();
    void updateQP(double v, double cx, double cy, double cyaw);
    double solveQP();

    rclcpp::Logger logger_{rclcpp::get_logger("MPCController")};
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    VehicleParams vp_;
    int    horizon_   = 15;
    double dt_        = 0.05;
    double Q_xy_      = 10.0, Q_theta_ = 5.0, R_steer_ = 0.5;
    double v_cruise_  = 0.4, v_max_ = 0.5, v_min_ = -0.35;

    nav_msgs::msg::Path global_plan_;
    int    target_idx_ = 0;

    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);
    double speed_limit_ = 0.5;
    bool   speed_limit_percent_ = false;
    double steer_filtered_ = 0.0;

    // ── OSQP ──────────────────────────────────────────────────
    OSQPWorkspace* work_    = nullptr;
    OSQPSettings*  settings_ = nullptr;
    OSQPData*      data_    = nullptr;
    int n_vars_ = 0, n_cons_ = 0;
    bool osqp_initialized_ = false;

    std::vector<double>   P_x_, q_, l_, u_, A_x_;
    std::vector<c_int>    P_i_, P_p_, A_i_, A_p_;
};

} // namespace saye_nav2_plugins
