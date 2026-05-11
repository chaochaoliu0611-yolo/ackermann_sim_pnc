/**
 * @file    pure_pursuit_controller.cpp
 * @brief   Pure Pursuit 控制器实现 (Nav2 Controller 插件)
 * @details
 *   移植自 my_pnc_core/src/controller.cpp PurePursuitController。
 *   公式: δ = arctan(2L·sin(α) / Ld)
 */
#include "saye_nav2_plugins/pure_pursuit_controller.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_util/node_utils.hpp>
#include <cmath>

namespace saye_nav2_plugins {

void PurePursuitController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent; costmap_ros_ = costmap_ros; tf_ = tf;
    auto node = node_.lock();
    logger_ = node->get_logger();

    nav2_util::declare_parameter_if_not_declared(node, name + ".wheelbase", rclcpp::ParameterValue(0.2255));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_steer", rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".lookahead_min", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, name + ".lookahead_max", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".lookahead_time", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(node, name + ".v_cruise", rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".v_max",   rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, name + ".v_min",   rclcpp::ParameterValue(-0.35));

    vp_.wheelbase = node->get_parameter(name + ".wheelbase").as_double();
    vp_.max_steer = node->get_parameter(name + ".max_steer").as_double();
    lookahead_min_  = node->get_parameter(name + ".lookahead_min").as_double();
    lookahead_max_  = node->get_parameter(name + ".lookahead_max").as_double();
    lookahead_time_ = node->get_parameter(name + ".lookahead_time").as_double();
    v_cruise_ = node->get_parameter(name + ".v_cruise").as_double();
    v_max_    = node->get_parameter(name + ".v_max").as_double();
    v_min_    = node->get_parameter(name + ".v_min").as_double();

    RCLCPP_INFO(logger_, "PurePursuitController configured: L=%.3f ld=[%.2f-%.2f] t=%.2f",
                vp_.wheelbase, lookahead_min_, lookahead_max_, lookahead_time_);
}

void PurePursuitController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
    speed_limit_ = speed_limit;
    speed_limit_percent_ = percentage;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
{
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;

    auto plan = transformGlobalPlan(pose);
    const int N = static_cast<int>(plan.poses.size());
    if (N < 2) return cmd;

    // ── 0. 目标到达检测 ──────────────────────────────────────
    if (goal_checker) {
        const auto& goal = plan.poses.back();
        if (goal_checker->isGoalReached(pose.pose, goal.pose, velocity))
            return cmd;
    }

    double cx = pose.pose.position.x, cy = pose.pose.position.y;
    double cyaw = tf2::getYaw(pose.pose.orientation);
    double v_actual = std::hypot(velocity.linear.x, velocity.linear.y);
    v_actual = std::max(0.05, v_actual);

    double costmap_safety = 1.0;
    auto* local_costmap = costmap_ros_->getCostmap();
    if (local_costmap) {
        double ox = cx, oy = cy, oyaw = cyaw;
        try {
            geometry_msgs::msg::PoseStamped in, out;
            in.header.frame_id = pose.header.frame_id;
            in.header.stamp = pose.header.stamp;
            in.pose.position.x = cx; in.pose.position.y = cy;
            tf2::Quaternion q; q.setRPY(0,0,cyaw);
            in.pose.orientation = tf2::toMsg(q);
            out = tf_->transform(in, costmap_ros_->getGlobalFrameID(),
                                 tf2::durationFromSec(0.1));
            ox = out.pose.position.x; oy = out.pose.position.y;
            oyaw = tf2::getYaw(out.pose.orientation);
        } catch (...) {}

        double max_cc = 0.0;
        const double offsets[3] = {-0.2618, 0.0, 0.2618};
        for (int r = 0; r < 3; ++r) {
            double dir = oyaw + offsets[r];
            for (int i = 0; i < 12; ++i) {
                double d = i * 0.2;
                double px = ox + d * std::cos(dir);
                double py = oy + d * std::sin(dir);
                unsigned int mx, my;
                if (local_costmap->worldToMap(px, py, mx, my))
                    max_cc = std::max(max_cc, static_cast<double>(local_costmap->getCost(mx, my)));
            }
        }
        if (max_cc >= 254.0)             costmap_safety = 0.0;
        else                             costmap_safety = 1.0 - max_cc / 254.0;
        costmap_safety = std::max(0.05, costmap_safety);
    }

    // ── 1. 速度自适应前视距离 ──────────────────────────────
    double Ld = lookahead_time_ * v_actual;
    Ld = std::max(lookahead_min_, std::min(lookahead_max_, Ld));

    // ── 2. 沿轨迹找前视点 (弧长法, 避免最近点弯道拉扯) ────
    int tgt_idx = target_idx_;
    double arc = 0.0;
    bool found = false;
    for (int i = target_idx_; i < N - 1; ++i) {
        double dx = plan.poses[i+1].pose.position.x - plan.poses[i].pose.position.x;
        double dy = plan.poses[i+1].pose.position.y - plan.poses[i].pose.position.y;
        arc += std::hypot(dx, dy);
        if (arc >= Ld) { tgt_idx = i + 1; found = true; break; }
    }
    if (!found) tgt_idx = N - 1;
    target_idx_ = tgt_idx;

    // ── 3. 目标点变换到车辆坐标系 ────────────────────────────
    const auto& tgt = plan.poses[tgt_idx].pose.position;
    double dx = tgt.x - cx, dy = tgt.y - cy;
    double lx =  dx * std::cos(cyaw) + dy * std::sin(cyaw);
    double ly = -dx * std::sin(cyaw) + dy * std::cos(cyaw);

    // ── 4. 纯追踪公式 + 前馈转向角 ──────────────────────────
    double alpha = std::atan2(ly, lx);
    double steer_pp = std::atan2(2.0 * vp_.wheelbase * std::sin(alpha), Ld);

    // 前馈: 参考路径曲率
    double steer_ff = 0.0;
    if (tgt_idx + 2 < N) {
        double kappa = computePathCurvature(
            plan.poses[tgt_idx].pose.position.x,
            plan.poses[tgt_idx].pose.position.y,
            plan.poses[tgt_idx+1].pose.position.x,
            plan.poses[tgt_idx+1].pose.position.y,
            plan.poses[tgt_idx+2].pose.position.x,
            plan.poses[tgt_idx+2].pose.position.y);
        steer_ff = std::atan2(vp_.wheelbase * kappa, 1.0);
    }
    double steer = steer_pp * 0.7 + steer_ff * 0.3;
    steer = std::max(-vp_.max_steer, std::min(vp_.max_steer, steer));

    // ── 5. 低通滤波 ──────────────────────────────────────────
    const double alpha_f = 0.4;
    steer_filtered_ = alpha_f * steer + (1.0 - alpha_f) * steer_filtered_;
    steer = steer_filtered_;

    // ── 6. 前瞻减速 + 终点衰减 ───────────────────────────────
    const auto& goal_pt = plan.poses.back().pose.position;
    double dist_goal = std::hypot(goal_pt.x - cx, goal_pt.y - cy);

    std::vector<SmoothedPoint> pts;
    pts.reserve(std::min(15, N - tgt_idx));
    for (int i = tgt_idx; i < std::min(tgt_idx + 15, N); ++i) {
        const auto& p = plan.poses[i].pose;
        double k = 0.0;
        if (i > 0 && i + 1 < N) {
            k = computePathCurvature(
                plan.poses[i-1].pose.position.x, plan.poses[i-1].pose.position.y,
                p.position.x, p.position.y,
                plan.poses[i+1].pose.position.x, plan.poses[i+1].pose.position.y);
        }
        pts.push_back({p.position.x, p.position.y, 0.0, tf2::getYaw(p.orientation), k});
    }
    double v = predictiveSpeed(pts, 0, vp_, v_cruise_, v_max_, v_min_, 15);
    if (dist_goal < 1.0) v *= std::max(0.15, dist_goal / 1.0);
    v *= costmap_safety;
    if (speed_limit_percent_) v = std::min(v, speed_limit_ * v_max_);
    else                      v = std::min(v, speed_limit_);

    cmd.twist.linear.x  = v;
    bool stopped = std::abs(v) < 0.001;
    cmd.twist.angular.z = stopped ? 0.0 : v * std::tan(steer) / vp_.wheelbase;
    return cmd;
}

nav_msgs::msg::Path PurePursuitController::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped& pose)
{
    if (global_plan_.poses.empty()) return global_plan_;
    if (global_plan_.header.frame_id == pose.header.frame_id) return global_plan_;

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_->lookupTransform(
            pose.header.frame_id,
            global_plan_.header.frame_id,
            tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(logger_, "Plan transform failed: %s", e.what());
        return global_plan_;
    }

    nav_msgs::msg::Path out;
    out.header.frame_id = pose.header.frame_id;
    out.header.stamp = pose.header.stamp;

    tf2::Transform t;
    tf2::fromMsg(tf_stamped.transform, t);

    for (const auto& p : global_plan_.poses) {
        geometry_msgs::msg::PoseStamped out_pose;
        out_pose.header = out.header;

        tf2::Vector3 v(p.pose.position.x, p.pose.position.y, 0.0);
        v = t * v;
        out_pose.pose.position.x = v.x();
        out_pose.pose.position.y = v.y();

        tf2::Quaternion q;
        tf2::fromMsg(p.pose.orientation, q);
        q = t.getRotation() * q;
        q.normalize();
        out_pose.pose.orientation = tf2::toMsg(q);

        out.poses.push_back(out_pose);
    }
    return out;
}

} // namespace saye_nav2_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(saye_nav2_plugins::PurePursuitController, nav2_core::Controller)
