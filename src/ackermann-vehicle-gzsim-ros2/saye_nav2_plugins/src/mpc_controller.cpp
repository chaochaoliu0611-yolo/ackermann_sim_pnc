/**
 * @file    mpc_controller.cpp
 * @brief   MPC 模型预测控制器实现 (Nav2 Controller 插件)
 * @details
 *   移植自 my_pnc_core/src/mpc_controller.cpp。
 *   基于运动学自行车模型线性化, 使用 OSQP 求解带约束 QP。
 *
 *   状态: [x, y, θ]  控制: [δ] (方向盘转角)
 *   min  Σ(Δzᵀ Q Δz + R·u²)  s.t. |u| ≤ δ_max
 */
#include "saye_nav2_plugins/mpc_controller.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_util/node_utils.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace saye_nav2_plugins {

using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector3d = Eigen::Vector3d;

// ════════════════════════════════════════════════════════════════
MPCController::MPCController()  = default;

MPCController::~MPCController() {
    if (work_) { osqp_cleanup(work_); work_ = nullptr; }
    if (data_) {
        if (data_->P) { c_free(data_->P); data_->P = nullptr; }
        if (data_->A) { c_free(data_->A); data_->A = nullptr; }
        c_free(data_); data_ = nullptr;
    }
    if (settings_) { c_free(settings_); settings_ = nullptr; }
}

// ════════════════════════════════════════════════════════════════
void MPCController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent; costmap_ros_ = costmap_ros; tf_ = tf;
    auto node = node_.lock();
    logger_ = node->get_logger();

    nav2_util::declare_parameter_if_not_declared(node, name + ".wheelbase",  rclcpp::ParameterValue(0.2255));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_steer",  rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".mpc_horizon", rclcpp::ParameterValue(15));
    nav2_util::declare_parameter_if_not_declared(node, name + ".mpc_dt",      rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(node, name + ".mpc_Q_xy",    rclcpp::ParameterValue(10.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".mpc_Q_theta", rclcpp::ParameterValue(5.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".mpc_R_steer", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, name + ".v_cruise",    rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".v_max",      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, name + ".v_min",      rclcpp::ParameterValue(-0.35));

    vp_.wheelbase = node->get_parameter(name + ".wheelbase").as_double();
    vp_.max_steer = node->get_parameter(name + ".max_steer").as_double();
    horizon_  = node->get_parameter(name + ".mpc_horizon").as_int();
    dt_       = node->get_parameter(name + ".mpc_dt").as_double();
    Q_xy_     = node->get_parameter(name + ".mpc_Q_xy").as_double();
    Q_theta_  = node->get_parameter(name + ".mpc_Q_theta").as_double();
    R_steer_  = node->get_parameter(name + ".mpc_R_steer").as_double();
    v_cruise_ = node->get_parameter(name + ".v_cruise").as_double();
    v_max_    = node->get_parameter(name + ".v_max").as_double();
    v_min_    = node->get_parameter(name + ".v_min").as_double();

    initOSQP();
    RCLCPP_INFO(logger_, "MPCController configured: horizon=%d dt=%.3f L=%.3f max_steer=%.3f",
                horizon_, dt_, vp_.wheelbase, vp_.max_steer);
}

void MPCController::cleanup() {
    if (work_) { osqp_cleanup(work_); work_ = nullptr; osqp_initialized_ = false; }
}

void MPCController::setSpeedLimit(const double& sl, const bool& pct) {
    speed_limit_ = sl; speed_limit_percent_ = pct;
}

// ════════════════════════════════════════════════════════════════
// initOSQP — 初始化 OSQP 求解器
// ════════════════════════════════════════════════════════════════
void MPCController::initOSQP()
{
    n_vars_ = horizon_;   // 决策变量: u₀...u_{N-1} (N 个 steer)
    n_cons_ = horizon_;   // 约束: -δ_max ≤ u_i ≤ δ_max

    // 预分配数组
    P_x_.resize(n_vars_ * n_vars_); P_i_.resize(n_vars_ * n_vars_);
    P_p_.resize(n_vars_ + 1);
    q_.resize(n_vars_);
    l_.resize(n_vars_); u_.resize(n_vars_);
    A_x_.resize(n_vars_); A_i_.resize(n_vars_); A_p_.resize(n_vars_ + 1);

    // A = I (box 约束的恒等映射)
    for (int i = 0; i < n_vars_; ++i) {
        A_x_[i] = 1.0; A_i_[i] = i; A_p_[i] = i;
    }
    A_p_[n_vars_] = n_vars_;

    // 设置
    settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose    = false;
    settings_->max_iter   = 400;
    settings_->eps_abs    = 1e-4;
    settings_->eps_rel    = 1e-4;
    settings_->polish     = 0;
    settings_->warm_start = 0;

    // 数据
    data_ = (OSQPData*)c_malloc(sizeof(OSQPData));
    data_->n = n_vars_; data_->m = n_cons_;

    // P (CSC)
    data_->P = (csc*)c_malloc(sizeof(csc));
    data_->P->nzmax = n_vars_ * n_vars_;
    data_->P->m = n_vars_; data_->P->n = n_vars_;
    data_->P->p = P_p_.data(); data_->P->i = P_i_.data();
    data_->P->x = P_x_.data(); data_->P->nz = -1;

    // A (CSC)
    data_->A = (csc*)c_malloc(sizeof(csc));
    data_->A->nzmax = n_vars_;
    data_->A->m = n_vars_; data_->A->n = n_vars_;
    data_->A->p = A_p_.data(); data_->A->i = A_i_.data();
    data_->A->x = A_x_.data(); data_->A->nz = -1;

    data_->q = q_.data(); data_->l = l_.data(); data_->u = u_.data();

    osqp_setup(&work_, data_, settings_);
    osqp_initialized_ = true;
}

// ════════════════════════════════════════════════════════════════
// updateQP — 每步更新 QP 矩阵 (线性化 MPC)
// ════════════════════════════════════════════════════════════════
void MPCController::updateQP(double v, double cx, double cy, double cyaw)
{
    if (!osqp_initialized_) return;
    const int N = static_cast<int>(global_plan_.poses.size());
    if (N < 2) return;

    int h = std::min(horizon_, N - target_idx_ - 1);
    if (h < 1) h = 1;

    Matrix3d Q = Matrix3d::Zero();
    Q(0,0) = Q_xy_; Q(1,1) = Q_xy_; Q(2,2) = Q_theta_;

    // 提取参考段
    std::vector<double> rx(h+1), ry(h+1), rth(h+1);
    for (int k = 0; k <= h; ++k) {
        int idx = std::min(target_idx_ + k, N-1);
        rx[k]  = global_plan_.poses[idx].pose.position.x;
        ry[k]  = global_plan_.poses[idx].pose.position.y;
        rth[k] = tf2::getYaw(global_plan_.poses[idx].pose.orientation);
    }

    // 初始误差
    Vector3d z0;
    z0(0) = cx - rx[0]; z0(1) = cy - ry[0];
    z0(2) = normalizeAngle(cyaw - rth[0]);

    // 递推计算 Phi 和 G
    std::vector<Matrix3d> Phi(h+1); Phi[0].setIdentity();
    std::vector<std::vector<Vector3d>> G(h);
    for (int k = 0; k < h; ++k) G[k].resize(k+1);

    for (int k = 0; k < h; ++k) {
        double th = rth[k], si = std::sin(th), ci = std::cos(th);
        Matrix3d A = Matrix3d::Identity();
        A(0,2) = -v * si * dt_; A(1,2) = v * ci * dt_;
        Vector3d B(0.0, 0.0, v * dt_ / vp_.wheelbase);
        Phi[k+1] = A * Phi[k];

        for (int j = 0; j <= k; ++j) {
            Matrix3d prod = Matrix3d::Identity();
            for (int i = k; i > j; --i) {
                double ti = rth[i], si2 = std::sin(ti), ci2 = std::cos(ti);
                Matrix3d Ai = Matrix3d::Identity();
                Ai(0,2) = -v * si2 * dt_; Ai(1,2) = v * ci2 * dt_;
                prod = Ai * prod;
            }
            G[k][j] = prod * B;
        }
    }

    // 组装 Hessian P
    Eigen::MatrixXd Pd(h, h); Pd.setZero();
    for (int i = 0; i < h; ++i) {
        for (int j = i; j < h; ++j) {
            double val = 0.0;
            for (int k = j; k < h; ++k) val += G[k][i].dot(Q * G[k][j]);
            if (i == j) val += R_steer_;
            Pd(i,j) = val * 2.0; Pd(j,i) = val * 2.0;
        }
    }

    // 组装线性项 q
    for (int i = 0; i < h; ++i) {
        double val = 0.0;
        for (int k = i; k < h; ++k) val += G[k][i].dot(Q * (Phi[k+1] * z0));
        q_[i] = 2.0 * val;
    }

    // 稠密 P → CSC
    int nz = 0;
    for (int col = 0; col < h; ++col) {
        P_p_[col] = nz;
        for (int row = 0; row < h; ++row) {
            if (std::abs(Pd(row, col)) > 1e-15) {
                P_x_[nz] = Pd(row, col); P_i_[nz] = row; ++nz;
            }
        }
    }
    P_p_[h] = nz;

    // Box 约束
    for (int i = 0; i < h; ++i) { l_[i] = -vp_.max_steer; u_[i] = vp_.max_steer; }
    for (int i = h; i < n_vars_; ++i) { l_[i] = 0.0; u_[i] = 0.0; q_[i] = 0.0; P_p_[i+1] = P_p_[i]; }
}

// ════════════════════════════════════════════════════════════════
// solveQP — 求解 QP 并返回第一步控制量
// ════════════════════════════════════════════════════════════════
double MPCController::solveQP()
{
    if (!osqp_initialized_ || !work_) return 0.0;

    osqp_update_P(work_, data_->P->x, data_->P->i, data_->P->nz);
    osqp_update_lin_cost(work_, data_->q);
    osqp_update_bounds(work_, data_->l, data_->u);
    osqp_solve(work_);

    if (work_->info->status_val == OSQP_SOLVED ||
        work_->info->status_val == OSQP_SOLVED_INACCURATE)
        return work_->solution->x[0];
    return 0.0;
}

// ════════════════════════════════════════════════════════════════
// computeVelocityCommands — Nav2 主接口
// ════════════════════════════════════════════════════════════════
geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
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

    // 动态避障: 沿路径向前查 5 个点, 取最大 cost 决定减速
    auto* local_costmap = costmap_ros_->getCostmap();
    double costmap_safety = 1.0;
    if (local_costmap) {
        // 将 map 帧 pose 转到 odom 帧 (local costmap 坐标系)
        double ox = cx, oy = cy, oyaw = cyaw;
        try {
            geometry_msgs::msg::PoseStamped in, out;
            in.header.frame_id = pose.header.frame_id;
            in.header.stamp = pose.header.stamp;
            in.pose.position.x = cx; in.pose.position.y = cy;
            tf2::Quaternion q; q.setRPY(0,0,cyaw);
            in.pose.orientation = tf2::toMsg(q);
            tf_->transform(in, out, costmap_ros_->getGlobalFrameID(),
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

    // ── 1. 前视参考点搜索 (避免最近点横向拉扯) ──────────────
    double Ld = std::max(0.3, std::min(1.0, v_actual * 2.0));
    int ref_idx = target_idx_;
    double arc = 0.0;
    for (int i = target_idx_; i < N - 1; ++i) {
        double dx = plan.poses[i+1].pose.position.x - plan.poses[i].pose.position.x;
        double dy = plan.poses[i+1].pose.position.y - plan.poses[i].pose.position.y;
        arc += std::hypot(dx, dy);
        if (arc >= Ld) { ref_idx = i + 1; break; }
    }
    if (arc < Ld) ref_idx = N - 1;
    target_idx_ = ref_idx;

    // ── 2. 前馈转向角 (从路径曲率) ───────────────────────────
    double steer_ff = 0.0;
    if (ref_idx + 2 < N) {
        double kappa = computePathCurvature(
            plan.poses[ref_idx].pose.position.x,
            plan.poses[ref_idx].pose.position.y,
            plan.poses[ref_idx+1].pose.position.x,
            plan.poses[ref_idx+1].pose.position.y,
            plan.poses[ref_idx+2].pose.position.x,
            plan.poses[ref_idx+2].pose.position.y);
        steer_ff = std::atan2(vp_.wheelbase * kappa, 1.0);
        steer_ff = std::max(-vp_.max_steer * 0.7, std::min(vp_.max_steer * 0.7, steer_ff));
    }

    // ── 3. MPC 反馈求解 (使用实际速度线性化) ────────────────
    updateQP(v_actual, cx, cy, cyaw);
    double steer_fb = solveQP();
    double steer = steer_fb + steer_ff;
    steer = std::max(-vp_.max_steer, std::min(vp_.max_steer, steer));

    // ── 4. 一阶低通滤波转向 ──────────────────────────────────
    const double alpha = 0.35;
    steer_filtered_ = alpha * steer + (1.0 - alpha) * steer_filtered_;
    steer = steer_filtered_;

    // ── 5. 前瞻减速 ──────────────────────────────────────────
    const auto& goal_pt = plan.poses.back().pose.position;
    double dist_goal = std::hypot(goal_pt.x - cx, goal_pt.y - cy);

    // 构造临时 SmoothedPoint 数组用于 predictiveSpeed
    std::vector<SmoothedPoint> lookahead_pts;
    lookahead_pts.reserve(std::min(horizon_ + 5, N - target_idx_));
    for (int i = target_idx_; i < std::min(target_idx_ + horizon_ + 5, N); ++i) {
        const auto& p = plan.poses[i].pose;
        double k = 0.0;
        if (i > 0 && i + 1 < N) {
            k = computePathCurvature(
                plan.poses[i-1].pose.position.x, plan.poses[i-1].pose.position.y,
                p.position.x, p.position.y,
                plan.poses[i+1].pose.position.x, plan.poses[i+1].pose.position.y);
        }
        lookahead_pts.push_back({p.position.x, p.position.y, 0.0,
                                 tf2::getYaw(p.orientation), k});
    }

    double v = predictiveSpeed(lookahead_pts, 0, vp_, v_cruise_, v_max_, v_min_, horizon_);
    if (dist_goal < 1.0) v *= std::max(0.15, dist_goal / 1.0);
    v *= costmap_safety;
    if (speed_limit_percent_) v = std::min(v, speed_limit_ * v_max_);
    else                      v = std::min(v, speed_limit_);

    cmd.twist.linear.x  = v;
    bool stopped = std::abs(v) < 0.001;
    cmd.twist.angular.z = stopped ? 0.0 : v * std::tan(steer) / vp_.wheelbase;
    return cmd;
}

nav_msgs::msg::Path MPCController::transformGlobalPlan(
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
PLUGINLIB_EXPORT_CLASS(saye_nav2_plugins::MPCController, nav2_core::Controller)
