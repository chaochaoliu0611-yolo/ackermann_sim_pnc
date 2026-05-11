/**
 * @file    hybrid_astar_planner.hpp
 * @brief   Hybrid A* 全局规划器 (Nav2 GlobalPlanner 插件)
 * @details
 *   移植自 my_pnc_core, 适配导航2代价图接口。
 *   使用自行车模型运动原语和五次多项式轨迹平滑。
 */
#pragma once

#include "saye_nav2_plugins/common.hpp"

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <queue>
#include <vector>

namespace saye_nav2_plugins {

class HybridAStarPlanner : public nav2_core::GlobalPlanner {
public:
    HybridAStarPlanner();
    ~HybridAStarPlanner() override;

    // ── Nav2 GlobalPlanner 接口 ────────────────────────────────
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   std::string name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;
    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal) override;

private:
    // ── 运动原语生成 ──────────────────────────────────────────
    void generatePrimitives();

    // ── 运动学扩展 ────────────────────────────────────────────
    SearchState kinematicExpand(const SearchState& node,
                                 const MotionPrimitive& prim,
                                 const nav2_costmap_2d::Costmap2D* costmap) const;

    // ── Dijkstra 障碍物感知启发式 ──────────────────────────
    void computeObstacleHeuristic(const nav2_costmap_2d::Costmap2D* costmap,
                                  double gx, double gy);
    double obstacleHeuristic(int gx, int gy) const;
    int    heuristic_w_ = 0;
    int    heuristic_h_ = 0;
    std::vector<double> heuristic_lookup_;

    // ── 离散化编码 (3D → 64-bit key) ─────────────────────────
    int64_t discreteKey(int gx, int gy, int gtheta) const;
    int thetaToBin(double theta) const;

    // ── 路径提取 ──────────────────────────────────────────────
    std::vector<SearchState> extractPath(const std::vector<SearchState>& nodes,
                                          int goal_idx) const;

    // ── Reeds-Shepp 解析扩展 ────────────────────────────────
    bool tryReedsSheppExpansion(
        const SearchState& node, int cur_idx,
        double gx, double gy, double gyaw,
        const nav2_costmap_2d::Costmap2D* costmap,
        std::vector<SearchState>& nodes, int& goal_idx) const;

    // ── 五次多项式平滑 (保留, 不再使用 — smoother_server 替代) ─
    std::vector<SmoothedPoint> smoothPath(const std::vector<SearchState>& raw,
                                           double dt) const;
    bool solveQuintic(double T, double p0, double p1,
                       double v0, double v1, double a0, double a1,
                       Eigen::Matrix<double, 6, 1>& coeff) const;

    // ── Nav2 接口 ─────────────────────────────────────────────
    rclcpp::Logger logger_{rclcpp::get_logger("HybridAStarPlanner")};
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    // ── 参数 ──────────────────────────────────────────────────
    VehicleParams vp_;
    double step_len_           = 0.2;
    double goal_xy_tolerance_  = 0.25;
    double goal_yaw_tolerance_ = 0.25;
    int    theta_bins_         = 72;
    double smooth_dt_          = 0.05;
    double smooth_time_factor_  = 0.5;
    uint8_t collision_threshold_ = 200;
    int    max_iterations_     = 500000;
    double cost_penalty_         = 2.0;
    double reverse_penalty_      = 2.0;

    // ── 运动原语 ──────────────────────────────────────────────
    std::vector<MotionPrimitive> primitives_;
};

} // namespace saye_nav2_plugins
