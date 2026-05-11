/**
 * @file    hybrid_astar_planner.cpp
 * @brief   Hybrid A* 全局规划器实现 (Nav2 GlobalPlanner 插件)
 * @details
 *   完整移植自 my_pnc_core/src/hybrid_astar.cpp,
 *   适配 Nav2 Costmap2D 接口。
 *
 *   算法流程:
 *     1. configure() — 读取参数, 生成运动原语
 *     2. createPlan() — 从 costmap 取地图, 执行 A* 搜索, 平滑路径
 */
#include "saye_nav2_plugins/hybrid_astar_planner.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_util/node_utils.hpp>

#include <Eigen/QR>
#include <algorithm>
#include <limits>
#include <queue>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace saye_nav2_plugins {

// ── A* 最小堆比较器 ────────────────────────────────────────────
namespace {
    struct NodeCompare {
        bool operator()(const SearchState& a, const SearchState& b) const {
            return a.f() > b.f();  // f 小优先
        }
    };
}

// ════════════════════════════════════════════════════════════════
HybridAStarPlanner::HybridAStarPlanner()  = default;
HybridAStarPlanner::~HybridAStarPlanner() = default;

// ════════════════════════════════════════════════════════════════
// configure — Nav2 生命周期回调
// ════════════════════════════════════════════════════════════════
void HybridAStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_        = parent;
    costmap_ros_ = costmap_ros;
    tf_          = tf;

    auto node = node_.lock();
    logger_ = node->get_logger();

    // ── 读取车辆参数 ──────────────────────────────────────────
    nav2_util::declare_parameter_if_not_declared(node, name + ".wheelbase",
                                                  rclcpp::ParameterValue(0.2255));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_steer",
                                                  rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(node, name + ".length",
                                                  rclcpp::ParameterValue(0.30));
    nav2_util::declare_parameter_if_not_declared(node, name + ".width",
                                                  rclcpp::ParameterValue(0.09));

    vp_.wheelbase = node->get_parameter(name + ".wheelbase").as_double();
    vp_.max_steer = node->get_parameter(name + ".max_steer").as_double();
    vp_.length    = node->get_parameter(name + ".length").as_double();
    vp_.width     = node->get_parameter(name + ".width").as_double();
    vp_.min_turn_radius = vp_.wheelbase / std::tan(vp_.max_steer);

    // ── 读取规划参数 ──────────────────────────────────────────
    nav2_util::declare_parameter_if_not_declared(node, name + ".step_len",
                                                  rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(node, name + ".theta_bins",
                                                  rclcpp::ParameterValue(72));
    nav2_util::declare_parameter_if_not_declared(node, name + ".goal_xy_tolerance",
                                                  rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(node, name + ".goal_yaw_tolerance",
                                                  rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(node, name + ".smooth_dt",
                                                  rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(node, name + ".smooth_time_factor",
                                                  rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, name + ".collision_threshold",
                                                  rclcpp::ParameterValue(200));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_iterations",
                                                  rclcpp::ParameterValue(500000));

    step_len_            = node->get_parameter(name + ".step_len").as_double();
    theta_bins_          = node->get_parameter(name + ".theta_bins").as_int();
    goal_xy_tolerance_   = node->get_parameter(name + ".goal_xy_tolerance").as_double();
    goal_yaw_tolerance_  = node->get_parameter(name + ".goal_yaw_tolerance").as_double();
    smooth_dt_           = node->get_parameter(name + ".smooth_dt").as_double();
    smooth_time_factor_  = node->get_parameter(name + ".smooth_time_factor").as_double();
    collision_threshold_ = static_cast<uint8_t>(
        node->get_parameter(name + ".collision_threshold").as_int());
    max_iterations_      = node->get_parameter(name + ".max_iterations").as_int();

    nav2_util::declare_parameter_if_not_declared(node, name + ".cost_penalty",
                                                  rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".reverse_penalty",
                                                  rclcpp::ParameterValue(2.0));
    cost_penalty_    = node->get_parameter(name + ".cost_penalty").as_double();
    reverse_penalty_ = node->get_parameter(name + ".reverse_penalty").as_double();

    generatePrimitives();

    RCLCPP_INFO(logger_, "HybridAStarPlanner configured: L=%.3f max_steer=%.3f step=%.2f "
                "bins=%d collision_threshold=%d",
                vp_.wheelbase, vp_.max_steer, step_len_, theta_bins_, collision_threshold_);
}

void HybridAStarPlanner::activate()   { RCLCPP_INFO(logger_, "HybridAStarPlanner activated"); }
void HybridAStarPlanner::deactivate() { RCLCPP_INFO(logger_, "HybridAStarPlanner deactivated"); }
void HybridAStarPlanner::cleanup()    { RCLCPP_INFO(logger_, "HybridAStarPlanner cleaned up"); }

// ════════════════════════════════════════════════════════════════
// generatePrimitives — 生成 6 个运动原语
// ════════════════════════════════════════════════════════════════
void HybridAStarPlanner::generatePrimitives()
{
    primitives_.clear();
    const double steer_set[] = {
        -vp_.max_steer, -vp_.max_steer*0.75, -vp_.max_steer*0.5, -vp_.max_steer*0.25,
         0.0,
         vp_.max_steer*0.25, vp_.max_steer*0.5, vp_.max_steer*0.75, vp_.max_steer
    };
    for (double s : steer_set) {
        primitives_.push_back({s,  1});  // 前进
        primitives_.push_back({s, -1});  // 后退
    }
}

// ════════════════════════════════════════════════════════════════
// kinematicExpand — 自行车模型运动学积分
// ════════════════════════════════════════════════════════════════
SearchState HybridAStarPlanner::kinematicExpand(
    const SearchState& node, const MotionPrimitive& prim,
    const nav2_costmap_2d::Costmap2D* costmap) const
{
    SearchState next = node;
    next.g += step_len_;
    next.forward = (prim.direction == 1);
    kinematicPropagate(next.x, next.y, next.theta,
                       prim.steer, prim.direction,
                       vp_.wheelbase, step_len_);

    if (costmap) {
        unsigned int gx, gy;
        if (costmap->worldToMap(next.x, next.y, gx, gy)) {
            auto cc = costmap->getCost(gx, gy);
            if (cc < collision_threshold_) {
                next.g += (static_cast<double>(cc) / 252.0) * cost_penalty_ * step_len_;
            }
        }
    }
    return next;
}

// ════════════════════════════════════════════════════════════════
// computeObstacleHeuristic — 2D BFS wavefront
//   从 goal 向 costmap 自由空间做 BFS, 输出单位是 costmap 栅格步数,
//   后续在 h() 中 × resolution 转为米与 g-cost (米) 对齐。
// ════════════════════════════════════════════════════════════════
void HybridAStarPlanner::computeObstacleHeuristic(
    const nav2_costmap_2d::Costmap2D* costmap, double gx, double gy)
{
    if (!costmap) return;
    const int W = static_cast<int>(costmap->getSizeInCellsX());
    const int H = static_cast<int>(costmap->getSizeInCellsY());
    heuristic_lookup_.assign(W * H, std::numeric_limits<double>::max());
    heuristic_w_ = W; heuristic_h_ = H;

    unsigned int ggx, ggy;
    if (!costmap->worldToMap(gx, gy, ggx, ggy)) return;
    if (costmap->getCost(ggx, ggy) >= collision_threshold_) return;

    const int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    const double res = costmap->getResolution();

    using QI = std::pair<double,int>;
    std::priority_queue<QI,std::vector<QI>,std::greater<QI>> pq;
    int gi = ggy * W + static_cast<int>(ggx);
    heuristic_lookup_[gi] = 0.0;
    pq.push({0.0, gi});

    while (!pq.empty()) {
        auto [c, idx] = pq.top(); pq.pop();
        if (c > heuristic_lookup_[idx] + 1e-9) continue;
        int cx = idx % W, cy = idx / W;
        for (int d = 0; d < 4; ++d) {
            int nx = cx + dirs[d][0], ny = cy + dirs[d][1];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            auto cc = costmap->getCost(static_cast<unsigned int>(nx),
                                        static_cast<unsigned int>(ny));
            if (cc >= collision_threshold_) continue;
            double nc = c + res;  // 步长=分辨率 → 单位: 米
            int ni = ny * W + nx;
            if (nc < heuristic_lookup_[ni] - 1e-9) {
                heuristic_lookup_[ni] = nc;
                pq.push({nc, ni});
            }
        }
    }
}

double HybridAStarPlanner::obstacleHeuristic(int gx, int gy) const
{
    if (heuristic_lookup_.empty()) return std::numeric_limits<double>::max();
    if (gx < 0 || gx >= heuristic_w_ || gy < 0 || gy >= heuristic_h_)
        return std::numeric_limits<double>::max();
    return heuristic_lookup_[gy * heuristic_w_ + gx];
}

// ════════════════════════════════════════════════════════════════
// tryReedsSheppExpansion — OMPL RS 解析扩展
//   用 OMPL 的 ReedsSheppStateSpace 算当前节点到目标的最短 RS 曲线，
//   碰撞检查通过后串联到搜索树，直接命中目标。
// ════════════════════════════════════════════════════════════════
bool HybridAStarPlanner::tryReedsSheppExpansion(
    const SearchState& node, int cur_idx,
    double gx, double gy, double gyaw,
    const nav2_costmap_2d::Costmap2D* costmap,
    std::vector<SearchState>& nodes, int& goal_idx) const
{
    if (!costmap) return false;
    double dist = euclideanDist(node.x, node.y, gx, gy);
    if (dist < step_len_ || dist > 15.0) return false;

    auto space = std::make_shared<ompl::base::ReedsSheppStateSpace>(vp_.min_turn_radius);
    auto* from = space->allocState()->as<ompl::base::SE2StateSpace::StateType>();
    auto* to   = space->allocState()->as<ompl::base::SE2StateSpace::StateType>();
    from->setXY(node.x, node.y);  from->setYaw(node.theta);
    to->setXY(gx, gy);            to->setYaw(gyaw);

    double rs_len = space->distance(from, to);
    if (rs_len > 15.0 || rs_len < step_len_) {
        space->freeState(from); space->freeState(to); return false;
    }

    const int n = std::max(4, static_cast<int>(rs_len / (step_len_ * 0.5)));
    auto* s = space->allocState()->as<ompl::base::SE2StateSpace::StateType>();
    for (int i = 1; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        space->interpolate(from, to, t, s);
        if (!isValidVehiclePose(costmap, s->getX(), s->getY(), s->getYaw(), vp_, collision_threshold_)) {
            space->freeState(s); space->freeState(from); space->freeState(to); return false;
        }
    }
    space->freeState(s);

    // 碰撞通过 → 加中间节点 + 终点
    int parent = cur_idx;
    double base_g = node.g, seg = rs_len / n;
    for (int i = 1; i < n; ++i) {
        double t = static_cast<double>(i) / n;
        auto* m = space->allocState()->as<ompl::base::SE2StateSpace::StateType>();
        space->interpolate(from, to, t, m);
        SearchState mid{m->getX(), m->getY(), normalizeAngle(m->getYaw()),
                        base_g + i * seg, 0.0, parent, true};
        space->freeState(m);
        nodes.push_back(mid);
        parent = static_cast<int>(nodes.size()) - 1;
    }
    SearchState gn{gx, gy, normalizeAngle(gyaw), base_g + rs_len, 0.0, parent, true};
    goal_idx = static_cast<int>(nodes.size());
    nodes.push_back(gn);
    space->freeState(from); space->freeState(to);
    return true;
}

// ════════════════════════════════════════════════════════════════
// discreteKey — (gx, gy, gtheta) → 64-bit key
// ════════════════════════════════════════════════════════════════
int64_t HybridAStarPlanner::discreteKey(int gx, int gy, int gtheta) const
{
    constexpr int64_t OFFSET = 1LL << 19;
    return ((static_cast<int64_t>(gx) + OFFSET) << 40)
         | ((static_cast<int64_t>(gy) + OFFSET) << 20)
         | (static_cast<int64_t>(gtheta) & 0xFFFFF);
}

// ════════════════════════════════════════════════════════════════
// thetaToBin — 角度 [-π,π) → [0, theta_bins_)
// ════════════════════════════════════════════════════════════════
int HybridAStarPlanner::thetaToBin(double theta) const
{
    theta = normalizeAngle(theta);
    double t = theta + M_PI;
    int bin = static_cast<int>(std::floor(t / (2.0 * M_PI) * theta_bins_));
    return bin % theta_bins_;
}

// ════════════════════════════════════════════════════════════════
// extractPath — 从节点链回溯路径
// ════════════════════════════════════════════════════════════════
std::vector<SearchState> HybridAStarPlanner::extractPath(
    const std::vector<SearchState>& nodes, int goal_idx) const
{
    std::vector<SearchState> path;
    int idx = goal_idx;
    while (idx >= 0) {
        path.push_back(nodes[idx]);
        idx = nodes[idx].parent_idx;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// ════════════════════════════════════════════════════════════════
// solveQuintic — 求解单段五次多项式系数
// ════════════════════════════════════════════════════════════════
bool HybridAStarPlanner::solveQuintic(double T,
                                       double p0, double p1,
                                       double v0, double v1,
                                       double a0, double a1,
                                       Eigen::Matrix<double, 6, 1>& coeff) const
{
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    Eigen::Matrix<double, 6, 6> A;
    // 行 0: f(0)   = p0 → c0 = p0
    A(0,0)=0; A(0,1)=0; A(0,2)=0; A(0,3)=0; A(0,4)=0; A(0,5)=1;
    // 行 1: f(T)   = p1
    A(1,0)=T5; A(1,1)=T4; A(1,2)=T3; A(1,3)=T2; A(1,4)=T; A(1,5)=1;
    // 行 2: f'(0)  = v0 → c1 = v0
    A(2,0)=0; A(2,1)=0; A(2,2)=0; A(2,3)=0; A(2,4)=1; A(2,5)=0;
    // 行 3: f'(T)  = v1
    A(3,0)=5*T4; A(3,1)=4*T3; A(3,2)=3*T2; A(3,3)=2*T; A(3,4)=1; A(3,5)=0;
    // 行 4: f''(0) = a0 → 2c2 = a0
    A(4,0)=0; A(4,1)=0; A(4,2)=0; A(4,3)=2; A(4,4)=0; A(4,5)=0;
    // 行 5: f''(T) = a1
    A(5,0)=20*T3; A(5,1)=12*T2; A(5,2)=6*T; A(5,3)=2; A(5,4)=0; A(5,5)=0;

    Eigen::Matrix<double, 6, 1> b;
    b << p0, p1, v0, v1, a0, a1;

    coeff = A.colPivHouseholderQr().solve(b);
    return true;
}

// ════════════════════════════════════════════════════════════════
// smoothPath — 五次多项式轨迹平滑
// ════════════════════════════════════════════════════════════════
std::vector<SmoothedPoint> HybridAStarPlanner::smoothPath(
    const std::vector<SearchState>& raw, double dt) const
{
    std::vector<SmoothedPoint> result;
    const size_t N = raw.size();
    if (N < 2) {
        for (const auto& s : raw)
            result.push_back({s.x, s.y, 0.0, s.theta});
        return result;
    }

    // 计算总弧长并分配时间
    std::vector<double> seg_lengths(N-1, 0.0);
    double total_length = 0.0;
    for (size_t i = 0; i < N-1; ++i) {
        seg_lengths[i] = euclideanDist(raw[i].x, raw[i].y, raw[i+1].x, raw[i+1].y);
        total_length += seg_lengths[i];
    }
    if (total_length < 1e-6) return result;

    double total_time = std::max(2.0, total_length * smooth_time_factor_);

    double vx0 = 0.0, vy0 = 0.0, ax0 = 0.0, ay0 = 0.0;
    for (size_t i = 0; i < N-1; ++i) {
        double Ti = total_time * seg_lengths[i] / total_length;
        if (Ti < dt) Ti = dt;

        double p0x = raw[i].x,   p0y = raw[i].y;
        double p1x = raw[i+1].x, p1y = raw[i+1].y;

        double vx1 = 0.0, vy1 = 0.0;
        if (i < N-2) { vx1 = (p1x - p0x) / Ti; vy1 = (p1y - p0y) / Ti; }

        Eigen::Matrix<double,6,1> cx, cy;
        solveQuintic(Ti, p0x, p1x, vx0, vx1, ax0, 0.0, cx);
        solveQuintic(Ti, p0y, p1y, vy0, vy1, ay0, 0.0, cy);

        // 多项式求值 lambda
        auto eval = [](const Eigen::Matrix<double,6,1>& c, double t) {
            double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
            return c(0)*t5 + c(1)*t4 + c(2)*t3 + c(3)*t2 + c(4)*t + c(5);
        };
        auto evalDeriv = [](const Eigen::Matrix<double,6,1>& c, double t) {
            double t2 = t*t, t3 = t2*t, t4 = t3*t;
            return 5*c(0)*t4 + 4*c(1)*t3 + 3*c(2)*t2 + 2*c(3)*t + c(4);
        };

        double t = 0.0;
        bool last_seg = (i == N-2);
        while (t < Ti - 1e-12) {
            double vx = evalDeriv(cx, t), vy = evalDeriv(cy, t);
            result.push_back({eval(cx, t), eval(cy, t),
                              std::hypot(vx, vy), std::atan2(vy, vx)});
            t += dt;
        }
        if (last_seg) {
            double vx = evalDeriv(cx, Ti), vy = evalDeriv(cy, Ti);
            result.push_back({eval(cx, Ti), eval(cy, Ti),
                              std::hypot(vx, vy), std::atan2(vy, vx)});
        }
        vx0 = evalDeriv(cx, Ti); vy0 = evalDeriv(cy, Ti);
        ax0 = 0.0; ay0 = 0.0;
    }
    return result;
}

// ════════════════════════════════════════════════════════════════
// createPlan — 主规划函数 (Nav2 接口)
// ════════════════════════════════════════════════════════════════
nav_msgs::msg::Path HybridAStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    RCLCPP_INFO(logger_, "createPlan: (%.2f,%.2f)→(%.2f,%.2f)",
                start.pose.position.x, start.pose.position.y,
                goal.pose.position.x,  goal.pose.position.y);

    nav_msgs::msg::Path result;
    result.header.frame_id = costmap_ros_->getGlobalFrameID();
    result.header.stamp = rclcpp::Clock().now();

    // ── 1. 获取 costmap ───────────────────────────────────────
    auto* costmap = costmap_ros_->getCostmap();
    if (!costmap) {
        RCLCPP_ERROR(logger_, "costmap is null");
        return result;
    }

    // ── 2. 起终点 (map 坐标系 = world 坐标系, worldToMap 内部处理 origin) ──
    double sx = start.pose.position.x;
    double sy = start.pose.position.y;
    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;

    double syaw = tf2::getYaw(start.pose.orientation);
    double gyaw = tf2::getYaw(goal.pose.orientation);

    // ── 3. 起终点有效性检查 ──────────────────────────────────
    if (!isValidVehiclePose(costmap, sx, sy, syaw, vp_, collision_threshold_)) {
        RCLCPP_ERROR(logger_, "start pose in collision");
        return result;
    }
    if (!isValidVehiclePose(costmap, gx, gy, gyaw, vp_, collision_threshold_)) {
        RCLCPP_ERROR(logger_, "goal pose in collision");
        return result;
    }

    // ── 3.5 Dijkstra 障碍物感知查表 ──────────────────────────
    computeObstacleHeuristic(costmap, gx, gy);
    bool has_dij = !heuristic_lookup_.empty();

    // 启发式 lambda: 查 Dijkstra 表 (米), 回退欧氏距离
    auto h = [&](double px, double py) -> double {
        if (has_dij) {
            unsigned int gi, gj;
            if (costmap->worldToMap(px, py, gi, gj)) {
                double base = obstacleHeuristic(
                    static_cast<int>(gi), static_cast<int>(gj));
                if (base < 1e9) return base * 1.0;  // 已是米，不缩放
            }
        }
        return std::hypot(gx - px, gy - py) * 1.0;
    };

    // ── 4. Hybrid A* 搜索 ────────────────────────────────────
    unsigned int gsx, gsy;
    costmap->worldToMap(sx, sy, gsx, gsy);

    // 动态数组存储节点 (避免 shared_ptr 开销)
    std::vector<SearchState> nodes;
    nodes.reserve(200000);

    SearchState start_node{sx, sy, normalizeAngle(syaw), 0.0,
                           h(sx, sy), -1, true};
    nodes.push_back(start_node);

    // 开启列表 (索引 + f 值)
    using PQEntry = std::pair<int, double>;
    auto pq_cmp = [](const PQEntry& a, const PQEntry& b) { return a.second > b.second; };
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(pq_cmp)> open_list(pq_cmp);
    open_list.push({0, start_node.f()});

    // 已访问表: key → 最小 g 值
    std::unordered_map<int64_t, double> visited;
    int start_key = discreteKey(static_cast<int>(gsx), static_cast<int>(gsy),
                                thetaToBin(start_node.theta));
    visited[start_key] = 0.0;

    int iterations = 0;
    bool found = false;
    int goal_idx = -1;

    while (!open_list.empty() && iterations < max_iterations_) {
        iterations++;
        auto [cur_idx, cur_f] = open_list.top(); open_list.pop();

        auto& cur = nodes[cur_idx];
        unsigned int cur_gx, cur_gy;
        costmap->worldToMap(cur.x, cur.y, cur_gx, cur_gy);
        if (cur.g > visited[discreteKey(static_cast<int>(cur_gx),
                                         static_cast<int>(cur_gy),
                                         thetaToBin(cur.theta))] + 1e-9)
            continue;

        // 目标检测
        if (euclideanDist(cur.x, cur.y, gx, gy) < goal_xy_tolerance_ &&
            std::abs(normalizeAngle(cur.theta - gyaw)) < goal_yaw_tolerance_) {
            found = true;
            goal_idx = cur_idx;
            break;
        }

        // Reeds-Shepp 解析扩展: 每 30 次迭代尝试直连目标
        if (iterations > 30 && iterations % 30 == 0) {
            if (tryReedsSheppExpansion(cur, cur_idx, gx, gy, gyaw,
                                        costmap, nodes, goal_idx)) {
                found = true;
                RCLCPP_INFO(logger_, "RS expansion hit at iter %d", iterations);
                break;
            }
        }

        // 扩张所有原语
        for (const auto& prim : primitives_) {
            SearchState next = kinematicExpand(cur, prim, costmap);

            if (!isValidArc(costmap, cur, next, vp_, collision_threshold_))
                continue;

            if (!next.forward)
                next.g += reverse_penalty_ * step_len_;

            next.h = h(next.x, next.y);
            next.parent_idx = cur_idx;

            unsigned int next_gx, next_gy;
            costmap->worldToMap(next.x, next.y, next_gx, next_gy);
            int64_t key = discreteKey(static_cast<int>(next_gx),
                                       static_cast<int>(next_gy),
                                       thetaToBin(next.theta));

            auto it = visited.find(key);
            if (it != visited.end() && next.g >= it->second - 1e-9)
                continue;

            visited[key] = next.g;
            int next_idx = static_cast<int>(nodes.size());
            nodes.push_back(next);
            open_list.push({next_idx, next.f()});
        }
    }

    if (!found) {
        RCLCPP_WARN(logger_, "HybridAStarPlanner: no path found after %d iterations", iterations);
        return result;
    }

    auto raw_path = extractPath(nodes, goal_idx);
    RCLCPP_INFO(logger_, "HybridAStarPlanner: path found (%zu nodes, %d iterations)",
                raw_path.size(), iterations);

    // ── 直接输出原始 A* 路径, 平滑由 smoother_server 处理 ──
    for (const auto& s : raw_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = result.header.frame_id;
        pose.pose.position.x = s.x;
        pose.pose.position.y = s.y;
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, s.theta);
        pose.pose.orientation = tf2::toMsg(q);
        result.poses.push_back(pose);
    }

    RCLCPP_INFO(logger_, "HybridAStarPlanner: result %zu poses", result.poses.size());
    return result;
}

} // namespace saye_nav2_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(saye_nav2_plugins::HybridAStarPlanner, nav2_core::GlobalPlanner)
