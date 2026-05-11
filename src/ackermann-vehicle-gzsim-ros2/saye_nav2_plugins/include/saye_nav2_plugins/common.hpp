/**
 * @file    common.hpp
 * @brief   SAYE Nav2 插件共享数据结构和工具函数
 * @details
 *   移植自 my_pnc_core/pnc_core/common.h, 适配 Nav2 Costmap2D 接口。
 *   提供车辆运动学模型、碰撞检测、坐标变换等底层工具。
 */
#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Core>

#include <nav2_costmap_2d/costmap_2d.hpp>

namespace saye_nav2_plugins {

// ── 车辆运动学参数 ──────────────────────────────────────────────
struct VehicleParams {
    double wheelbase       = 0.2255;  // 轴距 [m]
    double max_steer       = 0.4;     // 最大方向盘转角 [rad]
    double min_turn_radius = 0.53;    // 最小转弯半径 [m]
    double length          = 0.30;    // 车长 [m]
    double width           = 0.09;    // 车宽 [m]
};

// ── 搜索状态 (Hybrid A* 节点) ───────────────────────────────────
struct SearchState {
    double x      = 0.0;   // 连续坐标 x [m] (costmap 坐标系)
    double y      = 0.0;   // 连续坐标 y [m] (costmap 坐标系)
    double theta  = 0.0;   // 朝向角 [rad]
    double g      = 0.0;   // 已走代价
    double h      = 0.0;   // 启发式
    int    parent_idx = -1; // 父节点索引
    bool   forward = true; // 当前步是否前进

    double f() const { return g + h; }
};

// ── 运动原语 (刹车/加速 + 方向盘转角 + 方向) ──────────────────
struct MotionPrimitive {
    double steer;     // 方向盘转角 [rad]
    int    direction; // +1 前进, -1 后退
};

// ── 平滑输出路径点 ──────────────────────────────────────────────
struct SmoothedPoint {
    double x = 0, y = 0, v = 0, heading = 0, kappa = 0;
};

// ════════════════════════════════════════════════════════════════
// 工具函数
// ════════════════════════════════════════════════════════════════

// 角度归一化到 [-π, π)
inline double normalizeAngle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

// 欧氏距离
inline double euclideanDist(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

// Costmap 单格碰撞检测 (cost >= threshold → 障碍物)
inline bool isCellOccupied(const nav2_costmap_2d::Costmap2D* costmap,
                            int gx, int gy, uint8_t threshold) {
    if (!costmap) return true;
    // 越界 → 障碍物
    if (gx < 0 || gx >= static_cast<int>(costmap->getSizeInCellsX()) ||
        gy < 0 || gy >= static_cast<int>(costmap->getSizeInCellsY()))
        return true;
    return costmap->getCost(static_cast<unsigned int>(gx),
                            static_cast<unsigned int>(gy)) >= threshold;
}

// 自行车模型运动学积分 (前向欧拉)
//   ẋ = v·cos(θ),  ẏ = v·sin(θ),  θ̇ = v·tan(δ)/L
// direction: +1 前进 / -1 后退
inline void kinematicPropagate(double& x, double& y, double& theta,
                                double steer, int direction,
                                double wheelbase, double step) {
    if (std::abs(steer) < 1e-8) {
        // 近直线
        x     += direction * step * std::cos(theta);
        y     += direction * step * std::sin(theta);
        // theta 不变
    } else {
        double R      = wheelbase / std::tan(std::abs(steer));
        double dtheta = direction * step * std::tan(steer) / wheelbase;
        x     += R * (std::sin(theta + dtheta) - std::sin(theta));
        y     -= R * (std::cos(theta + dtheta) - std::cos(theta));
        theta += dtheta;
    }
    theta = normalizeAngle(theta);
}

// 车辆外形碰撞检测 (13 点采样, 5×3 网格覆盖矩形车身)
// 将车身矩形 13 个采样点映射到 costmap 栅格, 任一碰撞则拒绝
inline bool isValidVehiclePose(const nav2_costmap_2d::Costmap2D* costmap,
                                double x, double y, double theta,
                                const VehicleParams& vp, uint8_t threshold) {
    if (!costmap) return false;

    const double half_len = vp.length * 0.5;
    const double half_wid = vp.width  * 0.5;
    const double cos_t    = std::cos(theta);
    const double sin_t    = std::sin(theta);

    // 5 列 (纵向) × 3 行 (横向) = 15 个采样点
    const double lx[5] = { -half_len, -half_len*0.5, 0.0, half_len*0.5, half_len };
    const double ly[3] = { -half_wid, 0.0, half_wid };

    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 3; ++row) {
            double wx = x + cos_t * lx[col] - sin_t * ly[row];
            double wy = y + sin_t * lx[col] + cos_t * ly[row];

            unsigned int gx, gy;
            if (!costmap->worldToMap(wx, wy, gx, gy)) return false;
            if (costmap->getCost(gx, gy) >= threshold) return false;
        }
    }
    return true;
}

// 弧段碰撞检测: 沿 from→to 采样 n 个中间位姿
inline bool isValidArc(const nav2_costmap_2d::Costmap2D* costmap,
                        const SearchState& from, const SearchState& to,
                        const VehicleParams& vp, uint8_t threshold,
                        int n_samples = 10) {
    if (!isValidVehiclePose(costmap, to.x, to.y, to.theta, vp, threshold))
        return false;

    double dtheta = normalizeAngle(to.theta - from.theta);
    for (int i = 1; i < n_samples; ++i) {
        double alpha = static_cast<double>(i) / n_samples;
        double theta_mid = normalizeAngle(from.theta + alpha * dtheta);

        double x_mid, y_mid;
        if (std::abs(dtheta) < 1e-6) {
            x_mid = from.x + alpha * (to.x - from.x);
            y_mid = from.y + alpha * (to.y - from.y);
        } else {
            // 圆弧插值
            double dx = to.x - from.x, dy = to.y - from.y;
            double cx = (from.x + to.x) / 2.0 - dy / (2.0 * std::tan(dtheta / 2.0));
            double cy = (from.y + to.y) / 2.0 + dx / (2.0 * std::tan(dtheta / 2.0));
            double r  = std::hypot(from.x - cx, from.y - cy);
            double phi0 = std::atan2(from.y - cy, from.x - cx);
            double phi_mid = phi0 + alpha * dtheta;
            x_mid = cx + r * std::cos(phi_mid);
            y_mid = cy + r * std::sin(phi_mid);
        }
        if (!isValidVehiclePose(costmap, x_mid, y_mid, theta_mid, vp, threshold))
            return false;
    }
    return true;
}

// 曲率自适应速度计算
//   κ = |tan(δ)| / L,  κ_max = tan(δ_max) / L
//   v = v_cruise · max(0.15, 1 - 0.8·κ/κ_max)
inline double speedFromCurvature(double steer, const VehicleParams& vp,
                                  double v_cruise, double v_max, double v_min) {
    double L      = vp.wheelbase;
    double kappa  = std::abs(std::tan(steer)) / L;
    double k_max  = std::tan(vp.max_steer) / L;

    if (k_max < 1e-6) return v_cruise;

    double ratio = 1.0 - 0.8 * (kappa / k_max);
    ratio = std::max(0.15, std::min(1.0, ratio));
    double v = v_cruise * ratio;
    return std::max(v_min, std::min(v_max, v));
}

// 路径曲率计算 (离散三点法)
inline double computePathCurvature(double x0, double y0, double x1, double y1,
                                    double x2, double y2) {
    double dx1 = x1 - x0, dy1 = y1 - y0;
    double dx2 = x2 - x1, dy2 = y2 - y1;
    double ds = 0.5 * (std::hypot(dx1, dy1) + std::hypot(dx2, dy2));
    if (ds < 1e-6) return 0.0;
    double ddx = (dx2 - dx1) / ds;
    double ddy = (dy2 - dy1) / ds;
    double dtx = dx1 / std::max(1e-6, std::hypot(dx1, dy1));
    double dty = dy1 / std::max(1e-6, std::hypot(dx1, dy1));
    return (dtx * ddy - dty * ddx);
}

// 前瞻减速: 向前搜索最大曲率, 提前减速
//   lookahead_steps: 前瞻步数
inline double predictiveSpeed(const std::vector<SmoothedPoint>& path, int idx,
                               const VehicleParams& vp,
                               double v_cruise, double v_max, double v_min,
                               int lookahead_steps = 20) {
    double k_max = std::tan(vp.max_steer) / vp.wheelbase;
    if (k_max < 1e-6) return v_cruise;

    int end = std::min(idx + lookahead_steps, static_cast<int>(path.size()));
    double max_kappa = 0.0;
    for (int i = idx; i < end; ++i) {
        double k = std::abs(path[i].kappa);
        if (k > max_kappa) max_kappa = k;
    }

    double ratio = 1.0 - 0.85 * (max_kappa / k_max);
    ratio = std::max(0.12, std::min(1.0, ratio));
    double v = v_cruise * ratio;
    return std::max(v_min, std::min(v_max, v));
}

} // namespace saye_nav2_plugins
