// oncoming_yield.h — 会车让行 + 窄路减速纯逻辑核（planning Phase 5 抽核）
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 planning_node.cpp
// Phase 5「会车让行 + 窄路减速」的障碍扫描 + 降速决策从 1600 行主循环抽出，
// 用确定性单测钉住行为，并把故障表里那条恶性 bug 编码成回归守卫：
//
//   历史 bug（掉头返程幽灵刹车 / 会车过度保守）：
//   旧逻辑用世界 dx 窗口 (-10,80] 预筛 + `|dy|>2.0 && |dy|≤1.5×路宽` 判"对向车"
//   → ① 掉头返程朝 -x 时前方车 dx<-10 被跳过（方向盲，无对向保护）；
//     ② 相邻对向车道（3.5m 外，横向 10.5m）也被当迎头威胁 → 巡航压到 0.4× 全刹。
//   修复：沿车头方向投影（along=rx·cos h+ry·sin h、lat=-rx·sin h+ry·cos h、
//   rel_v=vx·cos h+vy·sin h）+ 横向相邻上界（0.65×路宽，只认同车道头对头）。
//   前进方向（heading=0）投影退化为原世界坐标 → 既有前进场景零回归。
//
// 与 safety_geometry.h 的 min_oncoming_ttc 同源（2026-08-05 两侧对齐），此处是
// planning 侧的会车让行判定。header-only 纯逻辑核，无框架依赖，可单文件单测。
//
// 行为保持约定：与 planning_node.cpp 原实现逐行等价——
//   · 遍历全部 obs_count 槽位，**不做有效性过滤**（空槽由 on_perception_obstacles
//     清零为 (0,0,0,0)，rel_v=0 恒 < -2.0 为假故永不误触 oncoming，与原逻辑一致）。
//   · 降速取 min（yield/narrow 只在小于当前 command_speed 时才覆盖）。
#pragma once

#include <cmath>

namespace planning {

// 非拥有视图：ego 标量 + 障碍数组指针 + 槽位数。与生成协议定长容量解耦——
// 调用方传 obs_count=kMaxObs，协议扩容自动跟随。
struct YieldView {
    double x = 0.0, y = 0.0, heading = 0.0;
    double target_speed = 0.0;    // 巡航速度（让行/窄路降速的基准）
    double command_speed = 0.0;   // 待调整的当前指令速度
    double lane_width = 0.0;      // ≤1.0 时回退 3.5m
    const double* obs_x = nullptr;
    const double* obs_y = nullptr;
    const double* obs_vx = nullptr;
    const double* obs_vy = nullptr;
    int obs_count = 0;
};

struct YieldParams {
    double oncoming_lat_ratio = 0.65;  // 会车横向上界 = ratio × 路宽（只认同车道头对头）
    double oncoming_range = 60.0;      // 前方检测距离
    double oncoming_rel_v = -2.0;      // 沿向相对速度阈值（< 视为迎面驶来）
    double yield_ratio = 0.4;          // 会车让行降速到 ratio × 巡航
    double narrow_lookahead = 20.0;    // 窄路前瞻窗
    double narrow_threshold = 1.5;     // 两侧间距和 < 此值触发窄路减速
    double narrow_min_gap = 0.3;       // 窄路降速比映射下沿
    double narrow_span = 1.2;          // 窄路降速比映射跨度
    double narrow_min_ratio = 0.1;     // 窄路降速比下限
};

struct YieldDecision {
    bool oncoming = false;
    double min_clearance_left = 1e9;
    double min_clearance_right = 1e9;
    double narrow_width = 1e9;
    double command_speed = 0.0;        // 调整后的指令速度
};

// 会车让行 + 窄路减速：沿车头方向投影扫描障碍，返回是否会车、左右最近净距、
// 以及降速后的 command_speed。逻辑逐行等价于 planning_node.cpp Phase 5。
inline YieldDecision oncoming_yield(const YieldView& s, const YieldParams& p) {
    YieldDecision d;
    d.command_speed = s.command_speed;

    const double lane_w = (s.lane_width > 1.0 ? s.lane_width : 3.5);
    const double oncoming_lat_max = lane_w * p.oncoming_lat_ratio;
    const double fwd_x = std::cos(s.heading);
    const double fwd_y = std::sin(s.heading);

    for (int i = 0; i < s.obs_count; ++i) {
        const double rx = s.obs_x[i] - s.x;
        const double ry = s.obs_y[i] - s.y;
        const double along = rx * fwd_x + ry * fwd_y;        // 沿车头前方
        const double lat_signed = -rx * fwd_y + ry * fwd_x;  // 左为正

        // 会车检测：同车道头对头（|lat| ≤ 0.65×路宽）、迎面驶来（rel_v < -2）、前方 60m 内
        const double rel_v = s.obs_vx[i] * fwd_x + s.obs_vy[i] * fwd_y;
        if (std::fabs(lat_signed) <= oncoming_lat_max &&
            along > 0.0 && along < p.oncoming_range && rel_v < p.oncoming_rel_v) {
            d.oncoming = true;
        }

        // 窄路检测：统计左右两侧最近障碍横向距离（前瞻窗沿车头方向）
        if (along > 0.0 && along < p.narrow_lookahead) {
            if (lat_signed < 0.0 && std::fabs(lat_signed) < d.min_clearance_right)
                d.min_clearance_right = std::fabs(lat_signed);
            if (lat_signed > 0.0 && lat_signed < d.min_clearance_left)
                d.min_clearance_left = lat_signed;
        }
    }

    // 会车让行：降速到 40% 巡航速度，让对向车先通过
    if (d.oncoming) {
        double yield_speed = s.target_speed * p.yield_ratio;
        if (yield_speed < d.command_speed)
            d.command_speed = yield_speed;
    }

    // 窄路减速：两侧间距 < 1.5m 时限制速度（1.5m→100%, 0.5m→33%）
    d.narrow_width = d.min_clearance_left + d.min_clearance_right;
    if (d.narrow_width < 1e8 && d.narrow_width < p.narrow_threshold) {
        double ratio = (d.narrow_width - p.narrow_min_gap) / p.narrow_span;
        if (ratio < p.narrow_min_ratio) ratio = p.narrow_min_ratio;
        if (ratio > 1.0) ratio = 1.0;
        double narrow_speed = s.target_speed * ratio;
        if (narrow_speed < d.command_speed)
            d.command_speed = narrow_speed;
    }

    return d;
}

}  // namespace planning
