// stop_light_gate.h — 归位/变道目标车道前方红灯闸门纯逻辑核（behavior 抽核）
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 behavior_planner_node.cpp 的
// lane_ahead_stop_light() 抽成纯函数，用确定性单测钉住行为，并把故障表里两个历史 bug
// 编码成回归守卫：
//
//   ① 「超车/归位后立刻在红灯前刹停（无效变道）」：归位/变入决策必须先查目标车道
//      前方 stop_range 内有无非绿灯，有则不切过去——否则切回内侧道即刹停，白变一次道。
//      守卫：目标车道前方有红灯 → 返回 true（禁止归位）；只查**目标车道**（横向
//      |tl_y − lane_c| ≤ 半车道宽），相邻车道的灯不算。
//   ② 「掉头返程方向盲」：返程朝 −x，前方的灯 dx = tl_x − ego_x < 0；旧世界系判定
//      `dx>0` 会漏掉返程前方的红灯 → 归位撞红灯。修复：on_return 时 dx 取反（灯在
//      前方时 −dx>0 才检测得到）。前进(on_return=false) 时退化为原世界坐标零回归。
//
// header-only 纯逻辑核，无框架依赖：红绿灯缓存以非拥有视图传入，车道几何复用
// road_geometry.h 的 lane_center_y（共享纯 inline）。
#pragma once

#include <cmath>

#include "road_geometry.h"  // lane_center_y（共享纯 inline，唯一车道中心事实源）

namespace behavior {

struct StopLightView {
    // ego 状态
    double ego_x = 0.0;
    double ego_v = 0.0;
    bool   on_return = false;      // flowsim 权威行进方向：true=掉头返程（朝 −x）
    // 红绿灯缓存（非拥有视图，长度 = tl_count）
    const double* tl_x = nullptr;
    const double* tl_y_lane = nullptr;
    const int*    tl_state = nullptr;  // 0=green 非 0=黄/红
    int    tl_count = 0;
    bool   has_traffic_lights = false;
    // 车道布局
    int    lane_count = 0;
    double lane_width = 0.0;
};

struct StopLightParams {
    double decel = 8.0;             // 刹车距离 v²/decel（≈4m/s² 减速度）
    double base_margin = 3.0;       // + 3m 余量（与 planning_node 红灯 override 同源）
    double extra_margin = 20.0;     // 再 + 20m 归位余量
    double min_range = 60.0;        // 下限 60m：距灯 60m 内都算"马上要停"
    double lane_half_ratio = 0.5;   // 横向门限 = lane_width × 此比例（半车道宽）
};

// 目标车道 lane_idx 前方 stop_range 内是否有非绿灯？有则不该归位/变入。
// 逐行等价于 behavior_planner_node.cpp lane_ahead_stop_light（原 551-570 行）。
inline bool lane_ahead_stop_light(int lane_idx, const StopLightView& s,
                                  const StopLightParams& p) {
    if (!s.has_traffic_lights || s.tl_count <= 0) return false;
    double lane_c = lane_center_y(lane_idx, s.lane_count, s.lane_width, 0.0, 0.0);
    double v = s.ego_v; if (v < 0.0) v = 0.0;
    double stop_range = v * v / p.decel + p.base_margin + p.extra_margin;
    if (stop_range < p.min_range) stop_range = p.min_range;
    for (int i = 0; i < s.tl_count; i++) {
        if (s.tl_state[i] == 0) continue;                              // 绿灯
        if (std::fabs(s.tl_y_lane[i] - lane_c) > s.lane_width * p.lane_half_ratio)
            continue;                                                  // 只查目标车道（bug① 修复）
        double dx = s.tl_x[i] - s.ego_x;
        if (s.on_return) dx = -dx;                                     // 返程方向翻转（bug② 修复）
        if (dx <= 0.0 || dx > stop_range) continue;
        return true;
    }
    return false;
}

}  // namespace behavior
