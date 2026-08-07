// safety_geometry.h — 安全层近场几何纯逻辑核（从 safety_control_node.cpp 抽出）
//
// 背景（Phase 0 立测试网，见 docs/ARCHITECTURE_REVIEW.md）：
//   safety 的 6 个近场风险度量（同车道 gap / 车辆 TTC / 对向 TTC / 横向穿越 /
//   行人碰撞 / 行人过街保持）此前内联在 safety_control_node.cpp，读一个巨大的
//   VehicleState（含 128 槽障碍数组）+ 生成协议 ObstacleList，无法脱离
//   transport/param 框架单测。故障表里最凶的一条——「掉头返程幽灵刹车 + 无同向
//   防撞（撞旁边车）」根因正是这几个函数**系统性用世界 +x 坐标**判前后：返程
//   （向西 heading≈π）时前车在 -x 被 skip → 同向防撞失效、同向车被误判迎头 →
//   head-on 硬刹。修复是全部改沿车头方向投影（ahead / lat / along_v）。这类
//   方向 bug 此前只能靠 45s 黑盒 demo 事后暴露；抽核后由 tests/test_safety_geometry.cpp
//   在毫秒级钉死「前进方向零回归 + 返程方向正确」两侧。
//
// 职责边界（对应 CLAUDE.md 模块职责铁律）：
//   这是 safety_control 的一部分——纯安全闸门的「测距」环节。只把「ego + 障碍
//   相对位姿」算成距离/TTC/风险度量，不理解任务意图、不改写指令（限幅+刹车决策
//   仍在 safety_control_node.cpp 的 task 循环里）。
//
// 纯逻辑：只依赖 <cmath>。用非拥有视图 EgoView（ego 标量 + 障碍数组指针 + 数量）
//   与生成协议 ObstacleList 的定长容量解耦——kMaxObs 仍在 .cpp 里由
//   sizeof(ObstacleList) 推导并作为 obs_count 传入，协议扩容自动跟随，本 header
//   不写死任何容量。header-only，同时编进 safety_control 插件与单测，杜绝第二份实现。
//
// 行为契约：所有阈值/常量 == 抽取时 safety_control_node.cpp 的字面量，逐行等价
//   移植，零行为变更。heading=0（前进/朝东）时沿车头投影退化为原世界坐标，
//   对既有前进场景零回归。
#pragma once
#include <algorithm>
#include <cmath>

namespace safety {

// 非拥有视图：ego 位姿 + 障碍数组指针 + 有效槽数量。调用方（safety_control_node
// 的 VehicleState，或测试）持有真实存储，本视图只读。obs_count = 需扫描的槽数
// （= .cpp 里的 kMaxObs），无效槽由 obs_valid[i]==false 跳过。
struct EgoView {
    double x = 0.0;
    double y = 0.0;
    double speed = 0.0;
    double heading = 0.0;
    const double* obs_x = nullptr;
    const double* obs_y = nullptr;
    const double* obs_v = nullptr;
    const double* obs_vy = nullptr;
    const bool*   obs_valid = nullptr;
    int           obs_count = 0;
    int           ped_index = -1;   // 首个行人障碍下标，-1 = 无
};

// 同车道最近前车净距（沿车头方向）。返回 1e9 = 无前车。
// 方向感知（2026-08-04 掉头返程）：旧实现 dx=obs_x-ego_x 世界 +x 判"前方"，
// 返程向西时前车在 -x（dx<0）被 skip → 同向 gap 恒 1e9 → 返程跟车失效。
// 沿车头方向投影 ahead 后前进/返程统一。
inline double nearest_same_lane_gap(const EgoView& s, double same_lane_tol) {
    double best_gap = 1e9;
    const double fwd_x = std::cos(s.heading);
    const double fwd_y = std::sin(s.heading);
    for (int i = 0; i < s.obs_count; ++i) {
        if (!s.obs_valid[i]) continue;
        const double ex = s.obs_x[i] - s.x;
        const double ey = s.obs_y[i] - s.y;
        const double lat = std::fabs(-ex * fwd_y + ey * fwd_x);
        if (lat > same_lane_tol) continue;
        const double along = ex * fwd_x + ey * fwd_y;
        const double gap = along - 4.6;
        if (along > 0.0 && gap < best_gap) best_gap = gap;
    }
    return best_gap;
}

// 前方行人碰撞净距（沿车头方向）。返回 1e9 = 无风险行人。
// 方向感知（2026-08-05）：沿车头投影，返程自适应。行人在 ego 后方（along ≤ 0）
// 无前向碰撞风险。
inline double pedestrian_collision_gap(const EgoView& s) {
    int pi = s.ped_index;
    if (pi < 0 || !s.obs_valid[pi]) return 1e9;
    const double fwd_x = std::cos(s.heading);
    const double fwd_y = std::sin(s.heading);
    const double ex = s.obs_x[pi] - s.x;
    const double ey = s.obs_y[pi] - s.y;
    const double along = ex * fwd_x + ey * fwd_y;            /* 沿车头前方距离 */
    const double lat = std::fabs(-ex * fwd_y + ey * fwd_x);  /* 横向（车体系） */
    if (along <= 0.0 || along > 70.0 || lat > 4.5) return 1e9;
    return along - 2.8;
}

// 行人过街保持净距（不停在过街线上）。返回 1e9 = 无需保持。
// 方向感知（2026-08-05）：同 pedestrian_collision_gap，沿车头投影，返程自适应。
inline double pedestrian_crossing_hold_gap(const EgoView& s) {
    int pi = s.ped_index;
    if (pi < 0 || !s.obs_valid[pi]) return 1e9;

    const double fwd_x = std::cos(s.heading);
    const double fwd_y = std::sin(s.heading);
    const double ex = s.obs_x[pi] - s.x;
    const double ey = s.obs_y[pi] - s.y;
    const double along = ex * fwd_x + ey * fwd_y;
    const double dy = std::fabs(-ex * fwd_y + ey * fwd_x);
    const double vyy = std::fabs(s.obs_vy[pi]);

    /* Guard zone: if pedestrian is crossing (or very close to lane center),
     * keep ego at least this distance behind the crossing line.
     *
     * Two-tier detection:
     *   dy < 3.0m     → always active (pedestrian ON the road, even if stopped)
     *   vyy > 0.05    → pedestrian is actively moving near the road (crossing intent)
     * Otherwise        → pedestrian is parked at curb → NOT crossing, release hold */
    const bool crossing_active = (dy < 3.0) || (vyy > 0.05 && dy < 6.5);
    if (!crossing_active) return 1e9;
    if (along < -2.0 || along > 35.0) return 1e9;

    constexpr double kCrossingBufferM = 6.0;
    return along - kCrossingBufferM;
}

// 同向最近车辆 TTC（沿车头方向，只算正在接近的前车）。返回 1e9 = 无风险。
// out_dx/out_dy 回填最危险候选的沿向距离/横向偏移（无候选时保持 0）。
// 方向感知（2026-08-04 掉头返程同向防撞失效）：旧实现用世界 dx=obs_x-ego_x，
// 返程 ego 向西时前车在 -x（dx<0）被 skip → 同向 TTC 完全失效，返程无防撞。
// 改为沿车头方向投影 ahead + 沿向速度，前进/返程统一。
inline double min_vehicle_ttc(const EgoView& s, double* out_dx = nullptr, double* out_dy = nullptr) {
    double best_ttc = 1e9;
    double best_dx = 0.0;
    double best_dy = 0.0;
    const double fwd_x = std::cos(s.heading);
    const double fwd_y = std::sin(s.heading);
    for (int i = 0; i < s.obs_count; ++i) {
        if (!s.obs_valid[i]) continue;
        const double ex = s.obs_x[i] - s.x;
        const double ey = s.obs_y[i] - s.y;
        const double along = ex * fwd_x + ey * fwd_y;      /* 沿车头前方距离 */
        const double lat = std::fabs(-ex * fwd_y + ey * fwd_x);  /* 横向偏移 */
        if (along < 0.0 || along > 35.0 || lat > 2.3) continue;

        const double along_v = s.obs_v[i] * fwd_x + s.obs_vy[i] * fwd_y;
        const double closing = s.speed - along_v;
        if (closing <= 0.4) continue;

        const double clearance = along - 4.8;
        const double ttc = clearance / std::max(0.1, closing);
        if (ttc < best_ttc) {
            best_ttc = ttc;
            best_dx = along;
            best_dy = lat;
        }
    }
    if (out_dx) *out_dx = best_dx;
    if (out_dy) *out_dy = best_dy;
    return best_ttc;
}

/* Phase 5: 对向来车 TTC.
 * 检查相邻对向车道 (2.0 < |dy| ≤ 6.0m) 是否有迎面驶来的车辆。
 * head-on closing speed = ego_speed + |obs_v_along|, 比同向 closing speed 大得多。
 *
 * |dy| 上界 6.0m ≈ 1.5×标准车道宽：只把**相邻车道**的对向车当迎头威胁。
 * 旧逻辑 |dy|>2.0 把对向任意车道都算上，多车道高速上 ego 在 lane3、对向车
 * 在 lane0（横向 10.5m，中间隔两条车道）也触发 → 巡航被压到 0.5~1.0 全刹
 * （2026-07-31 实跑：合并回 lane2 途中遇 2 车道外对向车刹停到 0）。
 *
 * 方向感知（2026-08-04 掉头返程幽灵刹车）：旧实现用世界 obs_v < -2 判"迎面"，
 * 掉头返程 ego 向西（世界 vx<0）时，**同向车**（世界 vx 也 <0）被误判为迎头 →
 * head-on TTC 用 speed+|obs_v| 硬刹 → 不敢超旁边车道同向车。改为沿车头方向
 * 投影沿向速度 along_v：沿向为负（朝 ego 靠近）才算迎头，同向车沿向恒为正。
 */
static constexpr double kOncomingSameLaneLatMax = 2.3;  /* 同车道横向容差（车宽半幅+余量） */
inline double min_oncoming_ttc(const EgoView& s, double* out_dx = nullptr) {
    double best_ttc = 1e9;
    double best_dx = 0.0;
    const double fwd_x = std::cos(s.heading);
    const double fwd_y = std::sin(s.heading);
    for (int i = 0; i < s.obs_count; ++i) {
        if (!s.obs_valid[i]) continue;
        const double dx = s.obs_x[i] - s.x;
        const double dy = s.obs_y[i] - s.y;
        /* 2026-08-05 同车道头对头修复：只把**同一条车道**、迎面驶来的车当迎头威胁。
         * 旧逻辑 2.0<|dy|≤6.0 把相邻对向车道（间隔 3.5m 的分隔车道）也当迎头 → 掉头
         * 返程（heading≈π 朝西）每次接近东行对向车 head-on TTC 硬刹 = 幽灵刹车。 */
        const double along = dx * fwd_x + dy * fwd_y;
        if (along < 0.0 || along > 60.0) continue;
        const double lat = std::fabs(-dx * fwd_y + dy * fwd_x); /* 横向（车体系投影） */
        if (lat > kOncomingSameLaneLatMax) continue;           /* 只在同车道才算迎头 */
        /* 沿车头方向速度：负 = 朝 ego 驶来（迎头）。同向/静止车沿向恒 ≥ -2，不算迎头 */
        const double along_v = s.obs_v[i] * fwd_x + s.obs_vy[i] * fwd_y;
        if (along_v > -2.0) continue;

        const double closing = s.speed + std::fabs(along_v);
        const double clearance = along - 4.0;  /* 车长余量 */
        const double ttc = clearance / std::max(0.1, closing);
        if (ttc < best_ttc) {
            best_ttc = ttc;
            best_dx = along;
        }
    }
    if (out_dx) *out_dx = best_dx;
    return best_ttc;
}

// 最近横向穿越风险度量（变道时防侧刮）。返回 1e9 = 无风险。
// out_dx 回填沿向距离，out_dy_signed 回填带符号横向偏移（车体系，左正右负）。
// 方向感知（2026-08-04 掉头返程）：旧实现用世界 dx 窗口 [-5,12]，返程时把车后
// 的障碍算进横向穿越风险 → 误刹。改沿车头投影（ahead 窗口 + 横向）。
inline double nearest_vehicle_lateral_cross_risk(const EgoView& s, double* out_dx = nullptr, double* out_dy_signed = nullptr) {
    double best = 1e9;
    double best_dx = 0.0;
    double best_dy_signed = 0.0;
    const double fwd_x = std::cos(s.heading);
    const double fwd_y = std::sin(s.heading);
    for (int i = 0; i < s.obs_count; ++i) {
        if (!s.obs_valid[i]) continue;
        const double ex = s.obs_x[i] - s.x;
        const double ey = s.obs_y[i] - s.y;
        const double along = ex * fwd_x + ey * fwd_y;
        const double dy_signed = -ex * fwd_y + ey * fwd_x;
        const double dy = std::fabs(dy_signed);
        if (along < -5.0 || along > 12.0) continue;
        if (dy > 2.2) continue;
        const double metric = std::fabs(along) + 2.0 * dy;
        if (metric < best) {
            best = metric;
            best_dx = along;
            best_dy_signed = dy_signed;
        }
    }
    if (out_dx) *out_dx = best_dx;
    if (out_dy_signed) *out_dy_signed = best_dy_signed;
    return best;
}

}  // namespace safety
