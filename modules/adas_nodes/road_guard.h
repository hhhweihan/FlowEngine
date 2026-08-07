// road_guard.h — 全域速度死锁恢复 + ROAD_GUARD 强制回正纯逻辑核（control 抽核）
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 control_node.cpp 主循环里
// 「SPEED_ZERO_RECOVERY + ROAD_GUARD」两段执行权覆盖决策抽出，用确定性单测钉住行为，
// 并把故障表里三个历史 bug 编码成回归守卫：
//
//   ① 「车速降到 0 后永久卡死」：旧 ROAD_GUARD 低速恢复要求 |y|≥road_center_limit，
//      但车可在 2.1<|y|<2.5 停下永不满足 → 卡死。修复：全域 speed_zero 计时器一到点
//      就给小油门（独立于 y，只要在目标车道阈值内 + 有轨迹 + 目标速度>0）。
//   ② 「掉头返程 ROAD_GUARD 把车拽出路面」：旧实现用世界系 lat_error 定 steer 符号，
//      假设车头恒朝 +x，返程(heading=π)时打反 → 车持续北漂越漂越远。修复：用参考系
//      投影误差 lat_err_n（正=左打，前进/返程都成立）。
//   ③ 「ROAD_GUARD 低速仍刹车 → 与死锁互锁」：低速(<2.5)时给小油门(0.18)而非刹车。
//
// 两分支互斥：SPEED_ZERO 要求 y_from_target ≤ 阈值，ROAD_GUARD 要求 > 阈值，同帧至多一个触发。
//
// header-only 纯逻辑核，无框架依赖：steer 限幅（依赖 g.wheelbase 的 steer_limit_for_speed）
// 由调用方算好作标量传入，速度死锁计时器 / prev_steer 由调用方按返回标志应用。
#pragma once

#include <cmath>

namespace control {

struct RoadGuardParams {
    double speed_zero_recover_s = 5.0;    // SPEED_ZERO_RECOVER_S：速度死锁触发秒数
    double road_guard_threshold_m = 3.0;  // ROAD_GUARD_THRESHOLD_M：偏离目标车道中心上限
    double low_speed_mps = 2.5;           // ROAD_GUARD 低速给油阈值
    double recovery_throttle = 0.15;      // SPEED_ZERO_RECOVERY 油门
    double guard_low_throttle = 0.18;     // ROAD_GUARD 低速给油
    double guard_brake = 0.65;            // ROAD_GUARD 高速刹车下限
    double min_target_speed = 1.0;        // recovery 要求 target_speed >
};

struct RoadGuardIn {
    double speed_zero_timer = 0.0;  // 速度持续近 0 的累计秒数
    double y_from_target = 0.0;     // |ego_y − 目标车道中心|
    bool   has_planning = false;
    double target_speed = 0.0;      // 轨迹末点目标速度
    double current_speed = 0.0;
    double lat_err_n = 0.0;         // 参考系投影横向误差（正=在目标线左侧，需右打回正）
    bool   maneuver_mode = false;   // 机动期（掉头/泊车）豁免 ROAD_GUARD
    double steer_limit = 0.0;       // = steer_limit_for_speed(|speed|, 2.4)
    // 当前控制量：未触发任何分支时原样返回
    double throttle_in = 0.0;
    double brake_in = 0.0;
    double steer_in = 0.0;
};

enum class RoadGuardMode { NONE, SPEED_ZERO_RECOVERY, ROAD_GUARD };

struct RoadGuardOut {
    double throttle = 0.0;
    double brake = 0.0;
    double steer = 0.0;
    RoadGuardMode mode = RoadGuardMode::NONE;
    bool reset_speed_zero_timer = false;  // 调用方据此清零 g.speed_zero_timer
    bool update_prev_steer = false;       // 调用方据此 g.prev_steer = out.steer
};

// 逐行等价于 control_node.cpp SPEED_ZERO_RECOVERY(958-967) + ROAD_GUARD(969-991) 两块。
inline RoadGuardOut road_guard_decide(const RoadGuardIn& in, const RoadGuardParams& p) {
    RoadGuardOut o;
    o.throttle = in.throttle_in;
    o.brake    = in.brake_in;
    o.steer    = in.steer_in;

    // ── 全域速度死锁恢复：无论 y 位置，速度死锁到点就给小油门（bug① 修复）──
    if (in.speed_zero_timer > p.speed_zero_recover_s &&
        in.y_from_target <= p.road_guard_threshold_m &&
        in.has_planning && in.target_speed > p.min_target_speed) {
        o.throttle = p.recovery_throttle;
        o.brake    = 0.0;
        o.mode     = RoadGuardMode::SPEED_ZERO_RECOVERY;
        o.reset_speed_zero_timer = true;
    }

    // ── ROAD_GUARD：偏离目标车道中心过远强制回正（与上互斥）──
    if (in.y_from_target > p.road_guard_threshold_m && !in.maneuver_mode) {
        // 参考系投影误差定 steer 符号：正误差=左打，前进/返程都成立（bug② 修复）
        o.steer = (in.lat_err_n > 0.0) ? in.steer_limit : -in.steer_limit;
        if (std::fabs(in.current_speed) < p.low_speed_mps) {
            o.throttle = p.guard_low_throttle;  // 低速给油而非刹车（bug③ 修复）
            o.brake    = 0.0;
            o.reset_speed_zero_timer = true;
        } else {
            o.throttle = 0.0;
            if (o.brake < p.guard_brake) o.brake = p.guard_brake;
        }
        o.mode = RoadGuardMode::ROAD_GUARD;
        o.update_prev_steer = true;
    }

    return o;
}

}  // namespace control
