// test_road_guard.cpp — 速度死锁恢复 + ROAD_GUARD 强制回正纯逻辑核单元测试
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 control_node.cpp 的
// SPEED_ZERO_RECOVERY + ROAD_GUARD 两块抽到 road_guard.h 后，用确定性单测钉住行为，
// 并把故障表里三个历史 bug 编码成回归守卫：
//
//   ① 「车速降到 0 后永久卡死」：恢复油门必须独立于 y 位置（只要在目标车道阈值内），
//      不能再要求 |y|≥road_center_limit（车可在 2.1<|y|<2.5 停下永不满足）。
//   ② 「掉头返程 ROAD_GUARD 把车拽出路面」：steer 符号由参考系投影误差 lat_err_n 定，
//      不能用世界系 lat_error（返程 heading=π 时打反 → 越漂越远）。
//   ③ 「ROAD_GUARD 低速仍刹车 → 与死锁互锁」：低速(<2.5) 给小油门而非刹车。
//
// 纯算法单测：只 include road_guard.h，无框架依赖。
#include <cmath>
#include <cstdio>

#include "road_guard.h"

using control::RoadGuardParams;
using control::RoadGuardIn;
using control::RoadGuardOut;
using control::RoadGuardMode;
using control::road_guard_decide;

static int g_checks = 0;
#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        return 1; \
    } \
} while (0)

static bool near_eq(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

// 构造一个"什么都不触发"的基线输入：小计时器、在车道中心、正常速度。
static RoadGuardIn base_in() {
    RoadGuardIn in;
    in.speed_zero_timer = 0.0;
    in.y_from_target = 0.5;
    in.has_planning = true;
    in.target_speed = 10.0;
    in.current_speed = 8.0;
    in.lat_err_n = 0.0;
    in.maneuver_mode = false;
    in.steer_limit = 0.10;
    in.throttle_in = 0.3;
    in.brake_in = 0.0;
    in.steer_in = 0.05;
    return in;
}

int main(void) {
    RoadGuardParams p;  // 默认：recover_s=5, threshold=3, low_speed=2.5, ...

    // ── 1. 两分支都不触发 → 控制量原样透传，mode=NONE ──
    {
        RoadGuardIn in = base_in();
        RoadGuardOut o = road_guard_decide(in, p);
        CHECK(o.mode == RoadGuardMode::NONE);
        CHECK(near_eq(o.throttle, 0.3));
        CHECK(near_eq(o.brake, 0.0));
        CHECK(near_eq(o.steer, 0.05));
        CHECK(o.reset_speed_zero_timer == false);
        CHECK(o.update_prev_steer == false);
    }

    // ── 2. 速度死锁恢复：计时到点 + 在目标车道内(y≤3) → 给小油门(bug① 修复) ──
    // 关键：y_from_target=2.1（车没偏多远，旧 |y|≥limit 逻辑不会触发恢复 → 卡死）。
    {
        RoadGuardIn in = base_in();
        in.speed_zero_timer = 6.0;  // > 5
        in.y_from_target = 2.1;     // ≤ 3，但旧逻辑要求 |y| 很大才恢复
        in.current_speed = 0.0;
        RoadGuardOut o = road_guard_decide(in, p);
        CHECK(o.mode == RoadGuardMode::SPEED_ZERO_RECOVERY);
        CHECK(near_eq(o.throttle, 0.15));
        CHECK(near_eq(o.brake, 0.0));
        CHECK(near_eq(o.steer, 0.05));            // steer 不动
        CHECK(o.reset_speed_zero_timer == true);
    }

    // ── 2b. 恢复门槛：无轨迹 / 目标速度≤1 → 不恢复（防止无目标时乱给油）──
    {
        RoadGuardIn in = base_in();
        in.speed_zero_timer = 6.0; in.y_from_target = 2.1; in.current_speed = 0.0;
        in.has_planning = false;
        CHECK(road_guard_decide(in, p).mode == RoadGuardMode::NONE);
        in.has_planning = true; in.target_speed = 0.5;  // ≤ 1
        CHECK(road_guard_decide(in, p).mode == RoadGuardMode::NONE);
    }

    // ── 3. ROAD_GUARD 低速：偏离>3 且速度<2.5 → 给小油门 0.18 而非刹车(bug③ 修复) ──
    {
        RoadGuardIn in = base_in();
        in.y_from_target = 4.0;   // > 3
        in.current_speed = 1.0;   // < 2.5
        in.lat_err_n = 0.5;       // 正 → 需右打回正 → +limit
        in.brake_in = 0.4;
        RoadGuardOut o = road_guard_decide(in, p);
        CHECK(o.mode == RoadGuardMode::ROAD_GUARD);
        CHECK(near_eq(o.throttle, 0.18));         // 低速给油
        CHECK(near_eq(o.brake, 0.0));
        CHECK(near_eq(o.steer, 0.10));            // +steer_limit
        CHECK(o.reset_speed_zero_timer == true);
        CHECK(o.update_prev_steer == true);
    }

    // ── 4. ROAD_GUARD 高速：偏离>3 且速度>2.5 → 刹车(≥0.65)，不给油 ──
    {
        RoadGuardIn in = base_in();
        in.y_from_target = 4.0;
        in.current_speed = 8.0;   // > 2.5
        in.lat_err_n = 0.5;
        in.brake_in = 0.3;        // < 0.65 → 抬到 0.65
        RoadGuardOut o = road_guard_decide(in, p);
        CHECK(o.mode == RoadGuardMode::ROAD_GUARD);
        CHECK(near_eq(o.throttle, 0.0));
        CHECK(near_eq(o.brake, 0.65));
        CHECK(o.reset_speed_zero_timer == false); // 高速分支不清计时器
    }

    // ── 4b. 高速已有更大刹车 → 不下调 ──
    {
        RoadGuardIn in = base_in();
        in.y_from_target = 4.0; in.current_speed = 8.0; in.lat_err_n = 0.5;
        in.brake_in = 0.9;
        CHECK(near_eq(road_guard_decide(in, p).brake, 0.9));
    }

    // ── 5. 回归(方向盲修复)：steer 符号跟随 lat_err_n，两侧对称 ──
    // 正误差(在目标线左侧)→ 右打回正 = +limit；负误差 → -limit。
    // 世界系 lat_error bug 会让返程 heading=π 时打反；这里用参考系 lat_err_n 恒对。
    {
        RoadGuardIn in = base_in();
        in.y_from_target = 4.0; in.current_speed = 1.0;
        in.lat_err_n = +0.5;
        CHECK(near_eq(road_guard_decide(in, p).steer, +0.10));
        in.lat_err_n = -0.5;
        CHECK(near_eq(road_guard_decide(in, p).steer, -0.10));
    }

    // ── 6. 机动期豁免：偏离>3 但 maneuver_mode → ROAD_GUARD 不夺权 ──
    // 掉头横穿整条路 y_from_target 必然大，不豁免则永远完不成掉头。
    {
        RoadGuardIn in = base_in();
        in.y_from_target = 5.0;   // 远偏
        in.maneuver_mode = true;
        RoadGuardOut o = road_guard_decide(in, p);
        CHECK(o.mode == RoadGuardMode::NONE);     // 不触发
        CHECK(near_eq(o.steer, 0.05));            // steer 原样
        CHECK(near_eq(o.throttle, 0.3));
    }

    // ── 7. 互斥性：恢复(y≤3) 与 ROAD_GUARD(y>3) 不会同帧都触发 ──
    {
        RoadGuardIn in = base_in();
        in.speed_zero_timer = 6.0; in.y_from_target = 2.9; in.current_speed = 0.0;
        RoadGuardOut o = road_guard_decide(in, p);
        CHECK(o.mode == RoadGuardMode::SPEED_ZERO_RECOVERY);  // 只走恢复，y≤3
    }

    std::printf("test_road_guard: all %d checks passed\n", g_checks);
    return 0;
}
