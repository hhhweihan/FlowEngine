// test_overtake_decision.cpp — 变道超车值得性判定纯逻辑核单元测试
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 behavior_planner_node.cpp
// 的超车判定抽到 overtake_decision.h 后，用确定性单测钉住行为，并把故障表里那条
// 恶性反逻辑 bug 编码成回归守卫：
//
//   `worthwhile = blocked && (best_gap < min_gap)`  —— 写反了
//   语义变成"目标车道 gap 越小越值得超车"，前方越挤越想变道 → 高速接近下来不及
//   → behavior 一直不进 FOLLOW/超车。正确是 `best_gap > min_gap`（gap 够大才值得）。
//
// 纯算法单测：只 include overtake_decision.h，无框架依赖。
#include <cmath>
#include <cstdio>

#include "overtake_decision.h"

using behavior::OvertakeParams;
using behavior::OvertakeDecision;
using behavior::overtake_decision;

static int g_checks = 0;
#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        return 1; \
    } \
} while (0)

int main(void) {
    // 典型配置（= behavior 默认量级）：base=25m, mult=2.0, cap=60m。
    OvertakeParams p;
    p.blocked_range_min = 40.0;
    p.blocked_range_mult = 3.0;
    p.follow_hysteresis = 1.5;
    p.min_overtake_gap_base = 25.0;
    p.min_overtake_gap_speed_mult = 2.0;
    p.min_overtake_gap_cap = 60.0;

    const double desired_gap = 20.0;  // → blocked_range = max(40, 60) = 60

    // ── 1. 被堵 + 目标车道空旷 → worthwhile（这才是超车的正确触发）──
    // best_gap=10（<60 被堵），本车道前车很近；min_gap = 25 + (12-4)*2 = 41。
    // 注意：这里 best_gap 是"本车道前车净距"，min_gap 判的也是本车道 gap 是否值得
    // ——两者同源。best_gap=10 < min_gap=41 → 按修复逻辑 worthwhile=false。
    // 用一个 gap 够大的被堵场景才触发（见 2）。
    {
        OvertakeDecision d = overtake_decision(10.0, desired_gap, false,
                                               /*ego_v=*/12.0, /*lead=*/4.0, p);
        CHECK(d.blocked == true);                 // 10 < 60
        CHECK(std::fabs(d.rel_speed - 8.0) < 1e-9);
        CHECK(std::fabs(d.min_gap - 41.0) < 1e-9);// 25 + 8*2 = 41
        CHECK(d.worthwhile == false);             // 10 > 41 假 → 不值得
    }

    // ── 2. 回归：被堵 且 gap 够大 → worthwhile=true（bug 时会翻成 false）──
    // best_gap=50（<60 仍算被堵，前车在中距），min_gap=41 → 50 > 41 → 值得。
    // 反逻辑 bug（best_gap < min_gap）下 50<41 假 → worthwhile=false，永不超车。
    {
        OvertakeDecision d = overtake_decision(50.0, desired_gap, false,
                                               12.0, 4.0, p);
        CHECK(d.blocked == true);                 // 50 < 60
        CHECK(std::fabs(d.min_gap - 41.0) < 1e-9);
        CHECK(d.worthwhile == true);              // 50 > 41 真（bug 时为 false）
    }

    // ── 3. 未被堵（前方空旷）→ 无论 gap 大小都不 worthwhile ──
    // best_gap=100 > blocked_range=60 → blocked=false → worthwhile=false。
    {
        OvertakeDecision d = overtake_decision(100.0, desired_gap, false,
                                               12.0, 4.0, p);
        CHECK(d.blocked == false);
        CHECK(d.worthwhile == false);
    }

    // ── 4. min_gap 上限夹紧 ──
    // 极大相对速度 → base + rel*mult 超 cap → 被夹到 cap=60。
    {
        OvertakeDecision d = overtake_decision(55.0, desired_gap, false,
                                               /*ego_v=*/40.0, /*lead=*/0.0, p);
        CHECK(std::fabs(d.min_gap - 60.0) < 1e-9); // 25+40*2=105 → cap 60
        CHECK(d.worthwhile == false);              // 55 > 60 假
    }

    // ── 5. rel_speed 负截零（前车比 ego 快，不追）──
    {
        OvertakeDecision d = overtake_decision(50.0, desired_gap, false,
                                               /*ego_v=*/5.0, /*lead=*/20.0, p);
        CHECK(std::fabs(d.rel_speed - 0.0) < 1e-9);
        CHECK(std::fabs(d.min_gap - 25.0) < 1e-9); // 25 + 0 = 25
    }

    // ── 6. FOLLOW 滞环：退出距离放大（进紧出松）──
    // in_follow=true → blocked_range × hysteresis = 60×1.5 = 90。
    // best_gap=75：非 FOLLOW 时 75>60 不算被堵；FOLLOW 中 75<90 仍算被堵（滞环保持）。
    {
        OvertakeDecision d_enter = overtake_decision(75.0, desired_gap, /*in_follow=*/false,
                                                     12.0, 4.0, p);
        CHECK(d_enter.blocked == false);          // 75 > 60
        OvertakeDecision d_hold = overtake_decision(75.0, desired_gap, /*in_follow=*/true,
                                                    12.0, 4.0, p);
        CHECK(d_hold.blocked == true);            // 75 < 90（滞环退出松）
    }

    std::printf("test_overtake_decision: all %d checks passed\n", g_checks);
    return 0;
}
