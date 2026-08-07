// test_stop_light_gate.cpp — 归位/变道目标车道前方红灯闸门纯逻辑核单元测试
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 behavior_planner_node.cpp 的
// lane_ahead_stop_light() 抽到 stop_light_gate.h 后，用确定性单测钉住行为，并把故障表
// 两个历史 bug 编码成回归守卫：
//
//   ① 「超车/归位后立刻在红灯前刹停（无效变道）」：只查**目标车道**前方（横向半车道宽
//      门限），相邻车道的灯不算；目标车道前方有非绿灯 → true（禁止归位）。
//   ② 「掉头返程方向盲」：返程朝 −x，前方的灯 dx<0；必须 on_return 时翻转 dx 才检测得到。
//
// 纯算法单测：只 include stop_light_gate.h（间接带入 road_geometry.h），无框架依赖。
#include <cmath>
#include <cstdio>

#include "stop_light_gate.h"

using behavior::StopLightView;
using behavior::StopLightParams;
using behavior::lane_ahead_stop_light;

static int g_checks = 0;
#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        return 1; \
    } \
} while (0)

// 4 车道 / 车道宽 3.5：lane_center_y(idx,4,3.5,0,0) = {0:+5.25, 1:+1.75, 2:−1.75, 3:−5.25}
// 目标车道统一取 idx=1（lane_c=+1.75）。灯缓存最多 4 槽。
struct Lights {
    double x[4]{}, y[4]{};
    int    state[4]{};  // 0=green 非0=红/黄
    int    count = 0;
};

static StopLightView make_view(const Lights& L, double ego_x, double ego_v, bool on_return) {
    StopLightView s;
    s.ego_x = ego_x;
    s.ego_v = ego_v;
    s.on_return = on_return;
    s.tl_x = L.x;
    s.tl_y_lane = L.y;
    s.tl_state = L.state;
    s.tl_count = L.count;
    s.has_traffic_lights = (L.count > 0);
    s.lane_count = 4;
    s.lane_width = 3.5;
    return s;
}

int main(void) {
    StopLightParams p;  // 默认：decel=8, +3+20, floor 60, 半车道宽 0.5

    // ── 1. 无灯 → false ──
    {
        Lights L;  // count=0
        StopLightView s = make_view(L, 100.0, 0.0, false);
        s.has_traffic_lights = false;
        CHECK(lane_ahead_stop_light(1, s, p) == false);
    }

    // ── 2. 目标车道前方是绿灯(state=0) → false ──
    {
        Lights L; L.count = 1; L.x[0] = 150.0; L.y[0] = 1.75; L.state[0] = 0;
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == false);
    }

    // ── 3. 目标车道前方红灯、在 range 内 → true（禁止归位）──
    {
        Lights L; L.count = 1; L.x[0] = 150.0; L.y[0] = 1.75; L.state[0] = 2;  // dx=50 ≤ 60
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == true);
    }

    // ── 4. 红灯在相邻车道(idx=2, y=−1.75，横向差 3.5 > 半车道宽 1.75) → false(bug① 只查目标车道) ──
    {
        Lights L; L.count = 1; L.x[0] = 150.0; L.y[0] = -1.75; L.state[0] = 2;
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == false);
    }

    // ── 5. 红灯在身后(前进系 dx≤0) → false ──
    {
        Lights L; L.count = 1; L.x[0] = 50.0; L.y[0] = 1.75; L.state[0] = 2;  // dx=−50
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == false);
    }

    // ── 6. 红灯超出 stop_range → false ──
    {
        Lights L; L.count = 1; L.x[0] = 170.0; L.y[0] = 1.75; L.state[0] = 2;  // dx=70 > 60(v=0)
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == false);
    }

    // ── 7. 高速时 stop_range 随 v² 增大：dx=200 在 v=40 时应命中(200<223) ──
    {
        Lights L; L.count = 1; L.x[0] = 300.0; L.y[0] = 1.75; L.state[0] = 2;  // dx=200
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 40.0, false), p) == true);
        // 同一灯在低速(v=0, range=60)时超出 → false
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == false);
    }

    // ── 8. 下限 60m：v=0 时刹车距离仅 23，但 dx=55 仍应命中(floor 60) ──
    {
        Lights L; L.count = 1; L.x[0] = 155.0; L.y[0] = 1.75; L.state[0] = 2;  // dx=55
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == true);
    }

    // ── 9. 掉头返程：前方(−x)的红灯 dx<0，翻转后命中 → true(bug② 方向修复) ──
    {
        Lights L; L.count = 1; L.x[0] = 50.0; L.y[0] = 1.75; L.state[0] = 2;  // 返程前方
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, true), p) == true);
        // 对照：同一灯前进系(on_return=false) 在身后 → false（证明翻转确有必要）
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == false);
    }

    // ── 10. 掉头返程：身后(+x)的红灯翻转后 dx≤0 → false ──
    {
        Lights L; L.count = 1; L.x[0] = 150.0; L.y[0] = 1.75; L.state[0] = 2;
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, true), p) == false);
    }

    // ── 11. 多灯混合：绿灯 + 相邻车道红灯 + 目标车道远红灯，只有目标车道近红灯命中 ──
    {
        Lights L; L.count = 3;
        L.x[0] = 140.0; L.y[0] = 1.75;  L.state[0] = 0;  // 目标车道绿灯 → 跳过
        L.x[1] = 130.0; L.y[1] = -1.75; L.state[1] = 2;  // 相邻车道红灯 → 跳过
        L.x[2] = 145.0; L.y[2] = 1.75;  L.state[2] = 2;  // 目标车道红灯 dx=45 → 命中
        CHECK(lane_ahead_stop_light(1, make_view(L, 100.0, 0.0, false), p) == true);
    }

    std::printf("test_stop_light_gate: all %d checks passed\n", g_checks);
    return 0;
}
