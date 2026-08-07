// test_oncoming_yield.cpp — 会车让行 + 窄路减速纯逻辑核单元测试
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 planning_node.cpp
// Phase 5 的会车让行 + 窄路减速抽到 oncoming_yield.h 后，用确定性单测钉住行为，
// 并把故障表里那条恶性 bug 编码成回归守卫：
//
//   历史 bug（掉头返程幽灵刹车 / 会车过度保守）：
//   ① 世界 dx 窗口 (-10,80] 预筛 → 掉头返程朝 -x 时前方对向车 dx<-10 被跳过
//      （方向盲，无对向保护）。修复：沿车头方向投影 along=rx·cos h+ry·sin h。
//   ② `|dy|≤1.5×路宽` 把相邻对向车道（3.5m 外）也当迎头 → 巡航压到 0.4× 全刹。
//      修复：横向上界 0.65×路宽，只认同车道头对头。
//   ③ 世界 obs_v<-2 判"迎面" → 返程同向车（vx 也<0）被误判迎头 → 幽灵刹车。
//      修复：沿向相对速度 rel_v=vx·cos h+vy·sin h。
//   前进方向（heading=0）三处投影均退化为原世界坐标 → 既有前进场景零回归。
//
// 纯算法单测：只 include oncoming_yield.h，无框架依赖。
#include <cmath>
#include <cstdio>

#include "oncoming_yield.h"

using planning::YieldView;
using planning::YieldParams;
using planning::YieldDecision;
using planning::oncoming_yield;

static int g_checks = 0;
#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        return 1; \
    } \
} while (0)

static bool near_eq(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

static constexpr double PI = 3.14159265358979323846;

// 障碍布景：世界坐标 + 世界速度，N 槽，未用槽保持 (0,0,0,0)。
struct Scene {
    static constexpr int N = 4;
    double ox[N], oy[N], ovx[N], ovy[N];
    Scene() { for (int i = 0; i < N; ++i) { ox[i] = oy[i] = ovx[i] = ovy[i] = 0.0; } }
    void set(int i, double x, double y, double vx, double vy) {
        ox[i] = x; oy[i] = y; ovx[i] = vx; ovy[i] = vy;
    }
    YieldView view(double ex, double ey, double heading, double target,
                   double cmd, double lane, int count) {
        YieldView v;
        v.x = ex; v.y = ey; v.heading = heading;
        v.target_speed = target; v.command_speed = cmd; v.lane_width = lane;
        v.obs_x = ox; v.obs_y = oy; v.obs_vx = ovx; v.obs_vy = ovy;
        v.obs_count = count;
        return v;
    }
};

int main(void) {
    YieldParams p;  // 默认：lat_ratio=0.65, range=60, rel_v=-2, yield=0.4, narrow<1.5
    // 路宽 3.5 → oncoming_lat_max = 0.65×3.5 = 2.275；target=10 → yield_speed = 4.0

    // ── 1. 前进(heading=0)：同车道正前方对向车迎面驶来 → 会车让行降速 ──
    {
        Scene sc;
        sc.set(0, /*world*/ 30.0, 0.0, /*vx*/ -5.0, 0.0);  // 前方 30m，逆向靠近
        YieldView v = sc.view(/*ex*/ 0.0, 0.0, /*h*/ 0.0, /*target*/ 10.0,
                              /*cmd*/ 10.0, /*lane*/ 3.5, /*count*/ 1);
        YieldDecision d = oncoming_yield(v, p);
        CHECK(d.oncoming == true);
        CHECK(near_eq(d.command_speed, 4.0));  // 10 → 0.4×10
    }

    // ── 2. 回归(幽灵刹车修复)：相邻对向车道(横向 3.5m > 2.275)不算迎头 ──
    // 旧 bug（|dy|≤1.5×3.5=5.25）会把它当威胁 → 全刹；修复后 command_speed 不变。
    {
        Scene sc;
        sc.set(0, 30.0, 3.5, -5.0, 0.0);  // 前方 30m、横向 3.5m（隔壁车道）
        YieldView v = sc.view(0.0, 0.0, 0.0, 10.0, 10.0, 3.5, 1);
        YieldDecision d = oncoming_yield(v, p);
        CHECK(d.oncoming == false);
        CHECK(near_eq(d.command_speed, 10.0));  // 不降速
    }

    // ── 3. 回归(方向盲修复)：掉头返程(heading=π)前方对向车在世界 -x ──
    // ego 在 (100,0) 朝 -x 行驶，对向车在世界 (70,0) 迎面(+x)驶来。
    // 世界 dx=-30<0，旧 dx 预筛跳过 → 无对向保护；方向投影后 along=+30 前方 → 命中。
    {
        Scene sc;
        sc.set(0, 70.0, 0.0, /*vx*/ +5.0, 0.0);  // 世界朝 +x（迎着返程 ego）
        YieldView v = sc.view(/*ex*/ 100.0, 0.0, /*h*/ PI, 10.0, 10.0, 3.5, 1);
        YieldDecision d = oncoming_yield(v, p);
        CHECK(d.oncoming == true);              // 方向投影后正确识别（bug 时为 false）
        CHECK(near_eq(d.command_speed, 4.0));
    }

    // ── 4. 回归(同向误判修复)：返程同向前车(世界 vx 也<0)不是迎头 ──
    // 旧 world obs_v<-2 判迎面 → 同向车 vx=-5<-2 被误判 → 幽灵刹车。
    // 方向投影 rel_v = -5·cos π = +5 > -2 → 正确不让行。
    {
        Scene sc;
        sc.set(0, 70.0, 0.0, /*vx*/ -5.0, 0.0);  // 世界朝 -x（与返程 ego 同向）
        YieldView v = sc.view(100.0, 0.0, PI, 10.0, 10.0, 3.5, 1);
        YieldDecision d = oncoming_yield(v, p);
        CHECK(d.oncoming == false);            // 同向不让行（bug 时误判 true → 全刹）
        CHECK(near_eq(d.command_speed, 10.0));
    }

    // ── 5. 前进同向前车(远离)不算迎头 ──
    {
        Scene sc;
        sc.set(0, 30.0, 0.0, /*vx*/ +5.0, 0.0);  // 前方同向、更快，远离
        YieldView v = sc.view(0.0, 0.0, 0.0, 10.0, 10.0, 3.5, 1);
        YieldDecision d = oncoming_yield(v, p);
        CHECK(d.oncoming == false);            // rel_v=+5 > -2
        CHECK(near_eq(d.command_speed, 10.0));
    }

    // ── 6. 窄路减速：左右两侧 20m 内障碍，间距和 < 1.5m → 降速 ──
    // 左 +0.6、右 -0.6 → narrow_width=1.2；ratio=(1.2-0.3)/1.2=0.75 → 10×0.75=7.5。
    {
        Scene sc;
        sc.set(0, 10.0,  0.6, 0.0, 0.0);  // 左侧（lat>0）
        sc.set(1, 10.0, -0.6, 0.0, 0.0);  // 右侧（lat<0）
        YieldView v = sc.view(0.0, 0.0, 0.0, 10.0, 10.0, 3.5, 2);
        YieldDecision d = oncoming_yield(v, p);
        CHECK(d.oncoming == false);
        CHECK(near_eq(d.min_clearance_left, 0.6));
        CHECK(near_eq(d.min_clearance_right, 0.6));
        CHECK(near_eq(d.narrow_width, 1.2));
        CHECK(near_eq(d.command_speed, 7.5));
    }

    // ── 7. 空槽惰性 + 无威胁：传满 N 槽(尾部清零)，ego 离原点，command_speed 不变 ──
    // 复刻真实调用（obs_count=kMaxObs，尾槽 (0,0,0,0)）。空槽 rel_v=0 恒不触 oncoming，
    // 且落在检测窗外，不污染决策。
    {
        Scene sc;  // 全零
        YieldView v = sc.view(/*ex*/ 100.0, 0.0, 0.0, 10.0, 10.0, 3.5, /*count*/ Scene::N);
        YieldDecision d = oncoming_yield(v, p);
        CHECK(d.oncoming == false);
        CHECK(near_eq(d.command_speed, 10.0));
        CHECK(d.narrow_width > 1e8);           // 无有效障碍
    }

    std::printf("test_oncoming_yield: all %d checks passed\n", g_checks);
    return 0;
}
