// test_safety_geometry.cpp — 安全层近场几何纯逻辑核单元测试
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 safety_control_node.cpp
// 的 6 个近场风险度量抽到 safety_geometry.h 后，用确定性单测钉住行为，并把故障表
// 里最凶的一条编码成回归守卫——「掉头返程幽灵刹车 + 无同向防撞（撞旁边车）」：
//
//   根因：这几个函数**系统性用世界 +x 坐标**判前后。ego 前进（朝东 heading=0）时
//   没问题，但掉头返程（朝西 heading≈π）时：
//     · 同向前车在世界 -x（dx<0）→ 旧 min_vehicle_ttc/nearest_same_lane_gap 跳过
//       → 同向防撞完全失效 → 追尾；
//     · 同向车世界 vx<0 → 旧 min_oncoming_ttc 用 obs_v<-2 判"迎面" → 误判 head-on
//       → 无谓硬刹（幽灵刹车）。
//   修复：全部改沿车头方向投影（along = ex·cos h + ey·sin h、沿向速度 along_v）。
//
// 本测试对每个函数验证两侧：
//   (A) heading=0（前进/朝东）→ 沿车头投影退化为原世界坐标，对既有场景零回归；
//   (B) heading=π（掉头返程/朝西）→ 方向正确（该抓的抓到、该放的放行）。
//
// 纯算法单测：只 include safety_geometry.h，无 transport/param/framework 依赖。
#include <cmath>
#include <cstdio>

#include "safety_geometry.h"

using safety::EgoView;

static int g_checks = 0;
#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        return 1; \
    } \
} while (0)

static bool near_eq(double a, double b, double eps) { return std::fabs(a - b) < eps; }

// 小容量障碍存储 + EgoView 构造器（测试持有真实数组，视图只读）。
struct Scene {
    static constexpr int N = 4;
    double ox[N]{}, oy[N]{}, ov[N]{}, ovy[N]{};
    bool valid[N]{};

    void set(int i, double x, double y, double vx, double vy) {
        ox[i] = x; oy[i] = y; ov[i] = vx; ovy[i] = vy; valid[i] = true;
    }
    EgoView view(double ex, double ey, double speed, double heading, int ped = -1) {
        EgoView v;
        v.x = ex; v.y = ey; v.speed = speed; v.heading = heading;
        v.obs_x = ox; v.obs_y = oy; v.obs_v = ov; v.obs_vy = ovy;
        v.obs_valid = valid; v.obs_count = N; v.ped_index = ped;
        return v;
    }
};

static constexpr double PI = 3.14159265358979323846;

int main(void) {
    const double tol = 2.0;  // same_lane_tol（SafetyParams 默认）

    // ── 1. nearest_same_lane_gap 前进（heading=0）零回归 ──
    // 前车 +x=20 同车道 → gap=20-4.6=15.4；后车 -x=20 必须忽略。
    {
        Scene s;
        s.set(0, 20.0, 0.0, 0.0, 0.0);   // 前方
        s.set(1, -20.0, 0.0, 0.0, 0.0);  // 后方（应忽略）
        double gap = safety::nearest_same_lane_gap(s.view(0, 0, 10, 0.0), tol);
        CHECK(near_eq(gap, 15.4, 1e-6));
    }

    // ── 2. nearest_same_lane_gap 掉头返程（heading=π）方向正确 ──
    // ego 朝西：同向前车在世界 -x=-20 → 沿车头 along=20 → gap=15.4（旧世界+x代码
    // 会 skip 返回 1e9 → 返程跟车失效）；车东侧 +x=20（现为后方）必须忽略。
    {
        Scene s;
        s.set(0, -20.0, 0.0, 0.0, 0.0);  // 返程前方（世界 -x）
        s.set(1, 20.0, 0.0, 0.0, 0.0);   // 返程后方（世界 +x，应忽略）
        double gap = safety::nearest_same_lane_gap(s.view(0, 0, 10, PI), tol);
        CHECK(near_eq(gap, 15.4, 1e-4));      // fix：抓到返程前车
        CHECK(gap < 1e8);                     // bug 时恒 1e9
    }

    // ── 3. min_vehicle_ttc 前进零回归 ──
    // ego v=10，前车 x=15 慢速 v=2 → closing=8, clearance=10.2, ttc=1.275。
    {
        Scene s;
        s.set(0, 15.0, 0.0, 2.0, 0.0);
        double dx = 0, dy = 0;
        double ttc = safety::min_vehicle_ttc(s.view(0, 0, 10, 0.0), &dx, &dy);
        CHECK(near_eq(ttc, 1.275, 1e-3));
        CHECK(near_eq(dx, 15.0, 1e-6));
    }

    // ── 4. min_vehicle_ttc 掉头返程同向防撞（回归 A：追尾修复）──
    // ego 朝西 v=10，同向前车世界 x=-15、世界 vx=-2（也朝西）→ 沿车头 along=15、
    // along_v=+2 → closing=8 → ttc=1.275。旧世界代码 dx=-15<0 被 skip → 1e9（返程无
    // 防撞，撞 entity14）。
    {
        Scene s;
        s.set(0, -15.0, 0.0, -2.0, 0.0);
        double dx = 0, dy = 0;
        double ttc = safety::min_vehicle_ttc(s.view(0, 0, 10, PI), &dx, &dy);
        CHECK(near_eq(ttc, 1.275, 1e-3));     // fix：返程也算同向 TTC
        CHECK(ttc < 1e8);                     // bug 时 1e9
    }

    // ── 5. min_oncoming_ttc 前进零回归 ──
    // ego v=10 朝东，真迎面车 x=30、世界 vx=-8（朝西冲 ego）→ along_v=-8<-2 → 迎头，
    // closing=18, clearance=26, ttc≈1.444 < 4。
    {
        Scene s;
        s.set(0, 30.0, 0.0, -8.0, 0.0);
        double dx = 0;
        double ttc = safety::min_oncoming_ttc(s.view(0, 0, 10, 0.0), &dx);
        CHECK(near_eq(ttc, 26.0 / 18.0, 1e-3));
        CHECK(ttc < 4.0);
    }

    // ── 6. min_oncoming_ttc 掉头返程幽灵刹车（回归 B：不误刹同向车）──
    // ego 朝西 v=10，同向车世界 x=-30、世界 vx=-8（也朝西）→ 沿车头 along_v=+8>-2 →
    // 非迎头 → 1e9（不刹）。旧世界代码 obs_v=-8<-2 → 误判迎头 head-on 硬刹 = 幽灵刹车。
    {
        Scene s;
        s.set(0, -30.0, 0.0, -8.0, 0.0);
        double dx = 0;
        double ttc = safety::min_oncoming_ttc(s.view(0, 0, 10, PI), &dx);
        CHECK(ttc > 1e8);                     // fix：同向车不算迎头（bug 时 ~1.44 硬刹）
    }

    // ── 6b. min_oncoming_ttc 掉头返程仍抓真迎头 ──
    // ego 朝西 v=10，真迎面车世界 x=-30、世界 vx=+8（朝东冲 ego）→ along_v=-8<-2 → 迎头。
    {
        Scene s;
        s.set(0, -30.0, 0.0, 8.0, 0.0);
        double dx = 0;
        double ttc = safety::min_oncoming_ttc(s.view(0, 0, 10, PI), &dx);
        CHECK(near_eq(ttc, 26.0 / 18.0, 1e-3)); // 返程真迎头仍刹
        CHECK(ttc < 4.0);
    }

    // ── 7. nearest_vehicle_lateral_cross_risk 前进 + 窗口过滤 ──
    // 近旁车 along=5, dy=1 → metric=5+2*1=7；远后车 along=-10（窗口 [-5,12] 外）忽略。
    {
        Scene s;
        s.set(0, 5.0, 1.0, 0.0, 0.0);
        s.set(1, -10.0, 0.0, 0.0, 0.0);   // 窗口外，忽略
        double dx = 0, dys = 0;
        double risk = safety::nearest_vehicle_lateral_cross_risk(s.view(0, 0, 8, 0.0), &dx, &dys);
        CHECK(near_eq(risk, 7.0, 1e-6));
        CHECK(near_eq(dx, 5.0, 1e-6));
        CHECK(near_eq(dys, 1.0, 1e-6));   // 左侧为正
    }

    // ── 8. pedestrian_collision_gap 前进 vs 返程方向自适应 ──
    {
        Scene s;
        s.set(0, 20.0, 0.0, 0.0, 0.0);    // 世界 +x=20 的行人
        // 前进：行人在前方 → gap=20-2.8=17.2
        double g_fwd = safety::pedestrian_collision_gap(s.view(0, 0, 5, 0.0, /*ped=*/0));
        CHECK(near_eq(g_fwd, 17.2, 1e-4));
        // 返程（朝西）：同一行人现在在后方（along<0）→ 无前向碰撞风险 → 1e9
        double g_rev = safety::pedestrian_collision_gap(s.view(0, 0, 5, PI, /*ped=*/0));
        CHECK(g_rev > 1e8);
    }

    // ── 9. 无障碍/无行人 → 全部返回 1e9（安全默认，不误刹）──
    {
        Scene s;  // 无 valid 障碍
        EgoView v = s.view(0, 0, 10, 0.0);
        CHECK(safety::nearest_same_lane_gap(v, tol) > 1e8);
        CHECK(safety::min_vehicle_ttc(v) > 1e8);
        CHECK(safety::min_oncoming_ttc(v) > 1e8);
        CHECK(safety::nearest_vehicle_lateral_cross_risk(v) > 1e8);
        CHECK(safety::pedestrian_collision_gap(v) > 1e8);   // ped_index=-1
        CHECK(safety::pedestrian_crossing_hold_gap(v) > 1e8);
    }

    std::printf("test_safety_geometry: all %d checks passed\n", g_checks);
    return 0;
}
