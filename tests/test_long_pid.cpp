// test_long_pid.cpp — 纵向 PID + anti-windup 纯逻辑核单元测试
//
// 立测试网（Phase 0，见 docs/ARCHITECTURE_REVIEW.md）：把 control_node.cpp 的
// 纵向控制核抽到 long_pid.h 后，用确定性单测钉住行为，并把故障表里两个历史
// bug 编码成回归守卫——它们此前只能靠 45s 黑盒 demo 事后暴露：
//
//   A) "控制遇慢车不减速、油门全开撞前车"：加速阶段积分饱和到 +500，切到减速
//      指令时残余正积分压过 P 项 → 油门全开。正确行为：pid_error 强烈翻负时
//      立刻清零正积分 → throttle=0、brake>0。
//   B) "掉头返程卡死 spd=0 target=0.5"：Phase 1 刹停使积分转负，目标翻正后
//      负积分把车钉死。正确行为：机动模式近停且目标为正时清负积分。
//
// 纯算法单测：只 include long_pid.h，无 transport/param/framework 依赖。
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "long_pid.h"

using longitudinal::LongPidParams;
using longitudinal::LongPidState;
using longitudinal::LongPidOutput;
using longitudinal::long_pid_step;

static int g_checks = 0;
#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        return 1; \
    } \
} while (0)

int main(void) {
    const LongPidParams p;  // 默认 = control_node.cpp 原字面量

    // ── 1. 加速：目标 > 当前 → throttle>0, brake=0, 模式 ACCEL ──
    {
        LongPidState st;
        LongPidOutput o = long_pid_step(st, p, 10.0, 0.0, false, false, false);
        CHECK(o.throttle > 0.0);
        CHECK(o.brake == 0.0);
        CHECK(std::strcmp(o.mode, "ACCEL") == 0);
    }

    // ── 2. HOLD：误差 < hold_error 且输出仍 >0 → 模式 HOLD ──
    {
        LongPidState st;
        // 小正误差：target 略高于 current，且无历史积分/微分冲击
        st.prev_error = 0.5;
        LongPidOutput o = long_pid_step(st, p, 10.5, 10.0, false, false, false);
        CHECK(o.throttle >= 0.0);
        CHECK(std::strcmp(o.mode, "HOLD") == 0);
    }

    // ── 3. 制动：目标 < 当前 → throttle=0, brake>0, 模式 BRAKE ──
    {
        LongPidState st;
        LongPidOutput o = long_pid_step(st, p, 0.0, 10.0, false, false, false);
        CHECK(o.throttle == 0.0);
        CHECK(o.brake > 0.0);
        CHECK(std::strcmp(o.mode, "BRAKE") == 0);
    }

    // ── 4. 积分限幅：任意驱动下积分恒 ∈ [integral_min, integral_max] ──
    {
        LongPidState st;
        st.integral = 1e6;   // 预置越界正值 → 一步后应被夹回 max
        long_pid_step(st, p, 6.0, 5.0, false, false, false);
        CHECK(st.integral <= p.integral_max + 1e-9);
        st.integral = -1e6;  // 预置越界负值 → 一步后应被夹回 min
        long_pid_step(st, p, 5.0, 6.0, false, false, false);
        CHECK(st.integral >= p.integral_min - 1e-9);
    }

    // ── 5. 回归 A："遇慢车不减速、油门全开撞前车" ──
    // 加速段积累的正饱和 windup（I 项压过 P 项）在切减速时必须被清除，
    // 否则减速帧 output 仍 >0 → 油门全开撞上去。anti-windup 是次帧修正
    // （output 先于清零计算），故验证"翻负帧清积分 + 次帧油门归零"两拍。
    {
        LongPidState st;
        st.integral   = 500.0;   // 模拟长加速积累的正饱和
        st.prev_error = -25.0;   // 已处稳定减速误差（隔离微分冲击）
        // 第 1 帧：误差强烈翻负（前车停，target=0 current=25）→ flip-clear 清零
        long_pid_step(st, p, 0.0, 25.0, false, false, false);
        CHECK(st.integral == 0.0);      // fix 的直接签名（bug 时仍 ~498 缓慢泄放）
        // 第 2 帧：稳定减速 → 无残余 windup → 油门归零、刹车起效
        LongPidOutput o2 = long_pid_step(st, p, 0.0, 25.0, false, false, false);
        CHECK(o2.throttle == 0.0);      // bug 时 integral 未清 → output>0 → throttle=1.0 撞车
        CHECK(o2.brake > 0.0);
        CHECK(std::strcmp(o2.mode, "BRAKE") == 0);
    }

    // ── 6. 倒车镜像：R 挡下 target/current 均负 → 正域 PID → 油门取负 ──
    {
        LongPidState st;
        LongPidOutput o = long_pid_step(st, p, -3.0, -1.0, /*is_reverse=*/true,
                                        false, false);
        CHECK(o.throttle < 0.0);        // 倒车加速 = 负油门（flowsim 约定）
        CHECK(std::strcmp(o.mode, "REV_ACCEL") == 0);
    }

    // ── 7. 换挡刹停：机动 + gear_pending → 全刹、清积分、模式 SHIFT_STOP ──
    {
        LongPidState st;
        st.integral = 300.0;  // 预置残余积分
        LongPidOutput o = long_pid_step(st, p, 5.0, 2.4, false,
                                        /*maneuver=*/true, /*gear_pending=*/true);
        CHECK(o.throttle == 0.0);
        CHECK(o.brake == 1.0);
        CHECK(st.integral == 0.0);
        CHECK(std::strcmp(o.mode, "SHIFT_STOP") == 0);
    }

    // ── 8. 回归 B："掉头返程卡死 spd=0 target=0.5" ──
    // Phase 1 刹停使积分转负，返程目标翻正后负积分残留把车钉死。
    // 机动模式下车近停且目标为正时必须清负积分，让车能起步。
    {
        LongPidState st;
        st.integral = -50.0;   // 模拟 Phase 1 刹停积累的负积分
        // 返程起步：目标翻正（0.5>maneuver_clear_error）、车近停（|0|<clear_speed）
        long_pid_step(st, p, /*target=*/0.5, /*current=*/0.0,
                      false, /*maneuver=*/true, false);
        CHECK(st.integral == 0.0);      // 负积分清零（bug 时残留 → 车钉死）
    }

    // ── 9. 非机动模式不做负积分清除（隔离 8 的特例，防越界到巡航） ──
    {
        LongPidState st;
        st.integral = -50.0;
        long_pid_step(st, p, 0.5, 0.0, false, /*maneuver=*/false, false);
        CHECK(st.integral < 0.0);       // 巡航模式：负积分不被特例清零
    }

    std::printf("test_long_pid: all %d checks passed\n", g_checks);
    return 0;
}
