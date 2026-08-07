// long_pid.h — 纵向 PID + anti-windup 纯逻辑核（从 control_node.cpp 抽出）
//
// 背景（Phase 0 立测试网，见 docs/ARCHITECTURE_REVIEW.md）：
//   control_node.cpp 的纵向控制逻辑此前内联在 ~170 行主循环块里，读一堆全局
//   `g.*`，无法单测。故障表里"控制遇慢车不减速、油门全开撞前车"这类 anti-windup
//   回归只能靠 45s 黑盒 demo 事后暴露。把纯数值核抽出来后，同样的 bug 可在
//   编译期/毫秒级由 test_long_pid.cpp 拦下。
//
// 职责边界（对应 CLAUDE.md 模块职责铁律）：
//   这是 control 的一部分——纯轨迹跟随的纵向执行量推导。只把"目标速度 vs 当前
//   速度"变成 throttle/brake，不发明目标速度（那是 planning 的唯一权威）。
//
// 纯逻辑：只依赖 <cmath>。无 transport/param/framework/全局态依赖。
//   状态（integral/prev_error）由调用方持有并传入，函数只读写传入的 state，
//   便于确定性单测（喂序列 → 断言输出）。header-only，同时编译进 control_node
//   插件与 test_long_pid 测试，杜绝第二份实现。
//
// 行为契约：所有默认参数 == 抽取时 control_node.cpp 的字面量，逐行等价移植，
//   零行为变更。改这些默认值前先跑 test_long_pid + demo_evaluator。
#pragma once
#include <cmath>

namespace longitudinal {

// PID + 输出映射 + anti-windup 的可调参数。默认值 = control_node.cpp 原字面量。
struct LongPidParams {
    double kp = 800.0;              // 比例增益（control.pid_kp）
    double ki = 50.0;              // 积分增益（control.pid_ki）
    double kd = 100.0;             // 微分增益（control.pid_kd）
    double dt = 0.05;              // 控制周期 s（CONTROL_DT_S，20Hz）

    double integral_max = 500.0;   // 积分上限
    double integral_min = -200.0;  // 积分下限
    double throttle_scale = 5000.0;// output>0 → throttle = output/scale
    double brake_scale = 8000.0;   // output<0 → brake = -output/scale
    double hold_error = 1.0;       // pid_error < 此值 → 模式 HOLD（不算主动加速）

    // Anti-windup：error 从正翻负（加速→减速切换）时残余正积分是追尾主因。
    double antiwindup_flip_error = -2.0;   // pid_error < 此值 且 integral>0 → 清零

    // 机动模式负积分对称清除：Phase 1 刹停使积分转负，目标翻正后负积分把车钉死。
    double maneuver_clear_error  = 0.2;    // pid_error > 此值
    double maneuver_clear_speed  = 1.0;    // 且 |current_speed| < 此值 → 清负积分
};

// 跨帧状态：由调用方（control_node 的 `g`，或测试）持有。
struct LongPidState {
    double integral = 0.0;
    double prev_error = 0.0;
};

// 单帧输出。mode 指向静态字符串字面量，可安全返回/长期持有。
struct LongPidOutput {
    double throttle = 0.0;
    double brake = 0.0;
    const char* mode = "NONE";
};

// 纯函数：推进一帧纵向 PID。读写 st，返回本帧 throttle/brake/mode。
//
// 参数语义：
//   acc_target     — 目标速度 m/s（已由 planning + slew 限幅，本函数不再限幅）
//   current_speed  — 当前车速 m/s（前进为正，倒车为负）
//   is_reverse     — 当前档位为 R（倒车镜像：让 PID 始终在正域工作）
//   maneuver_mode  — 机动轨迹（掉头/倒库），启用负积分清除等特例
//   gear_pending   — 带速待换挡：本帧物理刹停（掉头 D→R 交界的"停"）
//
// 逐行对应原 control_node.cpp 786–849。
inline LongPidOutput long_pid_step(LongPidState& st, const LongPidParams& p,
                                   double acc_target, double current_speed,
                                   bool is_reverse, bool maneuver_mode,
                                   bool gear_pending)
{
    // 倒车镜像速度符号，让 PID 始终在正域工作。
    // 倒车：target=-3, current=-1 → pid_target=3, pid_current=1 → error=2 → 正输出。
    double pid_target  = is_reverse ? -acc_target    : acc_target;
    double pid_current = is_reverse ? -current_speed : current_speed;
    double pid_error   = pid_target - pid_current;

    st.integral += pid_error * p.dt;
    if (st.integral > p.integral_max) st.integral = p.integral_max;
    if (st.integral < p.integral_min) st.integral = p.integral_min;

    double derivative = (pid_error - st.prev_error) / p.dt;
    double output = p.kp * pid_error + p.ki * st.integral + p.kd * derivative;

    LongPidOutput out;
    if (output > 0) {
        out.throttle = output / p.throttle_scale;
        if (out.throttle > 1.0) out.throttle = 1.0;
        out.brake = 0.0;
        out.mode = (pid_error < p.hold_error)
                       ? "HOLD"
                       : (is_reverse ? "REV_ACCEL" : "ACCEL");
    } else {
        out.throttle = 0.0;
        out.brake = (-output) / p.brake_scale;
        if (out.brake > 1.0) out.brake = 1.0;
        out.mode = is_reverse ? "REV_BRAKE" : "BRAKE";
    }
    // 倒车：油门取反（flowsim physics 用负油门触发倒车）。
    if (is_reverse) out.throttle = -out.throttle;

    // 机动换挡刹停：带速想换挡 → 物理全刹（掉头三把方向的"停"）。
    if (maneuver_mode && gear_pending) {
        out.throttle = 0.0;
        out.brake = 1.0;
        out.mode = "SHIFT_STOP";
        st.integral = 0.0;  // 全刹时清积分，防换挡后残余积分反向推车
    }

    // Anti-windup：error 从正翻负（加速→减速切换），残余正积分是追尾催命符。
    // 加速阶段积分可累积到 +500 → 减速时 I 项压过 P 项 → 油门全开撞上去。
    if (pid_error < p.antiwindup_flip_error && st.integral > 0) {
        st.integral = 0.0;  // 立刻清零
    } else {
        // 正常饱和时慢速泄放（仅在执行量已顶到限幅、误差仍同向时）。
        if (st.integral > 0 && out.throttle >= 1.0 && pid_error > 0)
            st.integral -= pid_error * p.dt;
        if (st.integral > 0 && out.brake >= 1.0 && pid_error < 0)
            st.integral += pid_error * p.dt;
    }

    // 机动模式负积分清除：Phase 1 刹停使积分转负，目标翻正（驻停/慢转）后
    // 负积分残留把车钉在原地（掉头返程 spd=0 target=0.5 卡死）。近停且目标为正时清。
    if (maneuver_mode && st.integral < 0 &&
        pid_error > p.maneuver_clear_error &&
        std::fabs(current_speed) < p.maneuver_clear_speed) {
        st.integral = 0.0;
    }

    st.prev_error = pid_error;
    return out;
}

}  // namespace longitudinal
