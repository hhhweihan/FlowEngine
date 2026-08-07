// overtake_decision.h — 变道超车值得性判定纯逻辑核（从 behavior_planner_node.cpp 抽出）
//
// 背景（Phase 0 立测试网，见 docs/ARCHITECTURE_REVIEW.md）：
//   behavior 的「本车道被堵 → 是否值得超车」判定此前内联在 1600 行 FSM 主循环里，
//   读一大把 g.*。故障表记过一条恶性反逻辑 bug：
//     `worthwhile = blocked && (best_gap < min_gap)`
//   语义写反——等目标车道 gap **小于** 阈值才觉得"值得超车"，即前方越挤越想变道。
//   高速接近下只剩 ~2s，变道来不及 → behavior 一直不进 FOLLOW/超车。正确是
//     `worthwhile = blocked && (best_gap > min_gap)`（gap 足够大才值得且安全）。
//   这类纯算术判定过去只能靠 45s 黑盒 demo 事后暴露；抽核后由
//   tests/test_overtake_decision.cpp 在毫秒级钉死。
//
// 职责边界（对应 CLAUDE.md 模块职责铁律）：
//   behavior 是离散状态机——决定"做什么"（是否值得发起变道超车），不输出连续
//   控制量。本函数只把 gap / 速度 / 阈值算成 blocked / worthwhile 布尔，供 FSM
//   转移消费。
//
// 纯逻辑：只依赖 <cmath>。所有阈值经 OvertakeParams 传入（= behavior 的 g.* 配置），
//   零全局态、零框架依赖。header-only，同时编进 behavior_planner 插件与单测，
//   杜绝第二份实现。
//
// 行为契约：逐行等价移植自 behavior_planner_node.cpp 超车判定块，零行为变更。
#pragma once
#include <cmath>

namespace behavior {

// 超车判定阈值（= behavior 的 g.* 配置项，可 flowctl param set 热调）。
struct OvertakeParams {
    double blocked_range_min;            // g.blocked_range_min：blocked 最小察觉距离
    double blocked_range_mult;           // g.blocked_range_mult：blocked 距离 = desired_gap × 此值
    double follow_hysteresis;            // g.follow_hysteresis：已在 FOLLOW 时退出距离放大（进紧出松）
    double min_overtake_gap_base;        // g.min_overtake_gap_base：值得超车的目标车道最小 gap 基线
    double min_overtake_gap_speed_mult;  // g.min_overtake_gap_speed_mult：随相对速度增大的 gap 需求
    double min_overtake_gap_cap;         // g.min_overtake_gap_cap：min_gap 上限
};

// 判定输出。下游 FSM 消费 blocked / worthwhile / min_gap / blocked_range。
struct OvertakeDecision {
    double blocked_range = 0.0;  // 本车道"被堵"察觉距离（随 desired_gap 伸缩）
    bool   blocked = false;      // 本车道前方有车影响通行
    double rel_speed = 0.0;      // 相对接近速度（ego − lead，负截零）
    double min_gap = 0.0;        // 值得且安全超车所需的目标车道最小 gap
    bool   worthwhile = false;   // 被堵 且 目标车道 gap 足够大 → 值得发起超车
};

// 纯函数：算超车值得性。逐行对应原 behavior_planner_node.cpp 超车判定块。
//
// 参数语义：
//   best_gap    — 本车道最近前车净距 m（1e9 = 无前车）
//   desired_gap — 期望跟车间距 m（时距×车速导出，blocked_range 以它为基准伸缩）
//   in_follow   — 当前 FSM 是否处于 FOLLOW（决定用进入阈值还是滞环退出阈值）
//   ego_v       — 本车速度 m/s
//   lead_speed  — 前车速度 m/s
inline OvertakeDecision overtake_decision(double best_gap, double desired_gap,
                                          bool in_follow, double ego_v,
                                          double lead_speed, const OvertakeParams& p)
{
    OvertakeDecision d;
    /* blocked 语义="本车道前方有车影响通行"，用 desired_gap 的倍数表达而非裸 80m，
     * 随车速自动伸缩（高速更早察觉、低速不误触发）。 */
    d.blocked_range = std::fmax(p.blocked_range_min, desired_gap * p.blocked_range_mult);
    /* 滞环：已在 FOLLOW 时用 hysteresis× 的退出距离，避免前车在阈值附近徘徊时
     * BLOCKED/LOST_LEAD 每帧翻转。进入紧、退出松。 */
    d.blocked = (best_gap < (in_follow ? d.blocked_range * p.follow_hysteresis
                                       : d.blocked_range));
    d.rel_speed = ego_v - lead_speed;
    if (d.rel_speed < 0.0) d.rel_speed = 0.0;
    d.min_gap = p.min_overtake_gap_base + d.rel_speed * p.min_overtake_gap_speed_mult;
    if (d.min_gap > p.min_overtake_gap_cap) d.min_gap = p.min_overtake_gap_cap;
    /* 值得超车 = 被堵 且 目标车道 gap 足够大（> min_gap）。注意是 >，不是 <——
     * 写成 < 是历史反逻辑 bug（前方越挤越想变道 → 高速下来不及 → 从不超车）。 */
    d.worthwhile = d.blocked && (best_gap > d.min_gap);
    return d;
}

}  // namespace behavior
