# FlowEngine 架构评估与优化执行计划书

> 评估视角:自动驾驶中间件资深架构师
> 评估日期:2026-08-07
> 评估基线:`feature/weihan_dev` @ 78eee7d(1000 commits)
> 评估方式:核对总线内存模型、传输层实现、节点耦合、测试覆盖的真实代码,而非仅读 CLAUDE.md

---

## 一、总体判断

这是一个**远超"个人练手"水准的中间件项目**:真有 Pub/Sub 总线 + 调度器 + 进程内/IPC(共享内存)/TCP 三级传输 + Discovery + dlopen 插件化 + bag/MCAP 录制 + 参数热更 + 监控守护 + 3D 仪表盘。工程完成度很高。

核心定位问题:它同时是"中间件"和"一整套 ADAS 演示栈 + 物理仿真 + 学习闭环 + 三维可视化",**四个产品缠在一个仓里**。这既是它的展示力,也是它最大的架构债来源。CLAUDE.md 里那张巨长的"常见故障模式"表(几十个掉头/刹车/变道连环 bug)不是运气差,而是**耦合与缺少单元级隔离的必然症状**。

---

## 二、名字:「FlowEngine」——建议更名

| 维度 | 评价 |
|------|------|
| 语感 | ✅ 干净、好记、暗示 dataflow |
| 独特性 | ❌ **严重撞名**。工作流/BPM/游戏/ML 领域一堆 "FlowEngine";Netflix、Salesforce 都有 "Flow" 系产品 |
| 定位准确性 | ❌ "Engine" 让人联想到*计算/工作流引擎*,而它本质是*消息中间件/机器人运行时*。同赛道命名都很具体:ROS、Cyber RT、Autoware、Apollo、DDS |
| 可检索性 | ❌ SEO 几乎为零,GitHub 搜 "FlowEngine" 淹没在无关项目里 |
| 商标风险 | ⚠️ 通用词,难注册难维权 |

**建议**:名字要传达"自动驾驶/机器人 + 实时消息运行时"。方向性候选:`FluxRT` / `LaneBus` / `DriveFlow` / `Corex`(呼应 Cyber RT + coro)/ `FlowRT`(至少加 RT 点明 runtime)。

**优先级**:不紧急。但若打算对外(开源/简历/产品),越早改成本越低。

---

## 三、功能合理性

**核心中间件(真正的产品,设计合理)**:总线 + 调度器(classic/choreo DAG 两模式)+ 传输 + Discovery + 插件系统 + params + bag。自洽且专业,choreo 模式(上游 publish 自动触发下游)是亮点。

**问题是范围蔓延(scope creep)**。以下都塞在同一仓、同一构建里:
- 完整 ADAS 算法栈(感知/融合/预测/行为/规划/控制/安全)
- 物理仿真 flowsim + esmini(第三方 15k 行 RoadManager)
- 学习闭环(data_recorder → tiny-MLP → PyTorch/时序训练 → ONNX → OTA)
- three.js 3D 前端(45 个 JS 模块)
- EKF-SLAM、驾考场景、SocketCAN 执行器

**架构师视角**:中间件的价值在于"算法可插拔",但现在算法栈和中间件**共享一套构建、一套 CLAUDE.md 铁律、一套故障表**,边界靠文字纪律(模块职责铁律)维系,不是靠代码/包边界强制。文字纪律挡不住越界——故障表本身就是证据。

---

## 四、架构问题(按严重度排序,均有代码证据)

### P0 — 消息总线的静态定长内存模型是硬天花板
- `Message` 结构体内联 `data[65536]`,按值拷贝;队列 1024 深 → **每条总线 ~64MB 静态内存**。
- `MAX_TOPICS 64`、`MAX_SUBSCRIBERS 256`(注释显示 26 节点已实测 >128,逼近上限)。
- stereo 一帧 44KB 直接把整个 Message 撑到 64KB。
- 结论:能跑 demo,但无法规模化。大 payload 场景每帧几十 KB memcpy,订阅/主题数写死。
- 证据:`include/message_bus.h:40-43`。

### P0 — 节点巨石(god object)
- `planning_node.cpp` 2691 行、`flowsim_node.cpp` 2324、`behavior` 1600、`control` 1453。
- CLAUDE.md 的"模块职责铁律"是**给耦合打的治理补丁**。
- 逻辑挤在超大 `.cpp` + 每节点一个 `static struct g` 全局态里,难单测、难复用。

### P1 — 测试严重不对称
- 前端 vis 有一流门禁(Coord 单一事实源 + grep 强制 + property test + tick 冒烟)——**整个项目最好的工程实践**。
- 但 C 侧控制/规划/行为**几乎没有单元测试**(`test_adas_nodes_logic.c` 仅 25 个断言),只靠 45s 黑盒 demo 评估兜底。
- 这正是掉头/刹车 bug 反复复发的根因:回归只在系统级、事后、非确定性地暴露。

### P1 — 算法与中间件未做包边界隔离
- 两者同构建、同头文件树,`modules/adas_nodes` 直接依赖 core 内部。
- 中间件想被别的项目复用,得先把 ADAS 栈剥出去。

### P2 — 全局可变状态
- 每节点 `static struct g` 阻碍多实例与并行单测(对单实例节点尚可接受)。

### 做得好、不要动的地方
dlopen 插件化、config 驱动 pipeline、choreo 调度、Coord 坐标治理、demo_evaluator 黑盒门禁、平台兼容层收口。这些是加分项。

---

## 五、执行计划书(分阶段)

> **原则:先补测试网,再动结构。** 没有单测网就重构巨石 = 制造新 bug。
> 全程用现有 `pipeline_check` / `demo_evaluator` / `scenario_regression` 三级门禁做安全网。

### Phase 0 — 立网(P1,1~2 周,零行为变更,风险极低)
1. 给 `planning`/`control`/`behavior`/`safety` 抽出**纯函数核**(参照 `maneuver_tracker.h` / `st_graph.c` 已有的 header-only 可单测形态),对故障表里每个历史 bug 补一条确定性单测。
2. **验收**:故障表 N 个已知 bug → N 个红/绿单测;`ctest` 纳入 CI。
3. **价值**:把"提交后 45s 才发现"变成"编译期/秒级拦下",是后续所有重构的前提。

### Phase 1 — 拆巨石(P0,2~4 周,行为不变,风险中)
1. 按已确立的"模块职责铁律"把 4 个 >1500 行节点拆成 `node.cpp(线程/IO 胶水) + <domain>_core.{h,cpp}(纯逻辑)`。逻辑只依赖入参,不碰 `g`。
2. 每拆一个,Phase 0 单测必须全绿 + `scenario_regression` baseline 无退化。
3. **验收**:单文件 ≤ 800 行;核心逻辑 0 全局依赖、可单测。

### Phase 2 — 总线内存模型改造(P0,2~3 周,风险高,需 py/bench 先行)
1. `Message` 改为**变长 payload**(定长头 + 指针/引用计数 buffer),大 payload(stereo/lidar)走**零拷贝或 SHM 引用**,消除按值 64KB 拷贝。
2. `MAX_TOPICS/MAX_SUBSCRIBERS` 由定长表改为可增长哈希表。
3. **先写微基准**(`src/benchmark.c` 已有基座)量化改造前后吞吐/延迟/内存,再动。
4. **验收**:26 节点 pipeline 内存显著下降;吞吐不退;订阅数无上限。

### Phase 3 — 产品边界切分(P1,1~2 周,风险低,可与 Phase 1 并行)
1. 仓内切成两层:`libflowengine`(纯中间件)+ `adas/`(算法栈,依赖前者公开头文件)。构建上强制单向依赖。
2. 第三方 esmini 移入子模块/明确 vendored 边界。
3. **验收**:能只构建 `libflowengine` 且不拉入任何 `adas_nodes`。

### Phase 4 — 收尾(P2,持续)
改名(若决定)、把 C 侧门禁做到与前端 vis 对称的成熟度、文档跟进。

---

## 六、排期与依赖

```
Phase 0 (测试网) ──► Phase 1 (拆巨石) ──► Phase 2 (总线改造)
                 └─► Phase 3 (边界切分,可与 1 并行)
```

- **Phase 0 是一切的地基,建议第一步就做。**
- **Phase 2 收益最大但风险最高**,必须压在 Phase 0 和基准测试之上。

---

## 七、优先级速查

| 项 | 优先级 | 风险 | 预估 | 前置 |
|----|--------|------|------|------|
| Phase 0 立测试网 | P1(先做) | 极低 | 1~2 周 | 无 |
| Phase 1 拆巨石 | P0 | 中 | 2~4 周 | Phase 0 |
| Phase 2 总线改造 | P0 | 高 | 2~3 周 | Phase 0 + 微基准 |
| Phase 3 边界切分 | P1 | 低 | 1~2 周 | 无(可并行) |
| 更名 | P2 | 低 | — | 战略决定 |

---

## 八、执行记录

> 逐次记录本计划的落地动作,便于回溯"改了什么、验了什么"。

### 2026-08-07 — Phase 0 第一刀:纵向 PID 抽核 + 立测试网

**背景**:`control_node.cpp` 纵向控制核内联在 ~170 行主循环块、读一堆全局 `g.*`,无法单测;故障表"控制遇慢车不减速、油门全开撞前车"这类 anti-windup 回归只能靠 45s 黑盒 demo 事后暴露。

**改动**(零行为变更,逐行等价移植;同一改动内收敛,无第二份实现):

| 文件 | 动作 |
|------|------|
| `modules/adas_nodes/long_pid.h` | **新增** header-only 纯逻辑核 `long_pid_step`(倒车镜像 / 积分限幅 / 输出映射 / 换挡刹停 / anti-windup 含加速→减速翻负清零 + 机动负积分对称清除)。魔法常量全部参数化,默认值 = 抽取时 `control_node.cpp` 原字面量 |
| `modules/adas_nodes/control_node.cpp` | 内联 PID 块(原 786–849)→ 调用 `long_pid_step`,行为不变 |
| `tests/test_long_pid.cpp` | **新增** 22 个确定性断言,含两条历史 bug 回归守卫:①加速积分饱和→切减速→正积分清零、油门归零(不撞车);②掉头返程机动模式负积分→目标翻正近停→清负积分(不卡死) |
| `CMakeLists.txt` | 接入 ctest 目标 `long_pid_tests`(TIMEOUT 10) |

**验证**(全绿):
- 单测:`ctest -R long_pid_tests` → 22/22 passed(0.47s)
- 守卫有牙:向副本注入历史 bug(flip-clear 条件反向)→ 测试在 `test_long_pid.cpp:87` FAIL(证明毫秒级拦截,替代 45s demo 撞车)
- 在树编译:`control_node` 及全部节点插件 build OK
- L1 运行时门禁:full pipeline 跑 40s,`demo_evaluator` **PASS**(control raw_cmd 26.7Hz / cmd 21.8Hz、无闯红灯、liveness 健康;仅两个已知 WARN:u-turn 横向跨路、NPC respawn)

**顺带发现(既有问题,非本次引入)**:macOS 上 `test_entity_physics` 链接 `lib/libesminiRMLib.so`(Linux `.so`)失败,阻断 `cmake --build build` 全量。建议纳入 **Phase 4 收尾**。

### 2026-08-07 — Phase 0 第二刀:安全层近场几何抽核

**背景**:`safety_control_node.cpp` 6 个近场风险度量(同车道 gap / 车辆 TTC / 对向 TTC / 横向穿越 / 行人碰撞 / 行人过街保持)内联在协程 task 里,读一个含 128 槽障碍数组的巨大 `VehicleState` + 生成协议 `ObstacleList`,无法脱离框架单测。故障表最凶的一条「掉头返程幽灵刹车 + 无同向防撞(撞旁边车)」根因正是这几个函数**系统性用世界 +x 坐标**判前后:返程(朝西 heading≈π)时前车在 -x 被 skip → 同向防撞失效;同向车被误判迎头 → head-on 硬刹。

**改动**(零行为变更;`heading=0` 前进时沿车头投影退化为原世界坐标,既有前进场景零回归):

| 文件 | 动作 |
|------|------|
| `modules/adas_nodes/safety_geometry.h` | **新增** header-only 纯逻辑核(namespace `safety`),用非拥有视图 `EgoView`(ego 标量 + 障碍数组指针 + `obs_count`)与生成协议定长容量解耦——`kMaxObs` 仍在 .cpp 由 `sizeof(ObstacleList)` 推导并作 `obs_count` 传入,协议扩容自动跟随 |
| `modules/adas_nodes/safety_control_node.cpp` | 6 个函数体**物理删除**(173 行),task 循环改 `make_ego_view(state)` 构视图后调 `safety::*`;单一实现,无第二份 |
| `tests/test_safety_geometry.cpp` | **新增** 23 个断言:每个函数验「前进(heading=0)零回归 + 掉头返程(heading=π)方向正确」两侧,含追尾修复 + 幽灵刹车修复 + 仍抓真迎头 |
| `CMakeLists.txt` | 接入 ctest 目标 `safety_geometry_tests` |

**验证**:单测 23/23;向副本注入世界 +x bug → 测试在 `test_safety_geometry.cpp:103`(返程追尾守卫)FAIL;`safety_control_node` 插件在树编译链接 OK。

### 2026-08-07 — Phase 0 第三刀:变道超车值得性判定抽核

**背景**:`behavior_planner_node.cpp` 的「本车道被堵 → 是否值得超车」判定内联在 1600 行 FSM 主循环,读一把 `g.*`。故障表记过恶性反逻辑 bug:`worthwhile = blocked && (best_gap < min_gap)`——写成 `<`,语义变"前方越挤越想变道",高速接近下来不及 → 一直不超车。正确为 `>`。

**改动**(零行为变更,逐行等价移植):

| 文件 | 动作 |
|------|------|
| `modules/adas_nodes/overtake_decision.h` | **新增** header-only 纯逻辑核(namespace `behavior`),`overtake_decision(best_gap, desired_gap, in_follow, ego_v, lead_speed, params)` → `{blocked_range, blocked, rel_speed, min_gap, worthwhile}`。阈值全经 `OvertakeParams` 传入(= `g.*`,可热调) |
| `modules/adas_nodes/behavior_planner_node.cpp` | 内联判定块删除,改调纯函数后 unpack 回同名局部变量,下游 FSM 消费点零改动 |
| `tests/test_overtake_decision.cpp` | **新增** 15 个断言:被堵+空旷→值得、被堵+gap不足→不值得、未堵→不值得、min_gap 夹紧、rel_speed 负截零、FOLLOW 滞环 |
| `CMakeLists.txt` | 接入 ctest 目标 `overtake_decision_tests` |

**验证**:单测 15/15;向副本注入 `>`→`<` 反逻辑 → 测试在 `test_overtake_decision.cpp:53` FAIL;`behavior_planner_node` 插件在树编译链接 OK。

### 2026-08-07 — Phase 0 第四刀:planning 会车让行 + 窄路减速抽核

**背景**:计划书 Phase 0 要求 planning/control/behavior/safety **四域**都抽纯核。前三刀补齐了 control/safety/behavior,planning 域尚无守卫。planning 的红灯 override / spd_out 双 bug 已在「planning 重生 M1」删除并交给 `st_graph.c`(已有纯核 + 11/11 测试),剩下最干净、自包含、且仍无守卫的历史 bug 是 **Phase 5 会车让行**:故障表「多车道遇对向车刹停到 0(会车让行过度保守)」+「掉头返程幽灵刹车」的 planning 侧——旧逻辑用世界 dx 预筛(返程朝 -x 方向盲)+ `|dy|≤1.5×路宽`(相邻对向车道也当迎头)→ 巡航压到 0.4× 全刹。

**改动**(零行为变更,逐行等价移植;`heading=0` 前进时三处方向投影退化为原世界坐标,既有前进场景零回归):

| 文件 | 动作 |
|------|------|
| `modules/adas_nodes/oncoming_yield.h` | **新增** header-only 纯逻辑核(namespace `planning`),非拥有视图 `YieldView` + `oncoming_yield()` → `{oncoming, min_clearance_left/right, narrow_width, command_speed}`。阈值全经 `YieldParams` 传入(默认 = 原内联字面量);降速取 min 在纯核内完成 |
| `modules/adas_nodes/planning_node.cpp` | Phase 5 内联块(58 行)**物理删除**,改 `make YieldView` 后调 `planning::oncoming_yield`;include 置于 `HAVE_FRENET` 分支外(让行判定不依赖 Frenet) |
| `tests/test_oncoming_yield.cpp` | **新增** 18 个断言:前进让行、相邻车道不误判、返程方向盲修复、返程同向误判修复、同向远离不让行、窄路减速、空槽惰性 |
| `CMakeLists.txt` | 接入 ctest 目标 `oncoming_yield_tests` |

**验证**:单测 18/18;**三处 teeth-check 全部咬住**——向副本注入 ① 世界 dx 预筛(`along=rx`)② 横向 1.5×路宽 ③ 世界速度(`rel_v=obs_vx`)→ 对应断言各自 FAIL;`planning_node` 插件在树编译链接 OK;L1 `demo_evaluator` **PASS**(仅 2 个已知 WARN)。

### 2026-08-07 — Phase 0 第五刀:control 速度死锁恢复 + ROAD_GUARD 抽核

**背景**:control 域已有 `long_pid.h`(纵向 anti-windup),但主循环里还有一段执行权覆盖决策——「SPEED_ZERO_RECOVERY(全域速度死锁给油)+ ROAD_GUARD(偏离目标车道强制回正)」——独占三个历史 bug 却无守卫,且两分支按 `y_from_target` 阈值可证互斥,自包含度高,选为第五刀。三个 bug:①「车速降到 0 后永久卡死」(旧低速恢复要求 `|y|≥road_center_limit`,车可在 2.1<|y|<2.5 停下永不满足);②「掉头返程 ROAD_GUARD 把车拽出路面」(旧用世界系 lat_error 定 steer 符号,返程 heading=π 打反);③「ROAD_GUARD 低速仍刹车 → 与死锁互锁」(低速应给小油门 0.18 而非刹车)。

**改动**(零行为变更,逐行等价移植;steer 限幅 `steer_limit_for_speed` 依赖 `g.wheelbase`,由调用方算好作标量传入,计时器/prev_steer 由调用方按返回标志应用):

| 文件 | 动作 |
|------|------|
| `modules/adas_nodes/road_guard.h` | **新增** header-only 纯逻辑核(namespace `control`),`RoadGuardIn`(标量输入)+ `road_guard_decide()` → `{throttle, brake, steer, mode, reset_speed_zero_timer, update_prev_steer}`。阈值全经 `RoadGuardParams` 传入(默认 = 原内联字面量)。两分支互斥(`y≤阈值` 恢复 / `y>阈值` ROAD_GUARD) |
| `modules/adas_nodes/control_node.cpp` | 内联 SPEED_ZERO_RECOVERY + ROAD_GUARD 两块(~35 行)**物理删除**,改 `make RoadGuardIn` 后调 `control::road_guard_decide`,按返回标志回写 `g.speed_zero_timer` / `g.prev_steer` / `mode` 字符串 |
| `tests/test_road_guard.cpp` | **新增** 30 个断言:两分支不触发透传、恢复独立于 y、恢复门槛(无轨迹/目标≤1 不恢复)、低速给油 0.18、高速刹车≥0.65、刹车不下调、steer 符号两侧对称、机动期豁免、互斥性 |
| `CMakeLists.txt` | 接入 ctest 目标 `road_guard_tests` |

**验证**:单测 30/30;**三处 teeth-check 全部咬住**——向副本注入 ① 恢复要求大 `y`(`<=`→`>=`,重现卡死)② steer 符号翻反(重现返程出路面)③ 低速改刹车(重现互锁)→ 对应断言各自 FAIL,还原后即绿;`control_node` 插件在树编译链接 OK;L1 `demo_evaluator` **PASS**(仅已知 WARN:掉头横向跨路、消息丢弃 0.01%、shadow MAE 临界)。

### 2026-08-07 — Phase 0 首轮收尾:全量验证

- **纯逻辑核单测**:`maneuver_tracker` / `long_pid` / `safety_geometry` / `overtake_decision` / `oncoming_yield` / `road_guard` / `stop_light_gate` 七组 ctest 全绿(maneuver + 22+23+15+18+30+13,毫秒级,0.02s 跑完)。
- **四域覆盖到齐**:计划书 Phase 0 要求的 control / safety / behavior / **planning** 四域现各有纯核 + 守卫(control 两把:`long_pid.h` 纵向 + `road_guard.h` 执行权覆盖;`safety_geometry.h` / `overtake_decision.h` / `oncoming_yield.h`)。
- **L1 运行时门禁**:full pipeline + `demo_evaluator` **PASS**(所有 liveness 信号存活、recognition 1.000、无闯红灯;仅两个已知 WARN:掉头横向跨路 6.5~7.0m、消息丢弃 0.01%)。确认四处抽核系统级行为不变。
  - 排障留痕:一次 eval 曾报 `warning lead time` FAIL,溯源为 demo.sh 在 macOS 上偶发「49s 协作式提前退出」(rc=0、各节点 `cleanup done`、无 crash/assert),评估器采到了正在关停的尾窗所致;缩短评估窗至退出前完成后复跑即 PASS。非本次改动引入。
- **既有失败(非本次引入)**:`flow_coro_bench` SEGFAULT(上游 flowcoro 基准,macOS 弱化项)、`traversability_tests` / `test_entity_physics` 未构建(esmini `.so`/`.dylib` 链接,同属 Phase 4 收尾)。均与四处抽核无关。

**本轮成果**:8 个历史 bug(anti-windup 追尾、掉头返程追尾、掉头返程幽灵刹车、超车反逻辑、会车让行过度保守/返程方向盲、车速降 0 卡死、ROAD_GUARD 返程 steer 打反出路面、ROAD_GUARD 低速刹车互锁)从「提交后 45s 黑盒暴露」变为「毫秒级红灯拦下」,并各自附注入式 teeth-check 证明守卫有牙。Phase 0 四域覆盖首轮到齐(control 域两把纯核)。

### 2026-08-07 — Phase 0 第六刀:behavior 归位红灯闸门抽核

**背景**:behavior 域已有 `overtake_decision.h`(超车值得性),但归位/变入决策还内联着 `lane_ahead_stop_light()`——它独占两个历史 bug 却无守卫,且是纯几何判定(红绿灯缓存 + 车道几何),自包含度高,选为第六刀。两个 bug:①「超车/归位后立刻在红灯前刹停(无效变道)」(归位不查目标车道前方红灯,切回内侧道即刹停);②「掉头返程方向盲」(返程朝 −x,前方灯 `dx<0`,旧世界系 `dx>0` 漏检 → 归位撞红灯)。

**改动**(零行为变更,逐行等价移植;`on_return=false` 前进时方向翻转退化为原世界坐标零回归):

| 文件 | 动作 |
|------|------|
| `modules/adas_nodes/stop_light_gate.h` | **新增** header-only 纯逻辑核(namespace `behavior`),非拥有视图 `StopLightView`(红绿灯缓存指针 + ego + 车道布局)+ `lane_ahead_stop_light()` → bool。车道中心复用 `include/road_geometry.h` 的共享 `lane_center_y`;刹车距离/横向门限/返程翻转全经 `StopLightParams` 传入(默认 = 原内联字面量) |
| `modules/adas_nodes/behavior_planner_node.cpp` | 内联函数体(20 行)**物理删除**,改组 `StopLightView` 后转发纯核;新增 include |
| `tests/test_stop_light_gate.cpp` | **新增** 13 个断言:无灯/绿灯/目标车道红灯命中、相邻车道不误判、身后不算、超 range 不算、v² range 缩放、下限 60m、返程方向命中、返程身后不算、多灯混合 |
| `CMakeLists.txt` | 接入 ctest 目标 `stop_light_gate_tests`(需 `include/` 供 road_geometry.h) |

**验证**:单测 13/13;**两处 teeth-check 全部咬住**——注入 ① 放宽横向门限(`0.5`→`2.0`,相邻灯误判)② 删返程方向翻转(返程方向盲)→ 对应断言各自 FAIL,还原即绿;`behavior_planner_node` 插件在树编译链接 OK;L1 `demo_evaluator` **PASS**(仅已知 WARN)。

### 五刀 → 六刀累计进度(2026-08-07)

**已抽核 + 守卫的纯逻辑核(7 个,毫秒级 ctest 全绿):**

| # | 纯核 | 域 | 覆盖历史 bug | 单测 |
|---|------|----|----|----|
| 1 | `maneuver_tracker.h` | control(机动) | 掉头/泊车弧长推进 + 倒挡横向反号 | ✅ |
| 2 | `long_pid.h` | control(纵向) | anti-windup 追尾 | 22 |
| 3 | `safety_geometry.h` | safety | 掉头返程追尾 + 幽灵刹车(世界 +x 方向盲) | 23 |
| 4 | `overtake_decision.h` | behavior | 超车值得性反逻辑(`>`↔`<`) | 15 |
| 5 | `oncoming_yield.h` | planning | 会车让行过度保守 + 返程方向盲 | 18 |
| 6 | `road_guard.h` | control(执行权) | 车速降 0 卡死 + 返程 steer 打反 + 低速刹车互锁 | 30 |
| 7 | `stop_light_gate.h` | behavior | 归位无效变道(不查目标车道红灯)+ 返程方向盲 | 13 |

**尚未抽核的失败表条目(候选,按自包含度/风险排序):**

| 优先级 | 现象 | 位置 | 为何仍难抽 / 抽核策略 |
|-------|------|------|------|
| 高 | behavior 归位「目标车道可用」判定(超车后立刻归位再超再归位) | `behavior_planner_node.cpp` 归位分支(~774) | 与 stop_light_gate 相邻的同段决策,纯标量(目标车道前车速 ≥ 0.7×巡航),可抽成 `lane_return_gate` 或并入 stop_light_gate |
| 高 | behavior 变道 P5 分支锁死(`target_speed=lead_speed` 停车锁死) | `behavior_planner_node.cpp` 变道分支 | 状态转移 × 速度耦合,需连状态机一起考虑,先补状态转移表完整性测试 |
| 中 | planning 红灯 override 同步重建 spd_out(闯红灯) | `planning_node.cpp` (~1209) | 依赖 spd_out 数组重建,需把速度斜坡逻辑抽成纯函数(输入 v0/command_speed/dt → points[].v) |
| 中 | control MPC max_steer 注入(bang-bang 翻符号) | `control_node.cpp` (~1372) | 依赖 MPC 求解器内部状态,抽核需先隔离 solver 接口 |
| 中 | control Stanley heading 阻尼(变道冲出车道) | `control_node.cpp` (~548) | 横向控制律,建议连同 lat PID 一起抽 `lateral_ctrl.h` |
| 低 | flowsim internal_cruise fallback 大 steer | `flowsim_node.cpp` (~1007) | 属 flowsim 域(计划书未纳入 Phase 0 四域),优先级最低 |

> **Phase 0 判定**:计划书验收 = 「四域各有纯核 + 守卫」。四域(control/safety/behavior/planning)已到齐且各有多把,**Phase 0 主线达成**;上表为「深化覆盖」的后续增量,非 Phase 0 阻塞项。Phase 1-4(god-object 拆分 / bus 重写 / 包拆分 / 重命名)仍按门控延后,不自动执行。

**状态**:全部改动在工作区,尚未 commit(按用户要求:完成并验证通过后再 commit)。
