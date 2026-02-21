# Reward Function Analysis: FL-UAV Research Papers vs. Current Implementation

**Date:** 2026-02-19
**Scope:** 16 top-rated (5/5) FL-UAV papers + current multi-agent RL environment
**Purpose:** Identify reward design patterns, diagnose current implementation weaknesses, and derive concrete improvement options
**Audience:** Graduate researcher implementing and refining a UAV multi-agent RL system for federated learning research

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Quick Reference Table: All 16 Papers](#2-quick-reference-table-all-16-papers)
3. [Detailed Analysis: Papers with Exact Equations (P1–P4)](#3-detailed-analysis-papers-with-exact-equations-p1p4)
   - 3.1 [P1: MATD3-TORA — Energy Efficient Multi-Agent DRL](#31-p1-matd3-tora--energy-efficient-multi-agent-drl)
   - 3.2 [P2: Rating-FL — Fast Fair Computation Offloading](#32-p2-rating-fl--fast-fair-computation-offloading)
   - 3.3 [P3: Fed-MARL-PF — Joint Trajectory Optimization](#33-p3-fed-marl-pf--joint-trajectory-optimization)
   - 3.4 [P4: HMDRL-UC — UAV Spectrum Sharing](#34-p4-hmdrl-uc--uav-spectrum-sharing)
4. [Pattern Analysis Across All 16 Papers](#4-pattern-analysis-across-all-16-papers)
   - 4.1 [Cost Minimization vs. Reward Maximization](#41-cost-minimization-vs-reward-maximization)
   - 4.2 [Individual vs. Global/Shared Rewards](#42-individual-vs-globalshared-rewards)
   - 4.3 [Sparse vs. Dense Reward Signals](#43-sparse-vs-dense-reward-signals)
   - 4.4 [Task Completion Term: Present vs. Absent](#44-task-completion-term-present-vs-absent)
   - 4.5 [Fairness: In Reward vs. Separate Metric](#45-fairness-in-reward-vs-separate-metric)
   - 4.6 [Collision Handling: Hard Constraint vs. Penalty](#46-collision-handling-hard-constraint-vs-penalty)
   - 4.7 [Energy Modeling: Included, Excluded, or How](#47-energy-modeling-included-excluded-or-how)
   - 4.8 [Latency Modeling: Included, Excluded, or How](#48-latency-modeling-included-excluded-or-how)
5. [Critical Comparison with Current Implementation](#5-critical-comparison-with-current-implementation)
   - 5.1 [What the Current Implementation Gets Right](#51-what-the-current-implementation-gets-right)
   - 5.2 [Identified Problems](#52-identified-problems)
   - 5.3 [Problem Severity Ranking](#53-problem-severity-ranking)
6. [Recommended Improvements](#6-recommended-improvements)
   - 6.1 [Option A: MATD3-TORA Style (Continuous Cost Minimization)](#61-option-a-matd3-tora-style-continuous-cost-minimization)
   - 6.2 [Option B: MAPPO-H Style (Task + QoS + Energy + Fairness)](#62-option-b-mappo-h-style-task--qos--energy--fairness)
   - 6.3 [Option C: Fed-MARL-PF Style (AoI + Potential Field Shaping)](#63-option-c-fed-marl-pf-style-aoi--potential-field-shaping)
   - 6.4 [Option D: Hybrid Recommended Approach](#64-option-d-hybrid-recommended-approach)
7. [Weights and Scale Analysis](#7-weights-and-scale-analysis)
8. [Summary Recommendations](#8-summary-recommendations)

---

## 1. Executive Summary

This document analyzes how 16 top-rated FL-UAV papers structure their reinforcement learning reward functions and compares them against a current multi-agent RL implementation for UAV task offloading.

### Key Findings

**Finding 1 — The current implementation is reward-maximization with sparse signals; the majority of papers use dense cost-minimization.**
Among the 4 papers with exact equations (P1–P4), 3 of 4 use a pure negative reward that directly encodes a continuous cost (latency, energy). The agent learns to minimize a sum that is non-zero at *every* timestep. The current implementation's primary positive signal (`r_success = +10`) fires only at task completion — a rare event in early training.

**Finding 2 — Binary task-completion rewards are rare in the literature.**
None of the 4 PDF-analyzed papers use a `+reward_when_task_done` term. Instead, they implicitly reward completion by penalizing the delay until completion. The papers that appear to reward "task success" (P6, P9) do so as a rate or probability, not a binary event bonus.

**Finding 3 — The current reward scale is severely imbalanced.**
The collision penalty (`-50`) is 5× larger than the success reward (`+10`), and the battery bonus (`+0.1`) is 500× smaller. This forces the agent to be dominated by collision avoidance and unable to meaningfully weight task success or energy.

**Finding 4 — Fairness appears in reward in only 2 of 16 papers (P4, P14); for most papers it is a monitored metric, not a training signal.**
Embedding fairness directly into the per-step reward adds noise to the learning signal in early training when the distribution is inherently unequal. Papers that achieve high fairness (JFI > 0.99) typically do so through structural choices (min-SNR objective in P4) rather than explicit penalty.

**Finding 5 — Collision avoidance is handled as a hard constraint in 3 of the 4 exact-equation papers (P1, P2, P3); P4 uses link-maintenance probability instead.**
Converting collision from a soft penalty to a hard constraint or potential field simplifies the reward landscape significantly.

**Finding 6 — Energy is modeled at every timestep in all 4 PDF-analyzed papers.**
The current implementation has no per-step energy penalty beyond the weak battery ratio bonus. Energy is the *primary* optimization objective in P1, P3, and P5; omitting it means the agent never learns to trade off computation cost against task latency.

### One-Line Verdict

> The current reward function is sparse, imbalanced, and missing the two signals — latency and energy — that dominate all comparable literature; the most impactful single change is replacing the binary success bonus with a continuous per-step latency/delay penalty.

---

## 2. Quick Reference Table: All 16 Papers

| ID | Paper (Short Name) | Algorithm | Reward Formula (Condensed) | Optimization Direction | Key Components | Reward Type | Individual or Shared | Collision Handling |
|----|-------------------|-----------|---------------------------|----------------------|----------------|-------------|---------------------|-------------------|
| **P1** | MATD3-TORA | MATD3 | `r_u = -η[w0·delay + w1·energy·ζ]` | Minimize cost | Delay, Energy | Dense, continuous | Per-agent | Hard constraint (C6) |
| **P2** | Rating-FL | DQL + FL Rating | `R = Avg(1/power, 1/latency)` | Maximize efficiency | Power, Latency | Dense, continuous | Per-drone | No collision term |
| **P3** | Fed-MARL-PF | MARL + PF | `r_C = -(ω1·AoI + ω2·delay + ω3·energy)` + PF shaping for UAVs | Minimize cost | AoI, Delay, Energy | Dense, central+shaped | Hybrid | Repulsive PF field |
| **P4** | HMDRL-UC | IPPO + MAPPO | CM: `R^m = Ση - ζ1·K_m - ζ2·avg_delay` / CH: `R^h = γ·throughput + (1-γ)·min_SNR` | Maximize utility | Link quality, Throughput, Fairness | Dense, dual-level | Per-role | Isolation penalty |
| **P5** | IEEE 10539111 | DRL (three-tier) | `r ∝ -(latency + energy_cost)` | Minimize cost | Latency, Energy | Dense | Per-agent | N/A (conceptual) |
| **P6** | MADDPG Trajectory | MADDPG | `r ∝ task_completion_rate - α·latency` | Maximize rate | Task rate, Latency | Mixed | Shared | Trajectory constraint |
| **P7** | FL in the Sky | FL optimization | Convergence rate (not RL reward) | Maximize convergence | Communication rounds | Non-RL | Global | N/A |
| **P8** | Model-Aided Fed-QMIX | QMIX + digital twin | `r ∝ data_collected / battery_used` | Maximize efficiency | Coverage, Battery | Dense | QMIX (shared) | N/A |
| **P9** | MAPPO Hierarchical | MAPPO (3-tier) | `r = task_success·QoS_weight - energy_penalty - collision_penalty` | Maximize QoS | Task, QoS, Energy, Collision | Mixed | Per-tier | Soft penalty |
| **P10** | MA-ETCO (Sky) | Multi-agent DRL | `r ∝ -(α·latency + β·energy)` | Minimize cost | Latency, Energy, Multi-hop | Dense | Per-agent | Trajectory constraint |
| **P11** | DECCo | DRL scheduler | `r ∝ -makespan - resource_fragmentation` | Minimize makespan | Makespan, Resource | Dense | Per-drone | Scheduling constraint |
| **P12** | FedULite | FL defense | Model accuracy under attack (not RL reward) | Maximize accuracy | Accuracy, Robustness | Non-RL | Global | N/A |
| **P13** | PPO-DC | PPO | `r = Σuser_throughput - α·trajectory_cost` | Maximize throughput | Sum rate, Coverage | Dense | Per-UAV | Trajectory boundary |
| **P14** | MA-PF-AD3PG | AD3PG + PF | `r = -latency + β·JFI_bonus` | Minimize latency + maximize fairness | Latency, JFI | Dense | Per-agent | Repulsive PF field |
| **P15** | FL-IDS | FL classification | Detection accuracy (not RL reward) | Maximize accuracy | F1, TPR | Non-RL | Federated | N/A |
| **P16** | MARL Cellular UAV | Multi-agent RL | `r ∝ offload_ratio × throughput - interference_penalty` | Maximize offload | Throughput, Interference | Dense | Per-agent | Interference constraint |

**Legend — Reward Type:**
- *Dense*: non-zero signal at every timestep
- *Sparse*: signal only on completion events
- *Mixed*: dense cost + sparse bonus
- *Non-RL*: paper is FL convergence/security, no RL reward

**Papers that are effectively non-RL** (P7, P12, P15): These optimize FL training objectives (convergence, accuracy, poisoning defense) and do not define an RL step reward. They are excluded from reward pattern analysis below.

---

## 3. Detailed Analysis: Papers with Exact Equations (P1–P4)

### 3.1 P1: MATD3-TORA — Energy Efficient Multi-Agent DRL

**Full Citation Context:** "Energy-Efficient Multi-Agent Deep Reinforcement Learning Task Offloading for UAV-Assisted Edge Computing"
**Algorithm:** Multi-Agent Twin Delayed DDPG (MATD3)
**Setting:** M UAVs serving L mobile devices via task offloading with flight energy constraints

#### Exact Reward Equations

The paper defines a **vector reward** for the multi-agent system:

```
R_t = (r_1, r_2, ..., r_u, ..., r_M)                             [Eq. 39]
```

Each UAV u receives its own scalar reward:

```
r_u = r(s_u, a_u) = -η_{l,u}(t) · [w_0 · t_u(t) + w_1 · e_u(t) · ζ]   [Eq. 40]
```

**Variable Definitions:**

| Symbol | Meaning | Range / Units |
|--------|---------|---------------|
| `η_{l,u}(t)` | Binary: 1 if UAV u is serving MD l at time t | {0, 1} |
| `t_u(t)` | Task processing delay (max of local vs. offload path) | seconds |
| `e_u(t)` | Total energy consumption = flight energy + computation energy | Joules |
| `w_0` | Weight for latency component | [0, 1], w_0 + w_1 = 1 |
| `w_1` | Weight for energy component | [0, 1], w_0 + w_1 = 1 |
| `ζ` | Balancing factor to normalize latency vs. energy to same order of magnitude | scalar > 0 |

**Sub-formulas referenced by Eq. 40:**

Task delay `t_u(t)` is the maximum of two sub-paths (Eq. 36–38):
```
t_u(t) = max(t_local(t), t_offload(t))

t_local(t)   = D_l(t) · (1 - α_{l,u}) / f_local
t_offload(t) = D_l(t) · α_{l,u} / R_{l,u}(t) + D_l(t) · α_{l,u} · S / f_u(t)
```

Energy `e_u(t)` (Eq. 18–22):
```
e_u(t) = E_fly(t) + E_comp(t)
E_fly(t) = φ · ||v_u(t)||²
E_comp(t) = κ · [f_u(t)]² · α_{l,u}(t) · D_l(t) · S
```

#### Design Rationale Analysis

**Why pure negative reward?** The system objective is a constrained minimization problem (minimize weighted sum of latency and energy subject to battery, task deadline, and collision constraints). Expressing the objective directly as the reward means the RL problem is identical to the optimization problem — no reward shaping mismatch.

**Why does the reward multiply by `η_{l,u}(t)`?** When a UAV is not serving any device, its reward is 0. This is important: the agent does not receive a penalty for flying to a new position, only a cost when it is actively computing. This incentivizes the UAV to complete service as quickly as possible to minimize the number of timesteps where negative reward accumulates.

**Why is collision a hard constraint?** Constraint C6 (collision avoidance) is enforced in the optimization problem structure, not in the reward. This cleanly separates the safety requirement from the performance objective and prevents the large negative collision penalty from overwhelming the latency/energy tradeoff signal.

**Characteristics Summary:**

- **Direction:** Minimize (reward always ≤ 0)
- **Density:** Dense when serving (η=1), zero otherwise
- **Scope:** Per-agent, individual
- **Task completion:** Implicit (delay goes to 0 when task finishes), no binary bonus
- **Fairness:** Absent from reward; fairness can emerge from distributed optimization
- **Energy:** Explicit, every step
- **Latency:** Explicit, every step
- **Collision:** Hard constraint, not in reward

---

### 3.2 P2: Rating-FL — Fast Fair Computation Offloading

**Full Citation Context:** "Fast and Fair Computation Offloading in Swarm of Drones Using Rating-Based Federated Learning"
**Algorithm:** Deep Q-Learning (DQL) with federated model aggregation + a post-hoc Rating system
**Setting:** Drone swarm with multiple computation modules per drone; FL used to share Q-function weights

#### Exact Reward Equation

```
R(s_t, a_t) = Avg(P_m^{D_i},  1/T_m^{D_i})                       [Eq. 24]
```

This is the **arithmetic mean** of two normalized scores:
- `P_m^{D_i}`: A score for computation power consumption of module m in drone D_i (higher = more power-efficient)
- `1/T_m^{D_i}`: Reciprocal of computation time of module m in drone D_i (higher = faster)

The Q-function is updated via Double DQL (Eq. 25):
```
Q(s_t, a_t) ← r_t + α · Q(s_{t+1}, argmax_{a'} Q_2(s_{t+1}, a'))
```

#### Fairness Metric (NOT in reward — Eq. 21)

Fairness is measured by **Jain's Fairness Index (JFI)** but only for monitoring:
```
F_i = [Σ_{k=1}^{K} f(E_k, L_k)]² / [K · Σ_{k=1}^{K} f(E_k, L_k)²]
```
- K: number of drones in the swarm
- `f(E_k, L_k)`: utility function combining energy E_k and latency L_k for drone k
- F_i ∈ [1/K, 1], where 1 = perfectly fair

**Key Insight:** The paper achieves fairness through the **Rating System** (Eq. 28), not through the reward signal:
```
Rating_j = [w1·B_j + w2·E_j + w3·f_j + w4·F_j + w5·P_j] / [w6·L_j]
```
- B_j: battery level
- E_j: energy efficiency score
- f_j: frequency availability
- F_j: Jain's Fairness Index score
- P_j: CPU performance score
- L_j: link quality (denominator — penalize poor links)

The Rating system determines **which drones aggregate their FL models** with the server, selectively including drones that balance efficiency and fairness — a structural fairness mechanism independent of the RL reward.

#### Design Rationale Analysis

**Why average of power_score and 1/latency?** This creates a balanced objective: an action that minimizes latency but wastes energy is scored the same as one that saves energy but is slow. Neither term dominates the other when they are averaged together.

**Why is fairness separate?** Fairness at the individual action level is impossible to compute — a single drone cannot know whether it is being "fair" without observing all other drones' states. The Rating system addresses this at aggregation time, not at step time.

**Characteristics Summary:**

- **Direction:** Maximize (reward always ≥ 0)
- **Density:** Dense, every action step
- **Scope:** Per-drone DQL; global fairness via Rating aggregation
- **Task completion:** Implicit in latency term
- **Fairness:** Separate mechanism (Rating system), not in step reward
- **Energy:** Explicit via power score
- **Latency:** Explicit via 1/T term
- **Collision:** Not addressed (paper focuses on computation offloading, not navigation)

---

### 3.3 P3: Fed-MARL-PF — Joint Trajectory Optimization

**Full Citation Context:** "Joint Trajectory and Offloading Optimization in UAV-Assisted MEC via Federated Multi-Agent RL and Potential Fields"
**Algorithm:** Federated MARL with Potential Field (PF) reward shaping; agents train locally and share policies via FL
**Setting:** Multiple UAVs collecting IoT sensor data; optimize Age of Information (AoI), delay, and energy simultaneously

#### Architecture: Two Reward Streams

P3 uses a **hybrid architecture** with a central coordination agent and individual UAV agents, each receiving different rewards.

#### Central Agent Reward (Eq. 19)

The central agent minimizes a weighted sum across the entire network:

```
P(t) = ω1 · Ā(t) + ω2 · Σ_d T_d(t) + ω3 · Σ_u E_u(t)
r_C(t) = -P(t)
```

| Symbol | Meaning | Notes |
|--------|---------|-------|
| `Ā(t)` | Average Age of Information across all transmitting devices | AoI measures data freshness |
| `T_d(t)` | Processing delay for device d (transmission + computation) | Summed over all devices |
| `E_u(t)` | Energy consumption of UAV u (flight + hover + compute) | Summed over all UAVs |
| `ω1, ω2, ω3` | Relative importance weights | ω1 + ω2 + ω3 = 1 |

**What is Age of Information (AoI)?** AoI measures the time elapsed since the last successfully received update from a sensor. It increases with every timestep the UAV is not collecting from that device and resets to zero on collection. AoI replaces a binary "task done" signal with a continuous freshness cost — the longer a device goes un-served, the more it hurts the central reward.

#### UAV Agent Reward: Potential Field Shaping

Each individual UAV agent u receives a reward shaped by an artificial potential field that:
1. **Attracts** the UAV toward sensor devices that have pending data (gravitational field)
2. **Repels** the UAV away from base stations and obstacles (repulsive field)

**Gravitational (attractive) field toward transmitting device d (Eq. 1):**
```
Φ_gra^(d)(x) = (1 / (2·N_d)) · ξ · α²(pos(x), pos(d))
```
- `N_d`: number of data packets pending at device d
- `ξ`: gravitational gain constant
- `α(pos(x), pos(d))`: Euclidean distance from UAV position x to device d

**Repulsive field from base station b (Eq. 2):**
```
Φ_rep^(b)(x) = { ½η · (1/α - 1/α₀)²   if α ≤ α₀
               { 0                       otherwise
```
- `η`: repulsive gain constant
- `α₀`: influence radius of the repulsive field (only active when close enough)
- `α`: distance from UAV to base station b

**Total potential field (Eq. 3):**
```
Φ(x) = Σ_b Φ_rep^(b)(x) + Σ_d Φ_gra^(d)(x)
```

**UAV agent reward:** The reward is the negative gradient of the total field — moving toward a high-data device reduces Φ, which decreases the negative reward. Navigation toward desired positions is thus continuously rewarded without requiring explicit collision penalties or discrete task-completion events.

#### Design Rationale Analysis

**Why AoI instead of task completion?** AoI provides a smooth, always-increasing cost that the agent can act on continuously. A binary "collected/not" would be sparse. AoI grows quadratically with time (since the AoI update formula squares the elapsed time in some variants), creating urgency to serve devices quickly.

**Why potential fields for UAV agents?** Potential fields provide smooth, analytical gradients that guide navigation without requiring the agent to discover through trial-and-error that flying toward a device is beneficial. This dramatically accelerates early training convergence.

**Why federated MARL?** Each UAV trains locally using its own trajectory experience; FL aggregation allows policy sharing without sharing raw sensor data (privacy), and the central agent's global reward provides the system-level coordination signal.

**Characteristics Summary:**

- **Direction:** Minimize (r_C always ≤ 0), UAV reward shaped by PF gradients
- **Density:** Dense at every step (AoI increments every step)
- **Scope:** Central agent (global), UAV agents (local PF)
- **Task completion:** Replaced by AoI (continuous freshness metric)
- **Fairness:** Implicit in ω1 weighting of average AoI across all devices
- **Energy:** Explicit, per timestep in central reward
- **Latency:** Explicit as T_d, per timestep
- **Collision:** Handled by repulsive potential field — no penalty term needed

---

### 3.4 P4: HMDRL-UC — UAV Spectrum Sharing

**Full Citation Context:** "Heterogeneous Multi-Agent Deep Reinforcement Learning for Cluster-Based Spectrum Sharing in UAV Swarms"
**Algorithm:** IPPO for Cluster Members (CM) + MAPPO for Cluster Heads (CH) — hence "Heterogeneous"
**Setting:** UAV swarm organized into clusters; optimize spectrum sharing, link quality, and cluster throughput

#### Architecture: Role-Differentiated Rewards

P4 uses **two completely different reward functions** for two agent roles. This is the most architecturally distinct design in the 16-paper set.

#### Cluster Member (CM) Reward — IPPO-M Algorithm (Eq. 16)

```
R^m = Σ_{j ∈ ε(m)} η_{m,j}  -  ζ_1 · K_m  -  (ζ_2 / |ε(m)|) · Σ_{f ∈ ε(m)} τ_{m,f}
```

| Symbol | Meaning | Effect |
|--------|---------|--------|
| `Σ η_{m,j}` | Sum of link maintenance probabilities to neighbors j in neighborhood ε(m) | Positive: reward stable links |
| `ζ_1 · K_m` | Isolation penalty (K_m = 1 if UAV m is isolated from cluster, 0 otherwise) | Negative: penalize isolation |
| `(ζ_2 / ε(m)) · Σ τ_{m,f}` | Average communication delay across all links in neighborhood | Negative: penalize high delay |
| `ζ_1, ζ_2` | Discount factors controlling relative weight of isolation vs. delay penalties | Hyperparameters |

**Design intent:** Each CM optimizes for its own connectivity health — maintaining reliable links to neighbors, avoiding isolation (which would disconnect it from the cluster), and minimizing its own communication delay. Since all CMs use IPPO (Independent PPO), they do not share a value function; they must learn cooperative behavior through interaction.

#### Cluster Head (CH) Reward — MAPPO-H Algorithm (Eqs. 18–20)

CHA receives a **convex combination** of global throughput and local fairness:

**Global throughput (Eq. 18):**
```
r_global = Σ_{h ∈ H} Σ_{e ∈ β(h)} B_{e,h} · log₂(1 + Φ_{e,h})
```
- `B_{e,h}`: bandwidth allocated to edge member e by cluster head h
- `Φ_{e,h}`: SNR of the link between e and h
- `Shannon capacity formula` — measures total information throughput

**Local fairness (Eq. 19):**
```
r_h,local = min_{e ∈ β(h)} Φ_{e,h}
```
This is the **min-max fairness** criterion: the reward for cluster head h is the *minimum* SNR among all its members. Maximizing this term forces the CH to improve the link quality of the worst-connected member, rather than focusing on already-strong links.

**Combined CH reward (Eq. 20):**
```
R^h = γ^h · r_global + (1 - γ^h) · r_h,local
```
- `γ^h ∈ [0, 1]`: tradeoff hyperparameter per cluster head
- `γ^h → 1`: prioritize total throughput
- `γ^h → 0`: prioritize worst-case member fairness

Since all CHs use MAPPO (Multi-Agent PPO with centralized critic), they share a global value function during training, enabling them to coordinate spectrum allocation across the entire swarm.

#### Design Rationale Analysis

**Why two reward functions?** CMs and CHs have fundamentally different roles: CMs maintain connectivity, CHs allocate spectrum. A single reward function would create conflicting objectives (spectrum allocation logic is irrelevant to a CM). Role-differentiated rewards allow each agent type to focus on its actual decision space.

**Why min-SNR for fairness?** The minimum operator creates a direct incentive to improve the weakest link. A JFI-based fairness term would create a softer gradient that allows the CH to neglect one poor-quality member as long as the average looks good. The min operator is more robust and simpler to compute.

**Why MAPPO for CHs but IPPO for CMs?** CHs make globally coordinated spectrum decisions that affect all clusters simultaneously — shared value function (MAPPO) enables them to account for inter-cluster interference. CMs operate locally and independently; IPPO is sufficient and scales better as the swarm grows.

**Characteristics Summary:**

- **Direction:** Maximize (both rewards are positive-valued objectives)
- **Density:** Dense, every timestep
- **Scope:** Per-agent-role (two distinct reward streams)
- **Task completion:** Not applicable (spectrum sharing, not task offloading)
- **Fairness:** Embedded in CH reward via min-SNR (structural), not JFI penalty
- **Energy:** Absent (spectrum sharing focus; energy is a separate constraint)
- **Latency:** Delay term in CM reward (τ_{m,f})
- **Collision:** Not in reward; cluster membership implicitly controls spatial separation

---

## 4. Pattern Analysis Across All 16 Papers

This section categorizes all 13 RL papers (P7, P12, P15 are FL-only and excluded) by reward design dimensions.

### 4.1 Cost Minimization vs. Reward Maximization

| Approach | Papers | Fraction |
|----------|--------|----------|
| **Cost minimization** (negative reward, always ≤ 0) | P1, P3, P5, P10, P11 | 5 / 13 (38%) |
| **Reward maximization** (positive utility) | P2, P4, P8, P13, P16 | 5 / 13 (38%) |
| **Mixed** (positive bonus + negative penalties) | P6, P9, P14 | 3 / 13 (23%) |

**Takeaway:** No single convention dominates. However, the cost-minimization papers tend to have *simpler, more theoretically grounded* objective functions because they directly encode the optimization problem. Reward-maximization papers tend to produce more intuitive rewards but require careful normalization.

The current implementation is **Mixed** (it has both positive bonuses and negative penalties), which is the least common approach and the most prone to scale imbalance.

### 4.2 Individual vs. Global/Shared Rewards

| Reward Scope | Papers | Fraction |
|-------------|--------|----------|
| **Per-agent individual** | P1, P2, P5, P10, P13, P16 | 6 / 13 (46%) |
| **Global/shared** (QMIX, MAPPO critic) | P8, P11 | 2 / 13 (15%) |
| **Hybrid** (per-agent + global critic or central agent) | P3, P4, P6, P9, P14 | 5 / 13 (38%) |

**Takeaway:** Hybrid designs are common in high-complexity settings (spectrum sharing, multi-tier computing). Pure per-agent rewards are the simplest and most common. The current implementation uses per-agent rewards computed from global state (fairness CV) — a design inconsistency where the signal is global but only one agent acts on it.

### 4.3 Sparse vs. Dense Reward Signals

| Signal Type | Papers | Fraction |
|-------------|--------|----------|
| **Dense** (non-zero every step) | P1, P2, P3, P4, P5, P8, P10, P11, P13, P14, P16 | 11 / 13 (85%) |
| **Sparse** (event-triggered only) | None | 0 / 13 (0%) |
| **Mixed** (dense background + sparse bonus) | P6, P9 | 2 / 13 (15%) |

**Takeaway:** This is the strongest and most uniform pattern in the literature. **Virtually every paper uses dense reward signals.** The current implementation's `r_success = +10` is sparse (fires only at task completion) and is the dominant positive component. This creates an 85% literature divergence on the most fundamental reward property.

### 4.4 Task Completion Term: Present vs. Absent

| Task Completion Handling | Papers | Fraction |
|--------------------------|--------|----------|
| **Absent** — implicit via latency/AoI minimization | P1, P2, P3, P5, P10, P11 | 6 / 13 (46%) |
| **Present as rate/probability** (not binary bonus) | P4, P8, P13, P16 | 4 / 13 (31%) |
| **Present as binary bonus** | P6, P9 | 2 / 13 (15%) |
| **Not applicable** (spectrum/security focus) | P4 | 1 / 13 (8%) |

**Takeaway:** The binary `+10 per completion` pattern used in the current implementation appears in only 2 of 13 papers (15%), and both of those papers pair it with a dense energy/latency cost signal. No paper uses a binary completion bonus as the *primary* reward component.

### 4.5 Fairness: In Reward vs. Separate Metric

| Fairness Handling | Papers | Fraction |
|-------------------|--------|----------|
| **In reward** (direct penalty or objective component) | P4 (min-SNR), P14 (JFI bonus) | 2 / 13 (15%) |
| **Monitored separately** (JFI computed, not trained on) | P2, P6, P9 | 3 / 13 (23%) |
| **Structural** (achieved via architecture, not reward) | P3 (AoI averaging), P4 (min-operator) | overlap |
| **Not addressed** | P1, P5, P8, P10, P11, P13, P16 | 7 / 13 (54%) |

**Takeaway:** Fairness in the reward signal is uncommon and paper-specific. When it appears, it uses the minimum operator (P4) or a JFI multiplier (P14) — not the Coefficient of Variation (CV) penalty used in the current implementation. The current implementation's fairness term adds noise to a signal that the literature suggests does not need to be in the reward at all.

### 4.6 Collision Handling: Hard Constraint vs. Reward Penalty

| Collision Handling | Papers | Fraction |
|-------------------|--------|----------|
| **Hard constraint** (infeasible action rejected) | P1 (C6), P2 (implicit), P11 | 3 / 13 (23%) |
| **Potential field repulsion** (smooth soft avoidance) | P3, P14 | 2 / 13 (15%) |
| **Soft penalty in reward** | P9 | 1 / 13 (8%) |
| **Not addressed / not applicable** | P4, P5, P6, P8, P10, P13, P16 | 7 / 13 (54%) |

**Takeaway:** When collision is modeled, hard constraints or potential fields are preferred over soft penalties. Only 1 of 13 papers uses a large collision penalty in the reward the way the current implementation does (`-50`). The large collision penalty in the current implementation dominates learning without being the typical choice.

### 4.7 Energy Modeling: Included, Excluded, or How

| Energy Handling | Papers | How Modeled |
|----------------|--------|-------------|
| **Explicit per-step penalty** | P1, P3, P5, P10 | `E_fly + E_comp` with physics formulas |
| **Efficiency ratio** | P2, P8 | `1/power_score`, `data/battery` |
| **Battery state** | P9 | Remaining battery as constraint |
| **Absent** | P4, P6, P11, P13, P14, P16 | Not in reward |

**Takeaway:** When energy appears in the reward, it is **modeled physically** (flight energy from velocity, computation energy from CPU frequency and task size). The current implementation's `r_battery = (battery/capacity) * 0.1` is a battery-state ratio, not an energy consumption penalty — it rewards having a full battery rather than penalizing energy consumption. This is backwards: the agent is rewarded for not using energy, not penalized for wasting it.

### 4.8 Latency Modeling: Included, Excluded, or How

| Latency Handling | Papers | How Modeled |
|-----------------|--------|-------------|
| **Explicit delay minimization** | P1, P3, P5, P10 | Transmission delay + computation delay per task |
| **Implicit** (1/T reciprocal) | P2 | `1/T_m` as reward component |
| **AoI** (freshness-based latency) | P3 | AoI increments per timestep |
| **Sum rate / throughput** | P13, P16 | User throughput as proxy for low latency |
| **QoS satisfaction** | P9 | Binary QoS class satisfaction |
| **Absent** | P4, P6, P8, P11, P14 | Not in step reward |

**Takeaway:** Latency is the second most common component after energy. Among task-offloading papers specifically, **latency appears in 100% of them** (P1, P2, P3, P5, P10). The current implementation has no latency signal whatsoever.

---

## 5. Critical Comparison with Current Implementation

### Current Implementation (Reference)

```python
# Per-step reward components:
r_success   = completed_this_step * 10.0        # +10 per task completed (rare event)
r_failure   = failed_this_step * (-5.0)         # -5 per task failed
r_battery   = (battery / capacity) * 0.1        # +0.0 to +0.1 (tiny continuous bonus)
r_fairness  = -fairness_penalty                 # -0.0 to -0.5 (CV-based, always negative)
r_collision = 0 or -50                          # large penalty when colliding

total_reward = r_success + r_failure + r_battery + r_fairness + r_collision
# Dead UAV penalty: -100
```

### 5.1 What the Current Implementation Gets Right

| Aspect | Assessment | Literature Support |
|--------|------------|-------------------|
| **Per-agent individual rewards** | Correct approach for MARL | P1, P2, P10 all use per-agent rewards |
| **Penalizing task failure** | Reasonable — distinguishes failed from unstarted tasks | Implicit in P1, P9 |
| **Including a battery/energy signal** | Correct direction (energy matters) | P1, P2, P3, P5, P8 all include energy |
| **Collision penalty present** | Addresses a real safety concern | P9 uses soft penalty; others use hard constraints |
| **Mixed reward structure** | Acceptable but requires careful scaling | P6, P9, P14 use mixed approach |

### 5.2 Identified Problems

#### Problem 1: Severe Reward Imbalance (Critical)

**Description:** The reward components span 4 orders of magnitude with no normalization:

| Component | Min Value | Max Value | Typical Frequency |
|-----------|-----------|-----------|-------------------|
| `r_success` | 0 | +10.0 | Rare (on task completion) |
| `r_failure` | -5.0 | 0 | Rare (on task failure) |
| `r_battery` | 0 | +0.1 | Every step |
| `r_fairness` | -0.5 | 0 | Every step |
| `r_collision` | -50 | 0 | Rare (on collision) |
| Dead penalty | -100 | 0 | Rare (once per episode) |

The collision penalty (`-50`) is:
- 5× larger than the success reward (`+10`)
- 500× larger than the battery bonus (`+0.1`)
- 100× larger than the fairness penalty (`-0.5`)

**Effect:** During early training when collisions are common, the agent learns almost exclusively to avoid collisions. By the time collision avoidance is learned, the gradient from `r_success` is too weak to guide task completion policy. The agent converges to a "hover safely and do nothing" policy.

**Literature contrast:** P1 uses `ζ` to normalize latency and energy to the same order. P4 uses `γ^h` to balance throughput vs. fairness explicitly. No paper in the set uses components that differ by more than 10× without explicit normalization.

#### Problem 2: Sparse Primary Signal (Critical)

**Description:** `r_success = +10` fires only at task completion. In an environment with many UAVs and limited tasks per episode, this may fire fewer than 5 times in a 1000-step episode. The agent receives almost no gradient information about whether its actions are leading *toward* task completion.

**Effect:** Extremely slow learning on the task objective. The agent cannot distinguish "I am currently serving a device and about to complete" from "I am flying away from all devices."

**Literature contrast:** Every task-offloading paper in the literature (P1, P2, P3, P5, P10) provides a per-step cost that is non-zero throughout task service (delay accumulating, energy consumed). The signal is dense: every timestep the agent receives feedback proportional to how efficiently it is serving.

#### Problem 3: Battery Bonus vs. Energy Penalty (Significant)

**Description:** `r_battery = (battery/capacity) * 0.1` rewards having a *high battery level*, not efficient energy use. This creates a perverse incentive: the agent is rewarded for doing nothing (conserving battery) rather than for completing tasks efficiently.

**Effect:** The agent may learn to hover instead of flying to tasks because hovering preserves battery level and thus maintains the battery bonus. This directly conflicts with the task completion objective.

**Literature contrast:** P1 explicitly penalizes `E_fly + E_comp` at every step. P2 rewards `1/power_consumption`. P8 rewards `data_collected / battery_used`. All of these penalize energy use relative to work accomplished, not reward energy possession.

**Correct formulation:** Energy reward should be `-energy_consumed_this_step` or `-(E_fly + E_compute)`, not `+battery_remaining`.

#### Problem 4: Global Fairness Signal in Per-Agent Reward (Significant)

**Description:** `r_fairness = -CV(workload_distribution)` is computed using the global distribution of task assignments across all UAVs. This global signal is applied uniformly to each agent's per-agent reward.

**Effect (two sub-problems):**
1. **Non-stationarity:** Each agent's reward depends on all other agents' actions, making the environment non-stationary from any single agent's perspective. This violates the CTDE (Centralized Training, Decentralized Execution) assumption when the fairness signal varies based on global state that no individual agent can control.
2. **Uninformative gradient:** The agent cannot determine which of its own actions caused the fairness to improve or degrade. The fairness penalty is essentially random noise from the agent's local perspective.

**Literature contrast:** P4 addresses fairness at the *cluster head* level using `min-SNR` — a local signal each CH can directly control. P14 uses JFI as a component but pairs it with an AD3PG architecture specifically designed for global-local reward decomposition. P2 handles fairness entirely outside the reward (Rating system).

#### Problem 5: No Latency Signal (Significant)

**Description:** The current reward has no term related to how long tasks take to complete. The agent receives `+10` whether a task completes in 1 step or 100 steps.

**Effect:** The agent has no incentive to prioritize fast task completion. It may learn to accept very slow service rates as long as tasks eventually complete.

**Literature contrast:** Latency is the *primary* objective in P1 (w_0 weight on t_u), P2 (1/T term), P3 (T_d in central reward and AoI accumulation), P5, P10. Among task-offloading papers, latency minimization appears in 100% of them.

#### Problem 6: Scale Mismatch Creates Asymmetric Pressure (Moderate)

**Description:** The current reward creates strong pressure to avoid collisions and very weak pressure to complete tasks. Even in collision-free episodes, the effective signal is dominated by the fairness penalty (`-0.0 to -0.5`) since `r_battery` is negligible and `r_success` is sparse.

**Effect:** In collision-free episodes, the agent essentially learns only from fairness feedback — which as noted above, is a global signal the agent cannot interpret locally.

### 5.3 Problem Severity Ranking

| Rank | Problem | Impact | Difficulty to Fix | Priority |
|------|---------|--------|-------------------|----------|
| 1 | Sparse primary signal (no per-step task progress) | Critical — slow convergence | Low — add per-step delay penalty | **Immediate** |
| 2 | Battery bonus vs. energy penalty (wrong direction) | Critical — perverse incentive | Low — flip sign and formula | **Immediate** |
| 3 | Reward scale imbalance (4 orders of magnitude) | Critical — dominance by collision term | Medium — requires normalization | **Immediate** |
| 4 | No latency signal | Significant — agent ignores speed | Low — add per-step delay | **High** |
| 5 | Global fairness in per-agent reward | Significant — non-stationary noise | Medium — restructure or remove | **High** |
| 6 | Collision as soft penalty vs. hard constraint | Moderate — learning dominated early on | Medium — requires env change | **Medium** |

---

## 6. Recommended Improvements

The following options are derived directly from paper patterns. Each option is self-contained.

### 6.1 Option A: MATD3-TORA Style (Continuous Cost Minimization)

**Based on:** P1 (Eq. 40)
**Design philosophy:** Convert the reward to a pure negative cost signal. Every step produces a non-zero reward proportional to current latency and energy consumption. No binary events.
**Best for:** Scenarios where task latency and energy are the primary optimization metrics.

```python
def compute_reward_option_a(uav_state, env_config):
    """
    P1-style continuous cost minimization.

    Reward is always <= 0. The agent minimizes the weighted sum of
    current task delay and energy consumption.

    Hyperparameters:
        w0: latency weight (default 0.5)
        w1: energy weight (default 0.5)
        zeta: normalization factor to equalize latency and energy scales
    """
    w0    = env_config.get("w_latency", 0.5)
    w1    = env_config.get("w_energy", 0.5)
    zeta  = env_config.get("zeta", 0.01)  # tune so w1*energy ~ w0*latency

    is_serving = uav_state.is_serving_device       # binary: 1 if serving, 0 otherwise
    current_delay  = uav_state.current_task_delay  # seconds or normalized time units
    energy_used    = uav_state.energy_this_step    # Joules = E_fly + E_compute

    # Core P1 reward: non-zero only when serving, always negative
    r_cost = -is_serving * (w0 * current_delay + w1 * energy_used * zeta)

    # Collision: hard constraint (return a very large penalty and terminate)
    # instead of soft penalty in the reward
    if uav_state.has_collision:
        return -1000.0  # effectively terminates episode

    return r_cost

# Expected reward range: [-1.0, 0.0] when zeta is tuned correctly
# Components should be same order of magnitude: |w0*delay| ~ |w1*energy*zeta|
```

**Pros:**
- Dense signal: non-zero every step during service
- Single scale: all terms normalized by w0+w1=1 and zeta
- Directly encodes the optimization objective
- No binary events to tune

**Cons:**
- Reward is zero when not serving — agent may not know to fly toward devices
- Requires accurate delay and energy measurement per step
- Learning that `r=0` (not serving) is bad requires exploration

**Recommended addition to Option A:** Combine with a navigation incentive (distance reduction bonus):
```python
# Add: reward for reducing distance to nearest unserved device
distance_reduction = prev_dist_to_target - curr_dist_to_target
r_navigation = 0.01 * distance_reduction  # small, dense, guides exploration
```

---

### 6.2 Option B: MAPPO-H Style (Task + QoS + Energy + Fairness)

**Based on:** P9 (MAPPO Hierarchical) + elements of P4 CH reward
**Design philosophy:** Keep the task-completion signal but make it denser by replacing binary completion with a probability/rate signal, and add QoS class bonuses. More complex but closer to the current implementation's intent.
**Best for:** Scenarios where different task types have different QoS requirements and explicit task completion tracking matters.

```python
def compute_reward_option_b(uav_state, env_config):
    """
    P9-style: task success (rate-based) + QoS bonus - energy - collision.

    Key change from current impl:
    - r_success is per-step progress toward completion (dense), not binary bonus
    - r_energy is per-step cost (negative), not battery bonus (positive)
    - r_fairness removed from per-agent reward (monitor separately)
    - collision is proportionally scaled to success signal
    """
    # --- Task progress (replaces sparse binary +10) ---
    # Instead of +10 on completion, give +progress_toward_completion per step
    # progress = fraction of task completed this step (0 to 1 per step)
    task_progress    = uav_state.task_progress_this_step  # in [0, 1]
    r_task_progress  = task_progress * 2.0                # +0 to +2.0 per step (dense)

    # Keep completion bonus but smaller, since progress already rewards the journey
    r_completion     = uav_state.completed_this_step * 3.0  # +3.0 on completion (sparse bonus)

    # --- QoS satisfaction bonus ---
    qos_satisfied    = uav_state.qos_met_this_step     # binary: 1 if task met latency deadline
    r_qos            = qos_satisfied * 1.0             # +1.0 if on-time delivery

    # --- Energy penalty (replaces battery bonus) ---
    energy_consumed  = uav_state.energy_this_step      # Joules: E_fly + E_compute
    energy_budget    = env_config.get("energy_budget_per_step", 10.0)  # normalize
    r_energy         = -(energy_consumed / energy_budget)  # -0.0 to -1.0 (normalized)

    # --- Failure penalty ---
    r_failure        = uav_state.failed_this_step * (-2.0)   # -2.0 per failure

    # --- Collision penalty (scaled proportionally to success signal) ---
    # Max collision penalty = 3x max per-step success signal
    r_collision      = uav_state.has_collision * (-(2.0 + 3.0))  # -5.0 (was -50, now -5)

    # NOTE: Fairness removed from per-agent reward
    # Monitor JFI or CV as evaluation metric but do NOT train on it
    # r_fairness = 0  (omitted)

    total = r_task_progress + r_completion + r_qos + r_energy + r_failure + r_collision

    # Dead UAV: scale to ~3x episode worth of worst-case reward
    if uav_state.is_dead:
        total += -20.0  # reduced from -100

    return total

# Expected range per step: approximately [-7.0, +5.0]
# Components are within 3x of each other (no 500x imbalance)
```

**Scale analysis for Option B:**

| Component | Range | Frequency | Expected per Episode |
|-----------|-------|-----------|----------------------|
| `r_task_progress` | [0, +2.0] | Dense | High |
| `r_completion` | 0 or +3.0 | Sparse | Low |
| `r_qos` | 0 or +1.0 | Sparse | Low |
| `r_energy` | [-1.0, 0] | Dense | Moderate |
| `r_failure` | -2.0 or 0 | Sparse | Low |
| `r_collision` | -5.0 or 0 | Rare | Very Low |

**Pros:**
- Preserves the task-completion feedback the current system uses
- Dense per-step progress signal resolves the sparse signal problem
- Energy penalty is in the correct direction
- Collision scaled proportionally to success

**Cons:**
- Requires defining `task_progress_this_step` (may require env changes)
- More components to tune
- Still has a mix of sparse and dense signals

---

### 6.3 Option C: Fed-MARL-PF Style (AoI + Potential Field Shaping)

**Based on:** P3 (Fed-MARL-PF)
**Design philosophy:** Replace task completion with AoI (a continuously-updating freshness metric) and use potential fields to provide navigation guidance without explicit collision penalties.
**Best for:** Scenarios where data freshness matters (IoT sensor collection, periodic updates) and where trajectory optimization is central to the problem.

```python
import numpy as np

def compute_gravitational_potential(uav_pos, device_pos, n_pending_tasks, xi=1.0):
    """
    P3 Eq. 1: Attractive potential toward devices with pending tasks.
    Returns NEGATIVE value (lower = closer, better).
    """
    dist = np.linalg.norm(uav_pos - device_pos)
    if dist < 1e-6:
        dist = 1e-6
    # Attractive: potential DECREASES as UAV moves toward device
    # Weighted by number of pending tasks (urgency)
    return (1.0 / (2.0 * max(n_pending_tasks, 1))) * xi * dist**2

def compute_repulsive_potential(uav_pos, obstacle_pos, eta=1.0, alpha0=20.0):
    """
    P3 Eq. 2: Repulsive potential from obstacles/base stations.
    Returns POSITIVE value (higher = closer, worse).
    """
    dist = np.linalg.norm(uav_pos - obstacle_pos)
    if dist >= alpha0:
        return 0.0  # outside influence radius
    return 0.5 * eta * (1.0/dist - 1.0/alpha0)**2

def compute_aoi(device_state, dt=1.0):
    """
    Age of Information: increases every step, resets on data collection.
    Higher AoI = data is stale = higher cost.
    """
    if device_state.collected_this_step:
        return 0.0  # AoI resets on collection
    return device_state.time_since_last_collection * dt

def compute_reward_option_c(uav_state, all_device_states, all_obstacle_positions,
                             env_config, omega=(0.4, 0.3, 0.3)):
    """
    P3-style: UAV agents use potential field navigation reward.
    A separate central agent (or global critic) receives AoI + delay + energy.

    omega: (omega_aoi, omega_delay, omega_energy) weights for central reward
    """
    omega_aoi, omega_delay, omega_energy = omega

    # --- Central reward (for global critic / central agent) ---
    # Average AoI across all devices
    avg_aoi = np.mean([compute_aoi(d) for d in all_device_states])

    # Total delay this step (transmission + computation for active tasks)
    total_delay = sum(d.current_delay for d in all_device_states if d.being_served)

    # Total energy this step
    total_energy = uav_state.energy_this_step

    r_central = -(omega_aoi * avg_aoi + omega_delay * total_delay +
                  omega_energy * total_energy)

    # --- UAV agent reward: potential field ---
    # Gravitational: attract toward pending devices
    phi_gra = sum(
        compute_gravitational_potential(
            uav_state.position,
            d.position,
            d.n_pending_tasks
        )
        for d in all_device_states if d.n_pending_tasks > 0
    )

    # Repulsive: push away from obstacles
    phi_rep = sum(
        compute_repulsive_potential(uav_state.position, obs_pos)
        for obs_pos in all_obstacle_positions
    )

    phi_total = phi_gra + phi_rep

    # UAV reward: negative total potential (minimize = move toward devices, away from obstacles)
    r_uav = -phi_total

    # In a CTDE setting: use r_central for centralized critic during training
    #                    use r_uav for decentralized execution
    # During inference: return r_uav
    return r_uav, r_central

# Expected range: r_uav in [-inf, 0], bounded by tuning xi, eta, alpha0
# Typical: [-5.0, 0.0] with reasonable hyperparameter choices
```

**Pros:**
- Eliminates sparse signals entirely — every step has meaningful gradient
- Collision avoidance is smooth and analytic (no hard -50 cliff)
- AoI provides urgency signal that grows over time (encourages priority scheduling)
- Most principled approach for trajectory + collection scenarios

**Cons:**
- Requires significant environment refactoring (AoI tracking, obstacle positions)
- Potential field hyperparameters (xi, eta, alpha0) require careful tuning
- Less interpretable than simple reward components
- AoI may not apply if tasks are not data-collection oriented

---

### 6.4 Option D: Hybrid Recommended Approach

**Based on:** Best practices across P1, P2, P9, P14
**Design philosophy:** Preserve the task-completion intent of the current implementation while fixing all identified critical and high-severity problems. Minimal refactoring required.
**Best for:** Immediate improvement to the current environment with the lowest implementation risk.

```python
def compute_reward_option_d(uav_state, env_config):
    """
    Hybrid recommended approach: fixes all critical problems in the current impl.

    Changes from current implementation:
    1. r_success: reduced from +10, added per-step progress signal (dense)
    2. r_energy: replace battery bonus with energy consumption penalty (correct direction)
    3. r_latency: new term — penalizes delay per step (was absent entirely)
    4. r_collision: scaled down from -50 to -5 (within 2x of success magnitude)
    5. r_fairness: REMOVED from per-agent reward (monitor as eval metric only)
    6. r_failure: kept at -2.0 (scaled down from -5)

    Target reward range: approximately [-4.0, +3.0] per step
    All components within 4x of each other
    """
    # Hyperparameters — tune these, do NOT change the formula structure
    PROGRESS_SCALE   = 1.5    # per-step task progress reward weight
    COMPLETION_BONUS = 2.0    # sparse bonus on task completion
    LATENCY_WEIGHT   = 0.5    # per-step delay penalty weight
    ENERGY_WEIGHT    = 0.5    # per-step energy penalty weight
    LATENCY_NORM     = 10.0   # expected max delay per step (normalize to [-1, 0])
    ENERGY_NORM      = 5.0    # expected max energy per step (normalize to [-1, 0])
    COLLISION_PENALTY = -5.0  # soft penalty (down from -50)
    FAILURE_PENALTY  = -2.0   # per task failure (down from -5)
    DEAD_PENALTY     = -20.0  # per episode (down from -100)

    # 1. Task progress (DENSE — replaces or supplements completion bonus)
    task_progress   = uav_state.task_fraction_completed_this_step  # [0.0, 1.0]
    r_progress      = PROGRESS_SCALE * task_progress               # [0, +1.5]

    # 2. Completion bonus (SPARSE — kept but reduced, since progress already signals journey)
    r_completion    = uav_state.completed_this_step * COMPLETION_BONUS  # 0 or +2.0

    # 3. Latency penalty (DENSE — was absent, now added)
    current_delay   = uav_state.current_service_delay             # time units
    r_latency       = -LATENCY_WEIGHT * min(current_delay / LATENCY_NORM, 1.0)  # [-0.5, 0]

    # 4. Energy penalty (DENSE — replaces battery bonus, correct direction)
    energy_step     = uav_state.energy_consumed_this_step         # Joules
    r_energy        = -ENERGY_WEIGHT * min(energy_step / ENERGY_NORM, 1.0)      # [-0.5, 0]

    # 5. Failure penalty (SPARSE)
    r_failure       = uav_state.failed_this_step * FAILURE_PENALTY   # 0 or -2.0

    # 6. Collision penalty (SPARSE — scaled proportionally)
    r_collision     = uav_state.has_collision * COLLISION_PENALTY     # 0 or -5.0

    # 7. Dead UAV penalty (TERMINAL)
    r_dead          = uav_state.is_dead * DEAD_PENALTY                # 0 or -20.0

    # NOTE: Fairness is computed and logged for evaluation but NOT added to reward
    # fairness_jfi = compute_jain_index(all_uav_workloads)  # log this
    # r_fairness = 0  (removed from training signal)

    total = r_progress + r_completion + r_latency + r_energy + r_failure + r_collision + r_dead

    return total

# Diagnostic: check component magnitudes after 100 episodes
def diagnose_reward_components(episode_logs):
    """
    After 100 episodes, verify no single component dominates.
    Rule of thumb: no component should contribute more than 50% of |total|.
    """
    components = ["progress", "completion", "latency", "energy", "failure",
                  "collision", "dead"]
    for comp in components:
        avg = abs(episode_logs[f"r_{comp}"].mean())
        total_avg = abs(episode_logs["r_total"].mean())
        pct = 100 * avg / (total_avg + 1e-8)
        print(f"  {comp:12s}: {avg:.3f} avg ({pct:.1f}% of total)")
        if pct > 60:
            print(f"  WARNING: {comp} dominates reward — consider re-scaling")
```

**Option D Scale Analysis:**

| Component | Was | Now | Change |
|-----------|-----|-----|--------|
| `r_success/r_progress` | 0 or +10 (sparse) | [0, +1.5] (dense) + 0/+2.0 (sparse) | Dense signal added |
| `r_battery` | [0, +0.1] (wrong direction) | [-0.5, 0] (energy cost) | Direction flipped |
| `r_latency` | Absent | [-0.5, 0] per step | New term |
| `r_fairness` | [-0.5, 0] (global noise) | Removed | Moved to eval metric |
| `r_failure` | -5.0 | -2.0 | Scaled down |
| `r_collision` | -50 | -5.0 | 10× reduction |
| `r_dead` | -100 | -20.0 | 5× reduction |

---

## 7. Weights and Scale Analysis

### 7.1 Weight Ranges Used in Literature

| Component | Paper | Weight Value | Notes |
|-----------|-------|-------------|-------|
| Latency weight w0 | P1 | 0.3–0.7 | w0 + w1 = 1 |
| Energy weight w1 | P1 | 0.3–0.7 | Complement of w0 |
| Energy normalizer ζ | P1 | 0.001–0.1 | Tune to equalize scale |
| AoI weight ω1 | P3 | 0.4 (typical) | ω1 + ω2 + ω3 = 1 |
| Delay weight ω2 | P3 | 0.3 (typical) | — |
| Energy weight ω3 | P3 | 0.3 (typical) | — |
| Throughput weight γ^h | P4 | 0.5–0.8 | Per-cluster-head |
| Isolation discount ζ_1 | P4 | 0.1–0.5 | Hyperparameter |
| Delay discount ζ_2 | P4 | 0.1–0.5 | Hyperparameter |
| Fairness weight β | P14 | 0.1–0.3 | JFI bonus multiplier |
| Power score weight | P2 | Equal (Avg) | Power and 1/T weighted equally |

### 7.2 Normalization Strategies

The papers use four distinct normalization approaches to keep reward components at the same scale:

**Strategy 1 — Explicit balancing factor (P1):**
```
r = -(w0 · t_u + w1 · e_u · ζ)
```
ζ is tuned empirically so that `w1 · e_u · ζ ≈ w0 · t_u` in typical operation.

**Strategy 2 — Reciprocal normalization (P2):**
```
R = Avg(power_score, 1/T)
```
Both terms are dimensionless ratios, guaranteed to be comparable scale.

**Strategy 3 — Unit-weight constraint (P3):**
```
ω1 + ω2 + ω3 = 1
```
The weighted sum is bounded by the worst-case value of the largest single component.

**Strategy 4 — Convex combination (P4):**
```
R^h = γ · r_global + (1 - γ) · r_local
```
Since γ ∈ [0,1], the combined reward is bounded by max(r_global, r_local).

### 7.3 Recommended Normalization for Current Implementation

Based on literature patterns, the following normalization targets are recommended for Option D:

```python
# Target: all components in range [-1.0, +1.0] before summing
# Then weight by importance

NORMALIZATION_TARGETS = {
    # Component     : (expected_raw_max, target_scale_after_norm)
    "task_progress" : (1.0,  1.5),   # already in [0,1], scale up slightly for importance
    "completion"    : (1.0,  2.0),   # binary, give slight bonus over progress
    "latency"       : (30.0, -0.5),  # normalize by 30-step max delay
    "energy"        : (10.0, -0.5),  # normalize by 10-unit max energy/step
    "failure"       : (1.0,  -2.0),  # binary, explicit penalty
    "collision"     : (1.0,  -5.0),  # binary, should be < 3x max success
    "dead"          : (1.0,  -20.0), # terminal, episode-level (appears once)
}

# Verify balance: for a typical episode (1000 steps):
# - Expected progress events: 100-200 steps (10-20% of steps)
# - Expected collision events: 0-5 (rare)
# - Cumulative collision contribution: 0 to -25
# - Cumulative progress contribution: +150 to +300
# => Collision can exceed progress only in very bad episodes — correct behavior
```

### 7.4 Scale Comparison: Current vs. Recommended vs. Literature

| Component | Current Scale | Recommended (Option D) | Typical Literature Scale |
|-----------|--------------|----------------------|--------------------------|
| Task progress (dense) | N/A (absent) | [0, +1.5]/step | [0, +1.0]/step (P1-style) |
| Task completion (sparse) | +10 (binary) | +2.0 (binary) | [0, +3.0] (P9) |
| Latency penalty (dense) | Absent | [-0.5, 0]/step | [-1.0, 0]/step (P1) |
| Energy penalty (dense) | +[0, 0.1]/step wrong direction | [-0.5, 0]/step | [-1.0, 0]/step (P1, P3) |
| Fairness penalty (dense) | [-0.5, 0]/step | Removed | Absent (majority) |
| Collision (sparse) | -50 (binary) | -5.0 (binary) | N/A (hard constraint, P1) |
| Dead penalty (terminal) | -100 | -20.0 | N/A |
| **Ratio: max_success / max_collision** | 10/50 = 0.2 | 3.5/5.0 = 0.7 | ~0.5–2.0 (literature range) |

---

## 8. Summary Recommendations

### Immediate Actions (Resolve Critical Problems)

1. **Add a per-step task progress signal.** Replace or supplement `r_success = +10` (sparse) with a dense per-step signal proportional to task completion progress. This is the single highest-impact change.

2. **Replace battery bonus with energy cost.** Change `r_battery = +(battery/capacity) * 0.1` to `r_energy = -(energy_consumed / energy_budget)`. This corrects the direction of the incentive and aligns with 100% of literature that models energy.

3. **Re-scale the collision penalty.** Reduce `r_collision = -50` to approximately `-5.0` (within 2–3× of the maximum per-step positive signal). Consider moving to a hard constraint with early episode termination.

### High Priority Actions (Resolve Significant Problems)

4. **Add a per-step latency penalty.** Penalize the elapsed delay per active task at every step. This provides the continuous cost signal that all task-offloading literature uses as its primary learning signal.

5. **Remove fairness from per-agent reward.** Move the CV-based fairness penalty to an evaluation metric only. Consider structural fairness approaches (e.g., min-operator for worst-case agent) if fairness must be trained.

### Medium Priority Actions (Alignment with Best Practices)

6. **Consider potential field navigation bonus.** Add a small attractive potential toward unserved devices and repulsive potential from obstacles. This dramatically accelerates early exploration (P3 approach).

7. **Implement reward diagnostics.** After every 100 training episodes, log the mean absolute contribution of each reward component. Flag any component that exceeds 60% of total signal magnitude.

8. **Consider role-differentiated rewards.** If the system includes both task-executing UAVs and coordinator agents, give them different reward functions aligned to their roles (P4 pattern).

### Recommended Implementation Path

```
Phase 1 (Week 1): Implement Option D (Hybrid)
  - Fix energy direction, add latency, re-scale collision, remove fairness
  - Run ablation: compare each component's contribution
  - Expected improvement: faster convergence, higher task completion rate

Phase 2 (Week 2-3): Add dense progress signal
  - Instrument environment to track per-step task fraction completed
  - Verify convergence speed improvement vs. Phase 1

Phase 3 (Month 2): Evaluate Option C (AoI + PF)
  - Implement AoI tracking for all devices
  - Implement potential field navigation reward
  - Compare against Phase 2 baseline
  - Best option for publications comparing against P3

Phase 4 (Month 2-3): Hyperparameter study
  - Grid search over LATENCY_WEIGHT, ENERGY_WEIGHT, COMPLETION_BONUS
  - Compare JFI as evaluation metric across reward designs
  - Document final weights for reproducibility
```

### Quick Decision Guide

| If your priority is... | Use option... | Key change from current |
|-----------------------|---------------|------------------------|
| Fastest implementation | **D (Hybrid)** | Fix scales, add latency, flip energy |
| Best theoretical grounding | **A (MATD3)** | Pure cost minimization |
| Task + QoS tracking | **B (MAPPO-H)** | Dense progress + QoS class |
| Trajectory + freshness | **C (Fed-MARL-PF)** | AoI + potential fields |

---

**Document Version:** 1.0
**Date:** 2026-02-19
**Coverage:** 16 papers (4 with exact equations, 9 with documented patterns, 3 non-RL/FL-only)
**Primary Sources:** MATD3-TORA (P1), Rating-FL (P2), Fed-MARL-PF (P3), HMDRL-UC (P4) — PDF-analyzed
**Secondary Sources:** Papers Background.pdf, 7_individual_backgrounds.md, papers_graded_list.md, COMPREHENSIVE_FORMULA_ANALYSIS.md
