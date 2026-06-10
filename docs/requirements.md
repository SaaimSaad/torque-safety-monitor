# Requirements — Torque Safety Monitor (ISO 26262 · E-Gas 3-Level)

Requirements are written to be testable and individually traceable to design
elements and test cases. IDs follow `REQ-SAF-###`. Each maps to a verification method
(analysis / MIL test / SIL test) in [test-plan.md](test-plan.md).

> **Safety goal (illustrative):** *No unintended torque / acceleration.* Per ISO 26262
> this is typically **ASIL D** for the torque path; the E-Gas three-level architecture
> with an independent monitor (L2) and an independent computer check (L3) supports ASIL
> decomposition of that goal.

## Interface

The monitor is a discrete component running at `Ts = 0.01 s` (100 Hz).

| Direction | Signal | Type / units | Range | Notes |
|---|---|---|---|---|
| In | `pedal` | single, % | 0 … 100 | Accelerator pedal (L2 independent path) |
| In | `torque_actual` | single, Nm | 0 … 400 | Torque the L1 function path is producing |
| In | `wd_ok` | boolean | 0 / 1 | Controller answered the L3 watchdog correctly this sample |
| Out | `state` | uint8 (enum) | 0 … 2 | `OK, L2_LIMIT, L3_SAFE` |
| Out | `tq_cmd` | single, Nm | 0 … 400 | Allowed torque command (≤ permissible when limiting) |
| Out | `safe_state` | boolean | 0 / 1 | A safety reaction is active (L2 or L3) |

## Level 2 — function monitoring

| ID | Type | Requirement | Verification |
|---|---|---|---|
| **REQ-SAF-001** | Functional | L2 shall compute a **permissible torque** independently from the pedal (`permissible = requested(pedal) + t_margin`); it shall be monotonic in pedal and bounded by `t_max + t_margin`. | Analysis · `verify_safety_monitor.js` |
| **REQ-SAF-002** | Safety | L2 shall detect a torque fault when `torque_actual` exceeds the permissible torque. | MIL: `tc_l2` |
| **REQ-SAF-003** | Safety | On a **confirmed** L2 fault the monitor shall command torque limitation (≤ `t_limp`) and reach that reaction within the **FTTI** (`ftti`). | MIL: `tc_l2` |
| **REQ-SAF-004** | Robustness | A torque excess shorter than the L2 debounce (`l2_cnt` samples) shall **not** trigger a reaction (no false trip on a transient). | MIL: `tc_l2_glitch` |

## Level 3 — controller / computer monitoring

| ID | Type | Requirement | Verification |
|---|---|---|---|
| **REQ-SAF-005** | Safety | L3 shall maintain a question/answer **watchdog**; a correct answer (`wd_ok`) each sample resets the watchdog fail counter. | Analysis |
| **REQ-SAF-006** | Safety | On watchdog timeout (`wd_cnt` consecutive missing/incorrect answers) the monitor shall force the **safe state** (`tq_cmd = 0`) within the **FTTI**, independent of L1/L2. | MIL: `tc_l3` |

## Supervisor requirements

| ID | Type | Requirement | Verification |
|---|---|---|---|
| **REQ-SAF-007** | Safety | Safety reactions shall be **latched**: once entered, the monitor holds the reaction even after the fault clears; recovery requires a reset / key cycle. | MIL: `tc_latch` |
| **REQ-SAF-008** | Safety | The L3 safe state shall have **priority** over the L2 torque limit; an L3 trip while in `L2_LIMIT` shall escalate to `L3_SAFE`. | MIL: `tc_priority` |
| **REQ-SAF-009** | Robustness | Nominal torque ripple within the L2 margin (`< t_margin`) shall **not** trigger any reaction. | MIL: `tc_nominal` |
| **REQ-SAF-010** | Interface | The component shall conform to the interface above; outputs shall stay within declared ranges (`state ∈ [0,2]`, `0 ≤ tq_cmd ≤ t_max`), and `tq_cmd` shall never exceed the permissible torque while a reaction is active. | Analysis + range checks in all MIL tests |

## Derived design constraints

- The L2 permissible-torque path uses only the pedal (and would use independently
  acquired engine speed in production) — **separate** from the L1 functional path, so a
  fault in L1 cannot corrupt the monitor.
- Reaction times are bounded by the FTTI; the L2 debounce (`l2_cnt`) and the watchdog
  timeout (`wd_cnt`) are both chosen so reaction completes within `ftti`.
- Both reactions latch; `tq_cmd` is monotonically non-increasing in severity
  (`OK ≥ L2_LIMIT ≥ L3_SAFE = 0`).

> Numeric values for `Ts`, `t_max`, `t_margin`, `t_limp`, `l2_cnt`, `wd_cnt`, and `ftti`
> live in `matlab/safety_params.m` so requirements and implementation share one source
> of truth.
