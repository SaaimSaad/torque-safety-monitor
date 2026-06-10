# Test Plan — Torque Safety Monitor (ISO 26262 · E-Gas 3-Level)

Verification is performed at **MIL** (model-in-the-loop) and **SIL**
(software-in-the-loop), with **structural coverage** collected during MIL. Every test
case traces back to one or more requirements in [requirements.md](requirements.md). The
monitor logic is additionally re-proven headlessly by `verify_safety_monitor.js`,
independent of Simulink.

## Coverage targets

| Metric | Target |
|---|---|
| Decision | 100 % |
| Condition | 100 % |
| MCDC (modified condition / decision) | 100 % |

Coverage is collected by `matlab/collect_coverage.m` and exported as an HTML report.
Any gaps are closed by adding test cases (not by disabling checks).

## Test cases

| ID | Scenario | Stimulus | Pass criteria | Traces to |
|---|---|---|---|---|
| **tc_l2** | Unintended torque | Hold `torque_actual` above permissible (excess > margin) | Detect; enter `L2_LIMIT` within FTTI; `tq_cmd ≤ t_limp` | REQ-SAF-002/003 |
| **tc_l2_glitch** | Transient excess | One-sample excess, then nominal | No reaction; stays `OK` | REQ-SAF-004 |
| **tc_l3** | Watchdog timeout | `wd_ok = 0` for ≥ `wd_cnt` samples | Enter `L3_SAFE` within FTTI; `tq_cmd = 0` | REQ-SAF-005/006 |
| **tc_latch** | Latching | Confirm a fault, then remove it | Reaction held; no self-clear | REQ-SAF-007 |
| **tc_priority** | L3 over L2 | Enter `L2_LIMIT`, then fail the watchdog | Escalate to `L3_SAFE`; `tq_cmd = 0` | REQ-SAF-008 |
| **tc_nominal** | Nominal ripple | `torque_actual = requested ± (< t_margin)` | No reaction throughout; stays `OK` | REQ-SAF-009 |
| **tc_range** | Output range | All of the above | `state ∈ [0,2]`, `0 ≤ tq_cmd ≤ t_max`, `tq_cmd ≤ permissible` when limiting | REQ-SAF-010 |

## MIL procedure (`run_mil.m`)

1. `safety_params` then `build_model` to ensure `torque_safety.slx` exists.
2. For each test case, set inputs, run `sim`, and evaluate pass criteria with assertions.
3. Plot `torque_actual`, `permissible`, `tq_cmd`, and `state` for visual inspection.
4. Print a pass/fail summary table mapping test cases to requirements, including the
   measured reaction time against the FTTI.

## SIL procedure (`run_sil.m`)

1. `gen_code` to generate C for the monitor subsystem.
2. Run the `tc_l2` and `tc_l3` stimuli through the SIL block.
3. Assert MIL vs SIL output equivalence (`state`, `tq_cmd` within tolerance).

## Coverage procedure (`collect_coverage.m`)

1. Enable decision / condition / MCDC recording.
2. Run the full test suite as a single coverage session.
3. Export the HTML report to `../assets/coverage/`.
4. Fail the run if any metric is below target.
