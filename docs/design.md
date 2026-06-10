# Design — Torque Safety Monitor (ISO 26262 · E-Gas 3-Level)

This document describes the three monitoring levels, the permissible-torque model, the
watchdog, the safe-state supervisor, and the timing budget. All numeric parameters live
in [`../matlab/safety_params.m`](../matlab/safety_params.m); the same values are mirrored
in the browser simulation so both tell the same story.

## 1. The E-Gas three-level concept

The E-Gas (EGAS) monitoring concept is the de-facto architecture for electronic
throttle / torque safety. Three independent layers guard the **no unintended torque**
goal:

```
 ┌──────────────────────────── Level 1 — FUNCTION ────────────────────────────┐
 │  driver pedal → torque request → air/fuel actuation  (the normal software)  │
 └─────────────────────────────────────────────────────────────────────────────┘
 ┌──────────────────────── Level 2 — FUNCTION MONITORING ──────────────────────┐
 │  independent permissible-torque model; if actual > permissible → LIMIT      │
 └─────────────────────────────────────────────────────────────────────────────┘
 ┌──────────────────── Level 3 — CONTROLLER / COMPUTER CHECK ──────────────────┐
 │  question/answer watchdog, program-flow & instruction checks → SAFE STATE   │
 └─────────────────────────────────────────────────────────────────────────────┘
```

This model implements L2 and L3 as the deployed monitor; L1 is represented by its
`torque_actual` output (the value the monitor polices).

## 2. Level 2 — permissible torque (REQ-SAF-001/002/003)

L2 recomputes, on an **independent** path, the most torque the driver could be asking
for, and compares it to what L1 is actually producing:

```
requested   = pedal/100 · t_max                 // independent driver-demand estimate
permissible = requested + t_margin              // allowance for normal transients
l2_bad      = torque_actual > permissible        // unintended-torque test
```

| Symbol | Meaning | Nominal |
|---|---|---|
| `t_max` | maximum engine torque | 300 Nm |
| `t_margin` | allowance above requested | 30 Nm |
| `t_limp` | limited (limp) torque in L2 reaction | 60 Nm |

A confirmed L2 fault commands **torque limitation**: `tq_cmd = min(torque_actual, t_limp)`.
In production this is a stepwise torque reduction or fuel cut; the model uses a single
limp ceiling.

## 3. Level 3 — question/answer watchdog (REQ-SAF-005/006)

An external/independent watchdog poses a question each cycle; the controller must return
the matching answer. The monitor counts missed/incorrect answers:

```
if !wd_ok : wdFail = min(wdFail + 1, wd_cnt)
else      : wdFail = 0
l3_trip   = wdFail >= wd_cnt                      // computer not healthy
```

A timeout forces the **safe state** (`tq_cmd = 0`, fuel cut) — independent of whether
L1/L2 are still computing. This catches stuck code, lost program flow, and clock/ALU
faults that L2 alone cannot.

| Symbol | Meaning | Nominal |
|---|---|---|
| `wd_cnt` | missed answers to trip L3 | 5 ( = 50 ms ) |

## 4. Safe-state supervisor (Stateflow)

```
        ┌────┐  l2_confirmed   ┌──────────┐
  ─────▶│ OK │ ──────────────▶ │ L2_LIMIT │
        └────┘                 └──────────┘
           │ l3_trip                │ l3_trip
           ▼                        ▼
        ┌─────────────────────────────┐
        │           L3_SAFE           │   (latched, highest priority)
        └─────────────────────────────┘
```

- **OK** — `tq_cmd = min(torque_actual, permissible)` (pass-through with a ceiling).
- **L2_LIMIT** — torque limited to `t_limp`; latched (REQ-SAF-007).
- **L3_SAFE** — `tq_cmd = 0`; latched; has **priority** over L2 and can be entered from
  any state (REQ-SAF-008).

`tq_cmd` is non-increasing in severity: `OK ≥ L2_LIMIT ≥ L3_SAFE = 0`.

## 5. Timing budget — FTTI (REQ-SAF-003/006)

The **fault-tolerant time interval** is the maximum time from fault occurrence to safe
state. Both reaction paths fit inside it:

| Reaction | Detection | Reaction time | Budget |
|---|---|---|---|
| L2 torque limit | `l2_cnt = 3` samples | **30 ms** | ≤ `ftti` = 50 ms |
| L3 safe state | `wd_cnt = 5` samples | **50 ms** | ≤ `ftti` = 50 ms |

The debounce counts are deliberately short enough that confirmation + reaction complete
within the FTTI, yet long enough that a single-sample transient (REQ-SAF-004) or nominal
ripple (REQ-SAF-009) never trips a reaction.

## 6. Why these choices (interview notes)

- **Independent L2 path** — the monitor must not share the failure modes of the function
  it checks; recomputing permissible torque from the pedal (and, in production,
  independently acquired engine speed) is the essence of E-Gas Level 2.
- **Question/answer watchdog at L3** — a simple "alive" watchdog can be fooled by a
  looping task; a challenge/response proves the program flow and ALU are actually
  executing, which is why ISO 26262 favours it for high-ASIL computer monitoring.
- **Latched reactions with key-cycle recovery** — safety reactions must not chatter or
  silently self-clear; latching until a controlled reset is the standard pattern.
- **L3 priority over L2** — a computer-integrity fault is more severe than a torque
  overshoot, so the safe state always wins.
