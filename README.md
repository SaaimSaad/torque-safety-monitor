# Torque Safety Monitor (ISO 26262 · E-Gas 3-Level)

[![Verification](https://img.shields.io/badge/safety--logic%20checks-passing-brightgreen)](verify_safety_monitor.js)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2022a%2B-orange)](matlab/)
[![Simulink](https://img.shields.io/badge/Simulink%20%7C%20Stateflow%20%7C%20Embedded%20Coder-required-blue)](matlab/build_model.m)
[![Safety](https://img.shields.io/badge/ISO%2026262-ASIL--D%20%7C%20E--Gas%20L1%2FL2%2FL3-555)](docs/design.md)
[![Standards](https://img.shields.io/badge/MISRA--C%20%7C%20MIL%2FSIL%20%7C%20MCDC-MBD%20workflow-555)](docs/test-plan.md)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

A model-based design (MBD) of an **ISO 26262 functional-safety torque monitor** for an
electronic throttle / torque path, built the way production safety software is built:
requirements → Simulink/Stateflow model → generated C → MIL/SIL verification with
structural coverage.

The monitor implements the well-known **E-Gas / EGAS three-level monitoring concept** —
the standard architecture for the **ASIL-D unintended-acceleration** safety goal:

| Level | Name | Role |
|---|---|---|
| **Level 1** | Function | The driver-demand torque path (computes the requested torque). |
| **Level 2** | Function monitoring | An **independent** permissible-torque model; limits torque if the actual torque exceeds what the pedal permits. |
| **Level 3** | Controller monitoring | A **question/answer watchdog** that forces a hardware safe state if the computer fails to respond correctly — independent of L1/L2. |

The monitor is defined **as code** — `matlab/build_model.m` constructs the Simulink
model and the Stateflow safety supervisor programmatically through the Simulink API, so
the design is reviewable, diffable, and version-controlled rather than living in an
opaque binary `.slx`.

> **This is an original, generic design** derived from public functional-safety
> first principles (the E-Gas monitoring concept). It contains no employer/client
> signals, calibrations, requirements, or proprietary structure.

---

## What it does

- **Level 2** computes a **permissible torque** from the accelerator pedal (an
  independent path from L1) and compares it to the actual torque. If actual exceeds
  permissible beyond a margin for a debounced interval, it commands **torque
  limitation** (a limp value) — reaching the safe reaction within the **fault-tolerant
  time interval (FTTI)**.
- **Level 3** services a **question/answer watchdog**. If the controller fails to
  answer correctly for `wd_cnt` samples, the monitor forces the **safe state** (fuel
  cut, zero torque) — a path independent of the functional software.
- The safety supervisor (`OK → L2_LIMIT → L3_SAFE`) **latches** reactions and gives
  L3 **priority** over L2; recovery requires a key cycle / reset.

See [docs/design.md](docs/design.md) for the level model and rationale, and
[docs/requirements.md](docs/requirements.md) for the traceable requirements.

---

## Repository layout

```
torque-safety-monitor/
├── README.md
├── docs/
│   ├── requirements.md      # REQ-SAF-### requirements + traceability
│   ├── design.md            # 3-level model, FTTI, watchdog, safe state
│   └── test-plan.md         # MIL/SIL scenarios + coverage targets
├── matlab/
│   ├── safety_params.m      # single source of monitor parameters (base workspace)
│   ├── safety_const.m       # code-gen-friendly constant companion
│   ├── build_model.m        # builds the Simulink + Stateflow model via API
│   ├── run_mil.m            # MIL simulation, requirement checks, plots
│   ├── run_sil.m            # SIL build + MIL/SIL equivalence
│   ├── gen_code.m           # Embedded Coder configuration + code generation
│   ├── collect_coverage.m   # decision / condition / MCDC coverage + report
│   └── test/
│       └── test_safety_monitor.m   # requirement-linked reference tests
├── autosar/
│   └── SafetyMonitor.arxml  # hand-authored AUTOSAR Classic SWC description
├── generated/               # generated C output (created by gen_code.m)
├── assets/                  # exported model + coverage screenshots
└── verify_safety_monitor.js # headless requirement check (no MATLAB needed)
```

---

## How to verify it today (no MATLAB required)

The monitor logic is re-proven headlessly, independent of Simulink:

```bash
node verify_safety_monitor.js
```

```
=== Torque safety-monitor logic verification ===
[PASS]  REQ-SAF-001 permissible monotonic + bounded
[PASS]  REQ-SAF-002 L2 detects excess torque
[PASS]  REQ-SAF-003 L2 reaction within FTTI              reaction=30 ms ≤ FTTI 50 ms
[PASS]  REQ-SAF-004 no trip on 1-sample excess
[PASS]  REQ-SAF-006 L3 safe-state within FTTI            reaction=50 ms ≤ FTTI 50 ms
[PASS]  REQ-SAF-007 reaction latched
[PASS]  REQ-SAF-008 L3 overrides L2
[PASS]  REQ-SAF-009 no spurious trip on ripple
[PASS]  REQ-SAF-010 outputs in range & bounded
ALL CHECKS PASSED
```

The same logic runs live in the project showcase page (`torque-safety.html`).

---

## How to run the MATLAB toolchain

> **Requires a personal MATLAB** — MATLAB Home, a 30-day trial, a student licence, or
> MATLAB Online — with **Simulink, Stateflow, and Embedded Coder** (SIL/coverage also
> use Simulink Coverage). **Do not build this on an employer/client MATLAB install.**

Tested target: MATLAB R2022a or later.

```matlab
cd matlab
safety_params          % load parameters into the base workspace
build_model            % create torque_safety.slx (Simulink + Stateflow)
run_mil                % simulate the safety scenarios, check requirements
gen_code               % generate MISRA-style C into ../generated/
collect_coverage       % run coverage and export an HTML report
```

After a run, export the model diagram and coverage report into `assets/`.

---

## MBD workflow this demonstrates

| Stage | Artifact |
|---|---|
| Requirements | `docs/requirements.md` (REQ-SAF-001 … 010) |
| Design (model-as-code) | `matlab/build_model.m` → `torque_safety.slx` |
| Model-in-the-loop (MIL) | `matlab/run_mil.m` + requirement assertions |
| Headless logic check | `verify_safety_monitor.js` (no MATLAB) |
| Code generation | `matlab/gen_code.m` → `generated/*.c/.h` (MISRA-C style) |
| Software-in-the-loop (SIL) | `matlab/run_sil.m` (MIL/SIL equivalence) |
| Structural coverage | `matlab/collect_coverage.m` (decision / condition / MCDC) |
| Architecture | `autosar/SafetyMonitor.arxml` (AUTOSAR Classic SWC) |

---

## Status

- [x] Requirements, design, and test plan authored
- [x] Monitor defined as code (`build_model.m`) + parameters + test/codegen/coverage scripts
- [x] Logic re-proven headlessly (`verify_safety_monitor.js`) and live in the showcase page
- [x] AUTOSAR Classic SWC description (`autosar/SafetyMonitor.arxml`)
- [ ] `.slx`, generated C, and coverage report produced on a clean personal MATLAB
- [ ] Model + coverage screenshots exported to `assets/`

## Licence

MIT — see header in source files. Original work; no third-party or proprietary content.
