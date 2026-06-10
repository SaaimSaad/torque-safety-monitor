// verify_safety_monitor.js — headless check that the torque safety monitor meets
// REQ-SAF-001…010. Replicates the logic in torque-safety.html and
// matlab/SafetyMonitor (Stateflow). Run:  node verify_safety_monitor.js
//
// Pattern: the E-Gas / EGAS three-level monitoring concept for an electronic
// throttle / torque path (the classic ASIL-D unintended-acceleration safety case).
//   Level 1 — Function        : driver-demand torque path (computes requested torque)
//   Level 2 — Function monitor : independent permissible-torque model; limits torque
//                                if actual exceeds permissible beyond a margin
//   Level 3 — Controller check : question/answer watchdog; forces safe state on timeout
const P = {
  Ts:0.01,
  t_max:300.0,       // max engine torque [Nm]
  t_margin:30.0,     // L2 allowance above requested torque [Nm]
  t_limp:60.0,       // limited (limp) torque in L2 reaction [Nm]
  l2_cnt:3,          // sustained samples to confirm an L2 fault (= 30 ms)
  wd_cnt:5,          // missed watchdog answers to trip L3   (= 50 ms)
  ftti:0.05          // fault-tolerant time interval [s] (reaction deadline)
};
const S = { OK:0, L2_LIMIT:1, L3_SAFE:2 };
const NAME = ['OK','L2_LIMIT','L3_SAFE'];

const requested   = pedal => pedal/100 * P.t_max;                 // L1 driver demand
const permissible = pedal => requested(pedal) + P.t_margin;       // L2 independent ceiling (REQ-SAF-001)

function makeState(){ return { state:S.OK, l2Cnt:0, wdFail:0, t:0, tq_cmd:0 }; }

// One discrete monitor step at Ts. Mirrors the L2 monitor + L3 watchdog + supervisor.
function step(st, inp){
  const perm   = permissible(inp.pedal);
  const l2_bad = inp.torque_actual > perm;                        // REQ-SAF-002
  if(l2_bad){ st.l2Cnt = Math.min(st.l2Cnt + 1, P.l2_cnt); } else { st.l2Cnt = 0; }
  const l2_confirmed = st.l2Cnt >= P.l2_cnt;                      // REQ-SAF-004 (debounced)

  if(!inp.wd_ok){ st.wdFail = Math.min(st.wdFail + 1, P.wd_cnt); } else { st.wdFail = 0; }
  const l3_trip = st.wdFail >= P.wd_cnt;                          // REQ-SAF-005/006

  // supervisor — L3 (most severe) has priority; both reactions latch (REQ-SAF-007/008)
  if(l3_trip)               st.state = S.L3_SAFE;
  else if(st.state===S.OK && l2_confirmed) st.state = S.L2_LIMIT;
  else if(st.state===S.L2_LIMIT && l3_trip) st.state = S.L3_SAFE;

  // torque command (REQ-SAF-010): pass-through when OK, limited in L2, zero in L3
  if(st.state===S.OK)            st.tq_cmd = Math.min(inp.torque_actual, perm);
  else if(st.state===S.L2_LIMIT) st.tq_cmd = Math.min(inp.torque_actual, P.t_limp);
  else                           st.tq_cmd = 0.0;                 // safe state: fuel cut

  st.t += P.Ts;
  return { perm, l2_bad, l3_trip, l2_confirmed };
}

function run(st, n, inFactory){
  for(let k=0;k<n;k++) step(st, inFactory(k));
}

let fails = 0;
const check = (name, cond, detail) => {
  console.log(`${cond?'[PASS]':'[FAIL]'}  ${name.padEnd(48)} ${detail}`); if(!cond) fails++;
};

console.log('\n=== Torque safety-monitor logic verification ===');

// ---- REQ-SAF-001 : permissible torque monotonic in pedal and bounded ----
{
  const mono = permissible(20) < permissible(80);
  const bounded = permissible(0) >= P.t_margin - 1e-9 && permissible(100) <= P.t_max + P.t_margin + 1e-9;
  check('REQ-SAF-001 permissible monotonic + bounded', mono && bounded,
        `P(20%)=${permissible(20).toFixed(0)} < P(80%)=${permissible(80).toFixed(0)} Nm`);
}

// ---- REQ-SAF-002 : L2 detects unintended torque above permissible ----
{
  let st = makeState();
  const r = step(st, { pedal:30, torque_actual:requested(30)+80, wd_ok:true }); // 80 Nm excess
  check('REQ-SAF-002 L2 detects excess torque', r.l2_bad,
        `actual=${(requested(30)+80).toFixed(0)} > perm=${r.perm.toFixed(0)} Nm`);
}

// ---- REQ-SAF-003 : L2 reaction reaches torque-limit within FTTI ----
{
  let st = makeState();
  let reactN = -1;
  for(let k=0;k<20;k++){
    step(st, { pedal:30, torque_actual:requested(30)+80, wd_ok:true });
    if(reactN<0 && st.state===S.L2_LIMIT) reactN = k+1;
  }
  const ms = reactN*P.Ts*1000;
  check('REQ-SAF-003 L2 reaction within FTTI', st.state===S.L2_LIMIT && reactN*P.Ts <= P.ftti + 1e-9 && st.tq_cmd<=P.t_limp+1e-9,
        `reaction=${ms.toFixed(0)} ms ≤ FTTI ${(P.ftti*1000).toFixed(0)} ms; tq_cmd=${st.tq_cmd.toFixed(0)} Nm`);
}

// ---- REQ-SAF-004 : a single-sample excess must NOT trip the limiter ----
{
  let st = makeState();
  step(st, { pedal:30, torque_actual:requested(30)+80, wd_ok:true });  // 1 bad sample
  run(st, 30, k => ({ pedal:30, torque_actual:requested(30), wd_ok:true }));
  check('REQ-SAF-004 no trip on 1-sample excess', st.state===S.OK,
        `state=${NAME[st.state]} (l2_cnt needs ${P.l2_cnt})`);
}

// ---- REQ-SAF-005/006 : L3 watchdog timeout forces safe state within FTTI ----
{
  let st = makeState();
  let reactN = -1;
  for(let k=0;k<20;k++){
    step(st, { pedal:30, torque_actual:requested(30), wd_ok:false }); // watchdog not serviced
    if(reactN<0 && st.state===S.L3_SAFE) reactN = k+1;
  }
  const ms = reactN*P.Ts*1000;
  check('REQ-SAF-006 L3 safe-state within FTTI', st.state===S.L3_SAFE && reactN*P.Ts <= P.ftti + 1e-9 && st.tq_cmd===0,
        `reaction=${ms.toFixed(0)} ms ≤ FTTI ${(P.ftti*1000).toFixed(0)} ms; tq_cmd=0 Nm`);
}

// ---- REQ-SAF-007 : reactions latch (fault clears, state holds) ----
{
  let st = makeState();
  run(st, 6, k => ({ pedal:30, torque_actual:requested(30)+80, wd_ok:true }));  // confirm L2
  const after = st.state;
  run(st, 50, k => ({ pedal:30, torque_actual:requested(30), wd_ok:true }));    // fault removed
  check('REQ-SAF-007 reaction latched', after===S.L2_LIMIT && st.state===S.L2_LIMIT,
        `held ${NAME[st.state]} after fault cleared`);
}

// ---- REQ-SAF-008 : L3 overrides L2 (priority + escalation) ----
{
  let st = makeState();
  run(st, 6, k => ({ pedal:30, torque_actual:requested(30)+80, wd_ok:true }));  // in L2_LIMIT
  run(st, P.wd_cnt + 2, k => ({ pedal:30, torque_actual:requested(30), wd_ok:false })); // wd fails
  check('REQ-SAF-008 L3 overrides L2', st.state===S.L3_SAFE && st.tq_cmd===0,
        `escalated to ${NAME[st.state]}; tq_cmd=0 Nm`);
}

// ---- REQ-SAF-009 : nominal torque ripple within margin must not trip ----
{
  let st = makeState(), ok = true;
  for(let k=0;k<500;k++){
    const ripple = 20 * Math.sin(k*0.4);                  // ±20 Nm < margin 30 Nm
    step(st, { pedal:40, torque_actual:requested(40)+ripple, wd_ok:true });
    if(st.state!==S.OK) ok = false;
  }
  check('REQ-SAF-009 no spurious trip on ripple', ok, `±20 Nm ripple < margin ${P.t_margin} Nm`);
}

// ---- REQ-SAF-010 : outputs in range; torque never exceeds permissible ----
{
  let st = makeState(), ok = true;
  const seq = [
    [40, k => ({ pedal:50, torque_actual:requested(50), wd_ok:true })],
    [20, k => ({ pedal:50, torque_actual:requested(50)+90, wd_ok:true })],   // L2
    [20, k => ({ pedal:50, torque_actual:requested(50)+90, wd_ok:false })]   // → L3
  ];
  for(const [n,f] of seq) for(let k=0;k<n;k++){ const r = step(st, f());
    const limitOk = (st.state===S.OK) || (st.tq_cmd <= r.perm + 1e-9);
    if(!(st.state>=0 && st.state<=2 && st.tq_cmd>=0 && st.tq_cmd<=P.t_max+1e-9 && limitOk)) ok=false; }
  check('REQ-SAF-010 outputs in range & bounded', ok, `state∈[0,2] · 0≤tq_cmd≤${P.t_max} · tq_cmd≤permissible when limiting`);
}

console.log(`\n${fails===0?'ALL CHECKS PASSED':fails+' CHECK(S) FAILED'}\n`);
process.exit(fails===0?0:1);
