function tests = test_safety_monitor()
% test_safety_monitor.m  —  Requirement-linked tests (MATLAB unit test).
% Original work, MIT licence.
%
% Pure-MATLAB reference checks of the safety-monitor logic, independent of
% Simulink, so the three-level logic can be verified quickly and traced to
% requirements. Mirrors verify_safety_monitor.js.
%
% Usage:  cd matlab; results = runtests('test/test_safety_monitor.m')

    tests = functiontests(localfunctions);
end

function setupOnce(tc)
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, '..'));           % for safety_const
    tc.TestData.S = safety_const();
end

% --- REQ-SAF-001 : permissible torque monotonic + bounded ----------------
function test_permissible_REQ001(tc)
    S = tc.TestData.S;
    verifyLessThan(tc, permissible(S,20), permissible(S,80), 'REQ-SAF-001 monotonic');
    verifyLessThanOrEqual(tc, permissible(S,100), S.t_max + S.t_margin + 1e-9, 'bounded');
end

% --- REQ-SAF-003 : L2 reaction within FTTI -------------------------------
function test_l2_reaction_REQ003(tc)
    S = tc.TestData.S;
    [st, rt] = run_seq(S, 30, @(k) requested(S,30)+80, @(k) true);
    verifyEqual(tc, double(st.state), 1, 'REQ-SAF-003 entered L2_LIMIT');
    verifyLessThanOrEqual(tc, rt.l2, S.ftti + 1e-9, 'L2 reaction within FTTI');
    verifyLessThanOrEqual(tc, st.tq_cmd, S.t_limp + 1e-6, 'torque limited');
end

% --- REQ-SAF-004 : single-sample excess must not trip --------------------
function test_l2_glitch_REQ004(tc)
    S = tc.TestData.S;
    st = run_seq(S, 30, @(k) requested(S,30) + (k==1)*80, @(k) true, 30);
    verifyEqual(tc, double(st.state), 0, 'REQ-SAF-004 no trip on glitch');
end

% --- REQ-SAF-006 : L3 watchdog timeout forces safe state within FTTI -----
function test_l3_reaction_REQ006(tc)
    S = tc.TestData.S;
    [st, rt] = run_seq(S, 30, @(k) requested(S,30), @(k) false);
    verifyEqual(tc, double(st.state), 2, 'REQ-SAF-006 entered L3_SAFE');
    verifyLessThanOrEqual(tc, rt.l3, S.ftti + 1e-9, 'L3 reaction within FTTI');
    verifyEqual(tc, st.tq_cmd, 0, 'safe state: zero torque');
end

% --- REQ-SAF-008 : L3 priority over L2 -----------------------------------
function test_priority_REQ008(tc)
    S = tc.TestData.S;
    st = run_seq(S, 30, @(k) requested(S,30)+80, @(k) k < 8, 20);  % L2 first, then wd fails
    verifyEqual(tc, double(st.state), 2, 'REQ-SAF-008 escalated to L3_SAFE');
    verifyEqual(tc, st.tq_cmd, 0, 'safe state torque = 0');
end

% --- REQ-SAF-007 : reactions latch after the fault clears ----------------
function test_latch_REQ007(tc)
    S = tc.TestData.S;
    st = run_seq(S, 30, @(k) requested(S,30) + (k<=6)*80, @(k) true, 40); % excess then cleared
    verifyEqual(tc, double(st.state), 1, 'REQ-SAF-007 L2 reaction latched after fault cleared');
end

% --- REQ-SAF-009 : nominal ripple within margin must not trip ------------
function test_nominal_ripple_REQ009(tc)
    S = tc.TestData.S;
    st = run_seq(S, 40, @(k) requested(S,40) + 20*sin(0.4*k), @(k) true, 200); % +/-20 < margin 30
    verifyEqual(tc, double(st.state), 0, 'REQ-SAF-009 ripple within margin: no reaction');
end

% =======================================================================
% Reference implementation (mirrors the Monitor block + supervisor).
function [st, rt] = run_seq(S, pedal, tqFcn, wdFcn, n)
    if nargin < 5; n = 20; end
    st.state = 0; st.tq_cmd = 0; l2c = 0; wdf = 0;
    rt.l2 = NaN; rt.l3 = NaN;
    for k = 1:n
        tq   = tqFcn(k);
        wdok = wdFcn(k);
        perm = permissible(S, pedal);
        if tq > perm; l2c = min(l2c + 1, S.l2_cnt); else; l2c = 0; end
        l2_confirmed = l2c >= S.l2_cnt;
        if ~wdok; wdf = min(wdf + 1, S.wd_cnt); else; wdf = 0; end
        l3_trip = wdf >= S.wd_cnt;

        if l3_trip; st.state = 2;
        elseif st.state == 0 && l2_confirmed; st.state = 1;
        elseif st.state == 1 && l3_trip; st.state = 2; end

        if st.state == 0; st.tq_cmd = min(tq, perm);
        elseif st.state == 1; st.tq_cmd = min(tq, S.t_limp);
        else; st.tq_cmd = 0; end

        if st.state == 1 && isnan(rt.l2); rt.l2 = k * S.Ts; end
        if st.state == 2 && isnan(rt.l3); rt.l3 = k * S.Ts; end
    end
end

function p = permissible(S, pedal); p = requested(S, pedal) + S.t_margin; end
function r = requested(S, pedal);   r = pedal/100 * S.t_max; end
