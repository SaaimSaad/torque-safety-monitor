function results = run_mil()
% run_mil.m  —  Model-in-the-loop verification of torque_safety.
% Original work, MIT licence.
%
% Runs the test cases from docs/test-plan.md, evaluates the requirement
% thresholds (including reaction time vs FTTI), plots the key signals, and
% prints a pass/fail summary that traces each test case to its requirement(s).
%
% Usage:  safety_params; build_model; run_mil

    mdl = 'torque_safety';
    if ~exist([mdl '.slx'], 'file'); build_model(); end
    S = evalin('base', 'S');

    cases = {
        'tc_l2'        , @scn_l2
        'tc_l2_glitch' , @scn_l2_glitch
        'tc_l3'        , @scn_l3
        'tc_latch'     , @scn_latch
        'tc_priority'  , @scn_priority
        'tc_nominal'   , @scn_nominal
        'tc_range'     , @scn_range
    };

    results = struct('name', {}, 'req', {}, 'pass', {}, 'detail', {});
    figure('Name', 'Torque Safety Monitor — MIL', 'Color', 'w');

    for i = 1:size(cases, 1)
        name = cases{i, 1};
        scn  = cases{i, 2}(S);
        out  = simulateScenario(mdl, scn, S);
        [pass, req, detail] = evaluate(name, out, S, scn);
        results(end+1) = struct('name', name, 'req', req, 'pass', pass, 'detail', detail); %#ok<AGROW>
        subplot(size(cases,1), 1, i); plotCase(name, out, S);
    end

    printSummary(results);
end

% =======================================================================
function out = simulateScenario(mdl, scn, S)
    in = Simulink.SimulationInput(mdl);
    in = in.setExternalInput(scn.signals);
    in = in.setModelParameter('StopTime', num2str(scn.tEnd));
    so = sim(in);
    out.t          = so.tout;
    out.state      = getLog(so, 'state');
    out.tq_cmd     = getLog(so, 'tq_cmd');
    out.safe_state = getLog(so, 'safe_state');
end

function v = getLog(so, name)
    try
        v = so.(name); if isa(v, 'timeseries'); v = v.Data; end
    catch
        v = so.yout.get(name).Values.Data;
    end
end

% =======================================================================
function [pass, req, detail] = evaluate(name, out, S, scn)
    reachedL2 = any(out.state == S.STATE.L2_LIMIT);
    reachedL3 = any(out.state == S.STATE.L3_SAFE);
    switch name
        case 'tc_l2'
            idx = find(out.state == S.STATE.L2_LIMIT, 1, 'first');
            rt  = isempty(idx)*NaN + ~isempty(idx)*(out.t(idx) - scn.tFault);
            pass = reachedL2 && rt <= S.ftti && all(out.tq_cmd(idx:end) <= S.t_limp + 1e-6);
            req  = 'REQ-SAF-002/003';
            detail = sprintf('L2 reaction = %.0f ms (FTTI %.0f ms); tq_cmd <= t_limp', 1000*rt, 1000*S.ftti);
        case 'tc_l2_glitch'
            pass = ~reachedL2 && ~reachedL3;
            req  = 'REQ-SAF-004';
            detail = 'single-sample excess did not trip';
        case 'tc_l3'
            idx = find(out.state == S.STATE.L3_SAFE, 1, 'first');
            rt  = isempty(idx)*NaN + ~isempty(idx)*(out.t(idx) - scn.tFault);
            pass = reachedL3 && rt <= S.ftti && out.tq_cmd(end) == 0;
            req  = 'REQ-SAF-005/006';
            detail = sprintf('L3 safe-state = %.0f ms (FTTI %.0f ms); tq_cmd = 0', 1000*rt, 1000*S.ftti);
        case 'tc_latch'
            % REQ-SAF-007: reaction holds after the fault clears (no self-recovery).
            held = out.state(end) == S.STATE.L2_LIMIT;
            pass = reachedL2 && held;
            req  = 'REQ-SAF-007';
            detail = sprintf('end state = %d (latched L2_LIMIT after fault cleared)', ...
                             double(out.state(end)));
        case 'tc_priority'
            pass = reachedL3 && out.state(end) == S.STATE.L3_SAFE && out.tq_cmd(end) == 0;
            req  = 'REQ-SAF-008';
            detail = 'escalated L2 -> L3 safe state';
        case 'tc_nominal'
            pass = ~reachedL2 && ~reachedL3 && all(out.state == S.STATE.OK);
            req  = 'REQ-SAF-009';
            detail = 'nominal ripple within margin: no reaction';
        case 'tc_range'
            % REQ-SAF-010: outputs stay within declared ranges over a mixed run.
            pass = all(out.state >= 0 & out.state <= 2) && ...
                   all(out.tq_cmd >= -1e-6 & out.tq_cmd <= S.t_max + 1e-6);
            req  = 'REQ-SAF-010';
            detail = sprintf('state∈[0,2] · 0≤tq_cmd≤%g', S.t_max);
        otherwise
            pass = false; req = '?'; detail = 'unknown case';
    end
    inRange = all(out.state >= 0 & out.state <= 2) && all(out.tq_cmd >= -1e-6 & out.tq_cmd <= S.t_max + 1e-6);
    pass    = pass && inRange;
    detail  = [detail sprintf(' | range_ok=%d', inRange)];
end

% =======================================================================
% ---- Scenario builders -------------------------------------------------
function scn = scn_l2(S)
    scn.tEnd = 0.6; scn.tFault = 0.2;
    req = @(p) p/100*S.t_max;
    scn.signals = sigBus(S, scn.tEnd, 30, @(t) req(30) + 80*(t >= scn.tFault), 1);
end
function scn = scn_l2_glitch(S)
    scn.tEnd = 0.4; scn.tFault = 0.2;
    req = @(p) p/100*S.t_max;
    spike = @(t) (abs(t - scn.tFault) < S.Ts/2);
    scn.signals = sigBus(S, scn.tEnd, 30, @(t) req(30) + 80*spike(t), 1);
end
function scn = scn_l3(S)
    scn.tEnd = 0.4; scn.tFault = 0.2;
    req = @(p) p/100*S.t_max;
    scn.signals = sigBus(S, scn.tEnd, 30, @(t) req(30), @(t) double(t < scn.tFault));
end
function scn = scn_latch(S)
    scn.tEnd = 0.8; scn.tFault = 0.2;
    req = @(p) p/100*S.t_max;
    % excess on [0.2, 0.4] only; after it clears the L2 reaction must stay latched
    scn.signals = sigBus(S, scn.tEnd, 30, @(t) req(30) + 80*(t >= scn.tFault & t < 0.4), 1);
end
function scn = scn_priority(S)
    scn.tEnd = 0.8; scn.tFault = 0.2;
    req = @(p) p/100*S.t_max;
    scn.signals = sigBus(S, scn.tEnd, 30, @(t) req(30) + 80*(t >= scn.tFault), @(t) double(t < 0.5));
end
function scn = scn_range(S)
    scn.tEnd = 0.8; scn.tFault = 0.2;
    req = @(p) p/100*S.t_max;
    % mixed run: OK -> L2 (excess at 0.2) -> L3 (watchdog fails at 0.5)
    scn.signals = sigBus(S, scn.tEnd, 50, @(t) req(50) + 90*(t >= scn.tFault), @(t) double(t < 0.5));
end
function scn = scn_nominal(S)
    scn.tEnd = 1.0; scn.tFault = Inf;
    req = @(p) p/100*S.t_max;
    scn.signals = sigBus(S, scn.tEnd, 40, @(t) req(40) + 20*sin(40*t), 1);  % +/-20 < margin 30
end

function bus = sigBus(S, tEnd, pedal, tq, wd)
    t = (0:S.Ts:tEnd)';
    bus = Simulink.SimulationData.Dataset;
    bus = bus.addElement(asTs(t, pedal, 'pedal'));
    bus = bus.addElement(asTs(t, tq,    'torque_actual'));
    bus = bus.addElement(asTs(t, wd,    'wd_ok'));
end

function ts = asTs(t, v, name)
    if isa(v, 'function_handle'); data = arrayfun(v, t); else; data = v + 0*t; end
    ts = timeseries(data, t, 'Name', name);
end

% =======================================================================
function plotCase(name, out, S)
    yyaxis left;  plot(out.t, out.tq_cmd, 'LineWidth', 1.2); ylabel('tq cmd [Nm]'); ylim([-10 S.t_max]);
    yyaxis right; stairs(out.t, double(out.state), 'LineWidth', 0.9); ylabel('state'); ylim([-0.3 2.3]);
    title(name, 'Interpreter', 'none'); grid on;
end

function printSummary(results)
    fprintf('\n=== Torque safety-monitor MIL summary ===\n');
    npass = 0;
    for i = 1:numel(results)
        r = results(i); tag = '[PASS]'; if ~r.pass; tag = '[FAIL]'; end
        npass = npass + r.pass;
        fprintf('%s  %-14s %-18s  %s\n', tag, r.name, r.req, r.detail);
    end
    fprintf('-----------------------------------------\n%d/%d test cases passed.\n\n', npass, numel(results));
    assert(npass == numel(results), 'run_mil: one or more requirement checks FAILED.');
end
