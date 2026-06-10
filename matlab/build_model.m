function build_model()
% build_model.m  —  Construct the torque safety monitor AS CODE.
% Original work, MIT licence. Generic E-Gas three-level torque monitor.
%
% Creates torque_safety.slx containing:
%   - SafetyMonitor : atomic subsystem = L2 monitor + L3 watchdog
%                     (MATLAB Function) + safe-state Supervisor (Stateflow).
%                     This is the code-generated unit. The supervisor drives
%                     the torque command (pass-through / limited / zero),
%                     latches reactions, and gives L3 priority over L2.
%
% Prerequisite: run `safety_params` first so struct S exists in the base workspace.
%
% Defining the model in code (rather than hand-drawing a .slx) keeps the
% design reviewable and diffable in version control.
%
% NOTE: authored for a clean personal MATLAB (Home / trial / Online) with
% Simulink + Stateflow; validate there. Do not build on a client install.

    mdl = 'torque_safety';
    if ~evalin('base', 'exist(''S'',''var'')')
        error('build_model:noParams', 'Run `safety_params` first to load struct S.');
    end

    close_system(mdl, 0); %#ok<*NASGU>
    if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end
    new_system(mdl);
    open_system(mdl);

    set_param(mdl, 'SolverType', 'Fixed-step', ...
                   'Solver',     'FixedStepDiscrete', ...
                   'FixedStep',  'S.Ts', ...
                   'StopTime',   '1');

    addInports(mdl);
    addMonitor(mdl);          % atomic subsystem: L2 + L3 + Supervisor
    addScopesAndLogging(mdl);
    wireModel(mdl);

    Simulink.BlockDiagram.arrangeSystem(mdl);
    save_system(mdl);
    fprintf('build_model: created %s.slx\n', mdl);
end

% =======================================================================
function addInports(mdl)
    names = {'pedal','torque_actual','wd_ok'};
    for i = 1:numel(names)
        add_block('simulink/Sources/In1', [mdl '/' names{i}], 'Position', pos(40, 40 + (i-1)*60));
    end
end

% =======================================================================
function addMonitor(mdl)
    sub = [mdl '/SafetyMonitor'];
    add_block('built-in/Subsystem', sub, 'Position', pos(300, 60));
    set_param(sub, 'TreatAsAtomicUnit', 'on');   % atomic -> code-gen unit

    % L2 monitor + L3 watchdog (MATLAB Function): produces the decision flags
    % plus permissible torque and the limp ceiling for the supervisor.
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [sub '/Monitor'], 'Position', pos(220, 60));
    setFcn(mdl, 'SafetyMonitor/Monitor', monitorCode());

    % Safe-state supervisor (Stateflow chart) placed INSIDE the subsystem
    add_block('sflib/Chart', [sub '/Supervisor'], 'Position', pos(460, 60));
    buildSupervisorChart(sub);

    addSubsysPorts(sub);
    wireMonitor(sub);
end

% =======================================================================
function buildSupervisorChart(sub)
% Build the OK / L2_LIMIT / L3_SAFE safe-state machine. L3 has priority and
% both reactions latch (recovery only via a key cycle / model reset).
    rt    = sfroot;
    chart = rt.find('-isa', 'Stateflow.Chart', '-and', 'Path', [sub '/Supervisor']);
    chart.ChartUpdate = 'DISCRETE';
    chart.SampleTime  = 'S.Ts';

    % ---- Chart data (ports appear in creation order) -------------------
    % Inputs: decision flags + the torque values needed to form tq_cmd.
    inNames = {'l2_confirmed','l3_trip','torque_actual','permissible','t_limp'};
    inTypes = {'boolean','boolean','double','double','double'};
    for i = 1:numel(inNames); addData(chart, inNames{i}, 'Input', inTypes{i}); end
    % Outputs: match the SWC / subsystem interface.
    outNames = {'state','tq_cmd','safe_state'};
    outTypes = {'uint8','double','boolean'};
    for i = 1:numel(outNames); addData(chart, outNames{i}, 'Output', outTypes{i}); end

    % ---- States --------------------------------------------------------
    names = {'OK','L2_LIMIT','L3_SAFE'};
    St = struct();
    for i = 1:numel(names)
        s = Stateflow.State(chart);
        s.Name     = names{i};
        s.Position = [60, 60 + (i-1)*120, 200, 80];
        St.(names{i}) = s;
    end
    % tq_cmd is recomputed every step (en, du) because it tracks the inputs;
    % state/safe_state are constant per state. tq_cmd is non-increasing in
    % severity: OK >= L2_LIMIT >= L3_SAFE = 0.
    St.OK.LabelString       = sprintf(['OK\n' ...
        'en, du: state = uint8(0); safe_state = false;\n' ...
        'tq_cmd = min(torque_actual, permissible);']);
    St.L2_LIMIT.LabelString = sprintf(['L2_LIMIT\n' ...
        'en, du: state = uint8(1); safe_state = true;\n' ...
        'tq_cmd = min(torque_actual, t_limp);']);   % latched (REQ-SAF-007)
    St.L3_SAFE.LabelString  = sprintf(['L3_SAFE\n' ...
        'en, du: state = uint8(2); safe_state = true;\n' ...
        'tq_cmd = 0;']);                            % latched safe state (REQ-SAF-006)

    % Default transition into OK
    dt = Stateflow.Transition(chart);
    dt.Destination = St.OK;

    tL2 = addTrans(chart, St.OK,       St.L2_LIMIT, 'l2_confirmed');   % debounced L2 fault
    tL3 = addTrans(chart, St.OK,       St.L3_SAFE,  'l3_trip');        % L3 can fire from OK
    addTrans(chart, St.L2_LIMIT, St.L3_SAFE,  'l3_trip');              % escalation L2 -> L3
    % L3 (safe state) has priority over L2 on a same-step double fault (REQ-SAF-008):
    % evaluate OK -> L3_SAFE before OK -> L2_LIMIT.
    tL3.ExecutionOrder = 1;
    tL2.ExecutionOrder = 2;
end

function d = addData(chart, name, scope, dtype)
    d = Stateflow.Data(chart);
    d.Name     = name;
    d.Scope    = scope;        % 'Input' | 'Output' | 'Local'
    d.DataType = dtype;
end

function t = addTrans(chart, src, dst, cond)
    t = Stateflow.Transition(chart);
    t.Source      = src;
    t.Destination = dst;
    t.LabelString = ['[' cond ']'];
end

% =======================================================================
function addScopesAndLogging(mdl)
    add_block('simulink/Sinks/Out1', [mdl '/state'],      'Position', pos(900, 60));
    add_block('simulink/Sinks/Out1', [mdl '/tq_cmd'],     'Position', pos(900, 120));
    add_block('simulink/Sinks/Out1', [mdl '/safe_state'], 'Position', pos(900, 180));
end

% =======================================================================
function addSubsysPorts(sub)
    inports = {'pedal','torque_actual','wd_ok'};
    for i = 1:numel(inports)
        add_block('simulink/Sources/In1', [sub '/' inports{i}], ...
                  'Position', pos(40, 40 + (i-1)*60));
    end
    outports = {'state','tq_cmd','safe_state'};
    for i = 1:numel(outports)
        add_block('simulink/Sinks/Out1', [sub '/' outports{i}], ...
                  'Position', pos(720, 40 + (i-1)*60));
    end
end

% =======================================================================
function wireMonitor(sub)
% Internal wiring of the SafetyMonitor subsystem. Chart ports follow the data
% creation order in buildSupervisorChart (inputs 1..5, outputs 1..3).
    M = 'Monitor';      % MATLAB Function: in (pedal,torque_actual,wd_ok)
                        %                  out (l2_confirmed,l3_trip,permissible,t_limp)
    C = 'Supervisor';   % Stateflow chart

    % inports -> Monitor / chart
    autoConnect(sub, 'pedal/1',         [M '/1']);   % pedal -> Monitor
    autoConnect(sub, 'torque_actual/1', [M '/2']);   % torque_actual -> Monitor
    autoConnect(sub, 'torque_actual/1', [C '/3']);   % torque_actual -> chart
    autoConnect(sub, 'wd_ok/1',         [M '/3']);   % wd_ok -> Monitor

    % Monitor outputs -> chart inputs
    autoConnect(sub, [M '/1'], [C '/1']);   % l2_confirmed
    autoConnect(sub, [M '/2'], [C '/2']);   % l3_trip
    autoConnect(sub, [M '/3'], [C '/4']);   % permissible
    autoConnect(sub, [M '/4'], [C '/5']);   % t_limp

    % chart outputs -> subsystem outports
    autoConnect(sub, [C '/1'], 'state/1');
    autoConnect(sub, [C '/2'], 'tq_cmd/1');
    autoConnect(sub, [C '/3'], 'safe_state/1');
end

% =======================================================================
function wireModel(mdl)
    autoConnect(mdl, 'pedal/1',         'SafetyMonitor/1');
    autoConnect(mdl, 'torque_actual/1', 'SafetyMonitor/2');
    autoConnect(mdl, 'wd_ok/1',         'SafetyMonitor/3');

    autoConnect(mdl, 'SafetyMonitor/1', 'state/1');
    autoConnect(mdl, 'SafetyMonitor/2', 'tq_cmd/1');
    autoConnect(mdl, 'SafetyMonitor/3', 'safe_state/1');
end

function autoConnect(sys, src, dst)
    try
        add_line(sys, src, dst, 'autorouting', 'smart');
    catch ME
        warning('build_model:wire', 'Could not connect %s -> %s (%s).', src, dst, ME.message);
    end
end

% =======================================================================
function setFcn(mdl, blockRelPath, code)
    S  = sfroot;
    fn = S.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/' blockRelPath]);
    fn.Script = code;
end

function p = pos(x, y)
    p = [x, y, x + 110, y + 40];
end

% =======================================================================
% ---- Embedded MATLAB: Monitor (L2 permissible-torque + L3 watchdog) ----
function c = monitorCode()
c = [ ...
"function [l2_confirmed, l3_trip, permissible, t_limp] = Monitor(pedal, torque_actual, wd_ok)" newline ...
"%#codegen" newline ...
"%  Level 2: independent permissible-torque model + debounced excess test." newline ...
"%  Level 3: question/answer watchdog timeout. Both feed the supervisor;" newline ...
"%  permissible and t_limp are forwarded so the chart can form tq_cmd." newline ...
"S = safety_const();" newline ...
"persistent l2c wdf; if isempty(l2c); l2c = 0; wdf = 0; end" newline ...
"" newline ...
"requested   = pedal/100 * S.t_max;          % independent driver-demand estimate" newline ...
"permissible = requested + S.t_margin;        % L2 ceiling (REQ-SAF-001)" newline ...
"t_limp      = S.t_limp;                       % limp ceiling forwarded to supervisor" newline ...
"" newline ...
"if torque_actual > permissible               % REQ-SAF-002" newline ...
"    l2c = min(l2c + 1, S.l2_cnt);" newline ...
"else" newline ...
"    l2c = 0;" newline ...
"end" newline ...
"l2_confirmed = l2c >= S.l2_cnt;              % debounced (REQ-SAF-004)" newline ...
"" newline ...
"if ~wd_ok                                    % REQ-SAF-005" newline ...
"    wdf = min(wdf + 1, S.wd_cnt);" newline ...
"else" newline ...
"    wdf = 0;" newline ...
"end" newline ...
"l3_trip = wdf >= S.wd_cnt;                   % REQ-SAF-006" newline ...
"end" ];
end
