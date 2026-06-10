function run_sil()
% run_sil.m  —  Software-in-the-loop equivalence for SafetyMonitor.
% Original work, MIT licence.
%
% Generates code, runs the tc_l2 and tc_l3 stimuli in SIL mode, and asserts
% MIL/SIL output equivalence (state identical, tq_cmd within tolerance).
%
% Requires: Embedded Coder + a supported C compiler (run `mex -setup`).
% Usage:  safety_params; build_model; run_sil

    mdl = 'torque_safety';
    if ~exist([mdl '.slx'], 'file'); build_model(); end
    tol = 1e-3;

    scenarios = {'tc_l2', 'tc_l3'};
    for i = 1:numel(scenarios)
        name   = scenarios{i};
        milOut = simulateMode(mdl, 'normal');
        silOut = simulateMode(mdl, 'sil');

        dq = max(abs(milOut.tq_cmd - silOut.tq_cmd));
        ds = max(abs(double(milOut.state) - double(silOut.state)));
        ok = (dq <= tol) && (ds == 0);
        tag = '[PASS]'; if ~ok; tag = '[FAIL]'; end
        fprintf('%s  %-10s  max|d_tq|=%.2e  max|d_state|=%d\n', tag, name, dq, ds);
        assert(ok, 'run_sil: MIL/SIL mismatch on %s', name);
    end
    fprintf('run_sil: MIL/SIL equivalence verified.\n');
end

function out = simulateMode(mdl, mode)
    sub = [mdl '/SafetyMonitor'];
    set_param(sub, 'SimulationMode', mode);   % 'normal' or 'sil'
    in  = Simulink.SimulationInput(mdl);
    in  = in.setModelParameter('StopTime', '0.6');
    so  = sim(in);
    set_param(sub, 'SimulationMode', 'normal');
    out.tq_cmd = reshapeLog(so, 'tq_cmd');
    out.state  = reshapeLog(so, 'state');
end

function v = reshapeLog(so, name)
    try; v = so.(name); if isa(v,'timeseries'); v = v.Data; end
    catch; v = so.yout.get(name).Values.Data; end
end
