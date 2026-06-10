function collect_coverage()
% collect_coverage.m  —  Structural coverage for torque_safety.
% Original work, MIT licence.
%
% Records decision / condition / MCDC coverage over the full MIL test suite
% and exports an HTML report to ../assets/coverage/. Fails the run if any
% metric is below the target in docs/test-plan.md.
%
% Requires: Simulink Coverage.
% Usage:  safety_params; build_model; collect_coverage

    mdl = 'torque_safety';
    if ~exist([mdl '.slx'], 'file'); build_model(); end

    target = 100;   % percent, for decision / condition / MCDC

    set_param(mdl, 'CovEnable',                'on');
    set_param(mdl, 'CovMetricStructuralLevel', 'MCDC');
    set_param(mdl, 'RecordCoverage',           'on');
    set_param(mdl, 'CovScope',                 'Subsystem');
    set_param(mdl, 'CovPath',                  '/SafetyMonitor');

    scenarios = {'tc_l2','tc_l2_glitch','tc_l3','tc_latch','tc_priority','tc_nominal','tc_range'};
    cumCov = [];
    for i = 1:numel(scenarios)
        in    = Simulink.SimulationInput(mdl);
        in    = in.setModelParameter('StopTime', '0.8');
        in    = in.setModelParameter('CovEnable', 'on');
        cdata = cvsim(in); %#ok<NASGU>
        cumCov = appendCoverage(cumCov, cdata);
    end

    outDir = fullfile('..', 'assets', 'coverage');
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    cvhtml(fullfile(outDir, 'index.html'), cumCov);

    d  = decisioninfo(cumCov, [mdl '/SafetyMonitor']);
    c  = conditioninfo(cumCov, [mdl '/SafetyMonitor']);
    m  = mcdcinfo(cumCov, [mdl '/SafetyMonitor']);
    pd = 100 * d(1) / max(d(2), 1);
    pc = 100 * c(1) / max(c(2), 1);
    pm = 100 * m(1) / max(m(2), 1);

    fprintf('\n=== Coverage (SafetyMonitor) ===\n');
    fprintf('Decision : %.1f%%\nCondition: %.1f%%\nMCDC     : %.1f%%\n', pd, pc, pm);
    fprintf('Report   : %s\n\n', fullfile(outDir, 'index.html'));

    assert(all([pd pc pm] >= target - 1e-9), ...
           'collect_coverage: coverage below %d%% target — add test cases.', target);
end

function acc = appendCoverage(acc, c)
    if isempty(acc); acc = c; else; acc = acc + c; end
end
