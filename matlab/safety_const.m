function S = safety_const()
%#codegen
% safety_const.m  —  Compile-time constants for the deployed safety monitor.
% Mirrors the tunable values in safety_params.m, but as a code-generation-friendly
% constant function (no base-workspace dependency). Keep the two in sync.
% Original work, MIT licence.

    S.Ts       = 0.01;
    S.ftti     = 0.05;

    S.t_max    = 300.0;
    S.t_margin = 30.0;
    S.t_limp   = 60.0;

    S.l2_cnt   = 3;
    S.wd_cnt   = 5;
end
