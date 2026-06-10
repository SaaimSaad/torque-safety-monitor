% safety_params.m  —  Single source of parameters for the torque safety monitor.
% Original work, MIT licence. Generic E-Gas three-level torque monitor;
% no proprietary content.
%
% Run this first; it populates struct `S` in the base workspace. The Simulink
% model, the test scripts, and the browser simulation all share these values.

S = struct();

% ---- Timing -------------------------------------------------------------
S.Ts        = 0.01;     % monitor sample time [s]  (100 Hz)
S.ftti      = 0.05;     % fault-tolerant time interval [s] (reaction deadline)

% ---- Torque model (Level 2) ---------------------------------------------
S.t_max     = 300.0;    % maximum engine torque [Nm]
S.t_margin  = 30.0;     % allowance above driver-requested torque [Nm]
S.t_limp    = 60.0;     % limited (limp) torque in L2 reaction [Nm]

% ---- Debounce / watchdog ------------------------------------------------
S.l2_cnt    = 3;        % sustained samples to confirm an L2 fault (= 30 ms)
S.wd_cnt    = 5;        % missed watchdog answers to trip L3        (= 50 ms)

% ---- State enum (kept in sync with Stateflow / generated code) ----------
S.STATE.OK       = uint8(0);
S.STATE.L2_LIMIT = uint8(1);
S.STATE.L3_SAFE  = uint8(2);

assignin('base', 'S', S);
fprintf('safety_params: loaded %d parameters (Ts = %g s, FTTI = %g s).\n', ...
        numel(fieldnames(S)), S.Ts, S.ftti);
