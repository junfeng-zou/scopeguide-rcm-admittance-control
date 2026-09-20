function report = warmupRcmQpSolver(cfg, options)
%WARMUPRCMQPSOLVER Compile/cache quadprog before a control timing audit.
% Warm-up solves are never control commands and are excluded from latency
% percentiles.  A later live runtime must call this before PREARM.

arguments
    cfg (1, 1) struct
    options.Iterations (1, 1) double = 12
    options.IncludeAllRcmModes (1, 1) logical = true
end
validateRcmAdmittanceConfig(cfg);
if exist('quadprog', 'file') ~= 2 || ...
        ~licenseAvailable('Optimization_Toolbox')
    error('scopeguide:control:QuadprogUnavailable', ...
        'quadprog and an Optimization Toolbox license are required.');
end
if options.Iterations < 1 || options.Iterations ~= fix(options.Iterations)
    error('scopeguide:control:InvalidWarmupIterations', ...
        'Warm-up iterations must be a positive integer.');
end

H = eye(6);
f = zeros(6, 1);
A = [eye(6); -eye(6)];
b = ones(12, 1);
qpOptions = optimoptions('quadprog', 'Display', 'off', ...
    'Algorithm', 'interior-point-convex', ...
    'ConstraintTolerance', cfg.qp.ConstraintTolerance, ...
    'OptimalityTolerance', cfg.qp.OptimalityTolerance, ...
    'MaxIterations', cfg.qp.MaximumIterations);
times = nan(options.Iterations, 1);
flags = nan(options.Iterations, 1);
for index = 1:options.Iterations
    startTime = tic;
    [~, ~, flags(index)] = quadprog(H, f, A, b, [], [], ...
        -ones(6, 1), ones(6, 1), [], qpOptions);
    times(index) = toc(startTime);
end
report = struct();
report.Iterations = options.Iterations;
report.SolveTimeSec = times;
report.ExitFlag = flags;
report.AllSolved = all(flags > 0);
report.LastSolveTimeSec = times(end);

if options.IncludeAllRcmModes
    modes = ["hard", "soft", "hybrid"];
else
    modes = string(cfg.rcm.Mode);
end
q = [0.2; -0.5; 0.8; -0.4; 0.3; -0.2];
kinematics = scopeguide.geometry.cr5ForwardKinematics(q, cfg);
pAxis = kinematics.EndoscopeTipPositionBaseM - ...
    0.15 * kinematics.ShaftAxisBase;
basis = scopeguide.geometry.computeRcmMotionBasis( ...
    kinematics.TBaseEndoscope, pAxis, cfg);
desired = basis.TwistBasisBase * [deg2rad(0.05); 0; 0.0001];
modeLastTime = nan(numel(modes), 1);
modeLastSolved = false(numel(modes), 1);
for modeIndex = 1:numel(modes)
    modeConfig = cfg;
    modeConfig.rcm.Mode = modes(modeIndex);
    if modes(modeIndex) == "hard"
        pRcm = pAxis;
    else
        pRcm = pAxis + 8e-4 * basis.PivotAxis1Base;
    end
    for iteration = 1:options.Iterations
        result = scopeguide.control.solveRcmConstrainedQp( ...
            q, zeros(6, 1), desired, zeros(3, 1), pRcm, ...
            cfg.runtime.NominalDtSec, modeConfig);
    end
    modeLastTime(modeIndex) = result.SolveTimeSec;
    modeLastSolved(modeIndex) = result.MotionCommandValid;
end
report.RcmModes = modes;
report.RcmModeLastSolveTimeSec = modeLastTime;
report.RcmModeLastSolved = modeLastSolved;
report.AllRcmModesReady = all(modeLastSolved) && ...
    all(modeLastTime <= cfg.qp.MaximumSolveTimeSec);
report.ReadyForTimingAudit = report.AllSolved && ...
    times(end) <= cfg.qp.MaximumSolveTimeSec && ...
    report.AllRcmModesReady;
report.DoesNotSendHardwareCommands = true;
end

function available = licenseAvailable(feature)
try
    available = logical(license('test', feature));
catch
    available = false;
end
end
