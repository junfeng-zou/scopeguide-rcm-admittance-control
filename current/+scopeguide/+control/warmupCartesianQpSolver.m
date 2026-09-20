function report = warmupCartesianQpSolver(cfg, options)
%WARMUPCARTESIANQPSOLVER Compile/cache the no-RCM QP before PREARM.

arguments
    cfg (1, 1) struct
    options.Iterations (1, 1) double = 12
end
validateCartesianAdmittanceDragConfig(cfg);
if exist('quadprog', 'file') ~= 2 || ...
        ~licenseAvailable('Optimization_Toolbox')
    error('scopeguide:cartesianDrag:QuadprogUnavailable', ...
        'quadprog and Optimization Toolbox are required.');
end
if options.Iterations < 1 || options.Iterations ~= fix(options.Iterations)
    error('scopeguide:cartesianDrag:InvalidWarmupIterations', ...
        'Iterations must be a positive integer.');
end
q = deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
desired = [0; 0; 1e-4; 0; 0; 0];
times = nan(options.Iterations, 1);
solved = false(options.Iterations, 1);
failureCodes = strings(options.Iterations, 1);
for index = 1:options.Iterations
    result = scopeguide.control.solveCartesianVelocityQp( ...
        q, zeros(6, 1), desired, zeros(6, 1), ...
        geometry.RotationBaseFromDrag, cfg.runtime.NominalDtSec, cfg);
    times(index) = result.SolveTimeSec;
    solved(index) = result.MotionCommandValid;
    failureCodes(index) = result.FailureCode;
end
report = struct();
report.Iterations = options.Iterations;
report.SolveTimeSec = times;
report.Solved = solved;
report.FailureCodes = failureCodes;
report.LastSolveTimeSec = times(end);
report.LastSolved = solved(end);
report.ReadyForTimingAudit = solved(end) && ...
    times(end) <= cfg.qp.MaximumSolveTimeSec;
report.NoRcmConstraint = true;
report.DoesNotSendHardwareCommands = true;
end

function available = licenseAvailable(feature)
try
    available = logical(license('test', feature));
catch
    available = false;
end
end
