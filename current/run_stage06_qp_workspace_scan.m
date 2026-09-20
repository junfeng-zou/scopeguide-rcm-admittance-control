function [scan, outputDirectory] = run_stage06_qp_workspace_scan(options)
%RUN_STAGE06_QP_WORKSPACE_SCAN Offline synthetic RCM QP feasibility scan.
% The scan evaluates representative CR5 joint configurations, but creates
% no RobotAdapter/ForceSensorAdapter and sends no motion command.

arguments
    options.Config = struct([])
    options.SampleCount (1, 1) double = 300
    options.RandomSeed (1, 1) double = 606
    options.WriteResults (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = options.Config;
end
validateRcmAdmittanceConfig(cfg);
if options.SampleCount < 3 || options.SampleCount ~= fix(options.SampleCount)
    error('scopeguide:stage06:InvalidScanSampleCount', ...
        'SampleCount must be an integer of at least 3.');
end
if exist('quadprog', 'file') ~= 2 || ...
        ~logical(license('test', 'Optimization_Toolbox'))
    error('scopeguide:stage06:QuadprogUnavailable', ...
        'Stage 6 scan requires quadprog and Optimization Toolbox.');
end

warmup = warmupAllModes(cfg);
rng(options.RandomSeed);
modes = ["hard", "soft", "hybrid"];
centers = safeWorkspaceCenters();
count = options.SampleCount;
modeLog = strings(count, 1);
jointPosition = nan(count, 6);
rcmErrorM = nan(count, 1);
singularityScale = nan(count, 1);
sigmaMinimum = nan(count, 1);
conditionNumber = nan(count, 1);
solveTimeSec = nan(count, 1);
exitFlag = nan(count, 1);
solved = false(count, 1);
failureCode = strings(count, 1);
maximumViolation = nan(count, 1);
predictedRcmErrorM = nan(count, 1);
relativePivotRad = nan(count, 1);
relativeInsertionM = nan(count, 1);

for index = 1:count
    mode = modes(1 + mod(index - 1, numel(modes)));
    cfgSample = cfg;
    cfgSample.rcm.Mode = mode;
    center = centers(:, 1 + mod(index - 1, size(centers, 2)));
    q = center + 0.06 * randn(6, 1);
    [pAxis, basis] = syntheticAxisPoint(q, cfgSample);
    if mode == "hard"
        errorMagnitude = 0;
    elseif mode == "soft"
        errorMagnitude = 0.4e-3 * rand;
    else
        errorMagnitude = 1.35e-3 * rand;
    end
    errorAngle = 2 * pi * rand;
    errorDirection = cos(errorAngle) * basis.PivotAxis1Base + ...
        sin(errorAngle) * basis.PivotAxis2Base;
    pRcm = pAxis + errorMagnitude * errorDirection;

    pivotRelativeMagnitude = 0.6 * ...
        cfg.control.RelativePivotLimitRad * rand;
    pivotRelativeAngle = 2 * pi * rand;
    relative = [pivotRelativeMagnitude * cos(pivotRelativeAngle); ...
        pivotRelativeMagnitude * sin(pivotRelativeAngle); ...
        0.6 * cfg.control.RelativeInsertionLimitM * (2 * rand - 1)];
    generalizedVelocity = [ ...
        deg2rad(0.08) * (2 * rand - 1); ...
        deg2rad(0.08) * (2 * rand - 1); ...
        0.0003 * (2 * rand - 1)];
    desiredTwist = basis.TwistBasisBase * generalizedVelocity;
    result = scopeguide.control.solveRcmConstrainedQp( ...
        q, zeros(6, 1), desiredTwist, relative, ...
        pRcm, cfg.runtime.NominalDtSec, cfgSample);

    modeLog(index) = mode;
    jointPosition(index, :) = q.';
    rcmErrorM(index) = result.CurrentRcmErrorNormM;
    singularityScale(index) = result.SingularityScale;
    sigmaMinimum(index) = result.ScaledJacobianSigmaMinimum;
    conditionNumber(index) = result.ScaledJacobianConditionNumber;
    solveTimeSec(index) = result.SolveTimeSec;
    exitFlag(index) = result.ExitFlag;
    solved(index) = result.MotionCommandValid;
    failureCode(index) = result.FailureCode;
    predictedRcmErrorM(index) = result.PredictedRcmErrorNormM;
    if result.MotionCommandValid
        maximumViolation(index) = ...
            result.ConstraintDiagnostics.MaximumViolation;
        relativePivotRad(index) = ...
            norm(result.PredictedRelativeCoordinate(1:2));
        relativeInsertionM(index) = ...
            abs(result.PredictedRelativeCoordinate(3));
    end
end

modeStatistics = repmat(timingStatistics("", [], false(0, 1)), ...
    numel(modes), 1);
for index = 1:numel(modes)
    selected = modeLog == modes(index);
    modeStatistics(index) = timingStatistics( ...
        modes(index), solveTimeSec(selected), solved(selected));
end
overallTiming = timingStatistics("all", solveTimeSec, solved);
singularStopCount = nnz(failureCode == "JACOBIAN_SINGULAR_STOP");
unexpectedFailure = ~solved & ...
    failureCode ~= "JACOBIAN_SINGULAR_STOP";
minimumFeasibilityRate = 0.95;
summary = struct();
summary.Stage = 6;
summary.GeneratedAtLocal = string(datetime('now', ...
    'TimeZone', 'Asia/Shanghai', ...
    'Format', 'yyyy-MM-dd HH:mm:ss Z'));
summary.Solver = "quadprog";
summary.QuadprogPath = string(which('quadprog'));
summary.OptimizationToolboxLicense = ...
    logical(license('test', 'Optimization_Toolbox'));
summary.SelectedControlRateHz = cfg.runtime.ControlRateHz;
summary.NominalControlPeriodSec = cfg.runtime.NominalDtSec;
summary.QpSolveBudgetSec = cfg.qp.MaximumSolveTimeSec;
summary.WarmupRequiredBeforePrearm = true;
summary.Warmup = warmup;
summary.WarmupExcludedFromTimingPercentiles = true;
summary.SampleCount = count;
summary.SolvedCount = nnz(solved);
summary.FeasibilityRate = nnz(solved) / count;
summary.MinimumAcceptedFeasibilityRate = minimumFeasibilityRate;
summary.SingularStopCount = singularStopCount;
summary.UnexpectedFailureCount = nnz(unexpectedFailure);
summary.ModeStatistics = modeStatistics;
summary.OverallTiming = overallTiming;
summary.TimingBudgetPassed = overallTiming.P99Sec <= ...
    cfg.qp.MaximumSolveTimeSec;
summary.MaximumConstraintViolation = ...
    max(maximumViolation, [], 'omitnan');
summary.MaximumPredictedRcmErrorM = ...
    max(predictedRcmErrorM, [], 'omitnan');
summary.MaximumRelativePivotRad = ...
    max(relativePivotRad, [], 'omitnan');
summary.MaximumRelativeInsertionM = ...
    max(relativeInsertionM, [], 'omitnan');
summary.AllSuccessfulConstraintsWithinTolerance = ...
    summary.MaximumConstraintViolation <= cfg.qp.ConstraintTolerance;
summary.AllModesHaveSuccessfulSamples = all( ...
    arrayfun(@(value) value.SolvedCount > 0, modeStatistics));
summary.WorkspaceScanPassed = ...
    summary.FeasibilityRate >= minimumFeasibilityRate && ...
    summary.UnexpectedFailureCount == 0 && ...
    summary.AllModesHaveSuccessfulSamples && ...
    summary.TimingBudgetPassed && ...
    summary.AllSuccessfulConstraintsWithinTolerance;
summary.GeometrySource = ...
    "per_pose_synthetic_rcm_on_representative_joint_workspace";
summary.RealRcmCalibrationClaimed = false;
summary.SolveTimeScope = "quadprog_call_only_after_explicit_warmup";
summary.OfflineOnly = true;
summary.HardwareConnectionsCreated = false;
summary.MotionCommandsSent = false;

samples = table((1:count).', modeLog, jointPosition, rcmErrorM, ...
    singularityScale, sigmaMinimum, conditionNumber, solveTimeSec, ...
    exitFlag, solved, failureCode, maximumViolation, ...
    predictedRcmErrorM, relativePivotRad, relativeInsertionM, ...
    'VariableNames', {'SampleIndex', 'Mode', 'JointPositionRad', ...
    'CurrentRcmErrorM', 'SingularityScale', 'SigmaMinimum', ...
    'ConditionNumber', 'SolveTimeSec', 'ExitFlag', 'Solved', ...
    'FailureCode', 'MaximumConstraintViolation', ...
    'PredictedRcmErrorM', 'RelativePivotRad', ...
    'RelativeInsertionM'});
scan = struct('Summary', summary, 'Samples', samples);

if options.WriteResults
    timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    outputDirectory = fullfile(projectRoot, 'results', ...
        "stage06_qp_workspace_scan_" + timestamp);
    mkdir(outputDirectory);
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
    writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
    writetable(samples, fullfile(outputDirectory, 'samples.csv'));
    save(fullfile(outputDirectory, 'scan.mat'), 'scan', 'cfg');
else
    outputDirectory = "";
end

fprintf('Stage 6 QP workspace scan: solved %d / %d (%.2f%%).\n', ...
    summary.SolvedCount, count, 100 * summary.FeasibilityRate);
fprintf('quadprog P50/P95/P99: %.3f / %.3f / %.3f ms; budget %.3f ms.\n', ...
    1e3 * overallTiming.P50Sec, 1e3 * overallTiming.P95Sec, ...
    1e3 * overallTiming.P99Sec, 1e3 * cfg.qp.MaximumSolveTimeSec);
fprintf('Hardware connections: 0; motion commands: 0.\n');
if strlength(outputDirectory) > 0
    fprintf('Results: %s\n', outputDirectory);
end
end

function warmup = warmupAllModes(cfg)
generic = scopeguide.control.warmupRcmQpSolver(cfg, Iterations=12);
q = safeWorkspaceCenters();
q = q(:, 1);
[pAxis, basis] = syntheticAxisPoint(q, cfg);
desired = basis.TwistBasisBase * [deg2rad(0.05); 0; 0.0001];
modes = ["hard", "soft", "hybrid"];
modeLastTime = nan(3, 1);
modeLastSolved = false(3, 1);
for modeIndex = 1:3
    modeConfig = cfg;
    modeConfig.rcm.Mode = modes(modeIndex);
    if modes(modeIndex) == "hard"
        pRcm = pAxis;
    else
        pRcm = pAxis + 8e-4 * basis.PivotAxis1Base;
    end
    for iteration = 1:12
        result = scopeguide.control.solveRcmConstrainedQp( ...
            q, zeros(6, 1), desired, zeros(3, 1), pRcm, ...
            cfg.runtime.NominalDtSec, modeConfig);
    end
    modeLastTime(modeIndex) = result.SolveTimeSec;
    modeLastSolved(modeIndex) = result.MotionCommandValid;
end
warmup = struct();
warmup.Generic = generic;
warmup.Modes = modes;
warmup.ModeLastSolveTimeSec = modeLastTime;
warmup.ModeLastSolved = modeLastSolved;
warmup.AllModesReady = all(modeLastSolved) && ...
    all(modeLastTime <= cfg.qp.MaximumSolveTimeSec);
warmup.DoesNotSendHardwareCommands = true;
end

function centers = safeWorkspaceCenters()
centers = [ ...
    0.2, -0.4, 0.5, -0.8; ...
    -0.5, 0.6, 0.3, 0.4; ...
    0.8, -0.7, -0.6, 0.5; ...
    -0.4, 0.5, -0.4, -0.6; ...
    0.3, -0.3, 0.7, 0.3; ...
    -0.2, 0.4, -0.5, 0.7];
end

function [pRcm, basis] = syntheticAxisPoint(q, cfg)
kinematics = scopeguide.geometry.cr5ForwardKinematics(q, cfg);
pRcm = kinematics.EndoscopeTipPositionBaseM - ...
    0.15 * kinematics.ShaftAxisBase;
basis = scopeguide.geometry.computeRcmMotionBasis( ...
    kinematics.TBaseEndoscope, pRcm, cfg);
end

function statistics = timingStatistics(mode, times, solved)
finiteTimes = times(isfinite(times));
statistics = struct();
statistics.Mode = string(mode);
statistics.SampleCount = numel(times);
statistics.TimedSolveCount = nnz(isfinite(times));
statistics.SolvedCount = nnz(solved);
statistics.FeasibilityRate = nnz(solved) / max(numel(times), 1);
statistics.P50Sec = percentile(finiteTimes, 50);
statistics.P95Sec = percentile(finiteTimes, 95);
statistics.P99Sec = percentile(finiteTimes, 99);
statistics.MaximumSec = maxOrNan(finiteTimes);
end

function value = percentile(values, percentage)
values = sort(values(:));
if isempty(values)
    value = NaN;
    return;
end
position = 1 + (numel(values) - 1) * percentage / 100;
lowerIndex = floor(position);
upperIndex = ceil(position);
fraction = position - lowerIndex;
value = values(lowerIndex) * (1 - fraction) + ...
    values(upperIndex) * fraction;
end

function value = maxOrNan(values)
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function writeJson(filePath, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:stage06:CannotWriteScan', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
