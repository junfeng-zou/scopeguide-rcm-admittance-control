function summary = run_cartesian_admittance_offline_validation(options)
%RUN_CARTESIAN_ADMITTANCE_OFFLINE_VALIDATION Synthetic no-hardware replay.
% This function creates no robot, force-sensor, dashboard or network object.

arguments
    options.Config = struct([])
    options.DurationSec (1, 1) double = 12
end
projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
if isempty(options.Config)
    cfg = current_cartesian_admittance_drag_config();
else
    cfg = options.Config;
end
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    cfg, "full_6dof");
validateCartesianAdmittanceDragConfig(cfg);
warmup = scopeguide.control.warmupCartesianQpSolver(cfg);
if ~warmup.ReadyForTimingAudit
    error('scopeguide:cartesianDrag:OfflineWarmupFailed', ...
        'No-RCM QP warm-up did not meet the production timing budget.');
end

dtSec = cfg.runtime.NominalDtSec;
sampleCount = floor(options.DurationSec / dtSec);
q = deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
initialGeometry = ...
    scopeguide.geometry.computeDragPointKinematics(q, cfg);
anchor = initialGeometry.TBaseDragPoint;
admittance = scopeguide.control.CartesianAdmittance6D(cfg);
qp = scopeguide.control.CartesianVelocityQpController(cfg);
enabled = struct('MotionPermitted', true, ...
    'ResetDynamicState', false, 'CommandScale', 1);

relativeLog = nan(6, sampleCount);
desiredLog = nan(6, sampleCount);
achievedLog = nan(6, sampleCount);
qdotLog = nan(6, sampleCount);
solveTime = nan(sampleCount, 1);
failureCodes = strings(sampleCount, 1);
solved = false(sampleCount, 1);
for index = 1:sampleCount
    timeSec = (index - 1) * dtSec;
    geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
    relative = scopeguide.geometry.relativeDragPose( ...
        anchor, geometry.TBaseDragPoint);
    wrench = syntheticWrench(timeSec);
    admittanceOutput = admittance.step(wrench, ...
        geometry.RotationBaseFromDrag, dtSec, enabled);
    [desired, ~] = scopeguide.control.limitCartesianTwistForTravel( ...
        admittanceOutput.DesiredTwistBase, relative, ...
        anchor(1:3, 1:3), dtSec, cfg);
    qpOutput = qp.step(q, desired, relative, anchor(1:3, 1:3), ...
        dtSec, enabled);
    solved(index) = qpOutput.MotionCommandValid;
    failureCodes(index) = qpOutput.FailureCode;
    solveTime(index) = qpOutput.SolveTimeSec;
    if qpOutput.RequiresFault
        break;
    end
    q = q + qpOutput.QdotCommandRadSec * dtSec;
    relativeLog(:, index) = relative;
    desiredLog(:, index) = desired;
    achievedLog(:, index) = qpOutput.AchievedTwistBase;
    qdotLog(:, index) = qpOutput.QdotCommandRadSec;
end
completed = find(~isnan(relativeLog(1, :)), 1, 'last');
if isempty(completed)
    completed = 0;
end
finalGeometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
finalRelative = scopeguide.geometry.relativeDragPose( ...
    anchor, finalGeometry.TBaseDragPoint);
limits = [cfg.cartesianDrag.Translation.RelativeLimitM(:); ...
    cfg.cartesianDrag.Rotation.RelativeLimitRad(:)];
maximumNormalizedTravel = max(abs(finalRelative) ./ limits);
if completed > 0
    maximumNormalizedTravel = max(maximumNormalizedTravel, ...
        max(abs(relativeLog(:, 1:completed)) ./ limits, [], 'all'));
end
summary = struct();
summary.Status = "PASS";
summary.RequestedSampleCount = sampleCount;
summary.CompletedSampleCount = completed;
summary.SolvedSampleCount = nnz(solved);
summary.SolveRate = nnz(solved) / max(sampleCount, 1);
summary.FailureCodes = unique(failureCodes(strlength(failureCodes) > 0));
summary.MaximumQpSolveTimeSec = max(solveTime, [], 'omitnan');
summary.MaximumNormalizedTravel = maximumNormalizedTravel;
summary.FinalRelativeCoordinate = finalRelative;
summary.MaximumJointSpeedRadSec = max(abs(qdotLog), [], 'all', 'omitnan');
summary.NoRcmConstraint = true;
summary.RcmPointReadCount = 0;
summary.HardwareConnectionCount = 0;
summary.MotionCommandCount = 0;
summary.Warmup = warmup;
if completed ~= sampleCount || ~all(solved) || ...
        maximumNormalizedTravel > 1 + cfg.qp.ConstraintTolerance
    summary.Status = "FAIL";
end
fprintf(['No-RCM Cartesian offline validation: %s; solved=%d/%d; ' ...
    'max travel=%.1f%%; hardware connections=0; commands=0.\n'], ...
    summary.Status, summary.SolvedSampleCount, sampleCount, ...
    100 * summary.MaximumNormalizedTravel);
end

function wrench = syntheticWrench(timeSec)
% One second of positive input and one second of release on each axis.
phase = floor(timeSec / 2);
within = mod(timeSec, 2);
wrench = zeros(6, 1);
if phase >= 0 && phase < 6 && within < 1
    if phase < 3
        wrench(phase + 1) = 4.0;
    else
        wrench(phase + 1) = 0.35;
    end
end
end
