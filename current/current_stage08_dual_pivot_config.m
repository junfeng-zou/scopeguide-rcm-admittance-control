function [cfg, profile] = current_stage08_dual_pivot_config(options)
%CURRENT_STAGE08_DUAL_PIVOT_CONFIG Accepted short-term dual-pivot tuning.
% This profile intentionally keeps the Stage 8 safety confirmation gates
% outside this function.  It fixes only the reviewed control parameters and
% loads the prerequisite completion record from the accepted pivot2 result.

arguments
    options.SourceAcceptedConfigFile (1, 1) string = ""
end

projectRoot = string(fileparts(mfilename('fullpath')));
sourceFile = options.SourceAcceptedConfigFile;
if strlength(sourceFile) == 0
    sourceFile = fullfile(projectRoot, 'resources', ...
        'validated_pivot2_config.mat');
end
if ~isfile(sourceFile)
    error('scopeguide:stage08:AcceptedConfigNotFound', ...
        'Accepted pivot2 configuration was not found: %s', sourceFile);
end

loaded = load(sourceFile, 'cfgAccepted');
if ~isfield(loaded, 'cfgAccepted') || ~isstruct(loaded.cfgAccepted)
    error('scopeguide:stage08:InvalidAcceptedConfigFile', ...
        'File does not contain a scalar cfgAccepted structure: %s', ...
        sourceFile);
end

cfg = scopeguide.runtime.upgradeRcmAdmittanceConfig( ...
    loaded.cfgAccepted);
if strlength(options.SourceAcceptedConfigFile) == 0
    cfg = scopeguide.runtime.relocatePackagedConfig(cfg, projectRoot);
end

% Fixed short-term profile accepted by the user on 2026-08-17:
% - keep the current pivot damping and 2.133 deg/s design target;
% - reduce virtual mass to 40% by changing only tau: 0.30 -> 0.12 s;
% - expand the combined dual-pivot relative range: 5.0 -> 7.5 deg;
% - retain soft RCM control and the independent 2 mm hard envelope.
profile = struct();
profile.Name = "stage08_dual_pivot_short_term_20260817";
profile.SourceAcceptedConfigFile = sourceFile;
profile.PivotVirtualMassScale = 0.40;
profile.ReferencePivotTimeConstantSec = 0.30;
profile.PivotTimeConstantSec = ...
    profile.ReferencePivotTimeConstantSec * ...
    profile.PivotVirtualMassScale;
profile.CombinedPivotRangeRad = deg2rad(7.5);
profile.RcmMode = "soft";

cfg.admittance.Pivot.TimeConstantSec = ...
    profile.PivotTimeConstantSec;
cfg.control.RelativePivotLimitRad = ...
    profile.CombinedPivotRangeRad;
cfg.rcm.Mode = profile.RcmMode;
cfg.meta.ActiveTuningProfile = profile.Name;
cfg.meta.ActiveTuningProfileSource = sourceFile;

validateRcmAdmittanceConfig(cfg);

parameters = ...
    scopeguide.control.deriveForceOnlyAdmittanceParameters(cfg);
profile.PivotTargetSteadySpeedRadSec = ...
    cfg.admittance.Pivot.TargetSteadySpeedRadSec;
profile.PivotMaximumSpeedRadSec = cfg.control.PivotMaxRadSec;
profile.PivotDampingNmSecPerRad = parameters.Damping(1);
profile.PivotVirtualMassKgM2 = parameters.VirtualMass(1);
profile.RcmHardRadiusM = cfg.rcm.HardRadiusM;
end
