function result = main_rcm_admittance_drag(options)
%MAIN_RCM_ADMITTANCE_DRAG Safe entry point for staged controller work.
% Stage 0 intentionally implements offline initialization only. It never
% constructs a robot/sensor client and cannot send a hardware command.

arguments
    options.Config = struct([])
    options.MotionConfirmation = ""
    options.GenerateEnvironmentReport (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'config'));

if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = options.Config;
end
validateRcmAdmittanceConfig(cfg);

cleanupGuard = onCleanup(@cleanupRuntime);
environment = scopeguide.runtime.collectEnvironmentInfo(projectRoot);
authorization = scopeguide.safety.evaluateMotionAuthorization( ...
    cfg, options.MotionConfirmation);

mode = string(cfg.runtime.Mode);
switch mode
    case "offline_replay"
        status = "stage00_offline_initialized";
    case "live_dry_run"
        error('scopeguide:runtime:StageNotImplemented', ...
            ['live_dry_run is reserved for Stage 7. Stage 0 does not ' ...
            'connect to hardware.']);
    case {"fixture_motion", "phantom_motion"}
        if ~authorization.Allowed
            failed = strjoin(authorization.FailedGates, ', ');
            error('scopeguide:safety:MotionNotAuthorized', ...
                'Physical motion denied. Failed gates: %s.', failed);
        end
        error('scopeguide:runtime:PhysicalMotionNotImplemented', ...
            ['The safety gates passed, but Stage 0 contains no hardware ' ...
            'adapter or motion implementation.']);
    otherwise
        error('scopeguide:config:InvalidMode', ...
            'Unsupported runtime mode: %s.', mode);
end

reportDirectory = "";
if options.GenerateEnvironmentReport && cfg.logging.WriteEnvironmentReport
    reportDirectory = ...
        scopeguide.runtime.writeStage00EnvironmentReport( ...
        projectRoot, cfg, environment);
end

result = struct();
result.Status = status;
result.Mode = mode;
result.Config = cfg;
result.Environment = environment;
result.MotionAuthorization = authorization;
result.DataContracts = struct( ...
    'RobotState', scopeguide.types.robotState(), ...
    'ForceSample', scopeguide.types.forceSample(), ...
    'HandleSample', scopeguide.types.handleSample(), ...
    'ControlDecision', scopeguide.types.controlDecision());
result.ReportDirectory = string(reportDirectory);
result.HardwareConnectionsCreated = false;
result.MotionCommandsSent = false;

clear cleanupGuard;
result.CleanupExecuted = true;
end

function cleanupRuntime()
% Later stages will close adapters here. Stage 0 owns no hardware.
% Keep this callback independent of the caller workspace so it is also
% safe while MATLAB unwinds the function after an expected exception.
end
