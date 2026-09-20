function assessment = assessHexHBiasOnlyCandidate( ...
        comparison, returnResidualChangeSensor, options)
%ASSESSHEXHBIASONLYCANDIDATE Gate a bias-only calibration candidate.
% This function is offline only. Passing means that a versioned candidate
% may be written for later review; it never authorizes live activation.

arguments
    comparison (1, 1) struct
    returnResidualChangeSensor (1, 6) double
    options.MaximumValidationForceNormN (1, 1) double = 1.3
    options.MaximumValidationMomentNormNm (1, 1) double = 0.18
    options.MaximumValidationForceVectorRmseN (1, 1) double = 0.9
    options.MaximumValidationMomentVectorRmseNm (1, 1) double = 0.12
    options.MaximumTrainingCenteredForceNormN (1, 1) double = 1.0
    options.MaximumTrainingCenteredMomentNormNm (1, 1) double = 0.12
    options.MaximumReturnForceChangeN (1, 1) double = 0.5
    options.MaximumReturnMomentChangeNm (1, 1) double = 0.05
    options.MaximumBiasShiftForceNormN (1, 1) double = 15
    options.MaximumBiasShiftMomentNormNm (1, 1) double = 0.5
    options.BiasVsJointRelativeRatio (1, 1) double = 1.25
    options.BiasVsJointForceMarginN (1, 1) double = 0.2
    options.BiasVsJointMomentMarginNm (1, 1) double = 0.03
end

validateComparison(comparison);
validateThresholds(options);
if any(~isfinite(returnResidualChangeSensor))
    error('scopeguide:biasCandidate:InvalidReturnChange', ...
        'returnResidualChangeSensor must be finite.');
end

bias = comparison.Models.BiasOnly;
joint = comparison.Models.Joint;
training = comparison.TrainingIndices;
sourceTrainingResidual = ...
    comparison.Models.Original.ResidualSensor(training, :);
centeredTraining = sourceTrainingResidual - ...
    comparison.BiasCorrectionSensor(:).';
trainingForceNorm = vecnorm(centeredTraining(:, 1:3), 2, 2);
trainingMomentNorm = vecnorm(centeredTraining(:, 4:6), 2, 2);

biasForceShift = norm(comparison.BiasCorrectionSensor(1:3));
biasMomentShift = norm(comparison.BiasCorrectionSensor(4:6));
returnForceChange = norm(returnResidualChangeSensor(1:3));
returnMomentChange = norm(returnResidualChangeSensor(4:6));

jointForceAllowance = max( ...
    options.BiasVsJointRelativeRatio * ...
        joint.Validation.ForceVectorRmseN, ...
    joint.Validation.ForceVectorRmseN + ...
        options.BiasVsJointForceMarginN);
jointMomentAllowance = max( ...
    options.BiasVsJointRelativeRatio * ...
        joint.Validation.MomentVectorRmseNm, ...
    joint.Validation.MomentVectorRmseNm + ...
        options.BiasVsJointMomentMarginNm);

gates = struct();
gates.ValidationMaximumForce = ...
    bias.Validation.MaximumForceNormN <= ...
        options.MaximumValidationForceNormN;
gates.ValidationMaximumMoment = ...
    bias.Validation.MaximumMomentNormNm <= ...
        options.MaximumValidationMomentNormNm;
gates.ValidationForceRmse = ...
    bias.Validation.ForceVectorRmseN <= ...
        options.MaximumValidationForceVectorRmseN;
gates.ValidationMomentRmse = ...
    bias.Validation.MomentVectorRmseNm <= ...
        options.MaximumValidationMomentVectorRmseNm;
gates.TrainingCenteredMaximumForce = ...
    max(trainingForceNorm) <= ...
        options.MaximumTrainingCenteredForceNormN;
gates.TrainingCenteredMaximumMoment = ...
    max(trainingMomentNorm) <= ...
        options.MaximumTrainingCenteredMomentNormNm;
gates.ReturnForceRepeatability = ...
    returnForceChange <= options.MaximumReturnForceChangeN;
gates.ReturnMomentRepeatability = ...
    returnMomentChange <= options.MaximumReturnMomentChangeNm;
gates.BiasShiftForcePlausible = ...
    biasForceShift <= options.MaximumBiasShiftForceNormN;
gates.BiasShiftMomentPlausible = ...
    biasMomentShift <= options.MaximumBiasShiftMomentNormNm;
gates.NotMateriallyWorseThanJointForce = ...
    bias.Validation.ForceVectorRmseN <= jointForceAllowance;
gates.NotMateriallyWorseThanJointMoment = ...
    bias.Validation.MomentVectorRmseNm <= jointMomentAllowance;

gateNames = fieldnames(gates);
gateValues = false(numel(gateNames), 1);
for index = 1:numel(gateNames)
    gateValues(index) = gates.(gateNames{index});
end

assessment = struct();
assessment.Passed = all(gateValues);
assessment.ActivationAuthorized = false;
assessment.Gates = gates;
assessment.FailedGates = string(gateNames(~gateValues)).';
assessment.BiasCorrectionSensor = ...
    comparison.BiasCorrectionSensor(:).';
assessment.BiasShiftForceNormN = biasForceShift;
assessment.BiasShiftMomentNormNm = biasMomentShift;
assessment.TrainingCenteredMaximumForceNormN = max(trainingForceNorm);
assessment.TrainingCenteredMaximumMomentNormNm = max(trainingMomentNorm);
assessment.ReturnForceChangeN = returnForceChange;
assessment.ReturnMomentChangeNm = returnMomentChange;
assessment.JointForceAllowanceN = jointForceAllowance;
assessment.JointMomentAllowanceNm = jointMomentAllowance;
assessment.Thresholds = options;
if assessment.Passed
    assessment.Status = "VALIDATED_CANDIDATE_NOT_ACTIVATED";
    assessment.RecommendedAction = ...
        "Review the versioned bias-only candidate; do not activate it automatically.";
elseif bias.Validation.ForceVectorRmseN > jointForceAllowance || ...
        bias.Validation.MomentVectorRmseNm > jointMomentAllowance
    assessment.Status = "BIAS_ONLY_REJECTED";
    assessment.RecommendedAction = ...
        "Bias-only correction is pose-dependent; inspect load/CoG, cable preload, and full recalibration.";
else
    assessment.Status = "BIAS_ONLY_REJECTED";
    assessment.RecommendedAction = ...
        "Resolve failed repeatability/residual gates and repeat the independent validation.";
end
end

function validateComparison(value)
requiredModels = {'Original','BiasOnly','Joint'};
if ~isfield(value, 'Models') || ...
        ~isfield(value, 'TrainingIndices') || ...
        ~isfield(value, 'BiasCorrectionSensor')
    error('scopeguide:biasCandidate:InvalidComparison', ...
        'comparison is incomplete.');
end
for index = 1:numel(requiredModels)
    if ~isfield(value.Models, requiredModels{index})
        error('scopeguide:biasCandidate:InvalidComparison', ...
            'comparison is missing model %s.', requiredModels{index});
    end
end
end

function validateThresholds(options)
names = fieldnames(options);
for index = 1:numel(names)
    value = options.(names{index});
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('scopeguide:biasCandidate:InvalidThreshold', ...
            '%s must be finite and positive.', names{index});
    end
end
end
