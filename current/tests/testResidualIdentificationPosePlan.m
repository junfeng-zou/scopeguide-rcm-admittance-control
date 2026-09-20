function tests = testResidualIdentificationPosePlan
tests = functiontests(localfunctions);
end

function testDefaultPlanHasIndependentSplitAndReturn(testCase)
offsets = defaultOffsets();
current = [100 200 300 170 -20 30];
plan = scopeguide.diagnostics.buildResidualIdentificationPosePlan( ...
    current, offsets, [1 2 3 5 6 8], [4 7 9], 15);
verifyEqual(testCase, plan.TargetCartesianPose(1, :), current, ...
    'AbsTol', 0);
verifyEqual(testCase, plan.TargetCartesianPose(end, :), current, ...
    'AbsTol', 0);
verifyEqual(testCase, plan.TrainingIndices, [1 2 3 5 6 8]);
verifyEqual(testCase, plan.ValidationIndices, [4 7 9]);
verifyLessThanOrEqual(testCase, max(plan.OrientationTransitionDeg), 15);
end

function testOverlappingSplitRejected(testCase)
verifyError(testCase, @() ...
    scopeguide.diagnostics.buildResidualIdentificationPosePlan( ...
        zeros(1, 6), defaultOffsets(), 1:6, [6 7 8], 15), ...
    'scopeguide:residualPlan:InvalidSplit');
end

function offsets = defaultOffsets()
offsets = [0 0 0; 10 0 0; 10 10 0; 0 10 0; -10 10 0; ...
    -10 0 0; -10 -10 0; 0 -10 0; 10 -10 0];
end
