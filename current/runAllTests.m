function results = runAllTests()
%RUNALLTESTS Run ScopeGuide offline tests without connecting to hardware.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));

testLocations = { ...
    fullfile(projectRoot, 'tests'), ...
    fullfile(projectRoot, 'force_sensor', 'tests')};
suite = testsuite(testLocations);
results = run(suite);

if nargout == 0
    disp(table(results));
    assert(all([results.Passed]), ...
        'scopeguide:tests:Failure', 'One or more tests failed.');
end
end
